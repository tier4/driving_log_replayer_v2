#!/usr/bin/env python3

# Copyright (c) 2025 TIER IV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_system_msgs.msg import AutowareState
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from std_msgs.msg import String
from tier4_metric_msgs.msg import MetricArray

from driving_log_replayer_v2.diagnostics import Conditions as DiagnosticsConditions
from driving_log_replayer_v2.diagnostics import DiagCondition
from driving_log_replayer_v2.diagnostics import DiagnosticsResult
from driving_log_replayer_v2.evaluator import DLREvaluatorV2
from driving_log_replayer_v2.evaluator import evaluator_main
from driving_log_replayer_v2.perception_reproducer import ConditionGroup
from driving_log_replayer_v2.perception_reproducer import EgoKinematicCondition
from driving_log_replayer_v2.perception_reproducer import EgoKinematicResult
from driving_log_replayer_v2.perception_reproducer import EgoKinematicTriggerCondition
from driving_log_replayer_v2.perception_reproducer import PerceptionReproducerScenario
from driving_log_replayer_v2.planning_control import MetricCondition
from driving_log_replayer_v2.planning_control import MetricResult
from driving_log_replayer_v2.planning_control import PlanningFactorCondition
from driving_log_replayer_v2.planning_control import PlanningFactorResult
from driving_log_replayer_v2.planning_control import stamp_to_float
from driving_log_replayer_v2.result import DummyResult
from driving_log_replayer_v2.result import ResultWriter


@dataclass
class EvaluatorState:
    """Shared state for all evaluators in the perception_reproducer use case."""

    # Time-related state
    engaged_time: float = 0.0  # Time when Autoware is engaged (0.0 means not engaged)
    timeout_s: float  # Test timeout in seconds

    # Evaluator registry
    all_evaluators: dict[str, "ConditionGroupEvaluator"]  # All evaluators by group_name

    # Test state flags
    test_ended: bool = False  # Whether test has ended
    fail_triggered: bool = False  # Whether any fail condition has been triggered


class ConditionGroupEvaluator:
    """Evaluate a condition group with time window management."""

    def __init__(
        self,
        group: ConditionGroup,
        state: EvaluatorState,
        parent_evaluator: "ConditionGroupEvaluator | None" = None,
    ) -> None:
        self.group = group
        self.state = state
        self.parent_evaluator = parent_evaluator

        self.passed = False
        self.pass_time: float | None = None
        self.is_active = False

        # Dependents: evaluators that depend on this one (via start_at/end_at)
        self._dependents: list[ConditionGroupEvaluator] = []

        # Result containers
        self.ego_kinematic_result: EgoKinematicResult | None = None
        self.metric_result: MetricResult | None = None
        self.diag_result: DiagnosticsResult | None = None
        self.planning_factor_result: PlanningFactorResult | None = None
        self.nested_evaluators: list[ConditionGroupEvaluator] = []

        self._initialize_results()

    def _initialize_results(self) -> None:
        """Initialize result containers based on condition_class in each condition."""
        # Handle nested condition groups first
        for nested_group in self.group.condition_list:
            if isinstance(nested_group, ConditionGroup):
                self.nested_evaluators.append(
                    ConditionGroupEvaluator(nested_group, self.state, parent_evaluator=self)
                )

        # Collect conditions by type using dictionary mapping
        ego_conditions: list[EgoKinematicCondition | EgoKinematicTriggerCondition] = []
        metric_conditions: list[MetricCondition] = []
        diag_conditions: list[DiagCondition] = []
        pf_conditions: list[PlanningFactorCondition] = []

        for item in self.group.condition_list:
            if isinstance(item, (EgoKinematicCondition, EgoKinematicTriggerCondition)):
                ego_conditions.append(item)
            elif isinstance(item, MetricCondition):
                metric_conditions.append(item)
            elif isinstance(item, DiagCondition):
                diag_conditions.append(item)
            elif isinstance(item, PlanningFactorCondition):
                pf_conditions.append(item)

        # Initialize result containers
        if ego_conditions:
            self.ego_kinematic_result = EgoKinematicResult(ego_conditions)
        if metric_conditions:
            self.metric_result = MetricResult(metric_conditions)
        if diag_conditions:
            diag_conditions_obj = DiagnosticsConditions(DiagConditions=diag_conditions)
            self.diag_result = DiagnosticsResult(diag_conditions_obj)
        if pf_conditions:
            self.planning_factor_result = PlanningFactorResult(pf_conditions)

    def activate(self) -> None:
        """Activate this evaluator and notify dependents."""
        if self.is_active:
            return
        self.is_active = True
        self._notify_children(activate=True)
        self._notify_dependents(activate=True)

    def deactivate(self) -> None:
        """Deactivate this evaluator and notify dependents."""
        if not self.is_active:
            return
        self.is_active = False
        self._notify_children(activate=False)
        self._notify_dependents(activate=False)

    def on_parent_activated(self) -> None:
        """Handle parent evaluator activation. Check if we should activate."""
        if self.group.start_at is not None:
            # Wait for start_at dependency to pass
            ref_evaluator = self.state.all_evaluators.get(self.group.start_at)
            if ref_evaluator is not None and ref_evaluator.passed:
                self.activate()
        elif self.state.engaged_time > 0:
            # No start_at, activate immediately when engaged
            self.activate()

    def on_parent_deactivated(self) -> None:
        """Handle parent evaluator deactivation. Deactivate this evaluator."""
        self.deactivate()

    def on_dependency_passed(self, dependency_name: str) -> None:
        """Handle dependency (start_at) passing. Activate if parent is active."""
        if self.group.start_at == dependency_name and (
            self.parent_evaluator is None or self.parent_evaluator.is_active
        ):
            self.activate()

    def on_dependency_ended(self, dependency_name: str) -> None:
        """Handle dependency (end_at) passing. Deactivate this evaluator."""
        if self.group.end_at == dependency_name:
            self.deactivate()

    def _notify_children(self, *, activate: bool) -> None:
        """Notify all child evaluators to activate or deactivate."""
        for child in self.nested_evaluators:
            if activate:
                child.on_parent_activated()
            else:
                child.on_parent_deactivated()

    def _notify_dependents(self, *, activate: bool) -> None:
        """Notify all dependents (via start_at/end_at) that this evaluator state changed."""
        for dependent in self._dependents:
            if activate and dependent.group.start_at == self.group.group_name:
                dependent.on_dependency_passed(self.group.group_name)
            elif not activate and dependent.group.end_at == self.group.group_name:
                dependent.on_dependency_ended(self.group.group_name)

    def evaluate(self) -> tuple[bool, str]:
        """Evaluate this condition group. Returns (success, summary)."""
        if not self.is_active:
            return (False, f"{self.group.group_name} (NotActive)")

        results, summaries = self._collect_all_results()

        # Combine results based on group_type
        if self.group.group_type == "all_of":
            success = all(results) if results else True
        else:  # any_of
            success = any(results) if results else True

        summary = f"{self.group.group_name} ({'Passed' if success else 'Failed'}): " + "; ".join(
            summaries
        )
        return success, summary

    def _collect_all_results(self) -> tuple[list[bool], list[str]]:
        """Collect results from all condition types and nested groups."""
        results: list[bool] = []
        summaries: list[str] = []

        if self.ego_kinematic_result is not None:
            self.ego_kinematic_result.update()
            results.append(self.ego_kinematic_result._success)
            summaries.append(f"EgoKinematic: {self.ego_kinematic_result._summary}")

        if self.metric_result is not None:
            self.metric_result.update()
            results.append(self.metric_result._success)
            summaries.append(f"Metrics: {self.metric_result._summary}")

        if self.diag_result is not None:
            self.diag_result.update()
            results.append(self.diag_result._success)
            summaries.append(f"Diagnostics: {self.diag_result._summary}")

        if self.planning_factor_result is not None:
            self.planning_factor_result.update()
            results.append(self.planning_factor_result._success)
            summaries.append(f"PlanningFactors: {self.planning_factor_result._summary}")

        for nested_evaluator in self.nested_evaluators:
            nested_success, nested_summary = nested_evaluator.evaluate()
            results.append(nested_success)
            summaries.append(f"{nested_evaluator.group.group_name}: {nested_summary}")

        return results, summaries


class PerceptionReproducerEvaluator(DLREvaluatorV2):
    def __init__(
        self,
        name: str,
        scenario_class: Callable = PerceptionReproducerScenario,
        result_class: Callable = DummyResult,
    ) -> None:
        super().__init__(
            name,
            scenario_class,
            result_class,
            "/driving_log_replayer/perception_reproducer/results",
        )
        self._scenario: PerceptionReproducerScenario
        self._result: DummyResult

        # Get conditions
        self._conditions = self._scenario.Evaluation.Conditions

        self._test_start_time = stamp_to_float(self.get_clock().now().to_msg())
        self._fail_trigger_time: float | None = None

        # Initialize components
        self._initialize_evaluators()
        self._setup_result_writers()
        self._setup_subscriptions()
        self._setup_kinematic_subscriptions()
        self._write_initial_dummy_result()

    def _initialize_evaluators(self) -> None:
        """Initialize evaluators for pass and fail condition groups."""
        # Create shared state first (with empty dict, will be populated)
        all_evaluators: dict[str, ConditionGroupEvaluator] = {}

        self._evaluator_state = EvaluatorState(
            engaged_time=0.0,
            timeout_s=self._conditions.timeout_s,
            all_evaluators=all_evaluators,
        )

        # Create pass evaluators
        self._pass_evaluators: list[ConditionGroupEvaluator] = []
        for group in self._conditions.pass_conditions:
            evaluator = ConditionGroupEvaluator(group, self._evaluator_state)
            self._pass_evaluators.append(evaluator)
            all_evaluators[group.group_name] = evaluator

        # Create fail evaluators
        self._fail_evaluators: list[ConditionGroupEvaluator] = []
        for group in self._conditions.fail_conditions:
            evaluator = ConditionGroupEvaluator(group, self._evaluator_state)
            self._fail_evaluators.append(evaluator)
            all_evaluators[group.group_name] = evaluator

        # Build dependency relationships (dependents list in each evaluator)
        for evaluator in self._pass_evaluators + self._fail_evaluators:
            self._build_dependents(evaluator)

    def _build_dependents(self, evaluator: ConditionGroupEvaluator) -> None:
        """Recursively build dependency relationships (dependents list)."""
        # Add this evaluator to its dependencies' dependents list
        if evaluator.group.start_at is not None:
            ref_evaluator = self._evaluator_state.all_evaluators.get(evaluator.group.start_at)
            if ref_evaluator is not None:
                ref_evaluator._dependents.append(evaluator)

        if evaluator.group.end_at is not None:
            ref_evaluator = self._evaluator_state.all_evaluators.get(evaluator.group.end_at)
            if ref_evaluator is not None:
                ref_evaluator._dependents.append(evaluator)

        # Recursively process nested evaluators
        for nested_evaluator in evaluator.nested_evaluators:
            self._build_dependents(nested_evaluator)

    def _setup_result_writers(self) -> None:
        """Set up result writers for pass, fail, and main results."""
        self._pub_pass_result = self.create_publisher(
            String, "/driving_log_replayer/perception_reproducer/pass_results", 1
        )
        self._pub_fail_result = self.create_publisher(
            String, "/driving_log_replayer/perception_reproducer/fail_results", 1
        )
        self._pass_result_writer: ResultWriter = ResultWriter(
            self._result_archive_path.joinpath("pass_result.jsonl"),
            self.get_clock(),
            {"pass_conditions": [g.model_dump() for g in self._conditions.pass_conditions]},
        )
        self._fail_result_writer: ResultWriter = ResultWriter(
            self._result_archive_path.joinpath("fail_result.jsonl"),
            self.get_clock(),
            {"fail_conditions": [g.model_dump() for g in self._conditions.fail_conditions]},
        )

    def _setup_kinematic_subscriptions(self) -> None:
        """Set up subscriptions for kinematic state and acceleration."""
        self.latest_kinematic_state: Odometry | None = None
        self.latest_acceleration: AccelWithCovarianceStamped | None = None
        self.__sub_kinematic_state = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.kinematic_state_cb,
            10,
        )
        self.__sub_acceleration = self.create_subscription(
            AccelWithCovarianceStamped,
            "/localization/acceleration",
            self.acceleration_cb,
            10,
        )

    def _write_initial_dummy_result(self) -> None:
        """Write initial dummy result to main result.jsonl."""
        dummy_result = {
            "Result": {
                "Success": True,
                "Summary": "Perception Reproducer Evaluation initialized. Waiting for ENGAGE...",
            },
            "Stamp": {"System": 0.0},
            "Frame": {},
        }
        dummy_result_str = self._result_writer.write_line(dummy_result)
        self._pub_result.publish(String(data=dummy_result_str))

    def _setup_subscriptions(self) -> None:
        """Set up subscriptions for all conditions."""
        # Collect all unique topics
        metric_topics = set()
        pf_topics = set()
        diag_hardware_ids = set()

        def collect_topics(group: ConditionGroup) -> None:
            for item in group.condition_list:
                if isinstance(item, MetricCondition):
                    metric_topics.add(item.topic)
                elif isinstance(item, PlanningFactorCondition):
                    pf_topics.add(item.topic)
                elif isinstance(item, DiagCondition):
                    diag_hardware_ids.add(item.hardware_id)
                elif isinstance(item, ConditionGroup):
                    collect_topics(item)

        for group in self._conditions.pass_conditions:
            collect_topics(group)
        for group in self._conditions.fail_conditions:
            collect_topics(group)

        # Subscribe to metric topics
        self.__sub_metrics = {}
        for topic in metric_topics:
            self.__sub_metrics[topic] = self.create_subscription(
                MetricArray,
                topic,
                lambda msg, t=topic: self.metric_cb(msg, t),
                3,
            )

        # Subscribe to planning factor topics
        self.__sub_factors = {}
        for topic in pf_topics:
            self.__sub_factors[topic] = self.create_subscription(
                PlanningFactorArray,
                topic,
                lambda msg, t=topic: self.factor_cb(msg, t),
                3,
            )

        # Subscribe to diagnostics
        if diag_hardware_ids:
            self.__sub_diag = self.create_subscription(
                DiagnosticArray,
                "/diagnostics",
                self.diag_cb,
                300,
            )

        # Subscribe to AutowareState to detect engage
        self.__sub_autoware_state = self.create_subscription(
            AutowareState,
            "/autoware/state",
            self.autoware_state_cb,
            10,
        )

    def acceleration_cb(self, msg: AccelWithCovarianceStamped) -> None:
        """Update latest acceleration."""
        self.latest_acceleration = msg

    def autoware_state_cb(self, msg: AutowareState) -> None:
        """Detect Autoware entering DRIVING state and set engaged_time."""
        if msg.state == AutowareState.DRIVING and self._evaluator_state.engaged_time == 0.0:
            self._evaluator_state.engaged_time = stamp_to_float(self.get_clock().now().to_msg())
            # Activate root evaluators (no parent, no start_at)
            for evaluator in self._pass_evaluators + self._fail_evaluators:
                if evaluator.parent_evaluator is None and evaluator.group.start_at is None:
                    evaluator.activate()
            self.get_logger().info(
                f"Autoware engaged at time {self._evaluator_state.engaged_time}s"
            )
            self.destroy_subscription(self.__sub_autoware_state)
            self.__sub_autoware_state = None

    def _collect_result(self, evaluators: list[ConditionGroupEvaluator]) -> dict:
        """Collect groups and their conditions results in unified structure."""
        groups: dict[str, dict] = {}
        conditions: dict[str, dict] = {}
        summaries: list[str] = []

        for evaluator in evaluators:
            if not evaluator.is_active:
                continue

            success, summary = evaluator.evaluate()
            group_name = evaluator.group.group_name
            groups[group_name] = {"Success": success, "Summary": summary}
            summaries.append(summary)

            result_containers = [
                evaluator.ego_kinematic_result,
                evaluator.metric_result,
                evaluator.diag_result,
                evaluator.planning_factor_result,
            ]
            for result in result_containers:
                if result is not None:
                    result.update()
                    if result.frame:
                        for cond_name, cond_result in result.frame.items():
                            conditions[f"{group_name}.{cond_name}"] = cond_result

        return {
            "Result": {
                "Success": all(g["Success"] for g in groups.values()) if groups else True,
                "Summary": "; ".join(summaries) if summaries else "No active groups",
            },
            "Frame": {"Groups": groups, "Conditions": conditions},
        }

    def _collect_pass_result(self) -> dict:
        """Collect pass groups and their conditions results in unified structure."""
        return self._collect_result(self._pass_evaluators)

    def _collect_fail_result(self) -> dict:
        """Collect fail groups and their conditions results in unified structure."""
        return self._collect_result(self._fail_evaluators)

    def _write_results(self) -> None:
        """Write pass and fail results to their respective writers."""
        current_time = stamp_to_float(self.get_clock().now().to_msg())

        pass_result = self._collect_pass_result()
        pass_result["Stamp"] = {"System": current_time, "ROS": current_time}
        pass_result_str = self._pass_result_writer.write_line(pass_result)
        self._pub_pass_result.publish(String(data=pass_result_str))

        fail_result = self._collect_fail_result()
        fail_result["Stamp"] = {"System": current_time, "ROS": current_time}
        fail_result_str = self._fail_result_writer.write_line(fail_result)
        self._pub_fail_result.publish(String(data=fail_result_str))

    def _propagate_group_update(self, evaluator: ConditionGroupEvaluator, *, is_pass: bool) -> None:
        """Propagate condition update upward to parent groups and check termination conditions."""
        if not evaluator.is_active:
            return

        success, _ = evaluator.evaluate()
        group_name = evaluator.group.group_name
        state_changed = False
        current_time = stamp_to_float(self.get_clock().now().to_msg())

        if is_pass:
            if not evaluator.passed and success:
                evaluator.passed = True
                evaluator.pass_time = current_time
                state_changed = True
                self.get_logger().info(f"Pass group '{group_name}' passed at time {current_time}s")
                # Notify dependents that this evaluator passed (for start_at dependencies)
                evaluator._notify_dependents(activate=True)
                if all(e.passed for e in self._pass_evaluators):
                    self._handle_test_termination("All pass conditions met")
                    return
        elif not success and not self._evaluator_state.fail_triggered:
            self._evaluator_state.fail_triggered = True
            self._fail_trigger_time = current_time
            self.get_logger().error(
                f"Fail condition '{group_name}' triggered at time {current_time}s"
            )

        if state_changed and evaluator.parent_evaluator is not None:
            self._propagate_group_update(
                evaluator.parent_evaluator,
                is_pass=evaluator.parent_evaluator in self._pass_evaluators,
            )

    def kinematic_state_cb(self, msg: Odometry) -> None:
        """Update latest kinematic state and propagate ego kinematic updates."""
        self.latest_kinematic_state = msg
        if self._evaluator_state.test_ended:
            return

        # Fallback: if AutowareState subscription hasn't set engaged_time yet,
        # use first kinematic state as backup (should not happen in normal operation)
        if self._evaluator_state.engaged_time == 0.0:
            self._evaluator_state.engaged_time = stamp_to_float(msg.header.stamp)
            for evaluator in self._pass_evaluators + self._fail_evaluators:
                if evaluator.parent_evaluator is None and evaluator.group.start_at is None:
                    evaluator.activate()
            self.get_logger().warning(
                "engaged_time set from kinematic_state (fallback). AutowareState subscription should handle this."
            )
            return

        # Update ego kinematic for all active evaluators
        updated_evaluators: set[ConditionGroupEvaluator] = set()
        for evaluator in self._pass_evaluators + self._fail_evaluators:
            if evaluator.is_active and evaluator.ego_kinematic_result is not None:
                evaluator.ego_kinematic_result.set_frame(
                    self.latest_kinematic_state, self.latest_acceleration
                )
                updated_evaluators.add(evaluator)

        for evaluator in updated_evaluators:
            self._propagate_group_update(evaluator, is_pass=evaluator in self._pass_evaluators)
        if updated_evaluators:
            self._write_results()

    def _update_condition_result(
        self,
        condition_type: type,
        match_func: Callable[[Any, ConditionGroupEvaluator], bool],
        update_func: Callable[[ConditionGroupEvaluator], None],
    ) -> None:
        """Update condition results and propagate changes."""
        updated_evaluators: set[ConditionGroupEvaluator] = set()
        for evaluator in self._pass_evaluators + self._fail_evaluators:
            if not evaluator.is_active:
                continue
            for item in evaluator.group.condition_list:
                if isinstance(item, condition_type) and match_func(item, evaluator):
                    update_func(evaluator)
                    updated_evaluators.add(evaluator)
                    break

        for evaluator in updated_evaluators:
            self._propagate_group_update(evaluator, is_pass=evaluator in self._pass_evaluators)
        if updated_evaluators:
            self._write_results()

    def metric_cb(self, msg: MetricArray, topic: str) -> None:
        """Handle metric messages."""
        if self._evaluator_state.test_ended or self._evaluator_state.engaged_time == 0.0:
            return
        self._update_condition_result(
            MetricCondition,
            lambda cond, _: cond.topic == topic,
            lambda evaluator: evaluator.metric_result.set_frame(msg, topic),
        )

    def factor_cb(self, msg: PlanningFactorArray, topic: str) -> None:
        """Handle planning factor messages."""
        if self._evaluator_state.test_ended or self._evaluator_state.engaged_time == 0.0:
            return
        self._update_condition_result(
            PlanningFactorCondition,
            lambda cond, _: cond.topic == topic,
            lambda evaluator: evaluator.planning_factor_result.set_frame(
                msg, topic, self.latest_kinematic_state
            ),
        )

    def diag_cb(self, msg: DiagnosticArray) -> None:
        """Handle diagnostic messages."""
        if (
            self._evaluator_state.test_ended
            or self._evaluator_state.engaged_time == 0.0
            or not msg.status
        ):
            return
        hardware_ids = {status.hardware_id for status in msg.status}
        self._update_condition_result(
            DiagCondition,
            lambda cond, _: cond.hardware_id in hardware_ids,
            lambda evaluator: evaluator.diag_result.set_frame(msg),
        )

    def timer_cb(
        self,
        *,
        register_loop_func: Callable | None = None,
        register_shutdown_func: Callable | None = None,  # noqa: ARG002
    ) -> None:
        """Override timer_cb to only check timeout conditions."""
        if self._evaluator_state.test_ended:
            return

        current_time = stamp_to_float(self.get_clock().now().to_msg())

        # Check engage timeout (if not engaged yet)
        if self._evaluator_state.engaged_time == 0.0:
            if current_time - self._test_start_time >= self._evaluator_state.timeout_s:
                self._handle_test_termination("Engage timeout reached")
                return
            return

        # Check test timeout (after engaged) - deactivate all evaluators
        if current_time >= self._evaluator_state.engaged_time + self._evaluator_state.timeout_s:
            for evaluator in self._pass_evaluators + self._fail_evaluators:
                evaluator.deactivate()
            self._handle_test_termination("Timeout reached")
            return

        # Check if we should terminate after fail
        if (
            self._evaluator_state.fail_triggered
            and self._fail_trigger_time is not None
            and current_time - self._fail_trigger_time >= self._conditions.terminated_after_fail_s
        ):
            self._handle_test_termination("Fail condition triggered")
            return

        if register_loop_func is not None:
            register_loop_func()

    def _handle_test_termination(self, reason: str) -> None:
        """Handle test termination: evaluate final results and shutdown."""
        if self._evaluator_state.test_ended:
            return
        self._evaluator_state.test_ended = True

        self.get_logger().info(f"Test terminated: {reason}")

        current_time = stamp_to_float(self.get_clock().now().to_msg())

        pass_result = self._collect_pass_result()
        fail_result = self._collect_fail_result()

        overall_success = pass_result["Result"]["Success"] and fail_result["Result"]["Success"]

        # Write final sub-condition results (pass_result.jsonl, fail_result.jsonl)
        pass_result["Stamp"] = {"System": current_time, "ROS": current_time}
        pass_result["Frame"]["TerminationReason"] = reason
        pass_result_str = self._pass_result_writer.write_line(pass_result)
        self._pub_pass_result.publish(String(data=pass_result_str))

        fail_result["Stamp"] = {"System": current_time, "ROS": current_time}
        fail_result["Frame"]["TerminationReason"] = reason
        fail_result_str = self._fail_result_writer.write_line(fail_result)
        self._pub_fail_result.publish(String(data=fail_result_str))

        # Write main result.jsonl (summary, similar to planning_control)
        main_result = {
            "Result": {
                "Success": overall_success,
                "Summary": f"Pass: {pass_result['Result']['Summary']}; Fail: {fail_result['Result']['Summary']}",
            },
            "Stamp": {"System": current_time, "ROS": current_time},
            "Frame": {
                "TerminationReason": reason,
                "PassConditions": pass_result["Frame"]["Groups"],
                "FailConditions": fail_result["Frame"]["Groups"],
            },
        }
        main_result_str = self._result_writer.write_line(main_result)
        self._pub_result.publish(String(data=main_result_str))

        # Close writers
        self._pass_result_writer.close()
        self._fail_result_writer.close()
        self._result_writer.close()

        # Shutdown
        rclpy.shutdown()


@evaluator_main
def main() -> DLREvaluatorV2:
    return PerceptionReproducerEvaluator("perception_reproducer_evaluator")


if __name__ == "__main__":
    main()
