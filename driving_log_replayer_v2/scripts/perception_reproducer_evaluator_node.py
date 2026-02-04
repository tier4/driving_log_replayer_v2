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
from typing import Literal

from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_system_msgs.msg import AutowareState
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.publisher import Publisher
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
from driving_log_replayer_v2.result import DummyResult
from driving_log_replayer_v2.result import ResultWriter


@dataclass
class EvaluatorState:
    """Shared state for all evaluators in the perception_reproducer use case."""

    all_evaluators: dict[str, "ConditionGroupEvaluator"]  # All evaluators by group_name

    timeout_s: float  # Test timeout in seconds
    test_ended: bool = False  # Whether test has ended

    engaged_time: float | None = None  # Time when Autoware is engaged (None means not engaged)
    fail_trigger_time: float | None = (
        None  # Time when fail condition was triggered (None means not triggered)
    )


class ConditionGroupEvaluator:
    """Evaluate a condition group."""

    def __init__(
        self,
        group: ConditionGroup,
        state: EvaluatorState,
        parent_evaluator: "ConditionGroupEvaluator | None" = None,
    ) -> None:
        self.group = group
        self.state = state
        self.parent_evaluator = parent_evaluator

        self.triggered = False  # for trigger conditions
        self.is_active = False
        self.summary: str = f"{self.group.group_name} (NotStarted)"
        self.success: bool | None = None  # None means the evaluator is not evaluated even once.

        # Dependents: evaluators that depend on this one (via start_at/end_at)
        self.start_at_dependents: list[ConditionGroupEvaluator] = []
        self.end_at_dependents: list[ConditionGroupEvaluator] = []

        # Result containers
        self.ego_kinematic_result: EgoKinematicResult | None = None
        self.metric_result: MetricResult | None = None
        self.diag_result: DiagnosticsResult | None = None
        self.planning_factor_result: PlanningFactorResult | None = None
        self.nested_evaluators: list[ConditionGroupEvaluator] = []
        self.diag_target_hardware_ids: list[str] = []

        self._initialize_results()

    def _initialize_results(self) -> None:  # noqa: C901
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
            self.diag_target_hardware_ids = diag_conditions_obj.target_hardware_ids
            self.diag_result = DiagnosticsResult(diag_conditions_obj)
        if pf_conditions:
            self.planning_factor_result = PlanningFactorResult(pf_conditions)

    def _notify_children_activation(self, *, activate: bool) -> None:
        for child in self.nested_evaluators:
            if activate:
                child.on_parent_activated()
            else:
                child.on_parent_deactivated()

    def activate(self) -> None:
        """Activate this evaluator and notify children."""
        if self.is_active:
            return
        self.is_active = True
        self._notify_children_activation(activate=True)

    def deactivate(self) -> None:
        """Deactivate this evaluator and notify children."""
        if not self.is_active:
            return
        self.is_active = False
        self._notify_children_activation(activate=False)

    def on_parent_activated(self) -> None:
        """Handle parent evaluator activation. Check if we should activate."""
        if self.group.start_at is not None:
            # Wait for start_at dependency to pass
            ref_evaluator = self.state.all_evaluators.get(self.group.start_at)
            if ref_evaluator is not None and ref_evaluator.triggered:
                self.activate()
        else:
            self.activate()

    def on_parent_deactivated(self) -> None:
        """Handle parent evaluator deactivation. Deactivate this evaluator."""
        self.deactivate()

    def on_dependency_passed(self) -> None:
        """Handle dependency (start_at) passing. Activate if parent is active."""
        if self.parent_evaluator is None or self.parent_evaluator.is_active:
            self.activate()

    def on_dependency_ended(self) -> None:
        """Handle dependency (end_at) passing. Deactivate this evaluator."""
        self.deactivate()

    def _notify_dependents_first_pass(self) -> None:
        """Notify dependents when this evaluator first passes."""
        # Notify start_at dependents to activate
        for dependent in self.start_at_dependents:
            dependent.on_dependency_passed()
        # Notify end_at dependents to deactivate
        for dependent in self.end_at_dependents:
            dependent.on_dependency_ended()

    def evaluate(self) -> None:  # noqa: C901
        """Evaluate this condition group recursively and update internal state."""
        if not self.is_active:  # keep the internal state as is
            return

        results: list[bool] = []
        summaries: list[str] = []

        # Collect results from result containers
        if self.ego_kinematic_result is not None:
            results.append(self.ego_kinematic_result.success)
            summaries.append(f"EgoKinematic: {self.ego_kinematic_result.summary}")

        if self.metric_result is not None:
            results.append(self.metric_result.success)
            summaries.append(f"Metrics: {self.metric_result.summary}")

        if self.diag_result is not None:
            results.append(self.diag_result.success)
            summaries.append(f"Diagnostics: {self.diag_result.summary}")

        if self.planning_factor_result is not None:
            results.append(self.planning_factor_result.success)
            summaries.append(f"PlanningFactors: {self.planning_factor_result.summary}")

        # Recursively evaluate nested evaluators (skip inactive ones)
        for nested_evaluator in self.nested_evaluators:
            if not nested_evaluator.is_active:
                continue
            nested_evaluator.evaluate()  # Update nested evaluator's state
            results.append(nested_evaluator.success)
            summaries.append(f"{nested_evaluator.group.group_name}: {nested_evaluator.summary}")

        # Combine results based on group_type
        if self.group.group_type == "all_of":
            success = all(g for g in results if g is not None) if results else True
        else:  # any_of
            success = any(g for g in results if g is not None) if results else True

        # Update internal state
        self.success = success
        self.summary = (
            f"{self.group.group_name} ({'Passed' if success else 'Failed'}): "
            + "; ".join(summaries)
        )

        # Notify dependents when first passing (for both top-level and nested evaluators)
        if success and not self.triggered:
            self.triggered = True
            self._notify_dependents_first_pass()

    def set_ego_kinematic_frame(
        self, msg: Odometry, acceleration_msg: AccelWithCovarianceStamped | None = None
    ) -> bool:
        updated = False
        if self.is_active and self.ego_kinematic_result is not None:
            self.ego_kinematic_result.set_frame(msg, acceleration_msg)
            updated = True

        for nested_evaluator in self.nested_evaluators:
            updated |= nested_evaluator.set_ego_kinematic_frame(msg, acceleration_msg)
        return updated

    def set_metric_frame(self, msg: MetricArray, topic: str) -> bool:
        updated = False
        if self.is_active and self.metric_result is not None:
            self.metric_result.set_frame(msg, topic)
            updated = True

        for nested_evaluator in self.nested_evaluators:
            updated |= nested_evaluator.set_metric_frame(msg, topic)
        return updated

    def set_diag_frame(self, msg: DiagnosticArray) -> bool:
        if len(msg.status) == 0:
            return False

        updated = False
        diag_status: DiagnosticStatus = msg.status[0]
        if (
            diag_status.hardware_id in self.diag_target_hardware_ids
            and self.is_active
            and self.diag_result is not None
        ):
            self.diag_result.set_frame(msg)
            updated = True

        for nested_evaluator in self.nested_evaluators:
            updated |= nested_evaluator.set_diag_frame(msg)
        return updated

    def set_planning_factor_frame(
        self, msg: PlanningFactorArray, topic: str, latest_kinematic_state: Odometry | None
    ) -> bool:
        updated = False
        if self.is_active and self.planning_factor_result is not None:
            self.planning_factor_result.set_frame(msg, topic, latest_kinematic_state)
            updated = True

        for nested_evaluator in self.nested_evaluators:
            updated |= nested_evaluator.set_planning_factor_frame(
                msg, topic, latest_kinematic_state
            )
        return updated

    def collect_frame_data(self, group_name_prefix: str = "") -> dict[str, dict]:
        """
        Recursively collect frame data from all condition types and nested groups.

        Returns a dictionary mapping condition keys to their frame data.
        """
        conditions: dict[str, dict] = {}
        current_group_name = self.group.group_name
        full_group_name = (
            f"{group_name_prefix}.{current_group_name}" if group_name_prefix else current_group_name
        )

        # Collect frame data from result containers
        result_containers = [
            self.ego_kinematic_result,
            self.metric_result,
            self.diag_result,
            self.planning_factor_result,
        ]
        for result in result_containers:
            if result is not None and result.frame:
                for cond_name, cond_result in result.frame.items():
                    conditions[f"{full_group_name}.{cond_name}"] = cond_result

        # Recursively collect from nested evaluators
        for nested_evaluator in self.nested_evaluators:
            nested_conditions = nested_evaluator.collect_frame_data(full_group_name)
            conditions.update(nested_conditions)

        return conditions


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

        self._test_start_time = self.get_clock().now().nanoseconds / 1e9

        # Evaluators and state variables
        self._pass_evaluators: list[ConditionGroupEvaluator] = []
        self._fail_evaluators: list[ConditionGroupEvaluator] = []
        self.latest_kinematic_state: Odometry | None = None
        self.latest_acceleration: AccelWithCovarianceStamped | None = None

        # Initialize components
        self._initialize_evaluators()
        self._setup_result_writers()
        self._setup_subscriptions()
        self._write_initial_dummy_result()

    def _initialize_evaluators(self) -> None:
        """Initialize evaluators for pass and fail condition groups."""
        all_evaluators: dict[str, ConditionGroupEvaluator] = {}

        self._evaluator_state = EvaluatorState(
            all_evaluators=all_evaluators,
            timeout_s=self._conditions.timeout_s,
        )

        # Create pass evaluators(ConditionGroupEvaluator constructor will create sub-evaluators recursively).
        for group in self._conditions.pass_conditions:
            evaluator = ConditionGroupEvaluator(group, self._evaluator_state)
            self._pass_evaluators.append(evaluator)
            all_evaluators[group.group_name] = evaluator

        # Create fail evaluators (ConditionGroupEvaluator constructor will create sub-evaluators recursively).
        for group in self._conditions.fail_conditions:
            evaluator = ConditionGroupEvaluator(group, self._evaluator_state)
            self._fail_evaluators.append(evaluator)
            all_evaluators[group.group_name] = evaluator

        # Build dependency relationships (dependents list in each evaluator)
        for evaluator in self._pass_evaluators + self._fail_evaluators:
            self._build_dependents(evaluator)

    def _build_dependents(self, evaluator: ConditionGroupEvaluator) -> None:
        """Recursively build dependency relationships (dependents lists)."""
        # Add this evaluator to its dependencies' dependents lists
        if evaluator.group.start_at is not None:
            ref_evaluator = self._evaluator_state.all_evaluators.get(evaluator.group.start_at)
            if ref_evaluator is not None:
                ref_evaluator.start_at_dependents.append(evaluator)

        if evaluator.group.end_at is not None:
            ref_evaluator = self._evaluator_state.all_evaluators.get(evaluator.group.end_at)
            if ref_evaluator is not None:
                ref_evaluator.end_at_dependents.append(evaluator)

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
            self._conditions.pass_conditions,
        )
        self._fail_result_writer: ResultWriter = ResultWriter(
            self._result_archive_path.joinpath("fail_result.jsonl"),
            self.get_clock(),
            self._conditions.fail_conditions,
        )

    def _setup_subscriptions(self) -> None:  # noqa: C901
        """Set up all subscriptions."""
        # Subscribe to kinematic state
        self.__sub_kinematic_state = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.kinematic_state_cb,
            10,
        )

        # Subscribe to acceleration
        self.__sub_acceleration = self.create_subscription(
            AccelWithCovarianceStamped,
            "/localization/acceleration",
            self.acceleration_cb,
            10,
        )

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

        for group in self._conditions.pass_conditions + self._conditions.fail_conditions:
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

    def _write_and_publish_result(
        self, result: dict, writer: ResultWriter, publisher: Publisher
    ) -> None:
        """Write result to file and publish to ROS topic."""
        ros_time = self.get_clock().now().nanoseconds / 1e9
        system_time = writer._system_clock.now().nanoseconds / 1e9  # noqa: SLF001
        result["Stamp"] = {"System": system_time, "ROS": ros_time}
        result_str = writer.write_line(result)
        publisher.publish(String(data=result_str))

    def _write_initial_dummy_result(self) -> None:
        """Write initial dummy result to main result.jsonl."""
        dummy_result = {
            "Result": {
                "Success": True,
                "Summary": "Perception Reproducer Evaluation initialized. Waiting for ENGAGE...",
            },
            "Frame": {},
        }
        self._write_and_publish_result(dummy_result, self._result_writer, self._pub_result)

    def _aggregate_results(self, groups_type: Literal["pass", "fail"]) -> dict:
        """Collect groups and their conditions results in unified structure for top-level Pass / Fail groups."""
        evaluators = self._pass_evaluators if groups_type == "pass" else self._fail_evaluators

        groups: dict[str, dict] = {}
        conditions: dict[str, dict] = {}

        for evaluator in evaluators:
            group_name = evaluator.group.group_name
            groups[group_name] = {
                "Success": evaluator.success,  # it may be None/True/False
                "Summary": evaluator.summary,  # it may be "NotStarted"/"Passed"/"Failed"
            }

            # Recursively collect frame data from this evaluator and its nested evaluators
            evaluator_conditions = evaluator.collect_frame_data()
            conditions.update(evaluator_conditions)

        # Determine overall success based on groups_type
        if groups_type == "fail":
            # For fail conditions: success if no group has failed
            overall_success = not any(g["Success"] is False for g in groups.values())
        else:
            # For pass conditions: success only if all groups have passed
            overall_success = all(g["Success"] is True for g in groups.values())

        return {
            "Result": {
                "Success": overall_success,
            },
            "Frame": {"Groups": groups, "Conditions": conditions},
        }

    def _write_results(
        self,
    ) -> None:
        """Write pass and fail results to their respective writers."""
        # _aggregate_results reads current state (evaluate() should have been called before)
        pass_result = self._aggregate_results("pass")
        self._write_and_publish_result(pass_result, self._pass_result_writer, self._pub_pass_result)

        fail_result = self._aggregate_results("fail")
        self._write_and_publish_result(fail_result, self._fail_result_writer, self._pub_fail_result)

    def _check_termination_conditions(self) -> None:
        """Check termination conditions after evaluating all top-level evaluators."""
        if self._evaluator_state.fail_trigger_time is not None:
            return

        ros_time = self.get_clock().now().nanoseconds / 1e9

        # Check if all pass conditions are met
        if all(e.triggered for e in self._pass_evaluators):
            self._handle_test_termination("All pass conditions met", is_pass=True)
            return

        if any(e.success is False for e in self._fail_evaluators):
            self._evaluator_state.fail_trigger_time = ros_time
            self.get_logger().error(f"Fail condition triggered at time {ros_time}s")
            return

    def acceleration_cb(self, msg: AccelWithCovarianceStamped) -> None:
        self.latest_acceleration = msg

    def autoware_state_cb(self, msg: AutowareState) -> None:
        """Detect Autoware entering DRIVING state and set engaged_time."""
        if msg.state == AutowareState.DRIVING and self._evaluator_state.engaged_time is None:
            self._evaluator_state.engaged_time = self.get_clock().now().nanoseconds / 1e9
            # Activate root evaluators (no parent, no start_at)
            for evaluator in self._pass_evaluators + self._fail_evaluators:
                if evaluator.group.start_at is None:
                    evaluator.activate()
            self.get_logger().info(f"Autoware engaged at: {self._evaluator_state.engaged_time}")

            # write and publish engaged info
            engaged_result = {
                "Result": {
                    "Success": True,
                    "Summary": "Autoware engaged, evaluation started.",
                },
                "Frame": {},
            }
            self._write_and_publish_result(engaged_result, self._result_writer, self._pub_result)

            self.destroy_subscription(self.__sub_autoware_state)
            self.__sub_autoware_state = None

    def _update_and_evaluate(
        self, set_frame_func: Callable[[ConditionGroupEvaluator], bool]
    ) -> None:
        any_updated = False
        for evaluator in self._pass_evaluators + self._fail_evaluators:
            updated = set_frame_func(evaluator)
            if updated:
                evaluator.evaluate()
                any_updated = True

        if any_updated:
            self._write_results()
            self._check_termination_conditions()

    def kinematic_state_cb(self, msg: Odometry) -> None:
        """Update latest kinematic state and propagate ego kinematic updates."""
        self.latest_kinematic_state = msg
        if self._evaluator_state.test_ended or self._evaluator_state.engaged_time is None:
            return

        self._update_and_evaluate(
            lambda e: e.set_ego_kinematic_frame(
                self.latest_kinematic_state, self.latest_acceleration
            )
        )

    def metric_cb(self, msg: MetricArray, topic: str) -> None:
        """Handle metric messages."""
        if self._evaluator_state.test_ended or self._evaluator_state.engaged_time is None:
            return

        self._update_and_evaluate(lambda e: e.set_metric_frame(msg, topic))

    def factor_cb(self, msg: PlanningFactorArray, topic: str) -> None:
        """Handle planning factor messages."""
        if self._evaluator_state.test_ended or self._evaluator_state.engaged_time is None:
            return

        self._update_and_evaluate(
            lambda e: e.set_planning_factor_frame(msg, topic, self.latest_kinematic_state)
        )

    def diag_cb(self, msg: DiagnosticArray) -> None:
        """Handle diagnostic messages."""
        if self._evaluator_state.test_ended or self._evaluator_state.engaged_time is None:
            return

        self._update_and_evaluate(lambda e: e.set_diag_frame(msg))

    def _check_timeout_conditions(self) -> None:
        """Check timeout conditions and terminate test if needed."""
        if self._evaluator_state.test_ended:
            return

        ros_time = self.get_clock().now().nanoseconds / 1e9

        # Check engage timeout (if not engaged yet)
        if self._evaluator_state.engaged_time is None:
            if ros_time - self._test_start_time >= self._evaluator_state.timeout_s:
                self._handle_test_termination("Engage timeout reached", is_pass=False)
                return
            return

        # Check test timeout (after engaged) - deactivate all evaluators
        if ros_time >= self._evaluator_state.engaged_time + self._evaluator_state.timeout_s:
            for evaluator in self._pass_evaluators + self._fail_evaluators:
                evaluator.deactivate()
            self._handle_test_termination("Driving Timeout reached", is_pass=False)
            return

        # Check if we should terminate after fail
        if (
            self._evaluator_state.fail_trigger_time is not None
            and ros_time - self._evaluator_state.fail_trigger_time
            >= self._conditions.terminated_after_fail_s
        ):
            self._handle_test_termination("Fail condition triggered", is_pass=False)
            return

    def _handle_ros_clock_stopped(self) -> None:
        """Handle clock stop detection: terminate test when simulation time stops."""
        self._handle_test_termination("ROS Clock Stopped", is_pass=False)

    def timer_cb(
        self,
        *,
        register_loop_func: Callable | None = None,  # noqa: ARG002
        register_shutdown_func: Callable | None = None,  # noqa: ARG002
    ) -> None:
        """Override timer_cb to register timeout check and clock stop handler."""
        super().timer_cb(
            register_loop_func=self._check_timeout_conditions,
            register_shutdown_func=self._handle_test_termination,
        )

    def _handle_test_termination(self, reason: str, *, is_pass: bool = True) -> None:
        """Handle test termination: evaluate final results and shutdown."""
        if self._evaluator_state.test_ended:
            return
        self._evaluator_state.test_ended = True

        self.get_logger().info(f"Test terminated({'Pass' if is_pass else 'Fail'}: {reason}")

        pass_result = self._aggregate_results("pass")
        fail_result = self._aggregate_results("fail")

        main_result = {
            "Result": {
                "Success": is_pass,
                "Summary": f"Pass Conditions: {pass_result['Result']['Summary']}; Fail Conditions: {fail_result['Result']['Summary']}",
            },
            "Frame": {
                "TerminationReason": reason,
            },
        }
        self._write_and_publish_result(main_result, self._result_writer, self._pub_result)

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
