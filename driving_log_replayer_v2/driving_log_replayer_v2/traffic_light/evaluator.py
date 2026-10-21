# Copyright (c) 2023 TIER IV.inc
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

from __future__ import annotations

from pathlib import Path
import pickle
import sys
from typing import TYPE_CHECKING

import lanelet2  # noqa
from lanelet2.core import BasicPoint2d
from lanelet2.core import LineString3d
from lanelet2.geometry import distance
from lanelet2.geometry import to2D
from perception_eval.common.object2d import DynamicObject2D
from perception_eval.common.schema import FrameID
from perception_eval.config import PerceptionEvaluationConfig
from perception_eval.evaluation.result.perception_frame_config import CriticalObjectFilterConfig
from perception_eval.evaluation.result.perception_frame_config import PerceptionPassFailConfig
from perception_eval.manager import PerceptionEvaluationManager
from perception_eval.tool import PerceptionAnalyzer2D

from driving_log_replayer_v2.lanelet2_util import load_map
from driving_log_replayer_v2.post_process.evaluator import Evaluator
from driving_log_replayer_v2.post_process.evaluator import FrameResult
from driving_log_replayer_v2.post_process.evaluator import InvalidReason
from driving_log_replayer_v2.traffic_light import FailResultHolder
from driving_log_replayer_v2.traffic_light import get_traffic_light_label_str

if TYPE_CHECKING:
    from autoware_perception_msgs.msg import TrafficLightGroup
    from geometry_msgs.msg import TransformStamped
    from perception_eval.common.transform import TransformDict
    from perception_eval.evaluation.metrics import MetricsScore
    from perception_eval.evaluation.result.perception_frame_result import PerceptionFrameResult

    from driving_log_replayer_v2.post_process.evaluation_manager import IgnoreFrames
    from driving_log_replayer_v2.post_process.runner import ConvertedData
    from driving_log_replayer_v2.traffic_light.runner import TrafficLightData


class TrafficLightInvalidReason(InvalidReason):
    NO_GROUND_TRUTH = "No Ground Truth"


class TrafficLightEvaluator(Evaluator):
    def __init__(
        self,
        perception_evaluation_config: dict,
        critical_object_filter_config: dict,
        perception_pass_fail_config: dict,
        t4_dataset_path: str,
        result_archive_path: str,
        evaluation_topic: str,
        ignore_frames: IgnoreFrames,
    ) -> None:
        super().__init__(result_archive_path, evaluation_topic)
        self.__ignore_frames = ignore_frames  # not use yet
        self._result_archive_w_topic_path = Path(result_archive_path).joinpath(
            evaluation_topic.lstrip("/").replace("/", ".")
        )

        map_path = Path(t4_dataset_path, "map", "lanelet2_map.osm").as_posix()
        self.__lanelet_map = load_map(map_path)
        self.fail_result_holder = FailResultHolder(self._result_archive_w_topic_path)

        self.__p_cfg = perception_evaluation_config
        self.__c_cfg = critical_object_filter_config
        self.__f_cfg = perception_pass_fail_config
        self.__evaluation_task = self.__p_cfg["evaluation_config_dict"]["evaluation_task"]
        self.__p_cfg["evaluation_config_dict"]["label_prefix"] = "traffic_light"
        self.__p_cfg["evaluation_config_dict"]["count_label_number"] = True
        self.__camera_type: str = self.__p_cfg["camera_type"]
        if not self.check_evaluation_task():
            err_msg = f"Unexpected evaluation task: {self.__evaluation_task}"
            raise ValueError(err_msg)

        evaluation_config: PerceptionEvaluationConfig = PerceptionEvaluationConfig(
            dataset_paths=[t4_dataset_path],
            frame_id=self.__camera_type,
            result_root_directory=Path(
                self._result_archive_w_topic_path,
                "perception_eval_log",
                "result",
                "{TIME}",
            ).as_posix(),
            evaluation_config_dict=self.__p_cfg["evaluation_config_dict"],
            load_raw_data=False,
        )
        # どれを注目物体とするかのparam
        self.__critical_object_filter_config: CriticalObjectFilterConfig = (
            CriticalObjectFilterConfig(
                evaluator_config=evaluation_config,
                target_labels=self.__c_cfg["target_labels"],
            )
        )
        # Pass fail を決めるパラメータ
        self.__frame_pass_fail_config: PerceptionPassFailConfig = PerceptionPassFailConfig(
            evaluator_config=evaluation_config,
            target_labels=self.__f_cfg["target_labels"],
        )
        self.__evaluator = PerceptionEvaluationManager(
            evaluation_config=evaluation_config,
        )
        self.__skip_counter = 0

    def check_evaluation_task(self) -> bool:
        if self.__evaluation_task != "classification2d":
            self._logger.error(
                "Unexpected evaluation task: %s",
                self.__evaluation_task,
            )
            return False
        return True

    def get_final_metrics(self) -> dict:
        with self._result_archive_w_topic_path.joinpath("scene_result.pkl").open("wb") as f:
            pickle.dump(self.__evaluator.frame_results, f)
        self.get_final_result()
        score_dict = {}
        conf_mat_dict = {}
        analyzer = PerceptionAnalyzer2D(self.__evaluator.evaluator_config)
        analyzer.add(self.__evaluator.frame_results)
        result = analyzer.analyze()
        if result.score is not None:
            score_dict = result.score.to_dict()
        if result.confusion_matrix is not None:
            conf_mat_dict = result.confusion_matrix.to_dict()
        self.fail_result_holder.save()
        return {"Score": score_dict, "ConfusionMatrix": conf_mat_dict}

    def list_dynamic_object_2d_from_ros_msg(
        self,
        unix_time: int,
        traffic_light_groups: list[TrafficLightGroup],
        cam2map: TransformDict,
        map_to_baselink: TransformStamped,
    ) -> list[DynamicObject2D]:
        estimated_objects: list[DynamicObject2D] = []
        for signal in traffic_light_groups:
            label = self.__evaluator.evaluator_config.label_converter.convert_label(
                get_traffic_light_label_str(signal.elements),
            )
            confidence: float = max(signal.elements, key=lambda x: x.confidence).confidence
            signal_pos = self.get_traffic_light_pos(
                signal.traffic_light_group_id, cam2map, map_to_baselink
            )

            estimated_object = DynamicObject2D(
                unix_time=unix_time,
                frame_id=FrameID.CAM_TRAFFIC_LIGHT,
                semantic_score=confidence,
                semantic_label=label,
                roi=None,
                uuid=str(signal.traffic_light_group_id),
                position=signal_pos,
            )
            estimated_objects.append(estimated_object)
        return estimated_objects

    def get_traffic_light_pos(
        self,
        traffic_light_uuid: int,
        cam2map: TransformDict,
        map_to_baselink: TransformStamped,
    ) -> tuple[float, float, float]:
        traffic_light_obj = self.__lanelet_map.regulatoryElementLayer.get(traffic_light_uuid)
        ego_position = map_to_baselink.transform.translation
        p2d = BasicPoint2d(ego_position.x, ego_position.y)
        tl_x, tl_y, tl_z, _ = self.get_min_traffic_light_distance(
            traffic_light_obj.trafficLights, p2d
        )
        return cam2map.inv().transform((tl_x, tl_y, tl_z))

    def get_traffic_light_pos_and_dist(
        self,
        traffic_light_uuid: str,
        map_to_baselink: TransformStamped,
    ) -> tuple[float, float, float, float]:
        rtn_distance = self.__p_cfg["evaluation_config_dict"]["max_distance"] + 1.0
        try:
            int_uuid = int(traffic_light_uuid)
        except ValueError:
            return (0.0, 0.0, 0.0, rtn_distance)
        else:
            traffic_light_obj = self.__lanelet_map.regulatoryElementLayer.get(int_uuid)
            ego_position = map_to_baselink.transform.translation
            p2d = BasicPoint2d(ego_position.x, ego_position.y)
            return self.get_min_traffic_light_distance(traffic_light_obj.trafficLights, p2d)

    @staticmethod
    def get_min_traffic_light_distance(
        traffic_lights: list[LineString3d], p2d: BasicPoint2d
    ) -> tuple[float, float, float, float]:
        min_distance: float = sys.float_info.max
        for tl in traffic_lights:
            l2d = to2D(tl)
            cur_distance = distance(l2d, p2d)
            if cur_distance < min_distance:
                min_tl = tl
                min_distance = cur_distance
        min_tl_center = (
            (getattr(min_tl[0], attr) + getattr(min_tl[1], attr)) / 2.0 for attr in ("x", "y", "z")
        )
        return (*min_tl_center, min_distance)

    def evaluate_frame(self, converted_data: ConvertedData) -> FrameResult:
        data: TrafficLightData = converted_data.data
        map_to_baselink = data.map_to_baselink
        header_timestamp_microsec: int = converted_data.header_timestamp
        ground_truth_now_frame = self.__evaluator.get_ground_truth_now_frame(
            header_timestamp_microsec
        )

        if ground_truth_now_frame is None:
            self.__skip_counter += 1
            return FrameResult(
                is_valid=False,
                invalid_reason=TrafficLightInvalidReason.NO_GROUND_TRUTH,
                skip_counter=self.__skip_counter,
            )

        cam2map = ground_truth_now_frame.transforms[(FrameID.CAM_TRAFFIC_LIGHT, FrameID.MAP)]

        for obj in ground_truth_now_frame.objects:
            x, y, z, _ = self.get_traffic_light_pos_and_dist(obj.uuid, map_to_baselink)
            # transform to camera coordinate system
            position = cam2map.inv().transform((x, y, z))
            obj.set_position(position)

        estimated_objects = self.list_dynamic_object_2d_from_ros_msg(
            header_timestamp_microsec,
            data.traffic_light_groups,
            cam2map,
            map_to_baselink,
        )
        frame_result: PerceptionFrameResult = self.__evaluator.add_frame_result(
            unix_time=header_timestamp_microsec,
            ground_truth_now_frame=ground_truth_now_frame,
            estimated_objects=estimated_objects,
            critical_object_filter_config=self.__critical_object_filter_config,
            frame_pass_fail_config=self.__frame_pass_fail_config,
        )
        self.fail_result_holder.add_frame(frame_result)
        return FrameResult(is_valid=True, data=frame_result, skip_counter=self.__skip_counter)

    def get_final_result(self) -> MetricsScore:
        final_metric_score = self.__evaluator.get_scene_result()

        # final result
        self._logger.info("final metrics result %s", final_metric_score)
        return final_metric_score
