# Overview

driving_log_replayer_v2 is a package that runs Autoware in an open loop by supplying previously recorded input data using log(rosbag2) API.
The package gathers information and evaluates topics output produced by Autoware.
This software is used to verify the performance of each Autoware component and for software regression testing.

## Related Documents

1. [AutowareDocumentation](https://autowarefoundation.github.io/autoware-documentation/main/)
2. [WebAutoDocumentation](https://docs.web.auto/)

## Related repositories

1. [ros2bag_extensions](https://github.com/tier4/ros2bag_extensions)
2. [perception_eval](https://github.com/tier4/autoware_perception_evaluation)
3. [perception_dataset](https://github.com/tier4/tier4_perception_dataset)

## Architecture

driving_log_replayer_v2 package contains an evaluation node that extends Autoware's standard functionality.
The architecture graph is shown below.

![architecture](images/architecture.drawio.svg)

## Package structure

The evaluation node works in the following manner:

- reads a scenario describing the conditions of positive evaluation
- launches autoware
- outputs the evaluation result in a JSON file format

The details of the node's operation are shown in the figure below.

![overview](images/overview.drawio.svg)

## Example usage flow

1. Acquire rosbags for evaluation using a real-world vehicle.
2. Filter the acquired rosbags to contain only sufficient input topics in required period of time
   - For this purpose please use [ros2bag_extensions](https://github.com/tier4/ros2bag_extensions) package (developed by TIER IV). To properly filter the input rosbag:
   - See docs/use_case/ documentations for which topics to leave in the filter.
3. Create an evaluation scenario
   1. Example scenarios could be found in the repository's [sample folder](https://github.com/tier4/driving_log_replayer_v2/tree/main/sample)
   2. Refer to the [format definition](../scenario_format/index.md) section of this document for description contents.
4. Create a dataset
   1. localization, eagleye, yabloc, ar_tag_based_localizer, and performance_diag are optional if you will not use [Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction).
   2. Create up to T4 non-annotated format data with reference to [perception_dataset tools_overview](https://github.com/tier4/tier4_perception_dataset/blob/main/docs/tools_overview.md).
   3. If you create T4 non-annotated format data, it is possible to check the contents of the data set on [Vehicle Data Search](https://docs.web.auto/user-manuals/vehicle-data-search/quick-start#t4-dataset-%E3%81%AE%E5%8B%95%E7%94%BB%E8%A1%A8%E7%A4%BA).
5. If the node should test obstacle_segmentation, perception, perception_2d, or traffic_light stacks, please annotate with an annotation tool that supports conversion to t4_dataset.
   1. [Deepen.AI](https://www.deepen.ai/) is available.
   2. By adding conversion functionality to [perception_dataset](https://github.com/tier4/tier4_perception_dataset), it becomes possible to use other annotation tools as well.
6. Perform the evaluation.
