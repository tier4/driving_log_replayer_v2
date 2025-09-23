# Release Notes

## Version 3.13.1

Minor Tweak

| Module             | Feature          | Brief summary                                                             | Pull request                                                        | Contributor                                     |
| ------------------ | ---------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------------- | ----------------------------------------------- |
| `common`           | chore            | refactor time unit comment and implementation                             | [#218](https://github.com/tier4/driving_log_replayer_v2/issues/218) | [MasatoSaeki](https://github.com/MasatoSaeki)   |
| `localization`     | arguments        | add args to switch to enable to use localization analysis in post-process | [#217](https://github.com/tier4/driving_log_replayer_v2/issues/217) | [MasatoSaeki](https://github.com/MasatoSaeki)   |
| `perception`       | arguments        | add args to switch to enable to use perception analysis in post-process   | [#214](https://github.com/tier4/driving_log_replayer_v2/issues/214) | [MasatoSaeki](https://github.com/MasatoSaeki)   |
| `traffic_light`    | arguments        | enable to specify the traffic light topic                                 | [#213](https://github.com/tier4/driving_log_replayer_v2/issues/213) | [MasatoSaeki](https://github.com/MasatoSaeki)   |
| `planning_control` | result           | additional fix for planning-factor result                                 | [#216](https://github.com/tier4/driving_log_replayer_v2/issues/216) | [xtk8532704](https://github.com/xtk8532704)     |
| `planning_control` | result           | a init bug of planning-factor result                                      | [#215](https://github.com/tier4/driving_log_replayer_v2/issues/215) | [xtk8532704](https://github.com/xtk8532704)     |
| `common`           | result and topic | publish conditions and results to topics                                  | [#211](https://github.com/tier4/driving_log_replayer_v2/issues/211) | [xtk8532704](https://github.com/xtk8532704)     |
| `perception`       | remap            | add all perception remap                                                  | [#212](https://github.com/tier4/driving_log_replayer_v2/issues/212) | [badai-nguyen](https://github.com/badai-nguyen) |
| `localization`     | msg_type         | update srv type for InitializeLocalization                                | [#208](https://github.com/tier4/driving_log_replayer_v2/issues/208) | [TaikiYamada4](https://github.com/TaikiYamada4) |
| `localization`     | condition        | add check diagnostics flag condition                                      | [#205](https://github.com/tier4/driving_log_replayer_v2/issues/205) | [TaikiYamada4](https://github.com/TaikiYamada4) |

## Version 3.13.0

Major changes

| Module                | Feature   | Brief summary                                                                | Pull request                                                        | Contributor                                         |
| --------------------- | --------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------------- | --------------------------------------------------- |
| `perception`          | criteria  | add the new criteria stop_reason                                             | [#202](https://github.com/tier4/driving_log_replayer_v2/issues/202) | [MasatoSaeki](https://github.com/MasatoSaeki)       |
| `localization`        | scenario  | make conditions detachable                                                   | [#203](https://github.com/tier4/driving_log_replayer_v2/issues/203) | [TaikiYamada4](https://github.com/TaikiYamada4)     |
| `ground_segmentation` | algorithm | update ground segmentation evaluator to be compatible with lidar-seg dataset | [#201](https://github.com/tier4/driving_log_replayer_v2/issues/201) | [nanoshimarobot](https://github.com/nanoshimarobot) |

## Version 3.12.0

Major changes

| Module             | Feature  | Brief summary                                              | Pull request                                                        | Contributor                                             |
| ------------------ | -------- | ---------------------------------------------------------- | ------------------------------------------------------------------- | ------------------------------------------------------- |
| `planning_control` | scenario | add the behavior condition                                 | [#193](https://github.com/tier4/driving_log_replayer_v2/issues/193) | [xtk8532704](https://github.com/xtk8532704)             |
| `perception`       | criteria | add validation for CUSTOM of CriteriaLevel                 | [#188](https://github.com/tier4/driving_log_replayer_v2/issues/188) | [MasatoSaeki](https://github.com/MasatoSaeki)           |
| `common`           | criteria | add naming of condition and criteria                       | [#194](https://github.com/tier4/driving_log_replayer_v2/issues/194) | [xtk8532704](https://github.com/xtk8532704)             |
| `common`           | pre-task | publish the topic in rosbag as pre-task before play rosbag | [#192](https://github.com/tier4/driving_log_replayer_v2/issues/192) | [MasatoSaeki](https://github.com/MasatoSaeki)           |
| `lanelet`          | map      | support multiple map loading                               | [#189](https://github.com/tier4/driving_log_replayer_v2/issues/189) | [ManabuSasaki0811](https://github.com/ManabuSasaki0811) |

## Version 3.11.1

Minor Tweak

| Module             | Feature       | Brief summary                                                                | Pull request                                                        | Contributor                                   |
| ------------------ | ------------- | ---------------------------------------------------------------------------- | ------------------------------------------------------------------- | --------------------------------------------- |
| `perception`       | test          | update test caused by adding the `nuscene_object_results` in perception_eval | [#198](https://github.com/tier4/driving_log_replayer_v2/issues/198) | [MasatoSaeki](https://github.com/MasatoSaeki) |
| `planning_control` | remap         | fix the invalid remapping                                                    | [#196](https://github.com/tier4/driving_log_replayer_v2/issues/196) | [xtk8532704](https://github.com/xtk8532704)   |
| `perception`       | evaluate      | fix variable name                                                            | [#183](https://github.com/tier4/driving_log_replayer_v2/issues/183) | [vividf](https://github.com/vividf)           |
| `perception`       | fp_validation | add TN to jsonl for fp validation                                            | [#187](https://github.com/tier4/driving_log_replayer_v2/issues/187) | [MasatoSaeki](https://github.com/MasatoSaeki) |
| `perception`       | fp_validation | handle TrackedObjects and PredictedObjects type in fp validation             | [#186](https://github.com/tier4/driving_log_replayer_v2/issues/186) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.11.0

Major changes

| Module       | Feature       | Brief summary                                | Pull request                                                        | Contributor                                     |
| ------------ | ------------- | -------------------------------------------- | ------------------------------------------------------------------- | ----------------------------------------------- |
| `perception` | scenario      | fix scenario for perception                  | [#185](https://github.com/tier4/driving_log_replayer_v2/issues/185) | [MasatoSaeki](https://github.com/MasatoSaeki)   |
| `perception` | scenario      | add key to specify degradation topic         | [#182](https://github.com/tier4/driving_log_replayer_v2/issues/182) | [MasatoSaeki](https://github.com/MasatoSaeki)   |
| `perception` | remap         | add x2_perception remap profile              | [#178](https://github.com/tier4/driving_log_replayer_v2/issues/178) | [badai-nguyen](https://github.com/badai-nguyen) |
| `common`     | ci            | update ruff version                          | [#176](https://github.com/tier4/driving_log_replayer_v2/issues/176) | [hayato-m126](https://github.com/hayato-m126)   |
| `perception` | calculation   | use function get_num_gt() of perception_eval | [#177](https://github.com/tier4/driving_log_replayer_v2/issues/177) | [MasatoSaeki](https://github.com/MasatoSaeki)   |
| `perception` | fp_validation | enable to use fp_validation                  | [#175](https://github.com/tier4/driving_log_replayer_v2/issues/175) | [MasatoSaeki](https://github.com/MasatoSaeki)   |
| `perception` | common        | import from absolute source                  | [#172](https://github.com/tier4/driving_log_replayer_v2/issues/172) | [vividf](https://github.com/vividf)             |

## Version 3.10.2

Minor Tweak

| Module       | Feature    | Brief summary                     | Pull request                                                        | Contributor                                   |
| ------------ | ---------- | --------------------------------- | ------------------------------------------------------------------- | --------------------------------------------- |
| `perception` | prediction | add timestamps for predicted path | [#164](https://github.com/tier4/driving_log_replayer_v2/issues/164) | [ktro2828](https://github.com/ktro2828)       |
| `perception` | evaluate   | add topic to evaluate             | [#169](https://github.com/tier4/driving_log_replayer_v2/issues/169) | [MasatoSaeki](https://github.com/MasatoSaeki) |
| `perception` | result     | show final metrics (revert)       | [#168](https://github.com/tier4/driving_log_replayer_v2/issues/168) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.10.1

Minor Tweak

| Module         | Feature     | Brief summary              | Pull request                                                        | Contributor                                         |
| -------------- | ----------- | -------------------------- | ------------------------------------------------------------------- | --------------------------------------------------- |
| `localization` | result      | update localization result | [#165](https://github.com/tier4/driving_log_replayer_v2/issues/165) | [SakodaShintaro](https://github.com/SakodaShintaro) |
| `common`       | package.xml | update maintainer          | [#166](https://github.com/tier4/driving_log_replayer_v2/issues/166) | [MasatoSaeki](https://github.com/MasatoSaeki)       |

## Version 3.10.0

Major changes

| Module             | Feature    | Brief summary                  | Pull request                                                        | Contributor                                   |
| ------------------ | ---------- | ------------------------------ | ------------------------------------------------------------------- | --------------------------------------------- |
| `planning_control` | evaluation | add Planning Factor evaluation | [#155](https://github.com/tier4/driving_log_replayer_v2/issues/155) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.9.0

Major changes
Ensure that you are using Autoware.universe that incorporates [this PR](https://github.com/autowarefoundation/autoware_universe/pull/10567)

| Module   | Feature        | Brief summary                      | Pull request                                                        | Contributor                                         |
| -------- | -------------- | ---------------------------------- | ------------------------------------------------------------------- | --------------------------------------------------- |
| `common` | bag play remap | delete module status based remap   | [#159](https://github.com/tier4/driving_log_replayer_v2/issues/159) | [hayato-m126](https://github.com/hayato-m126)       |
| `common` | initial pose   | replace InitializeLocalization srv | [#161](https://github.com/tier4/driving_log_replayer_v2/issues/161) | [SakodaShintaro](https://github.com/SakodaShintaro) |

## Version 3.8.5

Minor Tweak

| Module             | Feature       | Brief summary                       | Pull request                                                        | Contributor                                   |
| ------------------ | ------------- | ----------------------------------- | ------------------------------------------------------------------- | --------------------------------------------- |
| `common`           | post-process  | post process reindex                | [#158](https://github.com/tier4/driving_log_replayer_v2/issues/158) | [hayato-m126](https://github.com/hayato-m126) |
| `planning_control` | scenario_type | use diagnostics in planning control | [#146](https://github.com/tier4/driving_log_replayer_v2/issues/146) | [hayato-m126](https://github.com/hayato-m126) |
| `common`           | i/o interface | add result editor                   | [#153](https://github.com/tier4/driving_log_replayer_v2/issues/153) | [hayato-m126](https://github.com/hayato-m126) |
| `common`           | i/o interface | add result reader                   | [#151](https://github.com/tier4/driving_log_replayer_v2/issues/151) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.8.4

Minor Tweak

| Module       | Feature | Brief summary                                      | Pull request                                                      | Contributor                                   |
| ------------ | ------- | -------------------------------------------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `perception` | result  | change args False to True to save scene_result.pkl | [#147](https://github.com/tier4/driving_log_replayer_v2/pull/147) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.8.3

Minor Tweak

| Module       | Feature | Brief summary            | Pull request                                                      | Contributor                                   |
| ------------ | ------- | ------------------------ | ----------------------------------------------------------------- | --------------------------------------------- |
| `perception` | analyze | add consecutive fn spans | [#130](https://github.com/tier4/driving_log_replayer_v2/pull/130) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.8.2

Minor Tweak

| Module       | Feature | Brief summary                      | Pull request                                                      | Contributor                                   |
| ------------ | ------- | ---------------------------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `perception` | launch  | add args range                     | [#142](https://github.com/tier4/driving_log_replayer_v2/pull/142) | [MasatoSaeki](https://github.com/MasatoSaeki) |
| `perception` | analyze | delete final metrics in perception | [#143](https://github.com/tier4/driving_log_replayer_v2/pull/143) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.8.1

Minor Tweak

| Module       | Feature | Brief summary                                 | Pull request                                                      | Contributor                                   |
| ------------ | ------- | --------------------------------------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `perception` | launch  | allow None to be treated as an argument value | [#137](https://github.com/tier4/driving_log_replayer_v2/pull/137) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.8.0

Major changes

| Module       | Feature | Brief summary          | Pull request                                                      | Contributor                                   |
| ------------ | ------- | ---------------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `perception` | rosbag  | handle storage options | [#136](https://github.com/tier4/driving_log_replayer_v2/pull/136) | [MasatoSaeki](https://github.com/MasatoSaeki) |
| `common`     | rosbag  | set mcap as default    | [#135](https://github.com/tier4/driving_log_replayer_v2/pull/135) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.7.3

Minor Tweak

| Module         | Feature | Brief summary                     | Pull request                                                      | Contributor                                         |
| -------------- | ------- | --------------------------------- | ----------------------------------------------------------------- | --------------------------------------------------- |
| `localization` | rosbag  | record /localization/acceleration | [#132](https://github.com/tier4/driving_log_replayer_v2/pull/132) | [SakodaShintaro](https://github.com/SakodaShintaro) |

## Version 3.7.2

Minor Tweak

| Module       | Feature         | Brief summary            | Pull request                                                      | Contributor                                   |
| ------------ | --------------- | ------------------------ | ----------------------------------------------------------------- | --------------------------------------------- |
| `perception` | error_handling  | remove dummy result file | [#131](https://github.com/tier4/driving_log_replayer_v2/pull/131) | [MasatoSaeki](https://github.com/MasatoSaeki) |
| `perception` | evaluation_task | add prediction           | [#127](https://github.com/tier4/driving_log_replayer_v2/pull/127) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.7.1

Minor Tweak

| Module       | Feature  | Brief summary | Pull request                                                      | Contributor                                   |
| ------------ | -------- | ------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `perception` | analyzer | add analyzer  | [#125](https://github.com/tier4/driving_log_replayer_v2/pull/125) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.7.0

Major changes

| Module       | Feature    | Brief summary                 | Pull request                                                      | Contributor                                   |
| ------------ | ---------- | ----------------------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `perception` | evaluation | evaluation in post-processing | [#120](https://github.com/tier4/driving_log_replayer_v2/pull/120) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.6.5

Minor Tweak

| Module           | Feature          | Brief summary                           | Pull request                                                      | Contributor                                   |
| ---------------- | ---------------- | --------------------------------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `all_components` | documentation    | add all_components documentation        | [#121](https://github.com/tier4/driving_log_replayer_v2/pull/121) | [hayato-m126](https://github.com/hayato-m126) |
| `common`         | linter formatter | update ruff and pre-commit tool version | [#122](https://github.com/tier4/driving_log_replayer_v2/pull/122) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.6.4

Minor Tweak

| Module             | Feature | Brief summary              | Pull request                                                      | Contributor                                 |
| ------------------ | ------- | -------------------------- | ----------------------------------------------------------------- | ------------------------------------------- |
| `performance_diag` | result  | do not count invalid frame | [#116](https://github.com/tier4/driving_log_replayer_v2/pull/116) | [TadaKazuto](https://github.com/TadaKazuto) |

## Version 3.6.3

Minor Tweak

| Module         | Feature     | Brief summary        | Pull request                                                      | Contributor                                   |
| -------------- | ----------- | -------------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `localization` | rosbag play | revert: remap topics | [#117](https://github.com/tier4/driving_log_replayer_v2/pull/117) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.6.2

Major changes

| Module          | Feature  | Brief summary                               | Pull request                                                      | Contributor                                   |
| --------------- | -------- | ------------------------------------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `all_component` | use case | fix use case name dlr_all -> all_components | [#114](https://github.com/tier4/driving_log_replayer_v2/pull/114) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.6.1

Minor Tweak

| Module          | Feature       | Brief summary                          | Pull request                                                      | Contributor                                         |
| --------------- | ------------- | -------------------------------------- | ----------------------------------------------------------------- | --------------------------------------------------- |
| `common`        | documentation | add instructions for importing nightly | [#112](https://github.com/tier4/driving_log_replayer_v2/pull/112) | [SakodaShintaro](https://github.com/SakodaShintaro) |
| `traffic_light` | bag record    | change the topic to save               | [#111](https://github.com/tier4/driving_log_replayer_v2/pull/111) | [MasatoSaeki](https://github.com/MasatoSaeki)       |

## Version 3.6.0

Major changes

| Module                 | Feature      | Brief summary                  | Pull request                                                      | Contributor                                   |
| ---------------------- | ------------ | ------------------------------ | ----------------------------------------------------------------- | --------------------------------------------- |
| `localization, common` | post_process | add localization post analysis | [#109](https://github.com/tier4/driving_log_replayer_v2/pull/109) | [hayato-m126](https://github.com/hayato-m126) |
| `common`               | refactor     | split use_case launch          | [#108](https://github.com/tier4/driving_log_replayer_v2/pull/108) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.5.0

Major changes

| Module            | Feature  | Brief summary                                     | Pull request                                                      | Contributor                                   |
| ----------------- | -------- | ------------------------------------------------- | ----------------------------------------------------------------- | --------------------------------------------- |
| `ndt_convergence` | use case | add ndt_convergence_evaluation (TIER IV internal) | [#105](https://github.com/tier4/driving_log_replayer_v2/pull/105) | [hayato-m126](https://github.com/hayato-m126) |
| `ndt_convergence` | use case | add ndt_convergence_evaluation (TIER IV internal) | [#103](https://github.com/tier4/driving_log_replayer_v2/pull/103) | [hayato-m126](https://github.com/hayato-m126) |
| `common`          | refactor | launch                                            | [#106](https://github.com/tier4/driving_log_replayer_v2/pull/100) | [hayato-m126](https://github.com/hayato-m126) |
| `common`          | refactor | launch                                            | [#104](https://github.com/tier4/driving_log_replayer_v2/pull/100) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.4.2

Bug fix
Refactoring

| Module                  | Feature        | Brief summary            | Pull request                                                      | Contributor                                   |
| ----------------------- | -------------- | ------------------------ | ----------------------------------------------------------------- | --------------------------------------------- |
| `obstacle_segmentation` | initialization | fix initialization order | [#101](https://github.com/tier4/driving_log_replayer_v2/pull/101) | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | ci             | update github actions    | [#100](https://github.com/tier4/driving_log_replayer_v2/pull/100) | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | refactor       | launch                   | [#99](https://github.com/tier4/driving_log_replayer_v2/pull/99)   | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | refactor       | launch                   | [#98](https://github.com/tier4/driving_log_replayer_v2/pull/98)   | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | refactor       | launch                   | [#97](https://github.com/tier4/driving_log_replayer_v2/pull/97)   | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | refactor       | launch                   | [#96](https://github.com/tier4/driving_log_replayer_v2/pull/96)   | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | refactor       | launch                   | [#95](https://github.com/tier4/driving_log_replayer_v2/pull/95)   | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | refactor       | launch                   | [#94](https://github.com/tier4/driving_log_replayer_v2/pull/94)   | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.4.1

Major changes

| Module    | Feature  | Brief summary                    | Pull request                                                    | Contributor                                   |
| --------- | -------- | -------------------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `dlr_all` | use case | fix use case name all -> drl_all | [#92](https://github.com/tier4/driving_log_replayer_v2/pull/92) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.4.0

Major changes

| Module | Feature  | Brief summary                | Pull request                                                    | Contributor                                   |
| ------ | -------- | ---------------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `all`  | use case | add all_component_evaluation | [#90](https://github.com/tier4/driving_log_replayer_v2/pull/90) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.3.3

Documentation

| Module   | Feature      | Brief summary       | Pull request                                                    | Contributor                                   |
| -------- | ------------ | ------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `common` | architecture | update architecture | [#86](https://github.com/tier4/driving_log_replayer_v2/pull/86) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.3.2

Documentation

| Module                | Feature   | Brief summary                                                     | Pull request                                                    | Contributor                                   |
| --------------------- | --------- | ----------------------------------------------------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `ground_segmentation` | add index | Added to index and changed so that it can be traced from use_case | [#84](https://github.com/tier4/driving_log_replayer_v2/pull/84) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.3.1

Bug fix

| Module      | Feature       | Brief summary              | Pull request                                                    | Contributor                                   |
| ----------- | ------------- | -------------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `pose_node` | message field | convert int value to float | [#82](https://github.com/tier4/driving_log_replayer_v2/pull/82) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.3.0

Major changes

| Module   | Feature    | Brief summary                                                                       | Pull request                                                   | Contributor                                   |
| -------- | ---------- | ----------------------------------------------------------------------------------- | -------------------------------------------------------------- | --------------------------------------------- |
| `common` | dependency | Update package name `component_state_monitor` -> `autoware_component_state_monitor` | [#591](https://github.com/tier4/driving_log_replayer/pull/591) | [mitsudome-r](https://github.com/mitsudome-r) |

## Version 3.2.0

Major changes

| Module   | Feature | Brief summary                                                  | Pull request                                                    | Contributor                                         |
| -------- | ------- | -------------------------------------------------------------- | --------------------------------------------------------------- | --------------------------------------------------- |
| `common` | message | replace `tier4_debug_msgs` with `autoware_internal_debug_msgs` | [#78](https://github.com/tier4/driving_log_replayer_v2/pull/78) | [SakodaShintaro](https://github.com/SakodaShintaro) |

## Version 3.1.3

Minor Tweak

| Module   | Feature              | Brief summary                       | Pull request                                                    | Contributor                                   |
| -------- | -------------------- | ----------------------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `common` | linter and formatter | update linter and formatter version | [#76](https://github.com/tier4/driving_log_replayer_v2/pull/76) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.1.2

Minor Tweak

| Module          | Feature           | Brief summary                                                  | Pull request                                                    | Contributor                                   |
| --------------- | ----------------- | -------------------------------------------------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `traffic_light` | calculation logic | enable to calculate correct distance from ego to traffic light | [#74](https://github.com/tier4/driving_log_replayer_v2/pull/74) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.1.1

Minor Tweak

| Module                      | Feature  | Brief summary                   | Pull request                                                    | Contributor                                   |
| --------------------------- | -------- | ------------------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `perception, traffic_light` | criteria | enable to specify region filter | [#68](https://github.com/tier4/driving_log_replayer_v2/pull/68) | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 3.1.0

Major changes

| Module   | Feature | Brief summary                            | Pull request                                                   | Contributor                                   |
| -------- | ------- | ---------------------------------------- | -------------------------------------------------------------- | --------------------------------------------- |
| `common` | library | use `autoware_lanelet2_extension_python` | [#586](https://github.com/tier4/driving_log_replayer/pull/586) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.0.2

Minor Tweak

| Module   | Feature   | Brief summary         | Pull request                                                    | Contributor                                   |
| -------- | --------- | --------------------- | --------------------------------------------------------------- | --------------------------------------------- |
| `common` | type hint | fix type hint         | [#65](https://github.com/tier4/driving_log_replayer_v2/pull/65) | [hayato-m126](https://github.com/hayato-m126) |
| `common` | CI        | update github actions | [#66](https://github.com/tier4/driving_log_replayer_v2/pull/66) | [hayato-m126](https://github.com/hayato-m126) |

## Version 3.0.1

Documentation
Minor Tweak

| Module                | Feature         | Brief summary                    | Pull request                                                    | Contributor                                         |
| --------------------- | --------------- | -------------------------------- | --------------------------------------------------------------- | --------------------------------------------------- |
| `ground_segmentation` | launch argument | set sensing false                | [#63](https://github.com/tier4/driving_log_replayer_v2/pull/63) | [hayato-m126](https://github.com/hayato-m126)       |
| `ground_segmentation` | documentation   | add ground_segmentation document | [#63](https://github.com/tier4/driving_log_replayer_v2/pull/63) | [nanoshimarobot](https://github.com/nanoshimarobot) |

## Version 3.0.0

Start version tagging in v2.
Major changes

| Module             | Feature  | Brief summary                  | Pull request                                                    | Contributor                                   |
| ------------------ | -------- | ------------------------------ | --------------------------------------------------------------- | --------------------------------------------- |
| `planning_control` | use case | add planning_control_evaluator | [#43](https://github.com/tier4/driving_log_replayer_v2/pull/43) | [hayato-m126](https://github.com/hayato-m126) |
| `diagnostics`      | use case | add diagnostics_evaluator      | [#42](https://github.com/tier4/driving_log_replayer_v2/pull/42) | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.7.0

Major changes

| Module   | Feature    | Brief summary                                                                       | Pull request                                                   | Jira | Contributor                                   |
| -------- | ---------- | ----------------------------------------------------------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | dependency | Update package name `component_state_monitor` -> `autoware_component_state_monitor` | [#591](https://github.com/tier4/driving_log_replayer/pull/591) | -    | [mitsudome-r](https://github.com/mitsudome-r) |

## Version 2.6.0

Major changes

| Module   | Feature | Brief summary                                                  | Pull request                                                   | Jira     | Contributor                                         |
| -------- | ------- | -------------------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------------- |
| `common` | message | replace `tier4_debug_msgs` with `autoware_internal_debug_msgs` | [#589](https://github.com/tier4/driving_log_replayer/pull/589) | RT1-9020 | [SakodaShintaro](https://github.com/SakodaShintaro) |

## Version 2.5.0

Major changes

| Module   | Feature | Brief summary                            | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------- | ---------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | library | use `autoware_lanelet2_extension_python` | [#586](https://github.com/tier4/driving_log_replayer/pull/586) | RT1-8898 | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.4.0

Major changes

| Module                | Feature  | Brief summary                     | Pull request                                                   | Jira | Contributor                                         |
| --------------------- | -------- | --------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------------- |
| `ground_segmentation` | use case | add ground_segmentation_evaluator | [#528](https://github.com/tier4/driving_log_replayer/pull/528) | -    | [nanoshimarobot](https://github.com/nanoshimarobot) |

## Version 2.3.11

Minor Tweak

| Module       | Feature     | Brief summary               | Pull request                                                   | Jira | Contributor                           |
| ------------ | ----------- | --------------------------- | -------------------------------------------------------------- | ---- | ------------------------------------- |
| `perception` | performance | perf: 2x processing speedup | [#581](https://github.com/tier4/driving_log_replayer/pull/581) | -    | [ralwing](https://github.com/ralwing) |

## Version 2.3.10

Bug fix

| Module                  | Feature   | Brief summary                    | Pull request                                                   | Jira | Contributor                                   |
| ----------------------- | --------- | -------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `obstacle_segmentation` | condition | fix condition to judge diag name | [#582](https://github.com/tier4/driving_log_replayer/pull/582) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.3.9

Minor Tweak

| Module   | Feature          | Brief summary              | Pull request                                                   | Jira | Contributor                                   |
| -------- | ---------------- | -------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | formatter linter | update pre-commit add ruff | [#579](https://github.com/tier4/driving_log_replayer/pull/579) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.3.8

Minor Tweak

| Module   | Feature | Brief summary   | Pull request                                                   | Jira | Contributor                                   |
| -------- | ------- | --------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | CI      | add sonar cloud | [#577](https://github.com/tier4/driving_log_replayer/pull/577) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.3.7

Minor Tweak

| Module         | Feature     | Brief summary                              | Pull request                                                   | Jira | Contributor                                         |
| -------------- | ----------- | ------------------------------------------ | -------------------------------------------------------------- | ---- | --------------------------------------------------- |
| `common`       | IDE setting | use vscode setting                         | [#574](https://github.com/tier4/driving_log_replayer/pull/574) | -    | [hayato-m126](https://github.com/hayato-m126)       |
| `localization` | bag record  | add record topics in localization scenario | [#575](https://github.com/tier4/driving_log_replayer/pull/575) | -    | [SakodaShintaro](https://github.com/SakodaShintaro) |

## Version 2.3.6

Minor Tweak, Bug fix

| Module   | Feature         | Brief summary                       | Pull request                                                   | Jira | Contributor                                   |
| -------- | --------------- | ----------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `cli`    | package manager | use uv                              | [#571](https://github.com/tier4/driving_log_replayer/pull/571) | -    | [hayato-m126](https://github.com/hayato-m126) |
| `yabloc` | error handling  | check DiagnosticArray status length | [#572](https://github.com/tier4/driving_log_replayer/pull/572) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.3.5

Minor Tweak

| Module       | Feature        | Brief summary                                     | Pull request                                                   | Jira | Contributor                           |
| ------------ | -------------- | ------------------------------------------------- | -------------------------------------------------------------- | ---- | ------------------------------------- |
| `perception` | refactor       | evaluator di                                      | [#566](https://github.com/tier4/driving_log_replayer/pull/566) | -    | [ralwing](https://github.com/ralwing) |
| `cli`        | error handling | Handle missing scenario file and invalid Datasets | [#567](https://github.com/tier4/driving_log_replayer/pull/567) | -    | [ralwing](https://github.com/ralwing) |

## Version 2.3.4

Minor Tweak

| Module       | Feature | Brief summary                       | Pull request                                                   | Jira | Contributor                           |
| ------------ | ------- | ----------------------------------- | -------------------------------------------------------------- | ---- | ------------------------------------- |
| `perception` | result  | add covariance information in jsonl | [#568](https://github.com/tier4/driving_log_replayer/pull/568) | -    | [YoshiRi](https://github.com/YoshiRi) |

## Version 2.3.3

Minor Tweak

| Module   | Feature       | Brief summary                | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------------- | ---------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | ci            | fix colcon test              | [#561](https://github.com/tier4/driving_log_replayer/pull/561) | RT1-7842 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | ci            | use shallow clone            | [#562](https://github.com/tier4/driving_log_replayer/pull/562) | RT1-7842 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | documentation | add transforms3d version fix | [#563](https://github.com/tier4/driving_log_replayer/pull/563) | RT1-7887 | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.3.2

Minor Tweak

| Module   | Feature       | Brief summary                | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------------- | ---------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | documentation | update usage flow            | [#554](https://github.com/tier4/driving_log_replayer/pull/554) | RT1-7788 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | dependency    | delete pytest version fix    | [#555](https://github.com/tier4/driving_log_replayer/pull/555) | RT1-7813 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | dependency    | add transforms3d version fix | [#557](https://github.com/tier4/driving_log_replayer/pull/557) | RT1-7813 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | dependency    | delete numpy version fix     | [#558](https://github.com/tier4/driving_log_replayer/pull/558) | RT1-7826 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | dependency    | add numpy version fix < 2    | [#560](https://github.com/tier4/driving_log_replayer/pull/560) | RT1-7826 | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.3.1

Documentation

| Module   | Feature       | Brief summary      | Pull request                                                   | Jira | Contributor                                   |
| -------- | ------------- | ------------------ | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | documentation | add use case pages | [#552](https://github.com/tier4/driving_log_replayer/pull/552) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.3.0

Major changes
Ensure that you are using Autoware.universe that incorporates [this PR](https://github.com/autowarefoundation/autoware.universe/pull/8421).

| Module                                                                    | Feature    | Brief summary                         | Pull request                                                   | Jira | Contributor                                   |
| ------------------------------------------------------------------------- | ---------- | ------------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `localization, performance_diag, yabloc, eagleye, ar_tag_based_localizer` | dependency | update map_height_fitter package name | [#543](https://github.com/tier4/driving_log_replayer/pull/543) | -    | [a-maumau](https://github.com/a-maumau)       |
| `common`                                                                  | dependency | fix exec_depend                       | [#550](https://github.com/tier4/driving_log_replayer/pull/550) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.2.4

Documentation

| Module             | Feature  | Brief summary   | Pull request                                                   | Jira     | Contributor                                   |
| ------------------ | -------- | --------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `performance_diag` | criteria | apply pass_rate | [#545](https://github.com/tier4/driving_log_replayer/pull/545) | RT1-7569 | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.2.3

Documentation

| Module   | Feature       | Brief summary                      | Pull request                                                   | Jira | Contributor                       |
| -------- | ------------- | ---------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------- |
| `common` | documentation | docs: fix bag slice command option | [#539](https://github.com/tier4/driving_log_replayer/pull/539) | -    | [UMiho](https://github.com/UMiho) |

## Version 2.2.2

Bug fix

| Module   | Feature | Brief summary                                      | Pull request                                                   | Jira | Contributor                             |
| -------- | ------- | -------------------------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------- |
| `common` | library | use `pyquaternion` instead of `tf_transformations` | [#537](https://github.com/tier4/driving_log_replayer/pull/537) | -    | [kosuke55](https://github.com/kosuke55) |

## Version 2.2.1

Bug fix

| Module                                     | Feature        | Brief summary   | Pull request                                                   | Jira | Contributor                                     |
| ------------------------------------------ | -------------- | --------------- | -------------------------------------------------------------- | ---- | ----------------------------------------------- |
| `obstacle_segmentation`                    | error handling | fix index error | [#534](https://github.com/tier4/driving_log_replayer/pull/534) | -    | [1222-takeshi](https://github.com/1222-takeshi) |
| `ar_tag_based_localizer, performance_diag` | error handling | fix index error | [#535](https://github.com/tier4/driving_log_replayer/pull/535) | -    | [hayato-m126](https://github.com/hayato-m126)   |

## Version 2.2.0

Major changes
Ensure that you are using Autoware.universe that incorporates [this PR](https://github.com/autowarefoundation/autoware.universe/pull/8152).

| Module                      | Feature    | Brief summary            | Pull request                                                   | Jira | Contributor                             |
| --------------------------- | ---------- | ------------------------ | -------------------------------------------------------------- | ---- | --------------------------------------- |
| `annotationless_perception` | topic name | rename input diag topics | [#529](https://github.com/tier4/driving_log_replayer/pull/529) | -    | [kosuke55](https://github.com/kosuke55) |

## Version 2.1.5

Bug fix, Minor Tweak

| Module   | Feature    | Brief summary                   | Pull request                                                   | Jira | Contributor                                   |
| -------- | ---------- | ------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | dependency | remove old msg from package.xml | [#531](https://github.com/tier4/driving_log_replayer/pull/531) | -    | [hayato-m126](https://github.com/hayato-m126) |
| `common` | dependency | update dependency.repos         | [#497](https://github.com/tier4/driving_log_replayer/pull/497) | -    | [youtalk](https://github.com/youtalk)         |

## Version 2.1.4

Bug fix

| Module          | Feature | Brief summary                                    | Pull request                                                   | Jira | Contributor                                   |
| --------------- | ------- | ------------------------------------------------ | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `traffic_light` | message | remove autoware_perception_msgs in traffic_light | [#526](https://github.com/tier4/driving_log_replayer/pull/526) | -    | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 2.1.3

Documentation

| Module   | Feature       | Brief summary             | Pull request                                                   | Jira | Contributor                                   |
| -------- | ------------- | ------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | documentation | update can msg topic name | [#524](https://github.com/tier4/driving_log_replayer/pull/524) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.1.2

Bug fix

| Module       | Feature | Brief summary                  | Pull request                                                   | Jira | Contributor                             |
| ------------ | ------- | ------------------------------ | -------------------------------------------------------------- | ---- | --------------------------------------- |
| `perception` | result  | resolve invalid access to None | [#521](https://github.com/tier4/driving_log_replayer/pull/521) | -    | [ktro2828](https://github.com/ktro2828) |

## Version 2.1.1

Minor Tweak

| Module       | Feature | Brief summary                       | Pull request                                                   | Jira | Contributor                           |
| ------------ | ------- | ----------------------------------- | -------------------------------------------------------------- | ---- | ------------------------------------- |
| `perception` | result  | add perception object info to jsonl | [#452](https://github.com/tier4/driving_log_replayer/pull/452) | -    | [YoshiRi](https://github.com/YoshiRi) |

## Version 2.1.0

Minor Tweak

| Module   | Feature       | Brief summary             | Pull request                                                   | Jira | Contributor                                   |
| -------- | ------------- | ------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | visualization | use autoware project rviz | [#516](https://github.com/tier4/driving_log_replayer/pull/516) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.0.12

Bug fix

| Module          | Feature  | Brief summary              | Pull request                                                   | Jira | Contributor                                   |
| --------------- | -------- | -------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `traffic_light` | msg type | fix traffic light msg type | [#489](https://github.com/tier4/driving_log_replayer/pull/489) | -    | [MasatoSaeki](https://github.com/MasatoSaeki) |

## Version 2.0.11

Minor Tweak

| Module   | Feature  | Brief summary                   | Pull request                                                   | Jira     | Contributor                                   |
| -------- | -------- | ------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | refactor | delete unused file and function | [#509](https://github.com/tier4/driving_log_replayer/pull/509) | RT1-6800 | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.0.10

Minor Tweak

| Module                                     | Feature       | Brief summary                                           | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------------------ | ------------- | ------------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common`                                   | visualization | merge rviz                                              | [#507](https://github.com/tier4/driving_log_replayer/pull/507) | RT1-6800 | [hayato-m126](https://github.com/hayato-m126) |
| `perception, perception_2d, traffic_light` | interface     | interface in add_frame_result                           | [#499](https://github.com/tier4/driving_log_replayer/pull/499) | -        | [MasatoSaeki](https://github.com/MasatoSaeki) |
| `perception, perception_2d, traffic_light` | criteria      | resolve invalid access to critical_ground_truth_objects | [#503](https://github.com/tier4/driving_log_replayer/pull/503) | -        | [ktro2828](https://github.com/ktro2828)       |

## Version 2.0.9

Minor Tweak

| Module       | Feature  | Brief summary                 | Pull request                                                   | Jira     | Contributor                             |
| ------------ | -------- | ----------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------- |
| `perception` | criteria | add support of GT TP criteria | [#500](https://github.com/tier4/driving_log_replayer/pull/500) | RT1-6191 | [ktro2828](https://github.com/ktro2828) |

## Version 2.0.8

Bug Fix

| Module         | Feature        | Brief summary                | Pull request                                                   | Jira     | Contributor                                   |
| -------------- | -------------- | ---------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception`   | unit test      | update unit test             | [#494](https://github.com/tier4/driving_log_replayer/pull/494) | RT1-6191 | [hayato-m126](https://github.com/hayato-m126) |
| `localization` | error handling | check DiagnosticArray length | [#493](https://github.com/tier4/driving_log_replayer/pull/493) | RT1-6903 | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.0.7

Bug Fix

| Module       | Feature  | Brief summary                                                  | Pull request                                                   | Jira     | Contributor                             |
| ------------ | -------- | -------------------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------- |
| `perception` | analyzer | avoid error that conf_mat_dict is referenced before assignment | [#490](https://github.com/tier4/driving_log_replayer/pull/490) | RT1-6191 | [ktro2828](https://github.com/ktro2828) |

## Version 2.0.6

Minor Tweak

| Module       | Feature  | Brief summary                        | Pull request                                                   | Jira     | Contributor                             |
| ------------ | -------- | ------------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------- |
| `perception` | analyzer | apply PerceptionAnalysisResult       | [#483](https://github.com/tier4/driving_log_replayer/pull/483) | RT1-6191 | [ktro2828](https://github.com/ktro2828) |
| `perception` | criteria | if there is no objects returns 100.0 | [#486](https://github.com/tier4/driving_log_replayer/pull/486) | RT1-6191 | [ktro2828](https://github.com/ktro2828) |

## Version 2.0.5

Documentation

| Module   | Feature       | Brief summary             | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------------- | ------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | documentation | add trouble shooting page | [#484](https://github.com/tier4/driving_log_replayer/pull/484) | RT4-9983 | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.0.4

Minor Tweak

| Module       | Feature  | Brief summary               | Pull request                                                   | Jira | Contributor                             |
| ------------ | -------- | --------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------- |
| `perception` | criteria | add a criteria of yaw error | [#450](https://github.com/tier4/driving_log_replayer/pull/450) | -    | [ktro2828](https://github.com/ktro2828) |

## Version 2.0.3

Minor Tweak

| Module       | Feature  | Brief summary                    | Pull request                                                   | Jira | Contributor                             |
| ------------ | -------- | -------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------- |
| `perception` | criteria | add support of velocity criteria | [#465](https://github.com/tier4/driving_log_replayer/pull/465) | -    | [ktro2828](https://github.com/ktro2828) |

## Version 2.0.2

Minor Tweak

| Module                      | Feature      | Brief summary        | Pull request                                                   | Jira     | Contributor                                   |
| --------------------------- | ------------ | -------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `annotationless_perception` | metric_value | support metric_value | [#465](https://github.com/tier4/driving_log_replayer/pull/465) | RT1-6482 | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.0.1

Minor Tweak

| Module   | Feature       | Brief summary                       | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------------- | ----------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | documentation | update quick start                  | [#473](https://github.com/tier4/driving_log_replayer/pull/473) | RT1-6661 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | rosbag2       | add dependency rosbag2_storage_mcap | [#475](https://github.com/tier4/driving_log_replayer/pull/475) | -        | [hayato-m126](https://github.com/hayato-m126) |

## Version 2.0.0

Breaking Change
Requires Autoware to support autoware_msg

| Module   | Feature      | Brief summary        | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------------ | -------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | autoware_msg | support autoware_msg | [#472](https://github.com/tier4/driving_log_replayer/pull/472) | RT1-6661 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.18.1

Document update

| Module   | Feature  | Brief summary   | Pull request                                                   | Jira     | Contributor                                   |
| -------- | -------- | --------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | document | update document | [#470](https://github.com/tier4/driving_log_replayer/pull/470) | RT4-9983 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.18.0

Major Changes
The time in result_bag is now the same as the start end of input_bag.

| Module   | Feature | Brief summary           | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------- | ----------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | rosbag  | add use-sim-time option | [#468](https://github.com/tier4/driving_log_replayer/pull/468) | RT4-9983 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.17.0

Major Changes
Ensure that you are using Autoware.universe that incorporates [this PR](https://github.com/autowarefoundation/autoware.universe/pull/7048).

| Module         | Feature      | Brief summary              | Pull request                                                   | Jira     | Contributor                                   |
| -------------- | ------------ | -------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization` | initial pose | set initial pose directory | [#460](https://github.com/tier4/driving_log_replayer/pull/460) | RT4-8507 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.16.0

Major Changes
Use perception_eval v1.2.0 or later

| Module                  | Feature             | Brief summary                                             | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------- | ------------------- | --------------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `obstacle_segmentation` | function            | move function from evaluator.py                           | [#451](https://github.com/tier4/driving_log_replayer/pull/451) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | cli                 | delete json conversion                                    | [#457](https://github.com/tier4/driving_log_replayer/pull/457) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `traffic_light`         | criteria, interface | support multi criterion and update interface              | [#291](https://github.com/tier4/driving_log_replayer/pull/291) | RT4-8944 | [hayato-m126](https://github.com/hayato-m126) |
| `traffic_light`         | topic               | restore topic name                                        | [#458](https://github.com/tier4/driving_log_replayer/pull/458) | RT4-8944 | [hayato-m126](https://github.com/hayato-m126) |
| `traffic_light`         | frame_id            | rename FrameID.TRAFFIC_LIGHT to FrameID.CAM_TRAFFIC_LIGHT | [#460](https://github.com/tier4/driving_log_replayer/pull/460) | RT4-5928 | [ktro2828](https://github.com/ktro2828)       |

## Version 1.15.5

Minor Tweak

| Module                      | Feature | Brief summary                            | Pull request                                                   | Jira     | Contributor                                   |
| --------------------------- | ------- | ---------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `annotationless_perception` | launch  | set use_perception_online_evaluator=true | [#449](https://github.com/tier4/driving_log_replayer/pull/449) | RT1-6339 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.15.4

Bug Fix

| Module             | Feature       | Brief summary                | Pull request                                                   | Jira     | Contributor                                   |
| ------------------ | ------------- | ---------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common`           | cli           | fix suppress error msg       | [#441](https://github.com/tier4/driving_log_replayer/pull/441) | RT4-9983 | [hayato-m126](https://github.com/hayato-m126) |
| `common`           | documentation | update sample result.json    | [#443](https://github.com/tier4/driving_log_replayer/pull/443) | RT4-9983 | [hayato-m126](https://github.com/hayato-m126) |
| `perception`       | criteria      | use perception_eval function | [#436](https://github.com/tier4/driving_log_replayer/pull/436) | -        | [ktro2828](https://github.com/ktro2828)       |
| `performance_diag` | launch        | disable perception module    | [#444](https://github.com/tier4/driving_log_replayer/pull/444) | RT1-6321 | [hayato-m126](https://github.com/hayato-m126) |
| `perception`       | visualization | show full bounding box uuid  | [#445](https://github.com/tier4/driving_log_replayer/pull/445) | -        | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.15.3

Bug Fix

| Module                      | Feature    | Brief summary                 | Pull request                                                   | Jira     | Contributor                                   |
| --------------------------- | ---------- | ----------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception`                | result     | add object label list         | [#432](https://github.com/tier4/driving_log_replayer/pull/432) | RT4-9830 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                    | cli        | support base scenario         | [#437](https://github.com/tier4/driving_log_replayer/pull/437) | RT1-6169 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                    | cli        | fix suppress error msg        | [#438](https://github.com/tier4/driving_log_replayer/pull/438) | RT1-6169 | [hayato-m126](https://github.com/hayato-m126) |
| `annotationless_perception` | result bag | record pointcloud and objects | [#440](https://github.com/tier4/driving_log_replayer/pull/440) | -        | [kosuke55](https://github.com/kosuke55)       |

## Version 1.15.2

Bug Fix

| Module                                                                         | Feature     | Brief summary                                             | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------------------------------------------------------ | ----------- | --------------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `performance_diag`                                                             | diagnostics | Change processing to match the diagnostics data structure | [#433](https://github.com/tier4/driving_log_replayer/pull/433) | RT4-9826 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation, localization, yabloc, eagleye, ar_tag_based_localizer` | diagnostics | Change topic from diagnostics_agg to diagnostics          | [#434](https://github.com/tier4/driving_log_replayer/pull/434) | RT4-9826 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.15.1

Bug Fix, Minor Tweak

| Module                  | Feature          | Brief summary                                    | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------- | ---------------- | ------------------------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `obstacle_segmentation` | evaluation logic | set EvaluationItem.success False                 | [#419](https://github.com/tier4/driving_log_replayer/pull/419) | RT4-9627 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | target topic     | Change topic from diagnostics_agg to diagnostics | [#422](https://github.com/tier4/driving_log_replayer/pull/422) | RT4-9627 | [hayato-m126](https://github.com/hayato-m126) |
| `localization`          | evaluation logic | set EvaluationItem.success False                 | [#420](https://github.com/tier4/driving_log_replayer/pull/420) | RT4-9627 | [hayato-m126](https://github.com/hayato-m126) |
| `localization`          | target topic     | Change topic from diagnostics_agg to diagnostics | [#421](https://github.com/tier4/driving_log_replayer/pull/421) | RT4-9627 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | documentation    | Document reasons for using pipx                  | [#423](https://github.com/tier4/driving_log_replayer/pull/423) | RT4-9084 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.15.0

Bug Fix

| Module             | Feature          | Brief summary                                    | Pull request                                                   | Jira     | Contributor                                   |
| ------------------ | ---------------- | ------------------------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `performance_diag` | evaluation logic | set EvaluationItem.success False                 | [#415](https://github.com/tier4/driving_log_replayer/pull/415) | RT4-9627 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag` | target topic     | Change topic from diagnostics_agg to diagnostics | [#417](https://github.com/tier4/driving_log_replayer/pull/417) | RT4-9627 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.14.10

Bug Fix

| Module                                     | Feature          | Brief summary                   | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------------------ | ---------------- | ------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception, perception_2d, traffic_light` | evaluation logic | set EvaluationItem.success True | [#412](https://github.com/tier4/driving_log_replayer/pull/412) | RT4-9572 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.14.9

Minor Tweak

| Module                                     | Feature          | Brief summary                    | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------------------ | ---------------- | -------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common`                                   | test             | fix pytest version               | [#408](https://github.com/tier4/driving_log_replayer/pull/408) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `perception, perception_2d, traffic_light` | evaluation logic | do not count no gt and no object | [#409](https://github.com/tier4/driving_log_replayer/pull/409) | RT4-9572 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.14.8

Minor Tweak, Bug fix

| Module                      | Feature        | Brief summary              | Pull request                                                   | Jira     | Contributor                                   |
| --------------------------- | -------------- | -------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `annotationless_perception` | unit test      | fix unit test              | [#402](https://github.com/tier4/driving_log_replayer/pull/402) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `common`                    | pyproject.toml | update dependency          | [#404](https://github.com/tier4/driving_log_replayer/pull/404) | RT4-9477 | [hayato-m126](https://github.com/hayato-m126) |
| `annotationless_perception` | cli            | support scenario less mode | [#405](https://github.com/tier4/driving_log_replayer/pull/405) | RT4-9477 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.14.7

Bug fix

| Module                      | Feature | Brief summary   | Pull request                                                   | Jira | Contributor                             |
| --------------------------- | ------- | --------------- | -------------------------------------------------------------- | ---- | --------------------------------------- |
| `annotationless_perception` | metrics | fix min metrics | [#400](https://github.com/tier4/driving_log_replayer/pull/400) | -    | [kosuke55](https://github.com/kosuke55) |

## Version 1.14.6

Minor Tweak

| Module       | Feature   | Brief summary                                       | Pull request                                                   | Jira     | Contributor                                   |
| ------------ | --------- | --------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception` | exception | skip evaluation when 1 <= len(footprint.points) < 3 | [#397](https://github.com/tier4/driving_log_replayer/pull/397) | RT4-9407 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.14.5

Minor Tweak

| Module       | Feature | Brief summary                | Pull request                                                   | Jira | Contributor                             |
| ------------ | ------- | ---------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------- |
| `perception` | metrics | add support of label metrics | [#386](https://github.com/tier4/driving_log_replayer/pull/386) | -    | [ktro2828](https://github.com/ktro2828) |

## Version 1.14.4

Minor Tweak

| Module                      | Feature        | Brief summary                               | Pull request                                                   | Jira     | Contributor                                   |
| --------------------------- | -------------- | ------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization`              | initial pose   | revert #319 bag play pause                  | [#390](https://github.com/tier4/driving_log_replayer/pull/390) | RT4-8507 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                    | cli            | update cli run option                       | [#392](https://github.com/tier4/driving_log_replayer/pull/392) | RT4-8507 | [hayato-m126](https://github.com/hayato-m126) |
| `perception`                | error handling | set Result False when run time error occurs | [#387](https://github.com/tier4/driving_log_replayer/pull/387) | RT4-9407 | [hayato-m126](https://github.com/hayato-m126) |
| `annotationless_perception` | result         | add command to update scenario condition    | [#393](https://github.com/tier4/driving_log_replayer/pull/393) | RT4-9377 | [kosuke55](https://github.com/kosuke55)       |

## Version 1.14.3

Minor Tweak

This release is only an update of the CLI.

| Module                      | Feature | Brief summary                            | Pull request                                                   | Jira     | Contributor                                   |
| --------------------------- | ------- | ---------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common`                    | cli     | cli change structure                     | [#382](https://github.com/tier4/driving_log_replayer/pull/382) | RT4-9377 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                    | cli     | fix package name                         | [#383](https://github.com/tier4/driving_log_replayer/pull/383) | RT4-9377 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                    | cli     | check UseCaseName to create command      | [#384](https://github.com/tier4/driving_log_replayer/pull/384) | RT4-9377 | [hayato-m126](https://github.com/hayato-m126) |
| `annotationless_perception` | cli     | add command to update scenario condition | [#385](https://github.com/tier4/driving_log_replayer/pull/385) | RT4-9377 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.14.2

Minor Tweak

| Module                      | Feature | Brief summary                | Pull request                                                   | Jira | Contributor                             |
| --------------------------- | ------- | ---------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------- |
| `annotationless_perception` | result  | output details of fail items | [#379](https://github.com/tier4/driving_log_replayer/pull/379) | -    | [kosuke55](https://github.com/kosuke55) |

## Version 1.14.1

Minor Tweak

| Module                      | Feature    | Brief summary                 | Pull request                                                   | Jira     | Contributor                                   |
| --------------------------- | ---------- | ----------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `annotationless_perception` | evaluation | support object classification | [#377](https://github.com/tier4/driving_log_replayer/pull/377) | RT4-9083 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.14.0

Major Changes
To use this version, you need to have `perception_online_evaluator` in your Autoware workspace

| Module                      | Feature  | Brief summary                            | Pull request                                                   | Jira     | Contributor                                   |
| --------------------------- | -------- | ---------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `annotationless_perception` | use_case | add annotationless_perception evaluation | [#373](https://github.com/tier4/driving_log_replayer/pull/373) | RT4-8943 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.13.4

Minor Tweak

| Module                  | Feature     | Brief summary                     | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------- | ----------- | --------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `obstacle_segmentation` | performance | filter only lanelets close to ego | [#366](https://github.com/tier4/driving_log_replayer/pull/366) | RT4-8792 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | package.xml | Delete confusing comments         | [#371](https://github.com/tier4/driving_log_replayer/pull/371) | RT4-8792 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.13.3

Minor Tweak, Document update, Bug fix

| Module                  | Feature        | Brief summary                                         | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------- | -------------- | ----------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `obstacle_segmentation` | unit test      | update unit test                                      | [#364](https://github.com/tier4/driving_log_replayer/pull/364) | RT4-8792 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | documentation  | update document                                       | [#367](https://github.com/tier4/driving_log_replayer/pull/367) | RT4-8792 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | run time error | add handling of cases where non_detection is disabled | [#368](https://github.com/tier4/driving_log_replayer/pull/368) | RT4-8937 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.13.2

Minor Tweak

| Module                  | Feature   | Brief summary                | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------- | --------- | ---------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `obstacle_segmentation` | unit test | add unit test                | [#361](https://github.com/tier4/driving_log_replayer/pull/361) | RT4-8792 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`      | rosbag2   | drop concatenated/pointcloud | [#362](https://github.com/tier4/driving_log_replayer/pull/362) | RT4-8848 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.13.1

Minor Tweak

| Module   | Feature         | Brief summary | Pull request                                                   | Jira     | Contributor                                   |
| -------- | --------------- | ------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | package manager | use rye       | [#359](https://github.com/tier4/driving_log_replayer/pull/359) | RT4-8797 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.13.0

Breaking Change
To use this version, you need to have `lanelet2-extension-python` in your autoware workspace

| Module                  | Feature  | Brief summary                 | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------- | -------- | ----------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `obstacle_segmentation` | lanelet2 | use lanelet2-extension-python | [#356](https://github.com/tier4/driving_log_replayer/pull/356) | RT4-8497 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.12.5

Minor Tweak

| Module          | Feature | Brief summary          | Pull request                                                   | Jira | Contributor                           |
| --------------- | ------- | ---------------------- | -------------------------------------------------------------- | ---- | ------------------------------------- |
| `traffic_light` | rosbag2 | change recorded topics | [#357](https://github.com/tier4/driving_log_replayer/pull/357) | -    | [YoshiRi](https://github.com/YoshiRi) |

## Version 1.12.4

Minor Tweak

| Module          | Feature | Brief summary          | Pull request                                                   | Jira | Contributor                           |
| --------------- | ------- | ---------------------- | -------------------------------------------------------------- | ---- | ------------------------------------- |
| `traffic_light` | rosbag2 | change recorded topics | [#353](https://github.com/tier4/driving_log_replayer/pull/353) | -    | [kminoda](https://github.com/kminoda) |

## Version 1.12.3

Minor Tweak

| Module   | Feature | Brief summary     | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------- | ----------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | CI      | update CI setting | [#351](https://github.com/tier4/driving_log_replayer/pull/351) | RT4-8497 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.12.2

Minor Tweak

| Module       | Feature      | Brief summary                                      | Pull request                                                   | Jira | Contributor                           |
| ------------ | ------------ | -------------------------------------------------- | -------------------------------------------------------------- | ---- | ------------------------------------- |
| `perception` | Ground Truth | enable to interpolate gt when scenario is tracking | [#349](https://github.com/tier4/driving_log_replayer/pull/349) | -    | [YoshiRi](https://github.com/YoshiRi) |

## Version 1.12.1

Minor Tweak

| Module   | Feature | Brief summary      | Pull request                                                   | Jira | Contributor                                   |
| -------- | ------- | ------------------ | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | rviz    | update config/rviz | [#346](https://github.com/tier4/driving_log_replayer/pull/346) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.12.0

Breaking change
This update includes breaking changes to the `perception` scenario

| Module       | Feature  | Brief summary                                | Pull request                                                   | Jira     | Contributor                                   |
| ------------ | -------- | -------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception` | criteria | allow setting of criteria for each distance. | [#339](https://github.com/tier4/driving_log_replayer/pull/339) | RT4-8120 | [ktro2828](https://github.com/ktro2828)       |
| `perception` | criteria | allow omission of distance upper limit       | [#344](https://github.com/tier4/driving_log_replayer/pull/339) | RT4-8120 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.11.1

Minor Tweak

| Module       | Feature  | Brief summary                                  | Pull request                                                   | Jira | Contributor                           |
| ------------ | -------- | ---------------------------------------------- | -------------------------------------------------------------- | ---- | ------------------------------------- |
| `perception` | criteria | add maph criteria and allow multiple criterion | [#337](https://github.com/tier4/driving_log_replayer/pull/337) | -    | [kminoda](https://github.com/kminoda) |

## Version 1.11.0

Major change

| Module   | Feature | Brief summary             | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------- | ------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | cli     | allow arbitrary arguments | [#333](https://github.com/tier4/driving_log_replayer/pull/333) | RT4-7955 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.10.6

Bug fix

| Module       | Feature         | Brief summary          | Pull request                                                   | Jira     | Contributor                                   |
| ------------ | --------------- | ---------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception` | data validation | check footprint length | [#332](https://github.com/tier4/driving_log_replayer/pull/332) | RT4-7993 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.10.5

Minor Tweak

| Module   | Feature | Brief summary | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------- | ------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | cli     | refactor cli  | [#329](https://github.com/tier4/driving_log_replayer/pull/329) | RT4-7955 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.10.4

Minor Tweak, Bug fix

| Module   | Feature         | Brief summary                              | Pull request                                                   | Jira     | Contributor                                   |
| -------- | --------------- | ------------------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | launch argument | remove unnecessary functions and arguments | [#326](https://github.com/tier4/driving_log_replayer/pull/326) | RT4-7875 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | launch argument | fix overwrite pose_source and twist_source | [#327](https://github.com/tier4/driving_log_replayer/pull/327) | RT4-7954 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.10.3

Minor Tweak, Bug fix

| Module                           | Feature     | Brief summary                                                   | Pull request                                                   | Jira     | Contributor                                   |
| -------------------------------- | ----------- | --------------------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization, performance_diag` | bag control | Stop bag playback while the initial pose process is in progress | [#319](https://github.com/tier4/driving_log_replayer/pull/319) | RT4-7312 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                         | cli         | save the log displayed in the console as a file                 | [#320](https://github.com/tier4/driving_log_replayer/pull/320) | RT4-7883 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                         | ci          | fix github actions deprecating command                          | [#321](https://github.com/tier4/driving_log_replayer/pull/321) | RT4-7922 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.10.2

Minor Tweak, Bug fix

| Module                                     | Feature         | Brief summary                      | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------------------ | --------------- | ---------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception, perception_2d, traffic_light` | unit test       | add test for custom criteria level | [#316](https://github.com/tier4/driving_log_replayer/pull/316) | RT4-7872 | [hayato-m126](https://github.com/hayato-m126) |
| `perception, perception_2d, traffic_light` | perception_mode | fix override perception_mode       | [#317](https://github.com/tier4/driving_log_replayer/pull/316) | RT4-7872 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.10.1

Minor Tweak, Bug fix

| Module                                     | Feature             | Brief summary                       | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------------------ | ------------------- | ----------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common`                                   | type hint           | add type hint for rclpy.task.Future | [#312](https://github.com/tier4/driving_log_replayer/pull/312) | RT4-7872 | [hayato-m126](https://github.com/hayato-m126) |
| `cli`                                      | security            | fix ruff rule S602                  | [#313](https://github.com/tier4/driving_log_replayer/pull/313) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `perception, perception_2d, traffic_light` | scenario validation | fix criteria type                   | [#314](https://github.com/tier4/driving_log_replayer/pull/314) | -        | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.10.0

Minor Tweak

| Module   | Feature          | Brief summary                                              | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ---------------- | ---------------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | documentation    | add release process                                        | [#308](https://github.com/tier4/driving_log_replayer/pull/308) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `common` | scenario         | add scenario class                                         | [#306](https://github.com/tier4/driving_log_replayer/pull/306) | RT4-7583 | [hayato-m126](https://github.com/hayato-m126) |
| `cli`    | shell completion | add bash and fish completion                               | [#309](https://github.com/tier4/driving_log_replayer/pull/309) | RT4-7803 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | bag record       | add runtime arguments to override the topic to be recorded | [#301](https://github.com/tier4/driving_log_replayer/pull/301) | RT4-7669 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.9.1

Minor Tweak, Bug fix

| Module       | Feature     | Brief summary                 | Pull request                                                   | Jira     | Contributor                                   |
| ------------ | ----------- | ----------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `diag`       | refactoring | refactor diag blockage result | [#303](https://github.com/tier4/driving_log_replayer/pull/303) | RT4-7580 | [hayato-m126](https://github.com/hayato-m126) |
| `perception` | criteria    | fix no data criteria          | [#305](https://github.com/tier4/driving_log_replayer/pull/305) | -        | [YoshiRi](https://github.com/YoshiRi)         |

## Version 1.9.0

Minor Tweak

| Module                  | Feature | Brief summary                                                           | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------- | ------- | ----------------------------------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `obstacle_segmentation` | test    | add unit test for obstacle segmentation, update log format and analyzer | [#273](https://github.com/tier4/driving_log_replayer/pull/273) | RT4-7580 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.8.4

Minor Tweak

| Module       | Feature         | Brief summary       | Pull request                                                   | Jira     | Contributor                                   |
| ------------ | --------------- | ------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception` | launch argument | add perception_mode | [#299](https://github.com/tier4/driving_log_replayer/pull/299) | RT4-7580 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.8.3

Minor Tweak

| Module   | Feature       | Brief summary        | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------------- | -------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | documentation | update result format | [#297](https://github.com/tier4/driving_log_replayer/pull/297) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.8.2

Minor Tweak

| Module          | Feature | Brief summary                   | Pull request                                                   | Jira     | Contributor                                   |
| --------------- | ------- | ------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception`    | test    | add unit test for perception    | [#292](https://github.com/tier4/driving_log_replayer/pull/292) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `cli`           | runner  | fix create output directory     | [#294](https://github.com/tier4/driving_log_replayer/pull/294) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `perception_2d` | test    | add unit test for perception_2d | [#295](https://github.com/tier4/driving_log_replayer/pull/295) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.8.1

Minor Tweak

| Module                      | Feature       | Brief summary                        | Pull request                                                   | Jira     | Contributor                                   |
| --------------------------- | ------------- | ------------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `cli`                       | refactoring   | refactor command generation          | [#285](https://github.com/tier4/driving_log_replayer/pull/285) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                    | refactoring   | apply ruff PTH rules (use pathlib)   | [#286](https://github.com/tier4/driving_log_replayer/pull/286) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                    | refactoring   | refactor pathlib.Path initialization | [#287](https://github.com/tier4/driving_log_replayer/pull/287) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                    | refactoring   | refactor common module               | [#288](https://github.com/tier4/driving_log_replayer/pull/288) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `traffic_light`             | documentation | update traffic light ml model        | [#289](https://github.com/tier4/driving_log_replayer/pull/289) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `perception, perception_2d` | documentation | update ml model                      | [#290](https://github.com/tier4/driving_log_replayer/pull/290) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.8.0

Major change
Ensure that you are using Autoware.universe that incorporates [this PR](https://github.com/autowarefoundation/autoware.universe/pull/5319).

| Module         | Feature              | Brief summary                       | Pull request                                                   | Jira | Contributor                                         |
| -------------- | -------------------- | ----------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------------- |
| `localization` | evaluate convergence | use initial_to_result_relative_pose | [#282](https://github.com/tier4/driving_log_replayer/pull/282) | -    | [SakodaShintaro](https://github.com/SakodaShintaro) |
| `common`       | documentation        | scenario format 3.1.0               | [#283](https://github.com/tier4/driving_log_replayer/pull/283) | -    | [hayato-m126](https://github.com/hayato-m126)       |

## Version 1.7.0

Major change

| Module       | Feature         | Brief summary           | Pull request                                                   | Jira | Contributor                                   |
| ------------ | --------------- | ----------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `perception` | lint            | fix TCH002              | [#278](https://github.com/tier4/driving_log_replayer/pull/278) | -    | [hayato-m126](https://github.com/hayato-m126) |
| `perception` | scenario format | support perception_mode | [#279](https://github.com/tier4/driving_log_replayer/pull/279) | -    | [ktro2828](https://github.com/ktro2828)       |

## Version 1.6.7

Bug fix

| Module       | Feature | Brief summary    | Pull request                                                   | Jira | Contributor                             |
| ------------ | ------- | ---------------- | -------------------------------------------------------------- | ---- | --------------------------------------- |
| `perception` | library | fix import error | [#277](https://github.com/tier4/driving_log_replayer/pull/277) | -    | [ktro2828](https://github.com/ktro2828) |

## Version 1.6.6

Minor Tweak

| Module                                         | Feature    | Brief summary                                 | Pull request                                                   | Jira     | Contributor                                   |
| ---------------------------------------------- | ---------- | --------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `performance_diag`                             | test       | add unit test for performance_diag            | [#269](https://github.com/tier4/driving_log_replayer/pull/269) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `perception`                                   | test       | add unit test for perception_eval_conversions | [#271](https://github.com/tier4/driving_log_replayer/pull/271) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                                       | dependency | update dependency.repos                       | [#274](https://github.com/tier4/driving_log_replayer/pull/274) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `perception`, `perception_2d`, `traffic_light` | criteria   | update perception criteria                    | [#272](https://github.com/tier4/driving_log_replayer/pull/272) | RT4-7103 | [ktro2828](https://github.com/ktro2828)       |

## Version 1.6.5

Minor Tweak

| Module   | Feature              | Brief summary                                                          | Pull request                                                   | Jira | Contributor                                   |
| -------- | -------------------- | ---------------------------------------------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | Library installation | Install mkdocs library with poetry group docs                          | [#265](https://github.com/tier4/driving_log_replayer/pull/265) | -    | [hayato-m126](https://github.com/hayato-m126) |
| `common` | Library installation | Automatically install specific versions of libraries with colcon build | [#266](https://github.com/tier4/driving_log_replayer/pull/266) | -    | [ktro2828](https://github.com/ktro2828)       |

## Version 1.6.4

Document update

| Module   | Feature       | Brief summary                             | Pull request                                                   | Jira | Contributor                                   |
| -------- | ------------- | ----------------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | documentation | fix mkdocs github actions library install | [#263](https://github.com/tier4/driving_log_replayer/pull/263) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.6.3

Document update

| Module   | Feature       | Brief summary         | Pull request                                                   | Jira | Contributor                           |
| -------- | ------------- | --------------------- | -------------------------------------------------------------- | ---- | ------------------------------------- |
| `yabloc` | documentation | fix yabloc rosbag url | [#261](https://github.com/tier4/driving_log_replayer/pull/261) | -    | [kminoda](https://github.com/kminoda) |

## Version 1.6.2

Major change

| Module                   | Feature                  | Brief summary                               | Pull request                                                   | Jira     | Contributor                                         |
| ------------------------ | ------------------------ | ------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------------- |
| `common`                 | test                     | add unit test for result class              | [#246](https://github.com/tier4/driving_log_replayer/pull/246) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `localization`           | test                     | add unit test for localization              | [#248](https://github.com/tier4/driving_log_replayer/pull/248) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `localization`           | log                      | add topic result class                      | [#249](https://github.com/tier4/driving_log_replayer/pull/249) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `localization`           | log                      | update initial value of TopicResult.success | [#250](https://github.com/tier4/driving_log_replayer/pull/250) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `localization`           | log, test                | update localization availability            | [#251](https://github.com/tier4/driving_log_replayer/pull/251) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `yabloc`                 | test                     | add yabloc unit test                        | [#252](https://github.com/tier4/driving_log_replayer/pull/252) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `eagleye`                | test                     | add eagleye unit test                       | [#253](https://github.com/tier4/driving_log_replayer/pull/253) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `common`                 | log                      | change TopicResult to EvaluationItem        | [#254](https://github.com/tier4/driving_log_replayer/pull/254) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `common`                 | log, test                | add frame_success                           | [#256](https://github.com/tier4/driving_log_replayer/pull/256) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `common`                 | test                     | update variable name                        | [#257](https://github.com/tier4/driving_log_replayer/pull/257) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |
| `ar_tag_based_localizer` | `ar_tag_based_localizer` | add `ar_tag_based_localizer` evaluation     | [#258](https://github.com/tier4/driving_log_replayer/pull/258) | -        | [SakodaShintaro](https://github.com/SakodaShintaro) |
| `common`                 | documentation            | update document                             | [#259](https://github.com/tier4/driving_log_replayer/pull/259) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126)       |

## Version 1.6.1

Bug fix

| Module   | Feature | Brief summary                             | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------- | ----------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | launch  | fix restore launch argument default value | [#244](https://github.com/tier4/driving_log_replayer/pull/244) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.6.0

Breaking change
End of support for ROS 2 galactic

| Module                          | Feature          | Brief summary                  | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------- | ---------------- | ------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization, yabloc, eagleye` | refactoring      | refactor evaluator node        | [#228](https://github.com/tier4/driving_log_replayer/pull/228) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`              | refactoring      | refactor evaluator node        | [#229](https://github.com/tier4/driving_log_replayer/pull/229) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `perception`                    | refactoring      | refactor evaluator node        | [#230](https://github.com/tier4/driving_log_replayer/pull/230) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `perception_2d`                 | refactoring      | refactor evaluator node        | [#231](https://github.com/tier4/driving_log_replayer/pull/231) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `traffic_light`                 | refactoring      | refactor evaluator node        | [#232](https://github.com/tier4/driving_log_replayer/pull/232) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                        | refactoring      | refactor call lookup transform | [#233](https://github.com/tier4/driving_log_replayer/pull/233) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation`         | refactoring      | refactor evaluator node        | [#234](https://github.com/tier4/driving_log_replayer/pull/234) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                        | linter           | apply ruff linter rules        | [#237](https://github.com/tier4/driving_log_replayer/pull/237) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                        | linter           | apply type hint                | [#238](https://github.com/tier4/driving_log_replayer/pull/238) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                        | linter           | apply ruff linter rules        | [#239](https://github.com/tier4/driving_log_replayer/pull/239) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                        | linter           | apply ruff linter rules        | [#240](https://github.com/tier4/driving_log_replayer/pull/240) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                        | linter           | apply ruff linter rules        | [#241](https://github.com/tier4/driving_log_replayer/pull/241) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                        | CMake, type hint | drop galactic support          | [#242](https://github.com/tier4/driving_log_replayer/pull/242) | RT4-5832 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.5.4

Minor Tweak

| Module    | Feature         | Brief summary                   | Pull request                                                   | Jira | Contributor                                   |
| --------- | --------------- | ------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common`  | `documentation` | update mkdocs i18n setting      | [#224](https://github.com/tier4/driving_log_replayer/pull/224) | -    | [hayato-m126](https://github.com/hayato-m126) |
| `eagleye` | `documentation` | fix `eagleye` bag download link | [#225](https://github.com/tier4/driving_log_replayer/pull/225) | -    | [kminoda](https://github.com/kminoda)         |

## Version 1.5.3

Minor Tweak

| Module    | Feature         | Brief summary          | Pull request                                                   | Jira | Contributor                                   |
| --------- | --------------- | ---------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `cli`     | `poetry`        | update pyproject.toml  | [#220](https://github.com/tier4/driving_log_replayer/pull/220) | -    | [hayato-m126](https://github.com/hayato-m126) |
| `eagleye` | `documentation` | add `eagleye` tutorial | [#222](https://github.com/tier4/driving_log_replayer/pull/222) | -    | [kminoda](https://github.com/kminoda)         |

## Version 1.5.2

Minor Tweak

| Module          | Feature          | Brief summary                  | Pull request                                                   | Jira     | Contributor                                   |
| --------------- | ---------------- | ------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common`        | `linter`         | apply `ruff` linter EM rules   | [#211](https://github.com/tier4/driving_log_replayer/pull/211) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `common`        | `linter`         | apply `ruff` linter PIE rules  | [#212](https://github.com/tier4/driving_log_replayer/pull/212) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `common`        | `linter`         | apply `ruff` linter RET rules  | [#213](https://github.com/tier4/driving_log_replayer/pull/213) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `common`        | `linter`         | apply `ruff` linter SLF rules  | -                                                              | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `common`        | `linter`         | apply `ruff` linter TCH rules  | [#214](https://github.com/tier4/driving_log_replayer/pull/214) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `common`        | `linter`         | apply `ruff` linter UP rules   | [#215](https://github.com/tier4/driving_log_replayer/pull/215) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `common`        | `linter`         | apply `ruff` linter SIM rules  | [#216](https://github.com/tier4/driving_log_replayer/pull/216) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `common`        | `linter`         | apply `ruff` linter RUF        | [#217](https://github.com/tier4/driving_log_replayer/pull/217) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |
| `perception_2d` | `error_handling` | handle invalid evaluation task | [#218](https://github.com/tier4/driving_log_replayer/pull/218) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.5.1

Breaking change

| Module       | Feature           | Brief summary                   | Pull request                                                   | Jira     | Contributor                                   |
| ------------ | ----------------- | ------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception` | fp validation log | store fp result in result.jsonl | [#206](https://github.com/tier4/driving_log_replayer/pull/206) | RT4-5559 | [hayato-m126](https://github.com/hayato-m126) |
| `common`     | `library`         | update python library version   | [#207](https://github.com/tier4/driving_log_replayer/pull/207) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `common`     | `linter`          | use `ruff` linter               | [#208](https://github.com/tier4/driving_log_replayer/pull/208) | RT4-6363 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.5.0

Breaking change

| Module                                     | Feature         | Brief summary            | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------------------ | --------------- | ------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception`                               | `fp validation` | add `fp validation` mode | [#200](https://github.com/tier4/driving_log_replayer/pull/200) | RT4-5559 | [hayato-m126](https://github.com/hayato-m126) |
| `perception, perception_2d, traffic_light` | `object shape`  | support `unknown` label  | [#198](https://github.com/tier4/driving_log_replayer/pull/198) | RT4-5880 | [hayato-m126](https://github.com/hayato-m126) |
| `eagleye`                                  | `eagleye`       | add `eagleye` evaluation | [#203](https://github.com/tier4/driving_log_replayer/pull/203) | RT1-3254 | [kminoda](https://github.com/kminoda)         |
| `perception, perception_2d, traffic_light` | `scenario`      | update `sample scenario` | [#204](https://github.com/tier4/driving_log_replayer/pull/204) | RT4-6041 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.4.1

Major change

| Module   | Feature  | Brief summary           | Pull request                                                   | Jira     | Contributor                           |
| -------- | -------- | ----------------------- | -------------------------------------------------------------- | -------- | ------------------------------------- |
| `yabloc` | `yabloc` | add `yabloc` evaluation | [#201](https://github.com/tier4/driving_log_replayer/pull/201) | RT1-3255 | [kminoda](https://github.com/kminoda) |

## Version 1.4.0

Major change
Ensure that you are using Autoware.universe that incorporates [this PR](https://github.com/autowarefoundation/autoware.universe/pull/4128).

| Module             | Feature      | Brief summary                            | Pull request                                                   | Jira     | Contributor                                   |
| ------------------ | ------------ | ---------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization`     | initial pose | use map_height_fitter to correct z value | [#176](https://github.com/tier4/driving_log_replayer/pull/176) | RT4-5532 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag` | initial pose | use map_height_fitter to correct z value | [#179](https://github.com/tier4/driving_log_replayer/pull/179) | RT4-5532 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.17

Minor Tweak

| Module         | Feature              | Brief summary                                  | Pull request                                                   | Jira     | Contributor                                   |
| -------------- | -------------------- | ---------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization` | evaluation condition | delete converged condition to start evaluation | [#193](https://github.com/tier4/driving_log_replayer/pull/193) | RT4-5630 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.16

Bug fix, Minor Tweak

| Module                  | Feature          | Brief summary                         | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------- | ---------------- | ------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `obstacle_segmentation` | CMake            | fix build error due to lanelet2       | [#185](https://github.com/tier4/driving_log_replayer/pull/185) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `localization`          | summary          | add average and std_dev of likelihood | [#184](https://github.com/tier4/driving_log_replayer/pull/184) | RT4-5584 | [hayato-m126](https://github.com/hayato-m126) |
| `localization`          | NDT availability | update monitoring topic               | [#187](https://github.com/tier4/driving_log_replayer/pull/187) | RT4-5570 | [kminoda](https://github.com/kminoda)         |

## Version 1.3.15

Minor Tweak

| Module   | Feature | Brief summary          | Pull request                                                   | Jira | Contributor                                   |
| -------- | ------- | ---------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | ci      | add tier4 cspell-dicts | [#182](https://github.com/tier4/driving_log_replayer/pull/182) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.14

Minor Tweak

| Module         | Feature       | Brief summary        | Pull request                                                   | Jira     | Contributor                           |
| -------------- | ------------- | -------------------- | -------------------------------------------------------------- | -------- | ------------------------------------- |
| `localization` | state monitor | add ndt availability | [#178](https://github.com/tier4/driving_log_replayer/pull/178) | RT4-5570 | [kminoda](https://github.com/kminoda) |

## Version 1.3.13

Bug fix

| Module         | Feature  | Brief summary                | Pull request                                                   | Jira     | Contributor                                   |
| -------------- | -------- | ---------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization` | scenario | fill z value to initial pose | [#173](https://github.com/tier4/driving_log_replayer/pull/173) | RT4-5532 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.12

Documentation

| Module                              | Feature       | Brief summary                                  | Pull request                                                   | Jira     | Contributor                                   |
| ----------------------------------- | ------------- | ---------------------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception, obstacle_segmentation` | documentation | add description of t4_dataset conversion tools | [#171](https://github.com/tier4/driving_log_replayer/pull/171) | RT1-2597 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.11

Bug fix, Minor Tweak

| Module                     | Feature        | Brief summary             | Pull request                                                   | Jira     | Contributor                                   |
| -------------------------- | -------------- | ------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `localization, perception` | error handling | catch transform exception | [#169](https://github.com/tier4/driving_log_replayer/pull/169) | RT4-5289 | [hayato-m126](https://github.com/hayato-m126) |
| `common`                   | cli            | support scenario.yml      | [#167](https://github.com/tier4/driving_log_replayer/pull/167) | RT4-4995 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.10

Minor Tweak

| Module   | Feature       | Brief summary | Pull request                                                   | Jira | Contributor                               |
| -------- | ------------- | ------------- | -------------------------------------------------------------- | ---- | ----------------------------------------- |
| `common` | documentation | fix link      | [#165](https://github.com/tier4/driving_log_replayer/pull/165) | -    | [vios-fish](https://github.com/vios-fish) |

## Version 1.3.9

Minor Tweak

| Module   | Feature | Brief summary      | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------- | ------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | launch  | apply ShutdownOnce | [#163](https://github.com/tier4/driving_log_replayer/pull/163) | RT4-4690 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.8

Minor Tweak

| Module                     | Feature       | Brief summary             | Pull request                                                   | Jira     | Contributor                                   |
| -------------------------- | ------------- | ------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception, localization` | documentation | update result json format | [#161](https://github.com/tier4/driving_log_replayer/pull/161) | RT4-5057 | [hayato-m126](https://github.com/hayato-m126) |
| `perception`               | documentation | update bag topic          | [#160](https://github.com/tier4/driving_log_replayer/pull/160) | -        | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.7

Minor Tweak, Bug fix

| Module                                     | Feature         | Brief summary                    | Pull request                                                   | Jira     | Contributor                                   |
| ------------------------------------------ | --------------- | -------------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception, perception_2d`                | label attribute | support ignore attributes        | [#146](https://github.com/tier4/driving_log_replayer/pull/146) | RT4-4195 | [hayato-m126](https://github.com/hayato-m126) |
| `perception_2d`                            | target camera   | support multi camera             | [#148](https://github.com/tier4/driving_log_replayer/pull/148) | RT4-4195 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation`                    | scenario format | check if input polygon clockwise | [#153](https://github.com/tier4/driving_log_replayer/pull/153) | RT4-4162 | [hayato-m126](https://github.com/hayato-m126) |
| `perception, perception_2d, traffic_light` | scenario format | check scenario format            | [#155](https://github.com/tier4/driving_log_replayer/pull/155) | -        | [hayato-m126](https://github.com/hayato-m126) |
| `localization, performance_diag`           | initial pose    | fix call initial pose service    | [#157](https://github.com/tier4/driving_log_replayer/pull/157) | RT4-4961 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.6

Minor Tweak

| Module   | Feature | Brief summary                        | Pull request                                                   | Jira     | Contributor                                   |
| -------- | ------- | ------------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common` | launch  | use on_exit and delete event handler | [#151](https://github.com/tier4/driving_log_replayer/pull/151) | RT4-4162 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.5

Minor Tweak

| Module          | Feature    | Brief summary                 | Pull request                                                   | Jira     | Contributor                                   |
| --------------- | ---------- | ----------------------------- | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `perception_2d` | tracking2d | support tracking2d evaluation | [#142](https://github.com/tier4/driving_log_replayer/pull/142) | RT4-4048 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.4

Minor Tweak

| Module         | Feature              | Brief summary                        | Pull request                                                   | Jira     | Contributor                                   |
| -------------- | -------------------- | ------------------------------------ | -------------------------------------------------------------- | -------- | --------------------------------------------- |
| `common`       | linter and formatter | update linter and formatter settings | [#139](https://github.com/tier4/driving_log_replayer/pull/139) | RT4-3916 | [hayato-m126](https://github.com/hayato-m126) |
| `localization` | documentation        | update scenario sample               | [#138](https://github.com/tier4/driving_log_replayer/pull/138) | -        | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.3

Major Changes

| Module             | Feature                  | Brief summary                      | Pull request                                                   | Jira       | Contributor                                   |
| ------------------ | ------------------------ | ---------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `performance_diag` | update for ADAPI changes | delete fit_map_height service call | [#136](https://github.com/tier4/driving_log_replayer/pull/136) | T4PB-27288 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.2

Minor Tweak

| Module            | Feature      | Brief summary         | Pull request                                                   | Jira       | Contributor                                   |
| ----------------- | ------------ | --------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception_diag` | result.jsonl | fix output msg format | [#134](https://github.com/tier4/driving_log_replayer/pull/134) | T4PB-27178 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.1

Minor Tweak

| Module         | Feature | Brief summary             | Pull request                                                   | Jira       | Contributor                                   |
| -------------- | ------- | ------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `localization` | launch  | disable perception module | [#132](https://github.com/tier4/driving_log_replayer/pull/132) | T4PB-27344 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.3.0

Major Changes

Ensure that you are using Autoware.universe that incorporates [this PR](https://github.com/autowarefoundation/autoware.universe/pull/2724).

| Module         | Feature                  | Brief summary                      | Pull request                                                   | Jira       | Contributor                                   |
| -------------- | ------------------------ | ---------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `localization` | update for ADAPI changes | delete fit_map_height service call | [#129](https://github.com/tier4/driving_log_replayer/pull/129) | T4PB-27288 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.2.0

Major Changes

| Module                         | Feature       | Brief summary                                           | Pull request                                                   | Jira       | Contributor                                   |
| ------------------------------ | ------------- | ------------------------------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception_2d, traffic_light` | new evaluator | add perception_2d_evaluator and traffic_light_evaluator | [#122](https://github.com/tier4/driving_log_replayer/pull/122) | T4PB-25343 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.22

Minor Tweak

| Module             | Feature    | Brief summary     | Pull request                                                   | Jira | Contributor                                   |
| ------------------ | ---------- | ----------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `performance_diag` | bag record | drop camera topic | [#123](https://github.com/tier4/driving_log_replayer/pull/123) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.21

Documentation

| Module   | Feature  | Brief summary               | Pull request                                                   | Jira       | Contributor                                   |
| -------- | -------- | --------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `common` | document | update input bag topic list | [#120](https://github.com/tier4/driving_log_replayer/pull/120) | T4PB-26922 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.20

Minor Tweak

| Module       | Feature | Brief summary                    | Pull request                                                   | Jira       | Contributor                                   |
| ------------ | ------- | -------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception` | library | update for perception_eval 1.0.4 | [#118](https://github.com/tier4/driving_log_replayer/pull/118) | T4PB-26566 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.19

Bug fix

| Module       | Feature | Brief summary            | Pull request                                                   | Jira | Contributor                                   |
| ------------ | ------- | ------------------------ | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `perception` | metrics | fix count tp, fp, and fn | [#116](https://github.com/tier4/driving_log_replayer/pull/116) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.18

Minor Tweak, Bug fix

| Module       | Feature | Brief summary                          | Pull request                                                   | Jira       | Contributor                                   |
| ------------ | ------- | -------------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception` | library | update for perception_eval new feature | [#113](https://github.com/tier4/driving_log_replayer/pull/113) | T4PB-25343 | [hayato-m126](https://github.com/hayato-m126) |
| `cli`        | library | add dependent package                  | [#114](https://github.com/tier4/driving_log_replayer/pull/114) | T4PB-25810 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.17

Minor Tweak

| Module   | Feature       | Brief summary        | Pull request                                                   | Jira       | Contributor                                   |
| -------- | ------------- | -------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `common` | visualization | update rviz settings | [#111](https://github.com/tier4/driving_log_replayer/pull/111) | T4PB-25343 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.16

Bug fix

| Module       | Feature      | Brief summary                         | Pull request                                                   | Jira       | Contributor                                   |
| ------------ | ------------ | ------------------------------------- | -------------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception` | json library | use simplejson to convert NaN to null | [#109](https://github.com/tier4/driving_log_replayer/pull/109) | T4PB-25810 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.15

Bug fix

| Module     | Feature   | Brief summary | Pull request                                                   | Jira | Contributor                                     |
| ---------- | --------- | ------------- | -------------------------------------------------------------- | ---- | ----------------------------------------------- |
| `document` | html link | fix link      | [#105](https://github.com/tier4/driving_log_replayer/pull/105) | -    | [KeisukeShima](https://github.com/KeisukeShima) |
| `document` | html link | fix link      | [#106](https://github.com/tier4/driving_log_replayer/pull/106) | -    | [hayato-m126](https://github.com/hayato-m126)   |

## Version 1.1.14

Minor Tweak

| Module     | Feature        | Brief summary                                | Pull request                                                   | Jira | Contributor                                   |
| ---------- | -------------- | -------------------------------------------- | -------------------------------------------------------------- | ---- | --------------------------------------------- |
| `document` | mkdocs setting | update setting for mkdocs material version 9 | [#102](https://github.com/tier4/driving_log_replayer/pull/102) | -    | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.13

Minor Tweak

| Module     | Feature       | Brief summary                         | Pull request                                                 | Jira       | Contributor                                   |
| ---------- | ------------- | ------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `analyzer` | visualization | update obstacle segmentation analyzer | [#94](https://github.com/tier4/driving_log_replayer/pull/94) | T4PB-24956 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.12

Minor Tweak

| Module                  | Feature       | Brief summary                 | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------- | ----------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `analyzer`              | refactor      | refactor analyzer             | [#89](https://github.com/tier4/driving_log_replayer/pull/89) | T4PB-24956 | [hayato-m126](https://github.com/hayato-m126) |
| `ci`                    | github action | update pre-commit-shfmt       | [#95](https://github.com/tier4/driving_log_replayer/pull/95) | -          | [hayato-m126](https://github.com/hayato-m126) |
| `perception`            | dependency    | update perception eval        | [#98](https://github.com/tier4/driving_log_replayer/pull/98) | T4PB-25347 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | api           | use awapi to get stop reasons | [#99](https://github.com/tier4/driving_log_replayer/pull/99) | T4PB-25316 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.11

Bug fix

| Module | Feature             | Brief summary                                | Pull request                                                 | Jira       | Contributor                                     |
| ------ | ------------------- | -------------------------------------------- | ------------------------------------------------------------ | ---------- | ----------------------------------------------- |
| `cli`  | kill zombie process | Fixed to not kill processes unrelated to ros | [#92](https://github.com/tier4/driving_log_replayer/pull/92) | T4PB-25129 | [M(-\_-)Sakamoto](https://github.com/Motsu-san) |

## Version 1.1.10

Bug fix

| Module                  | Feature            | Brief summary             | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------------ | ------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `cli`                   | sub command        | fix command name          | [#80](https://github.com/tier4/driving_log_replayer/pull/80) | -          | [hayato-m126](https://github.com/hayato-m126) |
| `common`                | license            | use Apache-2.0            | [#82](https://github.com/tier4/driving_log_replayer/pull/82) | -          | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | dependency         | add exec depend           | [#83](https://github.com/tier4/driving_log_replayer/pull/83) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`      | exception handling | handle TransformException | [#85](https://github.com/tier4/driving_log_replayer/pull/85) | T4PB-24969 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.9

Bug fix

| Module                  | Feature       | Brief summary    | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------- | ---------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | visualization | fix import error | [#78](https://github.com/tier4/driving_log_replayer/pull/78) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.8

Minor Tweak, Bug fix

| Module                  | Feature       | Brief summary                                                                 | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------- | ----------------------------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | marker color  | The result of the topic's rate is reflected in the color of the bounding box. | [#72](https://github.com/tier4/driving_log_replayer/pull/72) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | visualization | add graph data to result.jsonl                                                | [#73](https://github.com/tier4/driving_log_replayer/pull/73) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`      | localization  | fix infinite wait when localization module off                                | [#75](https://github.com/tier4/driving_log_replayer/pull/75) | T4PB-24705 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.7

Minor Tweak, Bug fix

| Module                             | Feature       | Brief summary                                        | Pull request                                                 | Jira       | Contributor                                   |
| ---------------------------------- | ------------- | ---------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation`            | visualization | publish and record data for visualization            | [#67](https://github.com/tier4/driving_log_replayer/pull/67) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `performance_diag`                 | initial pose  | delete uninitialized publisher                       | [#68](https://github.com/tier4/driving_log_replayer/pull/68) | T4PB-24491 | [hayato-m126](https://github.com/hayato-m126) |
| `localization`, `performance_diag` | initial pose  | use map fit service to correct `initialpose z value` | [#70](https://github.com/tier4/driving_log_replayer/pull/70) | T4PB-24491 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.6

Minor Tweak

| Module                             | Feature      | Brief summary               | Pull request                                                 | Jira       | Contributor                                   |
| ---------------------------------- | ------------ | --------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `localization`, `performance_diag` | initial pose | set initial pose via ad-api | [#65](https://github.com/tier4/driving_log_replayer/pull/65) | T4PB-24491 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.5

Bug fix

| Module       | Feature      | Brief summary                                            | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | ------------ | -------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | marker color | different colors are applied to tp_gt, tp_est, fp and fn | [#61](https://github.com/tier4/driving_log_replayer/pull/61) | T4PB-24425 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.4

Major Changes

| Module       | Feature        | Brief summary                                 | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | -------------- | --------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | launch process | Removed waiting for conversion of onnx files. | [#59](https://github.com/tier4/driving_log_replayer/pull/59) | T4PB-24362 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.3

Major Changes

| Module                  | Feature             | Brief summary                                    | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------------- | ------------------------------------------------ | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle segmentation` | evaluation settings | set evaluation period for each bbox              | [#51](https://github.com/tier4/driving_log_replayer/pull/51) | T4PB-24138 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle segmentation` | result file         | output timestamp of bounding box and point cloud | [#52](https://github.com/tier4/driving_log_replayer/pull/52) | T4PB-24238 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.2

Minor Tweak

| Module   | Feature    | Brief summary                                         | Pull request                                                 | Jira       | Contributor                                   |
| -------- | ---------- | ----------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `common` | cli        | kill zombie process                                   | [#45](https://github.com/tier4/driving_log_replayer/pull/45) | T4PB-24124 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | cli        | fix generated command to kill zombie process          | [#46](https://github.com/tier4/driving_log_replayer/pull/46) | T4PB-24124 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | cli        | modified commands to avoid killing the running shell. | [#47](https://github.com/tier4/driving_log_replayer/pull/47) | T4PB-24124 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | dictionary | add local dictionary                                  | [#48](https://github.com/tier4/driving_log_replayer/pull/48) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | code style | fix lit check                                         | [#49](https://github.com/tier4/driving_log_replayer/pull/49) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.1.1

Major Changes

| Module     | Feature  | Brief summary                          | Pull request                                                 | Jira       | Contributor                                     |
| ---------- | -------- | -------------------------------------- | ------------------------------------------------------------ | ---------- | ----------------------------------------------- |
| `analyzer` | analyzer | add analyzer to visualize result.jsonl | [#42](https://github.com/tier4/driving_log_replayer/pull/42) | T4PB-23864 | [KeisukeShima](https://github.com/KeisukeShima) |
| `analyzer` | analyzer | fix pre-commit check                   | [#43](https://github.com/tier4/driving_log_replayer/pull/43) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126)   |
| `analyzer` | analyzer | add cli command and ros2 launch        | [#44](https://github.com/tier4/driving_log_replayer/pull/44) | T4PB-23864 | [hayato-m126](https://github.com/hayato-m126)   |

## Version 1.1.0

Major Changes

| Module                  | Feature                      | Brief summary                                            | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ---------------------------- | -------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | point cloud topic rate check | update diagnostics status name of point cloud topic rate | [#40](https://github.com/tier4/driving_log_replayer/pull/40) | T4PB-23868 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.12

Bug fix

| Module                  | Feature      | Brief summary                  | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | ------------ | ------------------------------ | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | frame result | fix status and judgement logic | [#37](https://github.com/tier4/driving_log_replayer/pull/37) | T4PB-23309 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.11

Major Changes

| Module                  | Feature   | Brief summary                                 | Pull request                                                 | Jira       | Contributor                                   |
| ----------------------- | --------- | --------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `obstacle_segmentation` | test mode | support 3 test mode for obstacle_segmentation | [#35](https://github.com/tier4/driving_log_replayer/pull/35) | T4PB-23309 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.10

Minor Tweaks

| Module   | Feature       | Brief summary                 | Pull request                                                 | Jira       | Contributor                                   |
| -------- | ------------- | ----------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `common` | documentation | update Japanese Documentation | [#30](https://github.com/tier4/driving_log_replayer/pull/30) | T4PB-23145 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.9

Bug Fix

| Module   | Feature       | Brief summary         | Pull request                                                 | Jira       | Contributor                                   |
| -------- | ------------- | --------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `common` | documentation | fix MkDocs Dependency | [#28](https://github.com/tier4/driving_log_replayer/pull/28) | T4PB-21730 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.8

Minor Tweaks

| Module   | Feature       | Brief summary             | Pull request                                                 | Jira       | Contributor                                   |
| -------- | ------------- | ------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `common` | documentation | add English Documentation | [#20](https://github.com/tier4/driving_log_replayer/pull/20) | T4PB-21730 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.7

Bug Fix

| Module       | Feature                  | Brief summary | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | ------------------------ | ------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | pass fail decision logic | fix logic     | [#25](https://github.com/tier4/driving_log_replayer/pull/25) | T4PB-22918 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.6

Bug Fix

| Module   | Feature              | Brief summary                                                               | Pull request                                                 | Jira       | Contributor                                   |
| -------- | -------------------- | --------------------------------------------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `common` | publish initial pose | Cast the value of the initial position described in the scenario to a float | [#22](https://github.com/tier4/driving_log_replayer/pull/22) | T4PB-22700 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.5

Major Changes

| Module       | Feature             | Brief summary                    | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | ------------------- | -------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | database evaluation | output database_result.json file | [#17](https://github.com/tier4/driving_log_replayer/pull/17) | T4PB-22358 | [hayato-m126](https://github.com/hayato-m126) |
| `perception` | database evaluation | fix exit with failure 1 file     | [#18](https://github.com/tier4/driving_log_replayer/pull/18) | T4PB-22358 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.4

Minor Tweaks

| Module   | Feature       | Brief summary                        | Pull request                                                 | Jira       | Contributor                                   |
| -------- | ------------- | ------------------------------------ | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `common` | documentation | Adopted MkDocs as documentation tool | [#14](https://github.com/tier4/driving_log_replayer/pull/14) | T4PB-22101 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.3

Major Changes

| Module       | Feature          | Brief summary                          | Pull request                                                 | Jira       | Contributor                                   |
| ------------ | ---------------- | -------------------------------------- | ------------------------------------------------------------ | ---------- | --------------------------------------------- |
| `perception` | launch parameter | delete use_pointcloud_container option | [#12](https://github.com/tier4/driving_log_replayer/pull/12) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.2

Minor Tweaks

| Module   | Feature       | Brief summary                     | Pull request                                               | Jira       | Contributor                                   |
| -------- | ------------- | --------------------------------- | ---------------------------------------------------------- | ---------- | --------------------------------------------- |
| `common` | documentation | update sample scenario and config | [#6](https://github.com/tier4/driving_log_replayer/pull/6) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |
| `common` | documentation | update sample scenario            | [#7](https://github.com/tier4/driving_log_replayer/pull/7) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.1

Minor Tweaks

| Module                  | Feature       | Brief summary                              | Pull request                                               | Jira       | Contributor                                   |
| ----------------------- | ------------- | ------------------------------------------ | ---------------------------------------------------------- | ---------- | --------------------------------------------- |
| `perception`            | visualization | shorten bounding box uuid to display       | [#2](https://github.com/tier4/driving_log_replayer/pull/2) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |
| `obstacle_segmentation` | remap topic   | remap concatenated point cloud in bag file | [#4](https://github.com/tier4/driving_log_replayer/pull/4) | T4PB-21729 | [hayato-m126](https://github.com/hayato-m126) |

## Version 1.0.0

Major Changes

| Module   | Feature | Brief summary  | Pull request                                               | Jira | Contributor                                   |
| -------- | ------- | -------------- | ---------------------------------------------------------- | ---- | --------------------------------------------- |
| `common` | oss     | publish as oss | [#0](https://github.com/tier4/driving_log_replayer/pull/0) | -    | [hayato-m126](https://github.com/hayato-m126) |
