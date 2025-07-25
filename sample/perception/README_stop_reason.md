# Stop Reason Evaluation for Perception

This feature adds stop reason evaluation to the perception simulation, allowing you to evaluate whether the vehicle stops for specific reasons (like intersections, traffic lights, obstacles) within specified time windows and distance ranges.

## Overview

The stop reason evaluation feature:

- Monitors `/awapi/autoware/get/status` messages for stop reasons
- Evaluates stop reasons against configurable criteria
- Supports both True Positive (TP) and True Negative (TN) evaluation modes
- Enforces minimum time intervals between events (tolerance_interval)
- Provides per-frame and summary evaluation results

## Configuration

Add a `StopReasonCriterion` section to your perception scenario YAML under `Evaluation.Conditions`:

```yaml
StopReasonCriterion:
  Intersection:
    start_time: 1649138880.0 # Start time for evaluation (Unix timestamp)
    end_time: 1649138900.0 # End time for evaluation (Unix timestamp)
    base_stop_line_dist:
      min: 0.0 # Minimum distance to stop line (meters)
      max: 1.0 # Maximum distance to stop line (meters)
    tolerance_interval: 0.5 # Minimum time between accepted events (seconds)
    pass_rate: 90.0 # Required pass rate percentage
    evaluation_type: TP # TP: True Positive (expect stop), TN: True Negative (expect no stop)
```

## Parameters

### `start_time` / `end_time`

- Unix timestamps defining the evaluation window
- Only stop reasons within this time range will be evaluated

### `base_stop_line_dist`

- `min`: Minimum acceptable distance to stop line (meters)
- `max`: Maximum acceptable distance to stop line (meters)
- Stop reasons with distances outside this range will be marked as "Fail"

### `tolerance_interval`

- Minimum time (seconds) required between accepted stop reason events
- Prevents counting rapid successive events as separate valid stops
- Events within this interval of the last accepted event will be skipped

### `pass_rate`

- Required percentage of valid stop reason events to pass evaluation
- Calculated as: (passed_events / total_events) \* 100

### `evaluation_type`

- `TP` (True Positive): Expect the stop event to occur during the evaluation window
- `TN` (True Negative): Expect NO stop event to occur during the evaluation window

## Evaluation Types

### True Positive (TP) Evaluation

- **Purpose**: Verify that the vehicle stops for the specified reason
- **Success Criteria**: Stop event occurs within the distance range during the time window
- **Use Case**: Verify that the vehicle stops at intersections, traffic lights, etc.

### True Negative (TN) Evaluation

- **Purpose**: Verify that the vehicle does NOT stop for the specified reason when it shouldn't
- **Success Criteria**: No stop event occurs for the specified reason during the time window
- **Use Case**: Verify that the vehicle doesn't stop unnecessarily (e.g., no false positives)

## Stop Reason Types

Common stop reason types that can be evaluated:

- `Intersection`: Stop at intersection
- `TrafficLight`: Stop at traffic light
- `ObstacleStop`: Stop due to obstacle
- `Crosswalk`: Stop at crosswalk
- `Walkway`: Stop at walkway

## Example Scenarios

### Basic Example (`scenario_stop_reason_simple.yaml`)

Evaluates intersection stops with basic criteria using TP evaluation.

### Advanced Example (`scenario_stop_reason.yaml`)

Evaluates multiple stop reason types with different criteria for each using TP evaluation.

### True Negative Example (`scenario_stop_reason_tn.yaml`)

Demonstrates both TP and TN evaluation modes using actual stop reason names:

```yaml
StopReasonCriterion:
  # True Positive: expect Intersection stop to occur
  Intersection:
    start_time: 1649138880.0
    end_time: 1649138900.0
    base_stop_line_dist:
      min: 0.0
      max: 1.0
    tolerance_interval: 0.5
    pass_rate: 90.0
    evaluation_type: TP # True Positive: expect stop event
  # True Negative: expect NO TrafficLight stop to occur
  TrafficLight:
    start_time: 1649138885.0
    end_time: 1649138895.0
    base_stop_line_dist:
      min: 0.0
      max: 2.0
    tolerance_interval: 0.3
    pass_rate: 85.0
    evaluation_type: TN # True Negative: expect NO stop event
  # True Negative: expect NO ObstacleStop to occur
  ObstacleStop:
    start_time: 1649138890.0
    end_time: 1649138905.0
    base_stop_line_dist:
      min: 0.0
      max: 3.0
    tolerance_interval: 1.0
    pass_rate: 80.0
    evaluation_type: TN # True Negative: expect NO stop event
```

**Note**: The stop reason names in the configuration must match exactly with the stop reason names that appear in the `/awapi/autoware/get/status` topic messages (e.g., "Intersection", "TrafficLight", "ObstacleStop", etc.).

## Running the Evaluation

1. Use one of the sample scenarios or create your own with `StopReasonCriterion` configuration
2. Run the perception simulation as usual:

   ```bash
   dlr2 simulation run /path/to/scenario/directory
   ```

3. The system will automatically enable planning and control components when stop reason evaluation is configured

## Results

### Per-Frame Results

Each frame in `result.jsonl` will include stop reason evaluation results:

#### True Positive Example

```json
{
  "Intersection": {
    "StopReason": {
      "Result": {
        "Total": "Success",
        "Frame": "Success"
      },
      "Info": {
        "Reason": "Intersection",
        "Distance": 0.5,
        "Timestamp": 1649138885.0,
        "EvaluationType": "TP"
      }
    }
  }
}
```

#### True Negative Example

```json
{
  "TrafficLight": {
    "StopReason": {
      "Result": {
        "Total": "Success",
        "Frame": "Success"
      },
      "Info": {
        "Reason": "ObstacleStop",
        "Distance": 2.0,
        "Timestamp": 1649138905.0,
        "EvaluationType": "TN"
      }
    }
  }
}
```

### Summary Results

The final result summary will include stop reason evaluation:

```text
Passed: criteria0 (Success): 95 / 100 -> 95.00%, stop reason(Success): 8 / 10 -> 80.00%
```

## CSV Output

Stop reason data is also saved to `stop_reasons.csv` in the result archive directory, containing:

- Timestamp
- Stop reason index
- Reason type
- Distance to stop pose
- Position (x, y, z)
- Orientation (qz, qw)

## Notes

- When `StopReasonCriterion` is configured, planning and control components are automatically enabled
- For TP evaluation: Only considers stop reasons that match the configured reason type
- For TN evaluation: Processes all stop reason messages during the evaluation window
- Events outside the time window or within the tolerance interval are skipped
- The final evaluation passes only if all configured stop reasons meet their pass rate requirements
- TN evaluation automatically succeeds if no target stop reason is received during the evaluation window
