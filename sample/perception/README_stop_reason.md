# Stop Reason Evaluation for Perception

This feature adds stop reason evaluation to the perception simulation, allowing you to evaluate whether the vehicle stops for specific reasons (like intersections, traffic lights, obstacles) within specified time windows and distance ranges.

## Overview

The stop reason evaluation feature:
- Monitors `/awapi/autoware/get/status` messages for stop reasons
- Evaluates stop reasons against configurable criteria
- Enforces minimum time intervals between events (tolerance_interval)
- Provides per-frame and summary evaluation results

## Configuration

Add a `StopReasonCriterion` section to your perception scenario YAML under `Evaluation.Conditions`:

```yaml
StopReasonCriterion:
  Intersection:
    start_time: 1649138880.0  # Start time for evaluation (Unix timestamp)
    end_time: 1649138900.0    # End time for evaluation (Unix timestamp)
    base_stop_line_dist:
      min: 0.0  # Minimum distance to stop line (meters)
      max: 1.0  # Maximum distance to stop line (meters)
    tolerance_interval: 0.5   # Minimum time between accepted events (seconds)
    pass_rate: 90.0           # Required pass rate percentage
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
- Calculated as: (passed_events / total_events) * 100

## Stop Reason Types

Common stop reason types that can be evaluated:
- `Intersection`: Stop at intersection
- `TrafficLight`: Stop at traffic light
- `ObstacleStop`: Stop due to obstacle
- `Crosswalk`: Stop at crosswalk
- `Walkway`: Stop at walkway

## Example Scenarios

### Basic Example (`scenario_stop_reason_simple.yaml`)
Evaluates intersection stops with basic criteria.

### Advanced Example (`scenario_stop_reason.yaml`)
Evaluates multiple stop reason types with different criteria for each.

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
        "Timestamp": 1649138885.0
      }
    }
  }
}
```
```

### Summary Results
The final result summary will include stop reason evaluation:
```
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
- The evaluation only considers stop reasons that match the configured reason type
- Events outside the time window or within the tolerance interval are skipped
- The final evaluation passes only if all configured stop reasons meet their pass rate requirements 