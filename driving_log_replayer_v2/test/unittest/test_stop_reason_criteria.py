import pytest
from driving_log_replayer_v2.perception.models import StopReasonEvaluation, StopReasonEvaluationItem

@pytest.fixture
def stop_reason_eval_item():
    config = StopReasonEvaluation(
        start_time=10.0,
        end_time=20.0,
        base_stop_line_dist={"min": 0.0, "max": 1.0},
        tolerance_interval=0.5,
        pass_rate=50.0,
    )
    return StopReasonEvaluationItem(name="Intersection", condition=config)

def test_stop_reason_eval_basic(stop_reason_eval_item):
    # Should skip: outside time window
    result = stop_reason_eval_item.set_frame({"reason": "Intersection", "timestamp": 5.0, "dist_to_stop_pos": 0.5})
    assert result["StopReason"]["Result"] == "Skip"
    # Should skip: not target reason
    result = stop_reason_eval_item.set_frame({"reason": "TrafficLight", "timestamp": 12.0, "dist_to_stop_pos": 0.5})
    assert result["StopReason"]["Result"] == "Skip"
    # Should pass: first valid event
    result = stop_reason_eval_item.set_frame({"reason": "Intersection", "timestamp": 11.0, "dist_to_stop_pos": 0.5})
    assert result["StopReason"]["Result"]["Frame"] == "Success"
    # Should skip: within tolerance interval
    result = stop_reason_eval_item.set_frame({"reason": "Intersection", "timestamp": 11.2, "dist_to_stop_pos": 0.5})
    assert result["StopReason"]["Result"] == "Skip"
    # Should fail: valid event but out of distance
    result = stop_reason_eval_item.set_frame({"reason": "Intersection", "timestamp": 12.0, "dist_to_stop_pos": 2.0})
    assert result["StopReason"]["Result"]["Frame"] == "Fail"
    # Should pass: valid event after tolerance interval
    result = stop_reason_eval_item.set_frame({"reason": "Intersection", "timestamp": 12.0 + 0.6, "dist_to_stop_pos": 0.8})
    assert result["StopReason"]["Result"]["Frame"] == "Success"
    # Check summary and pass rate
    assert stop_reason_eval_item.get_passed() == 2
    assert stop_reason_eval_item.get_total() == 3
    assert stop_reason_eval_item.get_pass_rate() == pytest.approx(2/3*100, rel=1e-2)
    assert "Intersection" in stop_reason_eval_item.get_summary()

def test_stop_reason_timeout(stop_reason_eval_item):
    # Check timeout before evaluation window starts
    result = stop_reason_eval_item.check_timeout(5.0)
    assert result is None  # Should not timeout outside window
    
    # Check timeout at start of window (no timeout yet)
    result = stop_reason_eval_item.check_timeout(10.0)
    assert result is None  # Should not timeout immediately
    
    # Check timeout after tolerance interval without receiving target reason
    result = stop_reason_eval_item.check_timeout(10.6)  # 10.0 + 0.5 + 0.1
    assert result is not None
    assert result["StopReason"]["Result"]["Frame"] == "Fail"
    assert result["StopReason"]["Info"]["Reason"] == "TIMEOUT"
    assert "No Intersection received within 0.5s" in result["StopReason"]["Info"]["Message"]
    
    # Check that timeout increases total count
    assert stop_reason_eval_item.get_total() == 1
    assert stop_reason_eval_item.get_passed() == 0
    
    # Check timeout again (should not spam)
    result = stop_reason_eval_item.check_timeout(10.7)
    assert result is None  # Should not spam timeouts
    
    # Check timeout after receiving a valid event
    stop_reason_eval_item.set_frame({"reason": "Intersection", "timestamp": 11.0, "dist_to_stop_pos": 0.5})
    result = stop_reason_eval_item.check_timeout(12.0)
    assert result is None  # Should not timeout after receiving valid event 