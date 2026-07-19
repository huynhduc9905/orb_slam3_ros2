import json
from pathlib import Path

def write_metrics(path: Path, loops=None, revisions=None, initialized=True, ok_ratio=0.9, deadlock=False):
    metrics = {
        "is_initialized": initialized,
        "is_deadlocked": deadlock,
        "tracking_ok_ratio": ok_ratio,
        "loops": loops if loops is not None else [],
        "revisions": revisions if revisions is not None else []
    }
    with open(path / "metrics.json", "w") as f:
        json.dump(metrics, f)

def test_core_loop_without_wrapper_event_is_unobserved(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[], revisions=[])
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["marker_counts"]["loop_detected"] == 1
    assert "core_loop_unobserved" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_observed_loop_with_published_rebuild_passes(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[{"graph_revision": 4}],
                  revisions=[{"state": "PUBLISHED", "graph_revision": 4}],
                  initialized=True, ok_ratio=0.9, deadlock=False)
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\nLocal Mapping STOP\nLocal Mapping RELEASE\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["diagnoses"] == ["observed_and_rebuilt"]
    assert evidence["passed"] is True

def test_bad_loop_is_unobserved(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[], revisions=[])
    (tmp_path / "orb_slam3_wrapper.log").write_text("BAD LOOP!!!\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["marker_counts"]["bad_loop"] == 1
    assert "core_loop_unobserved" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_unreleased_local_mapping_stop(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[], revisions=[])
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\nLocal Mapping STOP\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["marker_counts"]["local_mapping_stop"] == 1
    assert evidence["marker_counts"]["local_mapping_release"] == 0
    assert "local_mapping_stop_unreleased" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_missing_core_log(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[], revisions=[])
    evidence = evaluate_artifact(tmp_path)
    assert "core_log_missing" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_wrapper_loop_lacking_published_rebuild(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[{"graph_revision": 4}],
                  revisions=[{"state": "PUBLISHED", "graph_revision": 3}])
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\nLocal Mapping STOP\nLocal Mapping RELEASE\n")
    evidence = evaluate_artifact(tmp_path)
    assert "loop_rebuild_missing" in evidence["diagnoses"]
    assert evidence["passed"] is False
