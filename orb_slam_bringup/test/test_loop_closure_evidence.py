import json
from pathlib import Path

def write_metrics(path: Path, loops=None, map_revisions=None, initialized=True, ok_ratio=0.9, deadlock=False):
    metrics = {
        "is_initialized": initialized,
        "is_deadlocked": deadlock,
        "tracking_ok_ratio": ok_ratio,
        "loops": loops if loops is not None else [],
        "map_revisions": map_revisions if map_revisions is not None else []
    }
    with open(path / "metrics.json", "w") as f:
        json.dump(metrics, f)

def test_core_loop_without_wrapper_event_is_unobserved(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[], map_revisions=[])
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["marker_counts"]["loop_detected"] == 1
    assert "core_loop_unobserved" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_observed_loop_with_published_rebuild_passes(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[{"graph_revision": 4}],
                  map_revisions=[{"state": "PUBLISHED", "graph_revision": 4}],
                  initialized=True, ok_ratio=0.9, deadlock=False)
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\nLocal Mapping STOP\nLocal Mapping RELEASE\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["diagnoses"] == ["observed_and_rebuilt"]
    assert evidence["passed"] is True

def test_bad_loop_is_unobserved(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[], map_revisions=[])
    (tmp_path / "orb_slam3_wrapper.log").write_text("BAD LOOP!!!\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["marker_counts"]["bad_loop"] == 1
    assert "core_loop_unobserved" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_unreleased_local_mapping_stop(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[], map_revisions=[])
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\nLocal Mapping STOP\n")
    evidence = evaluate_artifact(tmp_path)
    assert evidence["marker_counts"]["local_mapping_stop"] == 1
    assert evidence["marker_counts"]["local_mapping_release"] == 0
    assert "local_mapping_stop_unreleased" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_missing_core_log(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[], map_revisions=[])
    evidence = evaluate_artifact(tmp_path)
    assert "core_log_missing" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_wrapper_loop_lacking_published_rebuild(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    write_metrics(tmp_path, loops=[{"graph_revision": 4}],
                  map_revisions=[{"state": "PUBLISHED", "graph_revision": 3}])
    (tmp_path / "orb_slam3_wrapper.log").write_text("*Loop detected\nLocal Mapping STOP\nLocal Mapping RELEASE\n")
    evidence = evaluate_artifact(tmp_path)
    assert "loop_rebuild_missing" in evidence["diagnoses"]
    assert evidence["passed"] is False

def test_observed_loop_with_only_building_rebuild_is_missing(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact

    write_metrics(
        tmp_path,
        loops=[{"graph_revision": 4}],
        map_revisions=[{"state": "BUILDING", "graph_revision": 5}],
    )
    (tmp_path / "orb_slam3_wrapper.log").write_text(
        "*Loop detected\nLocal Mapping STOP\nLocal Mapping RELEASE\n"
    )

    evidence = evaluate_artifact(tmp_path)

    assert evidence["diagnoses"] == ["loop_rebuild_missing"]
    assert evidence["passed"] is False

def test_atomic_write_creates_file(tmp_path: Path, monkeypatch):
    from orb_slam_bringup.loop_closure_evidence import main
    write_metrics(tmp_path, loops=[], map_revisions=[])
    (tmp_path / "orb_slam3_wrapper.log").write_text("")
    monkeypatch.setattr("sys.argv", ["test_loop_closure_evidence", "--artifact-dir", str(tmp_path)])
    
    result = main()
    
    # Check return code
    assert result == 1
    
    output_file = tmp_path / "loop_closure_evidence.json"
    assert output_file.exists()
    
    with open(output_file, "r") as f:
        evidence = json.load(f)
        
    assert "no_core_loop_detected" in evidence["diagnoses"]


def test_independent_diagnoses_multiple_errors(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import evaluate_artifact
    # wrapper_loops > 0, so loop_rebuild_missing check will run
    write_metrics(tmp_path, loops=[{"graph_revision": 4}], map_revisions=[])
    # core_loops == 0, so no_core_loop_detected will be added
    (tmp_path / "orb_slam3_wrapper.log").write_text("")
    
    evidence = evaluate_artifact(tmp_path)
    
    # Assert exact diagnoses in exact order
    assert evidence["diagnoses"] == ["no_core_loop_detected", "loop_rebuild_missing"]

def test_runner_script_contract():
    script_path = Path(__file__).parent.parent.parent / "tools" / "run_circle_loop_closure_evaluation.sh"
    if not script_path.exists():
        # Create a dummy to fail the exact test first
        pass
    
    text = script_path.read_text(encoding="utf-8")
    assert "rate:=1" in text
    assert "benchmark_mode:=off" in text
    assert "run-1" in text and "run-2" in text and "run-3" in text
    assert "loop_closure_evidence" in text
    assert "passed_runs" in text

def test_summarize_runs(tmp_path: Path):
    from orb_slam_bringup.loop_closure_evidence import summarize_runs
    
    paths = []
    for i, p_val in enumerate([True, True, False]):
        p = tmp_path / f"run-{i+1}.json"
        p.write_text(json.dumps({"passed": p_val}))
        paths.append(p)
        
    res = summarize_runs(paths)
    assert res["passed_runs"] == 2
    assert res["passed"] is True
    
    paths2 = []
    for i, p_val in enumerate([True, False, False]):
        p = tmp_path / f"run-{i+4}.json"
        p.write_text(json.dumps({"passed": p_val}))
        paths2.append(p)
        
    res2 = summarize_runs(paths2)
    assert res2["passed_runs"] == 1
    assert res2["passed"] is False
