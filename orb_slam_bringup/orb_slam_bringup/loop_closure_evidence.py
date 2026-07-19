import json
import argparse
from pathlib import Path
import sys

MARKERS = {
    "loop_detected": "*Loop detected",
    "bad_loop": "BAD LOOP!!!",
    "local_mapping_stop": "Local Mapping STOP",
    "local_mapping_release": "Local Mapping RELEASE",
    "merge_detected": "*Merge detected",
}

def evaluate_artifact(artifact_dir: Path) -> dict:
    metrics_path = artifact_dir / "metrics.json"
    log_path = artifact_dir / "orb_slam3_wrapper.log"
    
    if not metrics_path.exists():
        raise FileNotFoundError(f"Missing metrics file: {metrics_path}")
        
    with open(metrics_path, "r") as f:
        try:
            metrics = json.load(f)
        except json.JSONDecodeError:
            raise ValueError(f"Invalid JSON in metrics file: {metrics_path}")

    evidence = {
        "marker_counts": {k: 0 for k in MARKERS},
        "diagnoses": [],
        "passed": False
    }
    
    if not log_path.exists():
        evidence["diagnoses"].append("core_log_missing")
        return evidence
        
    with open(log_path, "r") as f:
        log_content = f.read()
        
    for marker_key, marker_text in MARKERS.items():
        evidence["marker_counts"][marker_key] = log_content.count(marker_text)
        
    core_loops = evidence["marker_counts"]["loop_detected"] + evidence["marker_counts"]["bad_loop"]
    wrapper_loops = len(metrics.get("loops", []))
    
    # Calculate diagnoses in deterministic order
    if core_loops == 0:
        evidence["diagnoses"].append("no_core_loop_detected")
    elif core_loops > 0 and wrapper_loops == 0:
        evidence["diagnoses"].append("core_loop_unobserved")

    # Check if wrapper loop lacks published rebuild
    loop_rebuild_missing = False
    if wrapper_loops > 0:
        published_revisions = [rev["graph_revision"] for rev in metrics.get("revisions", []) if rev.get("state") == "PUBLISHED"]
        
        for loop in metrics.get("loops", []):
            loop_rev = loop.get("graph_revision")
            if loop_rev is not None:
                # Need a PUBLISHED revision >= loop graph_revision
                if not any(pub_rev >= loop_rev for pub_rev in published_revisions):
                    loop_rebuild_missing = True
                    break
                    
    if loop_rebuild_missing:
        evidence["diagnoses"].append("loop_rebuild_missing")
            
    if evidence["marker_counts"]["local_mapping_stop"] > evidence["marker_counts"]["local_mapping_release"]:
        evidence["diagnoses"].append("local_mapping_stop_unreleased")
            
    if not evidence["diagnoses"]:
        evidence["diagnoses"].append("observed_and_rebuilt")
        evidence["passed"] = True
    
    return evidence

def main():
    parser = argparse.ArgumentParser(description="Evaluate loop closure evidence")
    parser.add_argument("--artifact-dir", type=Path, required=True, help="Path to artifact directory")
    args = parser.parse_args()
    
    try:
        evidence = evaluate_artifact(args.artifact_dir)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 2

    import tempfile
    import os

    output_path = args.artifact_dir / "loop_closure_evidence.json"
    
    fd, tmp_path = tempfile.mkstemp(dir=args.artifact_dir, prefix="loop_closure_evidence_", suffix=".tmp")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(evidence, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, output_path)
    except Exception as e:
        os.remove(tmp_path)
        raise
        
    return 0 if evidence["passed"] else 1

if __name__ == "__main__":
    sys.exit(main())
