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

from typing import Sequence

def summarize_runs(evidence_paths: Sequence[Path]) -> dict:
    runs = [json.loads(path.read_text(encoding="utf-8")) for path in evidence_paths]
    passed_runs = sum(bool(run.get("passed")) for run in runs)
    return {
        "runs": [str(path) for path in evidence_paths],
        "passed_runs": passed_runs,
        "required_passes": 2,
        "passed": passed_runs >= 2,
    }

def main():
    parser = argparse.ArgumentParser(description="Evaluate loop closure evidence")
    subparsers = parser.add_subparsers(dest="command", help="Commands")
    
    # Subcommand for single run evaluation
    eval_parser = subparsers.add_parser("evaluate", help="Evaluate a single artifact directory")
    eval_parser.add_argument("--artifact-dir", type=Path, required=True, help="Path to artifact directory")
    
    # Subcommand for summarizing multiple runs
    summary_parser = subparsers.add_parser("summary", help="Summarize multiple evidence files")
    summary_parser.add_argument("--inputs", type=Path, nargs="+", required=True, help="Paths to loop_closure_evidence.json files")
    summary_parser.add_argument("--output", type=Path, required=True, help="Path to save summary.json")
    
    # Allow backward compatibility for when the script was called directly without a subcommand
    if len(sys.argv) > 1 and sys.argv[1] not in ["evaluate", "summary", "-h", "--help"]:
        args = parser.parse_args(["evaluate"] + sys.argv[1:])
    else:
        args = parser.parse_args()
        
    import tempfile
    import os
    
    if args.command == "summary":
        try:
            summary = summarize_runs(args.inputs)
            out_path = args.output
            out_path.parent.mkdir(parents=True, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(dir=out_path.parent, prefix="summary_", suffix=".tmp")
            try:
                with os.fdopen(fd, "w") as f:
                    json.dump(summary, f, indent=2)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp_path, out_path)
            except Exception as e:
                os.remove(tmp_path)
                raise
            return 0 if summary["passed"] else 1
        except Exception as e:
            print(f"Error summarizing runs: {e}", file=sys.stderr)
            return 2
    else:
        try:
            evidence = evaluate_artifact(args.artifact_dir)
        except (FileNotFoundError, ValueError) as e:
            print(f"Error: {e}", file=sys.stderr)
            return 2
    
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
