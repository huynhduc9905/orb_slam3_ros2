import math
import sys
from array import array
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "orb_slam_bringup"))
import orb_slam_bringup
orb_slam_bringup.__path__.append(str(Path(__file__).resolve().parents[1] / "orb_slam_bringup"))

import pytest

from orb_slam_bringup.dashboard_model import (
    MAX_CORRECTED_POINTS,
    MAX_PROVISIONAL_POINTS,
    MAX_UNMATCHED,
    MAX_WHEEL_POINTS,
    DashboardModel,
    MapEnvelope,
    Pose2D,
    RevisionEnvelope,
    bounded_points,
)


def mp(stamp=(1, 2), cells=(0, 100)):
    return MapEnvelope(stamp, 0.1, -1.0, 2.0, 2, 1, cells)


def rv(stamp=(1, 2), state="PUBLISHED", graph=4, revision=1):
    return RevisionEnvelope(stamp, state, graph, revision, 8)


def assert_pair(pair, expected_map, expected_revision):
    actual_map, actual_revision = pair
    assert actual_map.stamp == expected_map.stamp
    assert actual_map.resolution == expected_map.resolution
    assert actual_map.origin_x == expected_map.origin_x
    assert actual_map.origin_y == expected_map.origin_y
    assert actual_map.width == expected_map.width
    assert actual_map.height == expected_map.height
    assert list(actual_map.cells) == list(expected_map.cells)
    assert actual_revision == expected_revision


def test_pairing_works_in_either_arrival_order():
    for revision_first in (False, True):
        model = DashboardModel()
        if revision_first:
            model.ingest_revision(rv())
            model.ingest_map(mp())
        else:
            model.ingest_map(mp())
            model.ingest_revision(rv())
        assert_pair(model.next_map_to_render(), mp(), rv())


def test_unmatched_inputs_are_bounded_and_failed_is_retained():
    model = DashboardModel()
    for i in range(MAX_UNMATCHED + 5):
        model.ingest_map(mp((i, 0)))
        model.ingest_revision(rv((i, 0), "FAILED", revision=i + 1))
    state = model.snapshot(0.0)
    assert state["map_revision"]["state"] == "FAILED"
    assert len(model._maps) <= MAX_UNMATCHED
    assert len(model._revisions) <= MAX_UNMATCHED


def test_only_published_newer_revision_is_render_candidate_and_newest_wins():
    model = DashboardModel()
    model.ingest_map(mp((1, 0)))
    model.ingest_revision(rv((1, 0), "BUILDING", revision=9))
    assert model.next_map_to_render() is None
    model.ingest_map(mp((2, 0)))
    model.ingest_revision(rv((2, 0), revision=2))
    model.ingest_map(mp((3, 0)))
    model.ingest_revision(rv((3, 0), revision=3))
    assert model.next_map_to_render()[1].map_revision == 3


def test_render_publication_is_atomic_and_revision_is_monotonic():
    model = DashboardModel()
    model.ingest_map(mp())
    model.ingest_revision(rv())
    assert model.map_png() == b""
    assert model.publish_rendered_map((99, 0), b"bad") is False
    assert model.next_map_to_render() is not None
    assert model.publish_rendered_map((1, 2), b"one") is True
    assert model.map_png() == b"one"
    assert model.snapshot(0)["map"]["revision"] == 1
    model.ingest_map(mp((2, 2)))
    model.ingest_revision(rv((2, 2), revision=1))
    assert model.next_map_to_render() is None


def test_render_candidate_cells_do_not_alias_internal_cache():
    model = DashboardModel()
    value = mp((4, 5), cells=(-1, 100))
    revision = rv((4, 5), graph=6, revision=7)
    model.ingest_map(value)
    model.ingest_revision(revision)

    exposed_map, exposed_revision = model.next_map_to_render()
    assert list(exposed_map.cells) == [-1, 100]
    exposed_map.cells.release()

    second_map, second_revision = model.next_map_to_render()
    assert second_map is not exposed_map
    assert list(second_map.cells) == [-1, 100]
    assert exposed_revision == second_revision == revision
    assert model.publish_rendered_map((4, 5), b"safe") is True
    assert model.map_png() == b"safe"
    metadata = model.snapshot(0)["map"]
    assert metadata["revision"] == 7
    assert metadata["graph_revision"] == 6
    assert metadata["width"] == 2


def test_ingested_compact_cells_do_not_alias_mutable_source():
    source = array("b", [-1, 100])
    model = DashboardModel()
    model.ingest_map(mp((8, 9), cells=source))
    source[0], source[1] = 42, 43
    model.ingest_revision(rv((8, 9), graph=3, revision=4))
    candidate, _ = model.next_map_to_render()
    assert list(candidate.cells) == [-1, 100]


def test_corrected_path_replacement_and_endpoint_bounds():
    model = DashboardModel()
    points = [Pose2D(float(i), float(i), 0.0) for i in range(MAX_CORRECTED_POINTS + 7)]
    model.replace_corrected_path(12, points)
    snap = model.snapshot(0)
    assert len(snap["paths"]["corrected"]) <= MAX_CORRECTED_POINTS
    assert snap["paths"]["corrected"][0] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert snap["paths"]["corrected"][-1]["x"] == float(len(points) - 1)
    model.replace_corrected_path(13, [Pose2D(9.0, 8.0, 0.5)])
    assert model.snapshot(0)["paths"]["corrected"] == [{"x": 9.0, "y": 8.0, "yaw": 0.5}]


def test_bounded_points_is_deterministic_and_preserves_endpoints():
    points = list(range(23))
    assert bounded_points(points, 5) == [0, 6, 12, 18, 22]
    assert len(bounded_points(points, 5)) <= 5
    assert bounded_points(points, 1) == [22]
    assert bounded_points(points, 0) == points


def test_nonfinite_values_are_rejected_before_storage():
    model = DashboardModel()
    with pytest.raises(ValueError):
        model.update_tracked("OK", Pose2D(math.nan, 0, 0), 1, 1.0)
    with pytest.raises(ValueError):
        model.replace_provisional_scan([(0.0, math.inf)])
    with pytest.raises(ValueError):
        model.update_wheel(Pose2D(0, 0, 0), math.nan)


def test_health_flags_mark_stale_inputs():
    model = DashboardModel()
    model.update_tracked("OK", Pose2D(1, 2, 0), 20, 1.0)
    model.update_wheel(Pose2D(1, 2, 0), 1.0)
    health = model.snapshot(10.0)["health"]
    assert health["tracking_stale"] is True
    assert health["wheel_stale"] is True


def test_duplicate_loss_and_relocalized_events_are_idempotent():
    model = DashboardModel()
    model.update_tracked("OK", Pose2D(3, 4, 0), 10, 1.0)
    model.update_wheel(Pose2D(0, 0, 0), 1.0)
    model.start_loss(7, 2.0)
    first = model.snapshot(2.0)
    model.start_loss(8, 3.0)
    assert model.snapshot(3.0)["fallback"] == first["fallback"]
    model.mark_relocalized(7, 4.0)
    trail = model.snapshot(4.0)["paths"]["wheel"]
    model.mark_relocalized(8, 5.0)
    assert model.snapshot(5.0)["paths"]["wheel"] == trail


def test_wheel_only_loss_trail_uses_anchor_transform_and_recovery_gate():
    model = DashboardModel()
    model.update_tracked("OK", Pose2D(10, 20, math.pi / 2), 10, 1.0)
    model.update_wheel(Pose2D(2, 3, 0), 1.0)
    model.start_loss(4, 2.0)
    model.update_wheel(Pose2D(3, 3, 0), 3.0)
    assert model.snapshot(3)["fallback"]["pose"]["x"] == pytest.approx(10.0)
    assert model.snapshot(3)["fallback"]["pose"]["y"] == pytest.approx(21.0)
    assert model.snapshot(3)["fallback"]["trail"][-1]["x"] == pytest.approx(10.0)
    assert model.snapshot(3)["fallback"]["trail"][-1]["y"] == pytest.approx(21.0)
    model.mark_relocalized(4, 4.0)
    assert model.snapshot(4)["state"] == "recovery_pending"
    model.update_tracked("OK", Pose2D(11, 21, 0), 20, 4.1)
    assert model.snapshot(4.1)["state"] == "recovery_pending"
    assert model.snapshot(4.1)["fallback"]["active"] is True
    model.ingest_map(mp((9, 0)))
    model.ingest_revision(rv((9, 0), graph=3, revision=5))
    assert model.snapshot(4)["fallback"]["active"] is True
    model.ingest_map(mp((10, 0)))
    model.ingest_revision(rv((10, 0), graph=4, revision=6))
    assert model.snapshot(5)["fallback"]["active"] is False
    model.update_wheel(Pose2D(4, 3, 0), 5.1)
    assert model.snapshot(5.1)["fallback"]["active"] is False


def test_visible_metadata_survives_cache_eviction_and_same_stamp_replacement():
    model = DashboardModel()
    visible = mp((1, 0))
    revision = rv((1, 0), revision=1)
    model.ingest_map(visible)
    model.ingest_revision(revision)
    candidate = model.next_map_to_render()
    assert_pair(candidate, visible, revision)
    assert model.publish_rendered_map((1, 0), b"visible")
    assert model._visible_map is not candidate[0]
    assert model._visible_revision_envelope is not candidate[1]
    for i in range(2, MAX_UNMATCHED + 4):
        model.ingest_map(mp((i, 0)))
    model.ingest_map(MapEnvelope((1, 0), 9.0, 9.0, 9.0, 1, 1, (42,)))
    model.ingest_revision(rv((1, 0), state="FAILED", graph=99, revision=1))
    snap = model.snapshot(0)
    assert snap["map"]["resolution"] == 0.1
    assert snap["map"]["origin_x"] == -1.0
    assert snap["map"]["width"] == 2
    assert snap["map"]["state"] == "PUBLISHED"
    assert snap["map"]["graph_revision"] == 4
    assert snap["map"]["committed_scan_count"] == 8
    assert model.map_png() == b"visible"


@pytest.mark.parametrize("replacement_state", ["FAILED", "BUILDING", "IDLE"])
def test_same_stamp_revision_replacement_invalidates_pending(replacement_state):
    model = DashboardModel()
    model.ingest_map(mp())
    model.ingest_revision(rv(revision=3))
    assert model.next_map_to_render() is not None
    model.ingest_revision(rv(state=replacement_state, revision=3))
    assert model.publish_rendered_map((1, 2), b"stale") is False
    assert model.map_png() == b""


def test_same_stamp_map_replacement_invalidates_pending():
    model = DashboardModel()
    model.ingest_map(mp())
    model.ingest_revision(rv(revision=3))
    assert model.next_map_to_render() is not None
    model.ingest_map(MapEnvelope((1, 2), 0.2, 0.0, 0.0, 1, 1, (0,)))
    assert model.publish_rendered_map((1, 2), b"stale") is False


def test_latest_revision_never_regresses_and_same_revision_is_deterministic():
    model = DashboardModel()
    model.ingest_revision(rv((4, 0), "BUILDING", graph=8, revision=7))
    model.ingest_revision(rv((3, 0), "FAILED", graph=99, revision=6))
    assert model.snapshot(0)["map_revision"]["map_revision"] == 7
    model.ingest_revision(rv((5, 0), "PUBLISHED", graph=8, revision=7))
    model.ingest_revision(rv((6, 0), "IDLE", graph=8, revision=7))
    assert model.snapshot(0)["map_revision"]["state"] == "PUBLISHED"
    model.ingest_revision(rv((7, 0), "FAILED", graph=8, revision=7))
    assert model.snapshot(0)["map_revision"]["state"] == "FAILED"
    reverse = DashboardModel()
    reverse.ingest_revision(rv((7, 0), "FAILED", graph=8, revision=7))
    reverse.ingest_revision(rv((5, 0), "PUBLISHED", graph=8, revision=7))
    reverse.ingest_revision(rv((6, 0), "IDLE", graph=8, revision=7))
    assert reverse.snapshot(0)["map_revision"] == model.snapshot(0)["map_revision"]


def test_same_revision_state_priority_dominates_graph_revision():
    for failed_first in (False, True):
        model = DashboardModel()
        failed = rv((20, 0), "FAILED", graph=4, revision=9)
        published = rv((21, 0), "PUBLISHED", graph=99, revision=9)
        for value in ((failed, published) if failed_first else (published, failed)):
            model.ingest_revision(value)
        latest = model.snapshot(0)["map_revision"]
        assert latest["state"] == "FAILED"
        assert latest["graph_revision"] == 4


@pytest.mark.parametrize("pair_first", [True, False])
def test_recovery_handles_both_pair_and_relocalized_arrival_orders(pair_first):
    model = DashboardModel()
    model.update_tracked("OK", Pose2D(1, 2, 0), 1, 0)
    model.update_wheel(Pose2D(0, 0, 0), 0)
    model.start_loss(10, 1)

    def ingest_pair():
        model.ingest_map(mp((30, 0)))
        model.ingest_revision(rv((30, 0), graph=10, revision=10))

    if pair_first:
        ingest_pair()
        assert model.snapshot(1)["fallback"]["active"] is True
        model.mark_relocalized(10, 2)
    else:
        model.mark_relocalized(10, 2)
        assert model.snapshot(2)["state"] == "recovery_pending"
        ingest_pair()

    recovered = model.snapshot(3)
    assert recovered["state"] == "OK"
    assert recovered["fallback"]["active"] is False
    model.mark_relocalized(10, 4)
    model.update_wheel(Pose2D(5, 0, 0), 4)
    assert model.snapshot(4)["fallback"]["active"] is False


def test_recovery_requires_equal_stamp_coherent_pair_and_not_rendering():
    model = DashboardModel()
    model.update_tracked("OK", Pose2D(0, 0, 0), 1, 0)
    model.update_wheel(Pose2D(0, 0, 0), 0)
    model.start_loss(10, 1)
    model.mark_relocalized(10, 2)
    model.ingest_map(mp((10, 0)))
    model.ingest_revision(rv((11, 0), graph=10, revision=2))
    assert model.snapshot(2)["fallback"]["active"] is True
    model.ingest_revision(rv((10, 0), graph=10, revision=3))
    assert model.snapshot(2)["fallback"]["active"] is False
    assert model.map_png() == b""


def test_loss_and_relocalization_guards_reset_for_a_second_cycle():
    model = DashboardModel()
    model.update_tracked("OK", Pose2D(1, 1, 0), 1, 0)
    model.update_wheel(Pose2D(0, 0, 0), 0)
    model.start_loss(1, 1)
    model.mark_relocalized(1, 2)
    model.ingest_map(mp((1, 0)))
    model.ingest_revision(rv((1, 0), graph=1, revision=1))
    model.update_tracked("OK", Pose2D(5, 6, 0), 1, 3)
    model.update_wheel(Pose2D(10, 10, 0), 3)
    model.start_loss(2, 4)
    assert model.snapshot(4)["fallback"]["pose"] == {"x": 5.0, "y": 6.0, "yaw": 0.0}
    model.mark_relocalized(2, 5)
    assert model.snapshot(5)["state"] == "recovery_pending"


@pytest.mark.parametrize(
    "bad_map",
    [
        MapEnvelope((1, 2), math.nan, 0.0, 0.0, 1, 1, (0,)),
        MapEnvelope((1, 2), 0.1, math.inf, 0.0, 1, 1, (0,)),
        MapEnvelope((1, 2), 0.1, 0.0, 0.0, 0, 1, ()),
        MapEnvelope((1, 2), 0.1, 0.0, 0.0, 2, 1, (0,)),
        MapEnvelope((1, 2), 0.1, 0.0, 0.0, 1, 1, (101,)),
        MapEnvelope((1, 2), 0.1, 0.0, 0.0, 1, 1, (0.5,)),
    ],
)
def test_invalid_map_envelopes_are_rejected_without_storage(bad_map):
    model = DashboardModel()
    with pytest.raises(ValueError):
        model.ingest_map(bad_map)
    assert len(model._maps) == 0


def test_provisional_points_are_bounded():
    model = DashboardModel()
    model.replace_provisional_scan([(float(i), 0.0) for i in range(MAX_PROVISIONAL_POINTS + 1)])
    assert len(model.snapshot(0)["paths"]["provisional"]) <= MAX_PROVISIONAL_POINTS


def test_transformed_fallback_trail_is_bounded():
    model = DashboardModel()
    model.update_tracked("OK", Pose2D(0, 0, 0), 1, 0)
    model.update_wheel(Pose2D(0, 0, 0), 0)
    model.start_loss(1, 1)
    for i in range(MAX_WHEEL_POINTS + 7):
        model.update_wheel(Pose2D(float(i), 0, 0), float(i + 2))
    trail = model.snapshot(float(MAX_WHEEL_POINTS + 9))["fallback"]["trail"]
    assert len(trail) <= MAX_WHEEL_POINTS
    assert trail[0] == {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert trail[-1]["x"] == float(MAX_WHEEL_POINTS + 6)
