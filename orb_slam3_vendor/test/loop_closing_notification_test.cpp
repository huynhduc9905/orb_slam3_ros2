#include <gtest/gtest.h>
#define private public
#include <System.h>
#include <Atlas.h>
#include <Map.h>
#include <KeyFrame.h>
#undef private
#include <SystemSnapshots.h>

#include <cstdlib>
#include <vector>
#include <algorithm>

namespace {

bool ContainsLoopEdge(const ORB_SLAM3::GraphSnapshot& snapshot, std::uint64_t from_kf_id, std::uint64_t to_kf_id) {
    auto it = std::find_if(snapshot.keyframes.begin(), snapshot.keyframes.end(),
                           [from_kf_id](const ORB_SLAM3::KeyframeSnapshot& kf) { return kf.id == from_kf_id; });
    if (it == snapshot.keyframes.end()) return false;
    return std::find(it->loop_edge_ids.begin(), it->loop_edge_ids.end(), to_kf_id) != it->loop_edge_ids.end();
}

} // namespace

TEST(LoopClosingNotification, NotificationPrecedesLoopEdgeAdditionInCorrectLoop) {
    const char* vocabulary = std::getenv("ORB_TEST_VOCAB");
    const char* settings = std::getenv("ORB_TEST_SETTINGS");
    ASSERT_NE(vocabulary, nullptr);
    ASSERT_NE(settings, nullptr);

    ORB_SLAM3::System system(vocabulary, settings, ORB_SLAM3::System::STEREO, false);

    // Initial snapshot call to consume initial map state
    (void)system.GetGraphSnapshot();
    (void)system.MapChanged();

    // Access active map via private system member mpAtlas
    ORB_SLAM3::Map* active_map = system.mpAtlas->GetCurrentMap();
    ASSERT_NE(active_map, nullptr);

    ORB_SLAM3::KeyFrame current_kf;
    current_kf.mnId = 1;
    current_kf.UpdateMap(active_map);

    ORB_SLAM3::KeyFrame matched_kf;
    matched_kf.mnId = 2;
    matched_kf.UpdateMap(active_map);

    active_map->AddKeyFrame(&current_kf);
    active_map->AddKeyFrame(&matched_kf);

    // Reproduce fixed ordering in CorrectLoop:
    // 1. Reciprocal loop edges added first
    matched_kf.AddLoopEdge(&current_kf);
    current_kf.AddLoopEdge(&matched_kf);

    // 2. mpAtlas->InformNewBigChange() notifies observers after edges are added
    system.mpAtlas->InformNewBigChange();

    // 3. Observer receives MapChanged() notification and fetches snapshot via system.GetGraphSnapshot()
    ASSERT_TRUE(system.MapChanged());
    const ORB_SLAM3::GraphSnapshot snapshot = system.GetGraphSnapshot();

    // Assert that the snapshot obtained upon MapChanged contains both reciprocal edges.
    // Under current ordering, this fails because the snapshot was captured before edges were added.
    ASSERT_TRUE(ContainsLoopEdge(snapshot, 1, 2));
    ASSERT_TRUE(ContainsLoopEdge(snapshot, 2, 1));
}
