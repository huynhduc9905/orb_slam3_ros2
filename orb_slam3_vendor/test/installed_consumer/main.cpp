#include <System.h>
#include <SystemSnapshots.h>
#include <DBoW2/BowVector.h>
#include <g2o/core/base_vertex.h>

int main()
{
  ORB_SLAM3::GraphSnapshot snapshot;
  return static_cast<int>(ORB_SLAM3::System::STEREO) == 1 && snapshot.keyframes.empty() ? 0 : 1;
}
