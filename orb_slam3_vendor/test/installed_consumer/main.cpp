#include <System.h>
#include <DBoW2/BowVector.h>
#include <g2o/core/base_vertex.h>

int main()
{
  return static_cast<int>(ORB_SLAM3::System::STEREO) == 1 ? 0 : 1;
}
