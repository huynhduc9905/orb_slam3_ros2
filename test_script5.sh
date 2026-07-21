echo "# Search for GlobalBundleAdjustment related loop issues"
grep -n -C 5 "RunGlobalBundleAdjustment" ./orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc
echo "\n# Search for CorrectLoop"
grep -n -A 10 "CorrectLoop" ./orb_slam3_vendor/vendor/ORB_SLAM3/src/LoopClosing.cc
