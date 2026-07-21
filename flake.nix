{
  description = "orb_slam3_ros2 — standalone ROS 2 Kilted devshell";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        ros = pkgs.rosPackages.kilted;

        pyDeps = ps: with ps; [
          argcomplete
          numpy
          pyyaml
          pillow
          pytest
        ];
      in {
        devShells.default = pkgs.mkShell {
          name = "orb-slam3-ros2-kilted";

          packages = [
            pkgs.colcon
            pkgs.cmake
            pkgs.pkg-config
            pkgs.eigen
            pkgs.boost
            pkgs.opencv
            pkgs.openssl
            pkgs.nlohmann_json
            pkgs.nodejs_22

            ros.ament-cmake-vendor-package

            (with ros; buildEnv {
              paths = [
                ros-core
                ros2cli
                ros2run
                ros2launch
                ros2topic
                ros2node
                ros2param
                ros2bag
                rosbag2-storage-mcap
                rosbag2-storage-sqlite3
                rclcpp
                rclpy
                std-msgs
                geometry-msgs
                sensor-msgs
                nav-msgs
                visualization-msgs
                diagnostic-msgs
                builtin-interfaces
                tf2
                tf2-ros
                tf2-msgs
                tf2-geometry-msgs
                cv-bridge
                message-filters
                pcl-conversions
                pcl-ros
                rosidl-default-generators
                rosidl-default-runtime
              ];
            })
          ] ++ (pyDeps pkgs.python3Packages);

          shellHook = ''
            unset PROMPT_COMMAND

            if [ -f install/setup.bash ]; then
              source install/setup.bash
            fi

            export ROS_DOMAIN_ID=''${ROS_DOMAIN_ID:-0}
            export RMW_IMPLEMENTATION=''${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

            if command -v register-python-argcomplete >/dev/null 2>&1; then
              eval "$(register-python-argcomplete --shell bash ros2 2>/dev/null)" || true
              eval "$(register-python-argcomplete --shell bash colcon 2>/dev/null)" || true
            fi

            echo "orb_slam3_ros2 standalone devshell active (ROS 2 Kilted)"
          '';
        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };
}
