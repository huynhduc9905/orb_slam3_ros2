# Standalone Nix DevShell Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a self-contained `flake.nix` to `orb_slam3_ros2/` so the repo can be cloned onto any NixOS workstation and built with `nix develop` + `colcon build` — independent of `~/robot/flake.nix`.

**Architecture:** A single `flake.nix` at repo root pulls `nix-ros-overlay` directly, exposes one `devShells.default` containing only the deps required by the 6 packages in this repo, and configures `ros.cachix.org` for fast binary downloads. No other files are modified.

**Tech Stack:** Nix flakes, nix-ros-overlay, ROS 2 Kilted, colcon

## Global Constraints

- ROS 2 distribution: `kilted` (via `rosPackages.kilted`)
- `nix-ros-overlay` input: `github:lopsided98/nix-ros-overlay/master`
- Node.js: must be `nodejs_22` (dashboard CMake enforces major version 22)
- Binary cache: `https://ros.cachix.org` with key `ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=`
- No dependency on `~/robot/flake.nix` or any path outside `orb_slam3_ros2/`
- Colcon build command (from README): `colcon build --packages-select orb_slam3_vendor orb_slam3_msgs orb_slam3_wrapper orb_lidar_mapper orb_slam_dashboard orb_slam_bringup --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release`

---

### Task 1: Write `flake.nix`

**Files:**
- Create: `flake.nix` (repo root: `/home/duc/robot/src/orb_slam3_ros2/flake.nix`)

**Interfaces:**
- Produces: `devShells.default` — entered via `nix develop`

- [ ] **Step 1: Create `flake.nix`**

  Create `/home/duc/robot/src/orb_slam3_ros2/flake.nix` with this exact content:

  ```nix
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
  ```

- [ ] **Step 2: Verify the file was written**

  Run:
  ```bash
  head -5 /home/duc/robot/src/orb_slam3_ros2/flake.nix
  ```
  Expected: first line is `{`, second line contains `description = "orb_slam3_ros2`.

---

### Task 2: Generate `flake.lock`

**Files:**
- Create: `flake.lock` (repo root, generated by nix)

**Interfaces:**
- Consumes: `flake.nix` from Task 1
- Produces: pinned `flake.lock` for reproducible builds on workstation

- [ ] **Step 1: Run `nix flake update` to generate the lock file**

  Run from `/home/duc/robot/src/orb_slam3_ros2/`:
  ```bash
  nix flake update
  ```
  Expected: output like `• Updated input 'nix-ros-overlay': ...` and a `flake.lock` file appears.

  Note: This requires network access and may take a minute. It does NOT download the full closure — just resolves and pins the input revisions.

- [ ] **Step 2: Verify `flake.lock` was created**

  Run:
  ```bash
  ls -lh /home/duc/robot/src/orb_slam3_ros2/flake.lock
  ```
  Expected: file exists, non-zero size.

- [ ] **Step 3: Smoke-check flake evaluation (no build)**

  Run:
  ```bash
  nix flake show /home/duc/robot/src/orb_slam3_ros2
  ```
  Expected: output contains `devShells` → `x86_64-linux` → `default`.  
  If it errors with "unfree packages" or similar, that is a nixpkgs config issue on this machine — it does not affect portability to the workstation.

---

### Task 3: Commit and push

**Files:**
- Modify: git index only — no source files changed

**Interfaces:**
- Consumes: `flake.nix`, `flake.lock` from Tasks 1–2

- [ ] **Step 1: Check for uncommitted changes in the working tree**

  Run:
  ```bash
  git -C /home/duc/robot/src/orb_slam3_ros2 status --short
  ```
  Note any modified files beyond `flake.nix` and `flake.lock`.

- [ ] **Step 2: Stage any WIP changes as a temp commit (if present)**

  If there are uncommitted changes to other files, stash or commit them first:
  ```bash
  git -C /home/duc/robot/src/orb_slam3_ros2 add -A
  git -C /home/duc/robot/src/orb_slam3_ros2 commit -m "wip: temp commit before standalone flake"
  ```
  If working tree is clean (only `flake.nix`/`flake.lock` are new), skip this step.

- [ ] **Step 3: Stage and commit the flake files**

  ```bash
  git -C /home/duc/robot/src/orb_slam3_ros2 add flake.nix flake.lock
  git -C /home/duc/robot/src/orb_slam3_ros2 commit -m "chore: add standalone nix devshell"
  ```
  Expected: commit succeeds, shows `flake.nix` and `flake.lock` in the diff summary.

- [ ] **Step 4: Push to remote**

  ```bash
  git -C /home/duc/robot/src/orb_slam3_ros2 push
  ```
  Expected: `To git@github.com:huynhduc9905/orb_slam3_ros2.git` ... `feature/orb-lidar-thin-walls-p0-p1 -> feature/orb-lidar-thin-walls-p0-p1` (or current branch name).

---

### Task 4: Workstation setup instructions

This task produces no files in the repo — it is the verification checklist for the workstation side.

- [ ] **Step 1: On workstation — clone the repo**

  ```bash
  git clone git@github.com:huynhduc9905/orb_slam3_ros2.git ~/orb_slam3_ros2
  cd ~/orb_slam3_ros2
  git checkout feature/orb-lidar-thin-walls-p0-p1   # or whichever branch was pushed
  ```

- [ ] **Step 2: Initialize the ORB-SLAM3 submodule**

  ```bash
  git submodule update --init --recursive
  ```
  Expected: checks out `orb_slam3_vendor/vendor/ORB_SLAM3`.

- [ ] **Step 3: Enter the devshell**

  ```bash
  nix develop
  ```
  Expected: downloads ROS 2 Kilted binaries from `ros.cachix.org` (fast), then prints:
  ```
  orb_slam3_ros2 standalone devshell active (ROS 2 Kilted)
  ```
  If the workstation hasn't trusted the cachix key yet, nix may prompt to allow the substituter — answer yes, or add to `/etc/nix/nix.conf`:
  ```
  trusted-substituters = https://ros.cachix.org
  trusted-public-keys = ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=
  ```

- [ ] **Step 4: Build all packages**

  ```bash
  COLCON_DEFAULTS_FILE=/dev/null colcon build \
    --packages-select orb_slam3_vendor orb_slam3_msgs orb_slam3_wrapper \
                      orb_lidar_mapper orb_slam_dashboard orb_slam_bringup \
    --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)
  ```
  Expected: all 6 packages report `[100%]` and `Finished <<< <pkg>`.

- [ ] **Step 5: Verify ROS runtime works**

  ```bash
  source install/setup.bash
  ros2 topic list
  ```
  Expected: at minimum `/parameter_events` and `/rosout` are listed (ROS 2 daemon running).

---

## Self-Review Notes

- `nodejs_22` included — required by `orb_slam_dashboard/CMakeLists.txt` which calls `find_program(NODE_EXECUTABLE node REQUIRED)` and enforces major version 22.
- `--parallel-workers 1` from the README was a workaround for the slow current machine; workstation build uses `$(nproc)`.
- WIP commit step handles the case where there are uncommitted changes on `feature/orb-lidar-thin-walls-p0-p1` before pushing.
- The `nix flake show` smoke-check in Task 2 Step 3 intentionally does not attempt a full `nix develop` on the current machine (large closure, slow). Workstation is the actual validation target.
