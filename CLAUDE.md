# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

This is the team "つかむ会" repository for the CIT Future Robotics "ロボット設計製作論実習3 (2026, 知能コース)" course. The goal is a robot system using a **CRANE-X7** 7-axis manipulator and an **Intel RealSense** depth camera. The task itself is still undecided (README says "〇〇をするロボット").

At the moment the repo contains only `README.md`. The package layout below is the *planned* structure from the README; it has not been created yet. When adding code, follow this layout rather than inventing a new one.

## Target environment

- Ubuntu 24.04, ROS 2 Jazzy
- Lives in `~/ros2_ws/src/` alongside `rt-net/crane_x7_ros` and `rt-net/crane_x7_description` (cloned with `-b $ROS_DISTRO`)

## Planned package structure

An `ament_python` ROS 2 package named `tukamukai`:

- `package.xml`, `setup.py` — package metadata; new nodes must be registered as `console_scripts` entry points in `setup.py`, and `launch/`/`config/` must be installed via `data_files`
- `tukamukai/` — node implementations (perception from RealSense, motion planning for CRANE-X7)
- `launch/` — launch files; the README expects `main.launch.py` as the entry point
- `config/` — camera and parameter settings
- `docs/` — presentation slides and meeting notes

The package runs on top of the rt-net CRANE-X7 stack: the arm is brought up separately via `crane_x7_examples` (MoveIt-based), and this package's nodes command it.

## Commands

```bash
# Build (from workspace root)
cd ~/ros2_ws
rosdep install -r -y -i --from-paths src
colcon build --symlink-install --packages-select tukamukai
source install/setup.bash

# Bring up the real robot, then this package in another terminal
ros2 launch crane_x7_examples demo.launch.py port_name:=/dev/ttyUSB0
ros2 launch tukamukai main.launch.py

# Tests (once a test/ dir exists)
colcon test --packages-select tukamukai && colcon test-result --verbose
```

## Team conventions

- Never push directly to `main`; work on `feature/<topic>` branches and merge via Pull Request.
- Progress is tracked in GitHub Issues/Projects; milestones map to the three presentations (プラン発表 / 中間発表 / 最終発表).
- If code from senior students' repos is reused, record the source repo and license in the README's license section.
- README and docs are written in Japanese.
