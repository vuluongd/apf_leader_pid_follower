# D3 Phase 1 — Implementation Walkthrough

## What was built

ROS2 package [d3_dual_mode_mpc](file:///home/lunog/apf_leader_pid_follower/src/d3_dual_mode_mpc/resource/d3_dual_mode_mpc) in `/home/lunog/apf_leader_pid_follower/src/d3_dual_mode_mpc/`

### Package structure
```
d3_dual_mode_mpc/
├── d3_dual_mode_mpc/
│   ├── __init__.py
│   ├── apf_leader_node.py          # APF planner (turtle1)
│   ├── signal_simulator_node.py    # RhOct FoV/range emulator
│   ├── mpc_follower_node.py        # Dual-mode MPC (hover/mpc_a/mpc_b/mpc_visual)
│   ├── velocity_relay_node.py      # RF velocity broadcast (Case B)
│   ├── data_logger_node.py         # CSV metrics logger
│   └── mpc/
│       ├── quadrotor_model.py      # 2D double-integrator A_d, B_d, DARE
│       ├── mpc_solver.py           # L-BFGS-B MPC with warm-start
│       └── leader_estimator.py     # Dead reckoning + Q(τ) decay
├── config/default_params.yaml
├── launch/turtlesim_prototype.launch.py
├── package.xml, setup.py, setup.cfg
```

## Verification

| Test | Result |
|------|--------|
| `colcon build` | ✅ Built in 1.02s |
| Import all modules | ✅ No errors |
| A_d (4×4), B_d (4×2) correct shapes | ✅ |
| DARE terminal cost P symmetric | ✅ |
| MPC solver produces output | ✅ u = [5, 5] (max acceleration toward target) |
| LeaderEstimator visual + prediction mode | ✅ Q decay and ref position correct |

## How to run

```bash
# Build
cd ~/apf_leader_pid_follower
source /opt/ros/humble/setup.bash
colcon build --packages-select d3_dual_mode_mpc
source install/setup.bash

# Run with MPC-B (default)
ros2 launch d3_dual_mode_mpc turtlesim_prototype.launch.py

# Run with hover baseline
ros2 launch d3_dual_mode_mpc turtlesim_prototype.launch.py controller_mode:=hover

# Run with different delay
ros2 launch d3_dual_mode_mpc turtlesim_prototype.launch.py delay_h:=1.0

# Run with different APF aggressiveness
ros2 launch d3_dual_mode_mpc turtlesim_prototype.launch.py k_rep:=4.0
```

Logs are saved to `/tmp/d3_logs/run_YYYYMMDD_HHMMSS.csv`.
