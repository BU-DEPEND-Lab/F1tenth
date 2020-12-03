# Parameterized DDPG (In Development)

- Original DDPG paper: https://arxiv.org/abs/1509.02971
- Baselines DDPG [post](https://blog.openai.com/better-exploration-with-parameter-noise/)
- `python main_controller.py --render` run the algorithm

## Required python library
- openai-gym [here](https://github.com/openai/gym)
- baselines [here](https://github.com/openai/baselines)

## Trajectory generation
The trajectory is generated from *bspline_path.py*, which is a piece-wise linear trajectory defined by the waypoints.

## Environment
A gym-like environment of F1tenth defined in [rear\_wheel\_control_gazebo.py](https://github.com/JmfanBU/F1tenth_BU/blob/master/racecar_simulator/racecar_control/scripts/rear_wheel_control_gazebo.py).

The gazebo simulation will slow down the training process.

## LQR Steering Control Simulation
The planned path is provided by cubic path planner. The waypoint can be set in the main function.
- `python lqr_steer_control.py`

The system parameters can be changed inside *lqr_steer_control.py* for tests with variance.
