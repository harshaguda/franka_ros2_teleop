# Force-Sensitive Teleoperation of Franka Robots in ROS 2

**franka_ros2_teleop** is a ROS 2 package that demonstrates the teleoperation of a **Franka FR3** robot with another **Franka FR3** robot as input device.
Due to the robot's force-torque sensors we can send contact forces measured by the follower to the leader robot.
The leader robot will then replay the forces so the teleoperator can feel the contacts as well.

## Bilateral control architecture (4-channel)

The teleoperation controllers implement a **4-channel bilateral control** scheme: position and
force are each transmitted explicitly, in both directions, and each direction has its own
independently tunable gain. This decouples *tracking* from *feel*, so one gain no longer has to
compromise between the two:

| # | Channel | Direction | Signal | Gain(s) | Tunes |
|---|---|---|---|---|---|
| 1 | Position coupling | leader → follower | leader's measured joint position/velocity | `k_gains` / `d_gains` (follower) | Tracking accuracy: how tightly the follower's motion follows the leader |
| 2 | Force reflection | follower → leader | follower's sensed external (contact) torque | `force_reflection_gains` (leader) | Feel: how strongly the operator perceives the follower's contact forces |
| 3 | Force feedforward | leader → follower | leader's own sensed external (operator-applied) torque | `force_feedforward_gains` (follower) | Feel: how directly the operator's applied force acts on the follower, without needing to raise the position gains |

Channel 1 alone would make `k_gains`/`d_gains` the single knob governing both how well the
follower tracks the leader *and*, indirectly, how "stiff" or "soft" the follower feels when it
contacts the environment. Adding channels 2 and 3 as explicit, separately-gained torque terms means
`k_gains`/`d_gains` can be tuned purely for tracking, while `force_reflection_gains` and
`force_feedforward_gains` can be tuned purely for feel.

Both force channels reuse the same `franka_robot_state_broadcaster/external_joint_torques` topic
that each robot (leader and follower) already publishes: the leader controller subscribes to the
follower's external torques (channel 2), and the follower controller subscribes to the leader's own
external torques (channel 3). Setting a `force_*_gains` parameter to all zeros disables that
channel.

To run the teleoperation, you need to have at least two **Franka FR3** robots with the **FCI (Franka Control Interface)** feature.
One will be the leader and the other will be the follower.
The leader can be handguided and the follower will match the leaders position.
The forces encountered by the follower can be felt by the person hand-guiding the leader.

> [!CAUTION] 
> Avoid hand-guiding the follower, as it might lead to abrupt movements of the leader!

### Verifying 1 kHz timing and channel latency

`controller_manager.update_rate` is configured to 1000 Hz ([config/teleop_controllers.yaml](config/teleop_controllers.yaml)),
but the configured rate is only a request: real-time scheduling issues, a non-PREEMPT_RT kernel,
or network jitter between the leader and follower hosts can silently degrade it. Both controllers
publish lightweight timing diagnostics (via `realtime_tools::RealtimePublisher`, so they never
block the control loop) that let you check the *actual* rate and per-channel latency rather than
assuming the configured one:

| Topic (relative to the controller node) | Meaning |
|---|---|
| `~/diagnostics/loop_period_us` | The `update()` period actually measured by `controller_manager`, in microseconds. Published by both `leader_controller` and `follower_controller`. |
| `~/diagnostics/position_channel_latency_us` | Age of the leader's position message (channel 1) when consumed by the follower. Published by `follower_controller`. |
| `~/diagnostics/force_reflection_channel_latency_us` | Age of the follower's contact-torque message (channel 2) when consumed by the leader. Published by `leader_controller`. |
| `~/diagnostics/force_feedforward_channel_latency_us` | Age of the leader's own torque message (channel 3) when consumed by the follower. Published by `follower_controller`. |

The channel-latency numbers are `now - header.stamp`, computed on the *consuming* robot, so they
are only meaningful if the leader and follower hosts' clocks are synchronized (e.g. via `chrony` or
PTP) -- verify and report that separately before trusting them.

`scripts/verify_bilateral_timing.py` subscribes to these topics for a fixed duration, checks the
achieved loop rate and channel latencies against configurable thresholds, and writes a latency
histogram, a CSV of raw samples, and a JSON report:

```bash
ros2 run franka_ros2_teleop verify_bilateral_timing.py \
  --leader-namespace leader --follower-namespace follower \
  --duration 30 --output-dir bilateral_timing_report
```

It exits non-zero if the achieved rate falls outside `--rate-tolerance-pct` of `--target-rate-hz`
(default: 1000 Hz +/- 2%) or if p99 channel latency exceeds `--latency-warn-ms` (default: 2 ms).
Run `ros2 run franka_ros2_teleop verify_bilateral_timing.py --help` for all options.

## Getting started

To run any quickstart example, you must add the correct IP addresses of you robot to the configuration as described in the following subsections. All other parameters have sensible defaults. If you need to change them, they are datailed in the section [Configuration parameters](#configuration-parameters).

You can also find minimal config examples in the config folder: [config/fr3_teleop_config.yaml](config/fr3_teleop_config.yaml) for single arm setups and [config/fr3_duo_teleop_config.yaml](config/fr3_duo_teleop_config.yaml) for custom humanoid setups, **FR3 Duo** or **Mobile FR3 Duo**.

> [!IMPORTANT]
> Before starting, make sure you can reach your robots over the local network and they are in **FCI** Mode.

### Docker-based Quickstart

The Docker-based quickstart Methods get you started fast and you do not need to worry about depencies like ROS 2.
To run any of them, you must modify the configuration in the `x-teleop-config` section of the [docker-compose.yml](docker-compose.yml) file.

#### Using pre-built Docker image

Download the [docker-compose.yml](docker-compose.yml) file and open it in an editor. Add the IP addresses of your robots and run `docker compose up` in the directory containing the modified `docker-compose.yml`. This will download and run a pre-built image from GitHub and run it.

#### Using locally built Docker container

Clone the repo, then open the [docker-compose.yml](docker-compose.yml) file in an editor and add the IP addresses of your robots.
Then run `docker compose up --build` in the top-level directory of the repo. This will build a docker image locally and run it.

### ROS 2 Quickstart

If you are familiar with ROS 2 and want to get more into the code, we recommend using any of the following methods.

To add the IP addresses of your robots to the configuration, you will need to edit the file [config/fr3_teleop_config.yaml](config/fr3_teleop_config.yaml) after downloading the repo.
After building and sourcing your workspace (for more detailed instructions see below), you can run the teleoperation example using: `ros2 launch franka_ros2_teleop teleop.launch.py`.

Alternativley, you can also create a copy of either config file and supply that file to the launch command: `ros2 launch franka_ros2_teleop teleop.launch.py robot_config_file:=/path/to/your/copy/of/fr3_teleop_config.yaml`


#### Using a Dev Container in VS Code

Clone the repo, open the folder in VS Code and reopen the folder as container. 

Now modify the config files as described above.

In a terminal, build the ROS 2 workspace by navigating to `/home/franka/ros2ws/` and executing `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`.
After building the workspace, source it using: `source install/setup.bash`

You can now launch the teleoperation.

#### Integrate it into your own ROS 2 workspace

This repo is a standard ROS 2 package. You can download it into your `src` folder as usual. You might need to also download the [franka_ros2](https://github.com/frankarobotics/franka_ros2) dependency manually.

To set up a ROS 2 environment, follow the official ROS 2 Humble installation [instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

After creating a [workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html), download the repo to the `src` directory and modify the config files as described above.

In a terminal, build the ROS 2 workspace by navigating to `/path/to/your/ros2_workspace/` and executing `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`.

After building the workspace, source it using: `source install/setup.bash`

You can now launch the teleoperation.

## Configuration parameters

In this section the parameters of interest to the user are listed and explained here. You can set them in a config file (example: [config/fr3_teleop_config.yaml](config/fr3_teleop_config.yaml)) or in the `x-teleop-config` parameter of the `docker-compose.yml`.

```yaml
# Base parameters:
# These parameters are valid for all started robots
# Except when they overridden by pair or robot-specific parameters.
# If not set there are defaults for most of them.
# Example values shown here are the default values

urdf_file: "fr3/fr3.urdf.xacro" # Which robot model to use.
# Must match your actual robots.
# Possible values are: fr3/fr3.urdf.xacro, fr3v2/fr3v2.urdf or xacro or fp3/fp3.urdf.xacro

load_gripper: false # If 'true' launch additional nodes to allow teleoperation of the Franka Hand.
# Franka Hands must be attached to the robots if set to 'true'.
# If you have a third-party gripper or hand attached you must set this to `false`.

input_topic_timeout: 2500000 # If messages received from other robots are too old the robot goes into zero gravity mode

fake_hardware: false # For testing purposes you can use fake hardware interfaces

base_namespace: null # Which base namespace to use. All nodes will be started in the base namespace

# Collision thresholds will make the robot stop if exceeded
upper_torque_thresholds_acceleration: [85.0, 85.0, 85.0, 85.0, 11.0, 11.0, 11.0]
upper_torque_thresholds_nominal: [85.0, 85.0, 85.0, 85.0, 11.0, 11.0, 11.0]
upper_force_thresholds_acceleration: [85.0, 85.0, 85.0, 85.0, 11.0, 11.0]
upper_force_thresholds_nominal: [85.0, 85.0, 85.0, 85.0, 11.0, 11.0]

# Be careful when changing control-related parameters
k_gains: [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0] # Stiffness parameters of the follower's joint impedance controller.
d_gains: [30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0] # Damping parameters of the follower's joint impedance controller.

# Force channels of the 4-channel bilateral control scheme (see "Bilateral control architecture"
# above). Independent from k_gains/d_gains, so tracking and feel can each be tuned on their own.
force_reflection_gains: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] # Leader-side gain on the follower's contact torque reflected back to the operator (feel).
force_feedforward_gains: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0] # Follower-side gain on the leader's (operator's) own torque, fed forward into the follower (feel).

alpha: [3, 3, 3, 3, 1, 1, 1] # Feedback-avoidance damping on the leader, per joint. Prevents the leader from "jumping away" right after the follower makes contact.

pairs:
    - namespace: pair_one # each pair must have the 'namespace' parameter set
      # parameters set in a pair override base parameters
      load_gripper: true # this pair does not use grippers, so we override the base parameter
      leader:
        # you can also override some parameters for each robot separately
        # some like robot_ip, arm_id and arm_prefix can only be set for one robot specifically
        robot_ip: leader.pair_one.franka.de # IP address or hostname of robot
        arm_prefix: "" # Prefix for arm topics
      follower:
        robot_ip: follower.pair_one.franka.de

    - namespace: pair_two
      urdf_file: "fp3/fp3.urdf.xacro" # this pair uses fp3's and therefore overrides the base parameter
      leader:
        robot_ip: leader.pair_two.franka.de
      follower:
        robot_ip: follower.pair_two.franka.de
        urdf_file: "fr3v2/fr3v2.urdf.xacro" # You can also set the urdf_file as robot-specific parameter

    - namespace: pair_three
      k_gains: [800.0, 800.0, 800.0, 800.0, 350.0, 250.0, 75.0] # this pair has a different task and needs to override some control parameters, so we override the default parameters
      leader:
        robot_ip: leader.pair_three.franka.de
      follower:
        robot_ip: follower.pair_three.franka.de
```

## Tests and Linting

In a workspace or in the Dev Container you can use the following commands to run tests and linters for this package.

```bash
colcon test --packages-select franka_ros2_teleop
colcon test-result --verbose
```

Some of the tools can be used independently
```bash
cd path/to/ros2_ws/src/franka_ros2_teleop

ament_uncrustify # for c++

ament_flake8 # for python
ament_pep257
```