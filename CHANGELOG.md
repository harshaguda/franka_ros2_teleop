
# Change Log
 
## [Unreleased]

- [Feature] Upgraded the bilateral controller to a 4-channel architecture: the leader's own
  sensed external torque is now transmitted to the follower as an explicit force-feedforward
  term (`force_feedforward_gains`), and the existing force reflection from follower to leader now
  has its own explicit gain (`force_reflection_gains`). Position coupling (`k_gains`/`d_gains`)
  can now be tuned purely for tracking, while the two force channels are tuned purely for feel.

## [Unreleased] - 2025-09-15

- [Hotfix] Fixed bug where the Franka Hand of the follower could not be commanded
 
## [0.1.0] - 2025-08-18

Initial release of franka_ros2_teleop package