# Chainable controller examples

This is the reference example for putting your own controller on top of an existing command
controller. The upper controller sends targets through the reference interfaces of the command
controller. These examples can run on the mock without the robot hand.

```text
your algorithm or JTC
  -> command controller reference
  -> the existing command controller
  -> the robot hand or the mock
```

## What is provided

The four are the same template, so the sources live together in `src/upper_controllers/` and only
the parameters are split per controller. Each skeleton claims the 16 references of the command
controller named by its `target_controller` parameter as its command interfaces,
`{target_controller}/{side}_<name>/<suffix>`, and exports the same suffixes under its own name,
`{upper_controller}/{side}_<name>/<suffix>`. The suffixes are those of the command controller
references listed in [Reference shape](#reference-shape). Each skeleton reads observations from
the `HandState` topic.

| Upper skeleton (plugin class) | Source | Config | Command controller below |
|---|---|---|---|
| `aidin_hand2_examples/JointPositionUpperController` | `src/upper_controllers/joint_position_upper_controller.cpp` | `config/upper_controllers/joint_position_upper.yaml` | `JointPositionController` |
| `aidin_hand2_examples/JointImpedanceUpperController` | `src/upper_controllers/joint_impedance_upper_controller.cpp` | `config/upper_controllers/joint_impedance_upper.yaml` | `JointImpedanceController` |
| `aidin_hand2_examples/ActuatorPositionUpperController` | `src/upper_controllers/actuator_position_upper_controller.cpp` | `config/upper_controllers/actuator_position_upper.yaml` | `ActuatorPositionController` |
| `aidin_hand2_examples/ActuatorEffortUpperController` | `src/upper_controllers/actuator_effort_upper_controller.cpp` | `config/upper_controllers/actuator_effort_upper.yaml` | `ActuatorEffortController` |

Each file is a template you can copy and edit on its own, and they differ only in the reference
shape they claim and export. Two command controllers of different modes cannot be active at the
same time for one robot hand, so use one matching upper controller and command controller pair.

## HandState input

All four subscribe to `hand_state_topic` and copy the latest whole
`aidin_hand2_msgs::msg::HandState` into the member `hand_state_` through a realtime buffer, so one
variable holds all of it:

- header stamp and hand side
- 21 joint positions
- 16 actuator positions, velocities and currents
- every finger and palm tactile cell
- the nested `CommandState`

A valid state has arrived only while `has_hand_state_` is true. The copy sits at the very top of
`update_and_write_commands()`, which runs in chained mode as well. The default topic is
`/<side>_hand_state_broadcaster/hand_state` and the YAML can change it.

## Generating nothing, on purpose

The skeleton is a safe structural template, not an algorithm example.

- `on_activate()` fills its exported references with NaN.
- `update_reference_from_subscribers()` produces no command input.
- `update_and_write_commands()` stores the whole HandState, then forwards the references to the
  controller below only once every one of them is finite. The skeleton rejects a partly finite input
  as an error, even though the command controller supports partial updates after an initial complete target.
- With no input at all, no new target is applied. The SDK keeps the last command, including effort
  if the previous command selected effort control.

So activating the file as it stands produces no new target. To write a real upper controller, read
`hand_state_` where each file says `Write the algorithm here` and fill the whole matching
reference set with finite values in one update.

## Reference shape

With `target_controller` set to `left_joint_position_controller`, a full resource name looks like
`left_joint_position_controller/left_thumb_joint0/position`.

For a configurable target, use the full names below. `{side}` is `left` or `right`; joint and
actuator names and array order are in [Joint and actuator order](../aidin_hand2_msgs/README.ko.md#4-joint-and-actuator-order).

| Command controller | Reference name |
|---|---|
| JointPositionController | `{target_controller}/{side}_{joint}/position` ×16 |
| JointImpedanceController | `{target_controller}/{side}_{joint}/position` ×16 |
| ActuatorPositionController | `{target_controller}/{side}_{actuator}/position_cnt` ×16 |
| ActuatorEffortController | `{target_controller}/{side}_{actuator}/effort_pct` ×16 |

The right hand takes `hand_side: right` and the right hand command controller names.
Joint impedance control is under development; do not use the impedance skeleton for robot hand control.

## Running a skeleton

Complete [Installation](../docs/ko/03_installation.md) and source the ROS 2 and workspace environments
in each terminal. Bring up the mock first, which leaves the joint position command controller active.
The mock does not publish `HandState`, so `has_hand_state_` stays false. This procedure verifies
controller connections only; an algorithm that requires measured state needs the robot hand backend.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_rviz:=false
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/your_ws/install/setup.bash
EXAMPLE_SHARE="$(ros2 pkg prefix aidin_hand2_examples)/share/aidin_hand2_examples"

ros2 run controller_manager spawner left_joint_position_upper \
  --inactive \
  --controller-manager /controller_manager \
  --controller-type aidin_hand2_examples/JointPositionUpperController \
  --param-file "$EXAMPLE_SHARE/config/upper_controllers/joint_position_upper.yaml"

ros2 control switch_controllers --strict \
  --activate left_joint_position_upper

ros2 control list_controllers
```

Once `left_joint_position_upper` claims the reference below it, `left_joint_position_controller`
enters chained mode and stops accepting command topic input. The skeleton generates no target,
so the mock pose does not change.

To try a skeleton of another mode, deactivate the current upper controller first, switch the two
command controllers atomically, and bring the new upper controller up last. Two command
controllers of different modes cannot be active at the same time for one robot hand.

Always take the upper controller down first.

```bash
ros2 control switch_controllers --strict \
  --deactivate left_joint_position_upper
ros2 control unload_controller left_joint_position_upper
```

## Implementation checklist

- Claim the exact reference names of the controller below in `command_interface_configuration()`.
- Export references of the same shape to allow one more chain step above.
- Write only a complete command into the realtime buffer from a subscriber callback.
- Keep the `HandState` subscriber callback free of computation, writing only the latest message
  into the realtime buffer.
- Reject NaN and Inf in the update.
- Leave the references NaN on activate and generate no target.
- Deactivate the upper controller before switching command controllers.
- Use the same controller and config on the robot hand and the mock.
