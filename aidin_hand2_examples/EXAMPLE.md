# Chainable controller examples

This is the reference example for putting your own controller on top of an existing command
controller. The command controller keeps its class and plugin name, and acts as the adapter that
sits right before the hardware command port.

```text
your algorithm or JTC
  -> command controller reference
  -> the existing command controller
  -> complete hardware command port + command_lock
  -> real or mock hardware
```

## What is provided

The four are the same template, so the sources live together in `src/upper_controllers/` and only
the parameters are split per controller.

| Upper skeleton (plugin class) | Source | Config | Command controller below |
|---|---|---|---|
| `aidin_hand2_examples/JointPositionUpperController` | `src/upper_controllers/joint_position_upper_controller.cpp` | `config/upper_controllers/joint_position_upper.yaml` | `JointPositionController` |
| `aidin_hand2_examples/JointImpedanceUpperController` | `src/upper_controllers/joint_impedance_upper_controller.cpp` | `config/upper_controllers/joint_impedance_upper.yaml` | `JointImpedanceController` |
| `aidin_hand2_examples/ActuatorPositionUpperController` | `src/upper_controllers/actuator_position_upper_controller.cpp` | `config/upper_controllers/actuator_position_upper.yaml` | `ActuatorPositionController` |
| `aidin_hand2_examples/ActuatorEffortUpperController` | `src/upper_controllers/actuator_effort_upper_controller.cpp` | `config/upper_controllers/actuator_effort_upper.yaml` | `ActuatorEffortController` |

Each file is a template you can copy and edit on its own, and they differ only in the reference
shape they claim and export. Two command controllers of different modes cannot be active at the
same time, because they share `command_lock`, so spawn one of the four configs and no more.

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
  controller below only once every one of them is finite. A partly finite input is rejected as an
  error.
- With no input at all, the command controller below writes NaN, meaning no command this cycle,
  and the hardware sends nothing. The hand holds the pose of its last command.

So activating the file as it stands produces no new target. To write a real upper controller, read
`hand_state_` where each file says `Write the algorithm here` and fill the whole matching
reference set with finite values in one update.

## Reference shape

With `target_controller` set to `left_joint_position_controller`, a full resource name looks like
`left_joint_position_controller/left_thumb_joint0/position`.

| Mode | Suffix |
|---|---|
| JointPosition | `left_<active_joint>/position` x16 |
| JointImpedance | `left_<active_joint>/position` x16 |
| ActuatorPosition | `left_<actuator>/position_cnt` x16 |
| ActuatorEffort | `left_<actuator>/effort_pct` x16 |

The right hand takes `hand_side: right` and the right hand command controller names.

## Running a skeleton

Bring up the mock first, which leaves the joint position command controller active.

```bash
ros2 launch aidin_hand2_bringup aidin_hand2_mock.launch.py use_rviz:=false
```

In another terminal:

```bash
source install/setup.bash
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
enters chained mode. The skeleton generates no value, so the hand holds the activation seed of the
controller below.

To try a skeleton of another mode, deactivate the current upper controller first, switch the two
command controllers atomically, and bring the new upper controller up last. Two command
controllers of different modes cannot be active at the same time, because they share
`command_lock`.

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
- Reject NaN, Inf and negative speed or gain in the update.
- Leave the references NaN on activate and generate no target.
- Switch modes by command controller, never by claiming hardware command interfaces piecemeal.
- Use the same controller and config on real and mock hardware.
