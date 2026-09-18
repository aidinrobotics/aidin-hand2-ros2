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
references listed in [Reference shape](#reference-shape). Each skeleton subscribes its own `~/cmd`
as `sensor_msgs/JointState` with the same name matching as the command controllers, and reads the
robot hand's state through state interfaces, in the same cycle, without a topic.

| Upper skeleton (plugin class) | Source | Config | Command controller below |
|---|---|---|---|
| `aidin_hand2_examples/JointPositionUpperController` | `src/upper_controllers/joint_position_upper_controller.cpp` | `config/upper_controllers/joint_position_upper.yaml` | `JointPositionController` |
| `aidin_hand2_examples/JointImpedanceUpperController` | `src/upper_controllers/joint_impedance_upper_controller.cpp` | `config/upper_controllers/joint_impedance_upper.yaml` | `JointImpedanceController` |
| `aidin_hand2_examples/ActuatorPositionUpperController` | `src/upper_controllers/actuator_position_upper_controller.cpp` | `config/upper_controllers/actuator_position_upper.yaml` | `ActuatorPositionController` |
| `aidin_hand2_examples/ActuatorEffortUpperController` | `src/upper_controllers/actuator_effort_upper_controller.cpp` | `config/upper_controllers/actuator_effort_upper.yaml` | `ActuatorEffortController` |

Each file is a template you can copy and edit on its own, and they differ only in the reference
shape they claim and export. Two command controllers of different modes cannot be active at the
same time for one robot hand, so use one matching upper controller and command controller pair.

## State input

All four claim the robot hand's state interfaces in `state_interface_configuration()` and
`read_state()` copies them into member arrays at the top of `update_and_write_commands()`. The
members sit in one commented block at the end of each file, in the SDK index order:

| Member | Size | Unit | State interface |
|---|---|---|---|
| `joint_position_rad_` | 21 | rad | `{side}_{joint}/position`, passive `joint4` included |
| `actuator_position_cnt_` | 16 | encoder count | `{side}_{actuator}/position_cnt` |
| `actuator_velocity_rpm_` | 16 | rpm | `{side}_{actuator}/velocity_rpm` |
| `actuator_current_ma_` | 16 | mA | `{side}_{actuator}/current_ma` |
| `tactile_finger_` | 5 × 17 | raw value | `{side}_{finger}_sensor/tactile_1..17` |
| `tactile_palm1_upper_` · `tactile_palm1_lower_` | 20 · 20 | raw value | `{side}_palm_sensor/palm1_upper_1..20` · `palm1_lower_1..20` |
| `tactile_palm2_` | 18 | raw value | `{side}_palm_sensor/palm2_1..18` |

The tactile interfaces are claimed only when the `read_tactile` parameter is true, because the mock
exports none and a controller whose state interface is missing fails to activate. With
`read_tactile: false` the tactile members stay NaN. State interfaces are shared, so claiming them
takes nothing away from the broadcasters.

## Scaling by zero, on purpose

The skeleton is a structural template, not an algorithm example. `update_and_write_commands()` is
split into three commented blocks:

- **READ** — `read_state()` refreshes the state members.
- **WRITE** — the algorithm. The template multiplies the input in `reference_interfaces_` by zero,
  so any command drives the target to zero. Replace this block.
- **FORWARD** — the target goes to the command controller below only when all 16 values are
  finite; a partly finite target is dropped with a warning. With no input the cycle writes nothing
  and the SDK keeps the last command.

The input arrives in `reference_interfaces_` either from the skeleton's own `~/cmd` topic
(standalone) or from a controller chained above it. `on_activate()` fills the references with NaN,
and a consumed input is reset to NaN.

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
The mock exports the joint and actuator state interfaces but no tactile, so keep `read_tactile: false`
there; the tactile members stay NaN.

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
enters chained mode and stops accepting command topic input. Publish a command to the skeleton
instead; the template scales it by zero, so the mock moves to the zero pose whatever the values.

```bash
ros2 topic pub --once /left_joint_position_upper/cmd sensor_msgs/msg/JointState \
  "{name: [left_thumb_joint0, left_thumb_joint1, left_thumb_joint2, left_thumb_joint3,
           left_index_joint1, left_index_joint2, left_index_joint3,
           left_middle_joint1, left_middle_joint2, left_middle_joint3,
           left_ring_joint1, left_ring_joint2, left_ring_joint3,
           left_baby_joint1, left_baby_joint2, left_baby_joint3],
    position: [0.20, 0.35, 0.10, 0.25, 0.08, 0.45, 0.30, 0.04, 0.55, 0.40,
               -0.04, 0.65, 0.50, -0.08, 0.75, 0.60]}"
```

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
- Resolve the `JointState` by name in the subscriber callback and hand the update a fixed-size array.
- Read state through state interfaces, not a topic, so observation and target stay in one cycle.
- Reject NaN and Inf in the update.
- Leave the references NaN on activate.
- Deactivate the upper controller before switching command controllers.
- Use the same controller and config on the robot hand and the mock, `read_tactile` aside.
