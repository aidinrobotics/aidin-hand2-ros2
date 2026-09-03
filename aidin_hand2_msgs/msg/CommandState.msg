# Values for controller_input_mode
uint8 CONTROLLER_INPUT_IDLE=0
uint8 CONTROLLER_INPUT_JOINT_POSITION=1
uint8 CONTROLLER_INPUT_JOINT_IMPEDANCE=2
uint8 CONTROLLER_INPUT_ACTUATOR_POSITION=3
uint8 CONTROLLER_INPUT_ACTUATOR_EFFORT=4

# Values for controller_output_type
uint8 OUTPUT_NONE=0
uint8 OUTPUT_ACTUATOR_POSITION=1
uint8 OUTPUT_ACTUATOR_EFFORT=2

# Values for selected_source, the command path taken this cycle
uint8 SOURCE_NONE=0
uint8 SOURCE_CONTROLLER=1
uint8 SOURCE_QUICK_STOP=2
uint8 SOURCE_HOMING=3

# Only the input matching controller_input_mode is valid
uint8 controller_input_mode
JointPositionCommand joint_position_input
JointImpedanceCommand joint_impedance_input
ActuatorPositionCommand actuator_position_input
ActuatorEffortCommand actuator_effort_input

# Only the array matching controller_output_type is valid
uint8 controller_output_type
float64[16] target_position_cnt
float64[16] target_effort_pct

uint8 selected_source

# Unit is 0.1% of rated current, 1000 = 100%
float64[16] max_effort_pct
