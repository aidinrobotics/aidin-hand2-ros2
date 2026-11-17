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

# The command the controller received, its unit set by controller_input_mode
#   JOINT_POSITION, JOINT_IMPEDANCE   rad per active joint, thumb_joint0..3 then index_joint1..3 and on
#   ACTUATOR_POSITION                 encoder count per actuator
#   ACTUATOR_EFFORT                   0.1% of rated current per actuator, 1000 = 100%
#   IDLE                              all NaN
uint8 controller_input_mode
float64[16] controller_input

# The setpoint sent to the drives, its unit set by controller_output_type
#   ACTUATOR_POSITION   encoder count per actuator
#   ACTUATOR_EFFORT     0.1% of rated current per actuator, 1000 = 100%
#   NONE                all NaN
uint8 controller_output_type
float64[16] controller_output

uint8 selected_source
