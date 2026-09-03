# Observation snapshot of one hand, health and faults are in HandDiagnostics
# header.stamp is when the hand was observed, not when this message was published
std_msgs/Header header
string hand_side          # "right" / "left"

# FK output in rad, order = thumb_joint0..4, then index_joint1..4, middle_joint1..4,
# ring_joint1..4, baby_joint1..4
float64[21] joint_position

# Order = thumb_actuator0..3, then index_actuator1..3, middle_actuator1..3,
# ring_actuator1..3, baby_actuator1..3
float64[16] actuator_position   # encoder count
float64[16] actuator_velocity   # rpm
float64[16] actuator_current    # mA

float64[17] tactile_thumb
float64[17] tactile_index
float64[17] tactile_middle
float64[17] tactile_ring
float64[17] tactile_baby
float64[20] tactile_palm1_upper
float64[20] tactile_palm1_lower
float64[18] tactile_palm2

# The output is not proof the drives applied it, selected_source says which path ran
CommandState command_state
