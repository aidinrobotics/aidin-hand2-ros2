# Observation snapshot of one hand, health and faults are in HandDiagnostics
# header.stamp is when the hand was observed, not when this message was published
std_msgs/Header header
string hand_side          # "right" / "left"

# FK output in rad, order = thumb_joint0..4, then index_joint1..4, middle_joint1..4,
# ring_joint1..4, baby_joint1..4
float64[21] joint_position

# Order = thumb_actuator0..3, then index_actuator1..3, middle_actuator1..3,
# ring_actuator1..3, baby_actuator1..3
int32[16] actuator_position     # encoder count
int32[16] actuator_velocity     # rpm
int16[16] actuator_current      # mA

# Raw sensor values, no unit and no normalization
# After ~/set_tactile_bias, the difference from the bias
int32[17] tactile_thumb
int32[17] tactile_index
int32[17] tactile_middle
int32[17] tactile_ring
int32[17] tactile_baby
int32[20] tactile_palm1_upper
int32[20] tactile_palm1_lower
int32[18] tactile_palm2

# The output is not proof the drives applied it, selected_source says which path ran
CommandState command_state
