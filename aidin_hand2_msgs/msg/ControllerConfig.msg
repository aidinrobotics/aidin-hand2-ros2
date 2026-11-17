# ControllerConfig
# The settings the SDK holds for one hand, published when they change
# header.stamp is the cycle that saw the change, at most one cycle after it was applied
std_msgs/Header header
string hand_side          # "right" / "left"

# 0.1% of rated current, 1000 being 100%
# Order = thumb_actuator0..3, then index_actuator1..3, middle_actuator1..3,
# ring_actuator1..3, baby_actuator1..3
float64[16] max_effort

JointPositionControllerConfig joint_position_controller
JointImpedanceControllerConfig joint_impedance_controller
