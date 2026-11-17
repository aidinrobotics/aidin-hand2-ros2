# JointImpedanceControllerConfig
# 0~1, 1 being the joint's hard stiffness
# Order = thumb_joint0..3, then index_joint1..3, middle_joint1..3, ring_joint1..3, baby_joint1..3
float64[16] stiffness

bool filter_enabled      # false takes the target straight through
float64 cutoff_freq      # Hz
