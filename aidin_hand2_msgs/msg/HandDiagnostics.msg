# HandState
std_msgs/Header header
string hand_side          # "right" / "left"

string lifecycle          # Disconnected/Connected/Running/Stopped/Faulted
uint64 nan_command_count  # commands rejected for a NaN or Inf target
uint64 control_cycles
uint64 deadline_misses    # cycles that overran the target period
float64 last_period_ms    # last cycle-to-cycle gap
float64 last_compute_ms   # last cycle processing time
string homing_state       # NotRun/Succeeded/InProgress/Failed

# Order = thumb_actuator0..3, then index_actuator1..3, middle_actuator1..3,
# ring_actuator1..3, baby_actuator1..3
bool[16] actuator_enabled
string[16] actuator_fault_name   # ActuatorFault name, empty while healthy
