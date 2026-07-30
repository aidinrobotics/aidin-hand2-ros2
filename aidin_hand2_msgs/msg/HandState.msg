# 손 한 개의 학습 관측(observation) snapshot. 순수 수치만 — fault/enabled 등 운영값은
# /diagnostics 로만 노출한다. 모든 배열은 고정 크기라 학습 텐서화에 그대로 쓴다.
# header.stamp = 관측 시점 wall-clock (SDK RX 직후). 발행 시각이 아니라 손이 관측된 순간이다.
std_msgs/Header header
string hand_side          # "right" / "left"

# kinematic (FK 결과, rad). 순서 = SDK joint 21 index.
float64[21] joint_position

# actuator proprioception. 순서 = thumb_actuator0..3, index/middle/ring/baby 각 actuator1..3.
float64[16] actuator_position   # encoder count
float64[16] actuator_velocity   # rpm
float64[16] actuator_current    # mA

# tactile 143 cell (SDK HandState.tactile 과 1:1).
float64[17] tactile_thumb
float64[17] tactile_index
float64[17] tactile_middle
float64[17] tactile_ring
float64[17] tactile_baby
float64[20] tactile_palm1_upper
float64[20] tactile_palm1_lower
float64[18] tactile_palm2

# controller 입력과 변환 출력. 출력은 drive 적용 보장이 아니며 selected_source와 함께 읽습니다.
CommandState command_state
