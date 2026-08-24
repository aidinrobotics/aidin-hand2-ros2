# 손 한 개의 운영/진단 snapshot. 고정 필드라 rqt Topic Monitor 에서 바로 읽힌다.
# 표준 diagnostic_msgs/DiagnosticArray(/diagnostics)와 같은 정보를 담되, 고정 필드 형태로
# 노출한 커스텀 버전(프로그램 구독·모니터 가시성용). SDK Diagnostics + per-actuator 운영 상태.
std_msgs/Header header
string hand_side          # "right" / "left"

# ── SDK/RT loop/통신 상태 ──
string lifecycle          # SDK 동작 상태 (Disconnected/Connected/Running/Stopped/Faulted)
uint64 nan_command_count  # 거부된 NaN/Inf command 누적 수
uint64 control_cycles
uint64 deadline_misses
float64 last_period_ms    # 직전 cycle 실측 주기 (목표 주기와의 편차 = jitter)
float64 last_compute_ms   # 직전 cycle 처리 시간
string homing_state       # 원점 상태 (NotRun/Succeeded/InProgress/Failed)

# ── per-actuator 운영 상태 (순서 = thumb_actuator0..3, index/middle/ring/baby 각 1..3) ──
bool[16] actuator_enabled
string[16] actuator_fault_name   # ActuatorFault 이름 (정상은 빈 문자열)
