#!/usr/bin/env python3
"""MANUS 글러브 → AIDIN Hand Gen2 텔레오퍼 캘리브레이션.

극단 자세에서 글러브 ergonomics value(degree)를 측정해, 각 active_joint 의 target 범위
(SDK clamp workspace 경계)에 선형 매핑하는 scale/offset 을 산출한다. 결과는 터미널에
glove_teleop_controller.yaml 의 terms 형식으로 출력한다(복붙).

측정 원리 (2점 선형회귀, deg 도메인):
    target_deg = glove_deg * scale + offset
    두 극단 자세의 (glove_value, target_deg) 쌍으로:
      scale  = (t_hi - t_lo) / (g_hi - g_lo)
      offset = t_lo - scale * g_lo
매핑 controller 는 최종적으로 target_deg * (pi/180) 을 관절 rad 로 지령한다.

target 범위 = SDK joint_clamp.cpp 의 workspace 경계(GUI 와 동일):
  long finger: abduction ±27.5°, MCP flex 0~97°, PIP flex 0~90°
  thumb:       CMC abd 0~110°, MCP flex 0~77.33°, MCP abd ±30°, DIP flex 0~70°

abduction 부호: URDF 각도는 모두 한 방향으로 +지만, "모으는" 동작은 손가락마다 다르다
(검지·중지는 +가 모으는 쪽, 약지·새끼는 -가 모으는 쪽). 아래 STEPS 테이블이 각 joint 의
극단(모음/벌림, 폄/오므림)을 어느 target 값에 대응시킬지 손가락별 부호로 명시한다.

실행:
  ros2 run aidin_hand2_examples glove_calibrate            # 왼손, manus_glove_0
  ros2 run aidin_hand2_examples glove_calibrate --side right --topic manus_glove_1
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from manus_ros2_msgs.msg import ManusGlove


# ── clamp workspace 경계 (deg) — target 범위 ──────────────────────────
LONG_ABD_MAX = 27.5     # long finger abduction plateau (clamp kLongBoundary)
LONG_MCP_MAX = 97.0     # long finger MCP flexion (clamp kLongFlexionMaxDeg)
LONG_PIP_MAX = 90.0     # long finger PIP flexion (clamp kLongDistalMaxDeg)
THUMB_J0_MAX = 110.0    # thumb CMC abduction (clamp kThumbJ0MaxDeg)
THUMB_J1_MAX = 77.33    # thumb MCP flexion (clamp kThumbFlexionMaxDeg)
THUMB_J2_MAX = 30.0     # thumb MCP abduction (clamp kThumbBoundary, URDF ±30)
THUMB_J3_MAX = 70.0     # thumb DIP flexion (clamp kThumbJ3MaxDeg)


# ── 캘리브 대상 joint 정의 ────────────────────────────────────────────
# 각 joint:
#   name        : active_joint base 이름 (yaml key)
#   ergonomics  : 이 joint 를 구동할 MANUS ergonomics type
#   t_lo, t_hi  : target 범위(deg) — pose_lo/pose_hi 자세에 각각 대응
#   pose_lo/hi  : 그 target 을 측정할 자세 이름(STEPS 의 pose)
# abduction 은 손가락별 부호가 달라 t_lo/t_hi 를 그에 맞게 지정한다.
#
# 자세 이름:
#   "open"  : 손 다 폄 + 손가락 벌림
#   "spread": 손가락 최대 벌림 (open 과 겹치면 open 재사용 가능)
#   "gather": 손가락 모음
#   "close" : 손 다 오므림 (최대 굽힘)
JOINTS = [
    # ─ thumb ─ (abduction 부호는 실측 후 조정 여지; 우선 +벌림 기준)
    dict(name="thumb_joint0", ergo="ThumbMCPSpread",  t_lo=0.0,  t_hi=THUMB_J0_MAX, pose_lo="gather", pose_hi="spread"),
    dict(name="thumb_joint1", ergo="ThumbMCPStretch", t_lo=0.0,  t_hi=THUMB_J1_MAX, pose_lo="open",   pose_hi="close"),
    # thumb_joint2(MCP abd)는 joint0 종속(yaml terms) — 캘리브 제외.
    dict(name="thumb_joint3", ergo="ThumbDIPStretch", t_lo=0.0,  t_hi=THUMB_J3_MAX, pose_lo="open",   pose_hi="close"),

    # ─ index (검지: 실측상 sign -1) ─
    dict(name="index_joint1", mode="abd_flexcomp2", spread="IndexSpread", mcp="IndexMCPStretch", sign=-1.0, range_deg=27.5),
    dict(name="index_joint2", ergo="IndexMCPStretch", t_lo=0.0, t_hi=LONG_MCP_MAX, pose_lo="open", pose_hi="close"),
    dict(name="index_joint3", ergo="IndexPIPStretch", t_lo=0.0, t_hi=LONG_PIP_MAX, pose_lo="open", pose_hi="close"),

    # ─ middle (중지: 실측상 sign -1) ─
    dict(name="middle_joint1", mode="abd_flexcomp2", spread="MiddleSpread", mcp="MiddleMCPStretch", sign=-1.0, range_deg=27.5),
    dict(name="middle_joint2", ergo="MiddleMCPStretch", t_lo=0.0, t_hi=LONG_MCP_MAX, pose_lo="open", pose_hi="close"),
    dict(name="middle_joint3", ergo="MiddlePIPStretch", t_lo=0.0, t_hi=LONG_PIP_MAX, pose_lo="open", pose_hi="close"),

    # ─ ring (약지: -가 모으는 쪽 → sign -1) ─
    dict(name="ring_joint1", mode="abd_flexcomp2", spread="RingSpread", mcp="RingMCPStretch", sign=-1.0, range_deg=27.5),
    dict(name="ring_joint2", ergo="RingMCPStretch", t_lo=0.0, t_hi=LONG_MCP_MAX, pose_lo="open", pose_hi="close"),
    dict(name="ring_joint3", ergo="RingPIPStretch", t_lo=0.0, t_hi=LONG_PIP_MAX, pose_lo="open", pose_hi="close"),

    # ─ baby/pinky (새끼: -가 모으는 쪽 → sign -1) ─
    dict(name="baby_joint1", mode="abd_flexcomp2", spread="PinkySpread", mcp="PinkyMCPStretch", sign=-1.0, range_deg=27.5),
    dict(name="baby_joint2", ergo="PinkyMCPStretch", t_lo=0.0, t_hi=LONG_MCP_MAX, pose_lo="open", pose_hi="close"),
    dict(name="baby_joint3", ergo="PinkyPIPStretch", t_lo=0.0, t_hi=LONG_PIP_MAX, pose_lo="open", pose_hi="close"),
]

# 매핑 방식:
#   "range"(기본): 2점 선형 — pose_lo/hi 의 glove value 를 t_lo/t_hi 로 매핑(flexion 용).
#   "abd_flexcomp2": abduction(벌림) flexcomp. 컨트롤러(cpp)가 아래 파라미터로 계산한다:
#       u = clamp01((mcp - m_flat)/(m_bent - m_flat))
#       zero = z_flat + u·(z_bent - z_flat)          # 벌림 중앙값(0점)
#       span = span_flat + u·(span_bent - span_flat) # 벌림 전체범위
#       target_rad = (spread - zero)/span · (2·range_deg·pi/180) · sign
#     굽힘 두 단계(flat/bent)에서 각각 spread 를 좌우로 스윕해 min/max 를 잡아:
#       z_*   = (spread_max + spread_min)/2   (벌림 중앙값)
#       span_* = spread_max - spread_min      (벌림 전체범위)
#       m_*   = mcp_mean                      (해당 스윕의 mcp 평균)
#     을 산출한다. 캘리브는 z/span/m 만 내보내고, 위 계산식은 컨트롤러가 수행한다.

# 측정할 자세 스텝 — 순서대로 사용자에게 지시. 필요하면 자세를 더 추가/분리할 수 있다.
#   flat_sweep/bent_sweep : abduction 용 스윕(min/max/mean). sample_sweep 로 측정.
#   open/close            : flexion(range) 용 순간평균 2점. sample_average 로 측정.
STEPS = [
    ("flat_sweep", "손가락을 편 상태로 유지하며 좌우로 최대한 벌렸다 모으기를 반복하세요"),
    ("bent_sweep", "손가락을 반쯤 굽힌 상태로 유지하며 좌우로 벌렸다 모으기를 반복하세요"),
    ("open",       "손을 활짝 펴세요 (모든 손가락 곧게 폄)"),
    ("close",      "손을 완전히 오므리세요 (주먹, 모든 관절 최대 굽힘)"),
]

# 스윕 스텝 여부(True=sample_sweep, False=sample_average).
SWEEP_STEPS = {"flat_sweep", "bent_sweep"}

SAMPLE_SEC = 1.5  # open/close 순간평균 시간(초)
SWEEP_SEC = 5.0   # flat_sweep/bent_sweep 스윕 측정 시간(초)


class GloveCalibrator(Node):
    def __init__(self, topic):
        super().__init__("glove_calibrate")
        self._latest = None
        self.create_subscription(ManusGlove, topic, self._cb, 10)
        self._topic = topic

    def _cb(self, msg):
        self._latest = {e.type: e.value for e in msg.ergonomics}

    def wait_for_data(self, timeout_sec=10.0):
        end = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while rclpy.ok() and self._latest is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().nanoseconds > end:
                return False
        return True

    def sample_average(self, seconds):
        """seconds 동안 ergonomics 를 모아 type 별 평균 반환."""
        acc, n = {}, 0
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest:
                for k, v in self._latest.items():
                    acc[k] = acc.get(k, 0.0) + v
                n += 1
        if n == 0:
            return {}
        return {k: v / n for k, v in acc.items()}

    def sample_sweep(self, seconds):
        """seconds 동안 계속 spin 하며 ergonomics type 별 min·max·mean 을 수집.

        벌렸다 모으기를 반복하는 스윕 구간용. 반환:
            {type: {"min": .., "max": .., "mean": ..}}
        """
        stats = {}
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest:
                for k, v in self._latest.items():
                    entry = stats.get(k)
                    if entry is None:
                        stats[k] = {"min": v, "max": v, "sum": v, "count": 1}
                    else:
                        if v < entry["min"]:
                            entry["min"] = v
                        if v > entry["max"]:
                            entry["max"] = v
                        entry["sum"] += v
                        entry["count"] += 1
        return {
            k: {"min": e["min"], "max": e["max"], "mean": e["sum"] / e["count"]}
            for k, e in stats.items()
        }


def main():
    parser = argparse.ArgumentParser(description="MANUS glove teleop calibration")
    parser.add_argument("--side", default="left", choices=["left", "right"])
    parser.add_argument("--topic", default="manus_glove_0")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = GloveCalibrator(args.topic)

    print(f"[calib] '{args.topic}' 구독, 글러브 데이터 대기 중...")
    if not node.wait_for_data():
        print("[calib] ERROR: 글러브 데이터 없음. manus_data_publisher 실행 확인.", file=sys.stderr)
        node.destroy_node(); rclpy.shutdown(); return 1

    # 자세별 ergonomics 측정. 스윕 스텝은 min/max/mean, 나머지는 순간평균.
    pose_values = {}
    for pose, instruction in STEPS:
        if pose in SWEEP_STEPS:
            input(f"\n[{pose}] {instruction}\n  준비되면 Enter (그 뒤 {SWEEP_SEC}s 동안 계속 스윕)...")
            print(f"  측정 중... ({SWEEP_SEC}s 동안 벌렸다 모으기 반복)")
            pose_values[pose] = node.sample_sweep(SWEEP_SEC)
        else:
            input(f"\n[{pose}] {instruction}\n  자세를 잡고 Enter (그 뒤 {SAMPLE_SEC}s 측정)...")
            print(f"  측정 중... ({SAMPLE_SEC}s 동안 정지 유지)")
            pose_values[pose] = node.sample_average(SAMPLE_SEC)
        if not pose_values[pose]:
            print("  WARN: 이 자세에서 데이터 없음")

    # joint 별 scale/offset 산출.
    print("\n" + "=" * 60)
    print("  캘리브 결과 — glove_teleop_controller.yaml 의 mapping 에 복붙")
    print("=" * 60)
    print("    mapping:")
    for j in JOINTS:
        mode = j.get("mode", "range")

        if mode == "abd_flexcomp2":
            # abduction flexcomp: flat/bent 스윕에서 spread min/max 와 mcp mean 산출.
            spread_type = j["spread"]; mcp_type = j["mcp"]
            sign = j["sign"]; range_deg = j["range_deg"]
            flat = pose_values.get("flat_sweep", {})
            bent = pose_values.get("bent_sweep", {})
            s_flat = flat.get(spread_type); s_bent = bent.get(spread_type)
            m_flat_e = flat.get(mcp_type); m_bent_e = bent.get(mcp_type)
            if None in (s_flat, s_bent, m_flat_e, m_bent_e):
                print(f"      # {j['name']}: 측정 실패 ({spread_type}/{mcp_type} @ flat|bent 스윕 없음)")
                continue
            z_flat = (s_flat["max"] + s_flat["min"]) / 2.0
            span_flat = s_flat["max"] - s_flat["min"]
            z_bent = (s_bent["max"] + s_bent["min"]) / 2.0
            span_bent = s_bent["max"] - s_bent["min"]
            m_flat = m_flat_e["mean"]
            m_bent = m_bent_e["mean"]
            warn = ""
            if span_flat < 1.0 or span_bent < 1.0:
                warn = f"  # WARN: span 이 너무 작음 (span_flat={span_flat:.2f} span_bent={span_bent:.2f}) — 스윕 재측정"
            print(f'      {j["name"]}:')
            print(f'        abd: {{ spread: {spread_type}, mcp: {mcp_type}, '
                  f'm_flat: {m_flat:.1f}, m_bent: {m_bent:.1f}, '
                  f'z_flat: {z_flat:.1f}, z_bent: {z_bent:.1f}, '
                  f'span_flat: {span_flat:.1f}, span_bent: {span_bent:.1f}, '
                  f'range_deg: {range_deg:.1f}, sign: {sign:.1f} }}{warn}')
            continue

        # range: 2점 선형.
        ergo = j["ergo"]
        g_lo = pose_values.get(j["pose_lo"], {}).get(ergo)
        g_hi = pose_values.get(j["pose_hi"], {}).get(ergo)
        if g_lo is None or g_hi is None:
            print(f"      # {j['name']}: 측정 실패 ({ergo} 값 없음) — 수동 확인 필요")
            continue
        if abs(g_hi - g_lo) < 1e-6:
            print(f"      # {j['name']}: glove 극단 차이 없음 (g_lo={g_lo:.2f} g_hi={g_hi:.2f}) — 자세 재측정")
            continue
        scale = (j["t_hi"] - j["t_lo"]) / (g_hi - g_lo)
        offset = j["t_lo"] - scale * g_lo
        print(f'      {j["name"]:<14}: {{ terms: "{ergo}*{scale:.4f}", offset: {offset:.2f} }}'
              f'  # g[{g_lo:.1f},{g_hi:.1f}]->t[{j["t_lo"]:.0f},{j["t_hi"]:.0f}]')
    # joint 종속(thumb_joint2)은 캘리브 대상 아님 — 안내.
    print("      # thumb_joint2 는 joint0 종속(terms: \"thumb_joint0*k\") — 수동 유지")
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
