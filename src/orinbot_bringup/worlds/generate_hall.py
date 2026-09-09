#!/usr/bin/env python3
"""대형 실내 물류창고/공장 월드. 최종 통합 검증용.

    python3 generate_hall.py > hall.sdf        (요약은 stderr)

왜 이 월드인가
==============
`room.sdf`(10x8 m)와 `maze.sdf`(6x6 m)는 **실제 배포 규모가 아닙니다.**
여기서 보려는 것: 넓은 공간의 탐사 수렴, 긴 주행의 SLAM 누적 오차와 루프
클로저, RTAB-Map 노드 수 증가, 통로와 개활지가 섞인 실제 AMR 환경.

치수 24 x 18 m (x -12..12, y -4..14). **남쪽 벽이 y = -4.0 인 것은
`room.sdf` 와 같은 자리라 `docking.yaml` 의 도크 좌표를 그대로 쓰기
위해서입니다** (아래에서 그 파일을 읽어 대조합니다).

구성
----
- 하역장 (y -4..1, 전폭): 개활지. 도크가 남쪽 벽에 있습니다
- 랙 블록 2개 (서/동), 각 4열: 랙 사이 복도 1.7 m — 실제 AMR 통로 폭
- 중앙 주통로 (x -2..1): 3 m
- 사무실 3칸 (북쪽): 문 0.9 m. 닫힌 공간이라 탐사가 들어갔다 나와야 합니다
- 기둥 4개: 구조 기둥이자 고유 랜드마크

**벽면마다 고유 텍스처**입니다 (`generate_textures.py hall`). 재사용하면
서로 다른 장소를 같은 곳으로 오인해 루프 클로저가 깨집니다(실측: map->odom
보정 1.3 m). 랙도 네 면이 각각 다른데, 한 랙의 양면은 **서로 다른 복도**에서
보이기 때문입니다.
"""

import os
import sys

TEX = 'model://room_materials/materials/textures'

# ---- 홀 치수 ----
X0, X1 = -12.0, 12.0
Y0, Y1 = -4.0, 14.0          # Y0 은 room.sdf 와 같아야 합니다 (도크 좌표)
WALL_H = 2.2
WALL_T = 0.15
SEG = 3.0                    # 외벽 세그먼트 길이

# ---- 랙 (2블록 x 4열) ----
RACK_LEN = 8.0
RACK_DEPTH = 0.8
RACK_H = 1.8
RACK_YS = (1.8, 4.3, 6.8, 9.3)      # 랙 중심 y -> 복도 폭 2.5-0.8 = 1.7 m
RACK_BLOCKS = ((-10.0, -2.0), (2.0, 10.0))   # (x 시작, x 끝)

# ---- 사무실 (북쪽) ----
OFFICE_Y = 11.2              # 앞벽 y
OFFICE_D = 2.6               # 깊이 (앞벽 ~ 북쪽 외벽)
OFFICE_XS = ((-9.0, -5.0), (-3.0, 1.0), (3.0, 7.0))
DOOR_W = 0.9

PILLARS = ((-6.0, 0.6), (6.0, 0.6), (-6.0, 10.6), (6.0, 10.6))
PILLAR = 0.4

out = []
_tex = 0


def w(s=''):
    out.append(s)


def next_tex():
    global _tex
    t = _tex
    _tex += 1
    return '%s/hall_%03d.png' % (TEX, t)


def box_visual(name, pose, size, texture):
    w(f"""      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <pbr><metal>
            <albedo_map>{texture}</albedo_map>
            <roughness>0.9</roughness>
          </metal></pbr>
        </material>
      </visual>""")


def box_collision(name, pose, size):
    w(f"""      <collision name="{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>""")


def colored_box(name, pose, size, rgb, collision=True):
    w(f"""      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>{rgb[0]} {rgb[1]} {rgb[2]} 1</ambient>
          <diffuse>{rgb[0]} {rgb[1]} {rgb[2]} 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>""")
    if collision:
        box_collision(f'{name}_c', pose, size)


def slab(name, cx, cy, sx, sy, h, z=None):
    """텍스처가 붙은 벽 조각 하나를 독립 모델로."""
    z = h / 2.0 if z is None else z
    w(f'    <model name="{name}">')
    w('      <static>true</static>')
    w(f'      <pose>{cx:.4f} {cy:.4f} {z:.4f} 0 0 0</pose>')
    w('      <link name="link">')
    box_collision('c', '0 0 0 0 0 0', f'{sx:.4f} {sy:.4f} {h:.4f}')
    box_visual('v', '0 0 0 0 0 0', f'{sx:.4f} {sy:.4f} {h:.4f}', next_tex())
    w('      </link>')
    w('    </model>')


# ====================================================================
w('<?xml version="1.0" ?>')
w('<!-- generate_hall.py 로 생성됨. 직접 수정하지 말고 생성기를 고치세요. -->')
w('<sdf version="1.9">')
w('  <world name="hall">')
w("""
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <!-- 카메라/뎁스 렌더링에 필요 -->
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>

    <gravity>0 0 -9.8</gravity>

    <scene>
      <ambient>0.65 0.65 0.65 1</ambient>
      <background>0.75 0.82 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 넓은 공간이라 광원을 셋으로 나눕니다. 그림자로 특징점이 사라지는
         구역이 생기면 VSLAM 이 그 자리에서 끊깁니다. -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <light type="directional" name="fill1">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.3 0.3 0.34 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.5 -0.4 -0.8</direction>
    </light>
    <light type="directional" name="fill2">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.25 0.25 0.28 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.1 0.9 -0.6</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
""")

# ---- 바닥 (윗면이 정확히 z=0 이라야 ground_plane 과 z-파이팅이 없습니다) ----
w('    <model name="floor">')
w('      <static>true</static>')
w(f'      <pose>{(X0+X1)/2:.3f} {(Y0+Y1)/2:.3f} -0.005 0 0 0</pose>')
w('      <link name="link"><visual name="v">')
w(f'        <geometry><box><size>{X1-X0:.2f} {Y1-Y0:.2f} 0.01</size></box></geometry>')
w('        <material><diffuse>1 1 1 1</diffuse><pbr><metal>')
w(f'          <albedo_map>{TEX}/floor.png</albedo_map><roughness>0.95</roughness>')
w('        </metal></pbr></material>')
w('      </visual></link>')
w('    </model>')


def frange(a, b, step):
    v = a
    while v < b - 1e-6:
        yield v
        v += step


# ---- 외벽 (세그먼트마다 고유 텍스처) ----
n_wall = 0
for x in frange(X0, X1, SEG):
    ln = min(SEG, X1 - x)
    slab(f'wall_s_{n_wall}', x + ln / 2, Y0 - WALL_T / 2, ln, WALL_T, WALL_H); n_wall += 1
    slab(f'wall_n_{n_wall}', x + ln / 2, Y1 + WALL_T / 2, ln, WALL_T, WALL_H); n_wall += 1
for y in frange(Y0, Y1, SEG):
    ln = min(SEG, Y1 - y)
    slab(f'wall_w_{n_wall}', X0 - WALL_T / 2, y + ln / 2, WALL_T, ln, WALL_H); n_wall += 1
    slab(f'wall_e_{n_wall}', X1 + WALL_T / 2, y + ln / 2, WALL_T, ln, WALL_H); n_wall += 1

# ---- 랙 (네 면을 각각 독립 패널로: 한 랙의 양면은 서로 다른 복도에서 보입니다) ----
PANEL_T = 0.06
n_rack = 0
for bx0, bx1 in RACK_BLOCKS:
    length = min(RACK_LEN, bx1 - bx0)
    cx = (bx0 + bx1) / 2.0
    for ry in RACK_YS:
        # 남/북 긴 면
        slab(f'rack_{n_rack}_s', cx, ry - RACK_DEPTH / 2, length, PANEL_T, RACK_H)
        slab(f'rack_{n_rack}_n', cx, ry + RACK_DEPTH / 2, length, PANEL_T, RACK_H)
        # 서/동 마구리
        slab(f'rack_{n_rack}_w', cx - length / 2, ry, PANEL_T, RACK_DEPTH, RACK_H)
        slab(f'rack_{n_rack}_e', cx + length / 2, ry, PANEL_T, RACK_DEPTH, RACK_H)
        n_rack += 1

# ---- 사무실 (앞벽에 문 하나씩) ----
n_off = 0
for ox0, ox1 in OFFICE_XS:
    # 앞벽: 문을 가운데 두고 좌우로 나눕니다
    door_c = (ox0 + ox1) / 2.0
    left = door_c - DOOR_W / 2 - ox0
    right = ox1 - (door_c + DOOR_W / 2)
    if left > 0.05:
        slab(f'off_{n_off}_fl', ox0 + left / 2, OFFICE_Y, left, WALL_T, WALL_H)
    if right > 0.05:
        slab(f'off_{n_off}_fr', ox1 - right / 2, OFFICE_Y, right, WALL_T, WALL_H)
    # 좌우 칸막이 (북쪽 외벽까지)
    depth = Y1 - OFFICE_Y
    slab(f'off_{n_off}_l', ox0, OFFICE_Y + depth / 2, WALL_T, depth, WALL_H)
    slab(f'off_{n_off}_r', ox1, OFFICE_Y + depth / 2, WALL_T, depth, WALL_H)
    n_off += 1

# ---- 기둥 ----
for i, (px, py) in enumerate(PILLARS):
    slab(f'pillar_{i}', px, py, PILLAR, PILLAR, WALL_H)

# ====================================================================
# 충전 도킹 스테이션
#
# **아래 상수는 generate_room.py / docking.yaml / orinbot.urdf.xacro 와
# 반드시 같아야 합니다.** 어긋나면 마커는 보이는데 접점이 안 닿거나,
# 도크 좌표가 지도와 달라 접근 자체가 실패합니다. 그래서 아래에서
# docking.yaml 을 실제로 읽어 대조하고, 다르면 생성을 중단합니다.
DOCK_X = 1.0
DOCK_WALL_Y = Y0 + WALL_T / 2.0
DOCK_PANEL_T = 0.05
DOCK_PANEL_W = 0.52
DOCK_PANEL_H = 0.45
DOCK_PLATE = 0.14
DOCK_MARKER_Z = 0.31
DOCK_MARKER_DX = (-0.16, 0.0, 0.16)
DOCK_MARKER_IDS = (1, 0, 2)
DOCK_PLATE_LAT = 0.055
DOCK_PLATE_W = 0.075
DOCK_PLATE_L = 0.10
DOCK_PLATE_Z = 0.04
DOCK_PLATE_BACK = 0.10
DOCK_COPPER = (0.72, 0.45, 0.20)
# 도킹 방향. docking.yaml 의 reverse_dock 과 **반드시 같아야 합니다** —
# 동판 위치가 이 값에 따라 앞뒤로 바뀝니다. 기하 근거는 generate_room.py.
REVERSE_DOCK = True
DOCK_ROBOT_Y = -3.67 if REVERSE_DOCK else -3.60

_panel_front = DOCK_WALL_Y + DOCK_PANEL_T
_plate_t = 0.008
_plate_cy = _panel_front - 0.002 + _plate_t / 2.0

w('    <model name="dock_station">')
w('      <static>true</static>')
w('      <link name="link">')
box_visual('dock_panel',
           f'{DOCK_X} {DOCK_WALL_Y + DOCK_PANEL_T/2.0} {DOCK_PANEL_H/2.0} 0 0 0',
           f'{DOCK_PANEL_W} {DOCK_PANEL_T} {DOCK_PANEL_H}', f'{TEX}/dock_body.png')
box_collision('dock_panel_c',
              f'{DOCK_X} {DOCK_WALL_Y + DOCK_PANEL_T/2.0} {DOCK_PANEL_H/2.0} 0 0 0',
              f'{DOCK_PANEL_W} {DOCK_PANEL_T} {DOCK_PANEL_H}')
for _dx, _mid in zip(DOCK_MARKER_DX, DOCK_MARKER_IDS):
    box_visual(f'dock_marker_{_mid}',
               f'{DOCK_X + _dx} {_plate_cy} {DOCK_MARKER_Z} 0 0 0',
               f'{DOCK_PLATE} {_plate_t} {DOCK_PLATE}',
               f'{TEX}/dock_marker_{_mid}.png')
# 포고핀이 로봇 뒤쪽이고 후진으로 들어가므로 동판은 벽 쪽으로 들어갑니다.
# 포고핀이 로봇 뒤쪽이라 후진 도킹이면 동판이 벽 쪽으로 들어갑니다.
_plate_y = DOCK_ROBOT_Y + (-DOCK_PLATE_BACK if REVERSE_DOCK else DOCK_PLATE_BACK)
for _k, _s in enumerate((-1, 1)):
    colored_box(
        f'dock_plate_{_k}',
        f'{DOCK_X + _s * DOCK_PLATE_LAT} {_plate_y} {DOCK_PLATE_Z/2.0} 0 0 0',
        f'{DOCK_PLATE_W} {DOCK_PLATE_L} {DOCK_PLATE_Z}',
        DOCK_COPPER)
w('      </link>')
w('    </model>')

w('  </world>')
w('</sdf>')

# ---- docking.yaml 과 대조 (드리프트 방지) ----
_yaml = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     '..', '..', 'orinbot_navigation', 'config', 'docking.yaml')
_checked = '확인 못 함'
try:
    import re
    with open(os.path.normpath(_yaml)) as f:
        m = re.search(r'home_dock:.*?pose:\s*\[([-0-9.]+),\s*([-0-9.]+)', f.read(), re.S)
    if m:
        px, py = float(m.group(1)), float(m.group(2))
        if abs(px - DOCK_X) > 1e-6 or abs(py - DOCK_ROBOT_Y) > 1e-6:
            sys.stderr.write(
                'ERROR: docking.yaml 의 home_dock.pose [%.3f, %.3f] 가 이 월드의 '
                '도크 [%.3f, %.3f] 와 다릅니다.\n' % (px, py, DOCK_X, DOCK_ROBOT_Y))
            sys.exit(1)
        _checked = '일치 [%.2f, %.2f]' % (px, py)
except OSError as e:                                        # noqa: BLE001
    _checked = '읽기 실패 (%s)' % e

sys.stdout.write('\n'.join(out) + '\n')

e = sys.stderr.write
e('홀 %.0f x %.0f m (%.0f m^2),  벽 높이 %.1f m\n'
  % (X1 - X0, Y1 - Y0, (X1 - X0) * (Y1 - Y0), WALL_H))
e('외벽 %d 조각 / 랙 %d 개(면 %d) / 사무실 %d 칸 / 기둥 %d\n'
  % (n_wall, n_rack, n_rack * 4, n_off, len(PILLARS)))
e('랙 복도 폭 %.2f m,  중앙 주통로 %.2f m,  사무실 문 %.2f m\n'
  % (RACK_YS[1] - RACK_YS[0] - RACK_DEPTH,
     RACK_BLOCKS[1][0] - RACK_BLOCKS[0][1], DOOR_W))
e('필요 텍스처: hall_000 ~ hall_%03d  (%d 장)\n' % (_tex - 1, _tex))
e('  -> generate_textures.py 의 N_HALL_SEGMENTS 를 %d 이상으로 두고\n' % _tex)
e('     python3 generate_textures.py hall\n')
e('도크: docking.yaml 대조 %s\n' % _checked)
e('로봇 시작 권장: (0, 0) — 하역장 개활지, 도크에서 %.1f m\n'
  % ((0 - DOCK_ROBOT_Y) ** 2) ** 0.5)
