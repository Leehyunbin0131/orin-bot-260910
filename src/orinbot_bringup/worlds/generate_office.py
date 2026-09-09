#!/usr/bin/env python3
"""사무실 월드 — 무늬 없는 벽 + 가구 + 움직이는 사람. 실전형 최종 시험.

    python3 generate_office.py > office.sdf      (요약은 stderr)

무엇을 시험하는가
=================
다른 월드는 벽이 전부 고대비 벽돌입니다. 실제 사무실은 그렇지 않으므로
여기서는 **의도적으로 어려운 조건**을 만듭니다:

1. **벽이 거의 평평합니다** (`office_*.png`, 저대비). 무늬 없는 벽 앞에서는
   뎁스가 정확해도 시각 오도메트리가 동작하지 않으므로, **특징점을 가구가
   제공**하도록 설계했습니다. 실제 실내가 그렇습니다.
2. **움직이는 사람 3명.** 없던 장애물이 갑자기 생기고, RTAB-Map 격자에
   유령으로 남고, 경로가 막혀 복구가 돕니다. `Wait` 복구가 의미를 갖는
   유일한 상황이기도 합니다 (기다리면 비켜 줍니다).
3. **가구가 낮은 장애물**입니다. 책상 상판 0.75 m 는 라이다 평면(0.49 m)에
   걸리지만 의자 다리 사이는 통과해 보이고 선반 아래는 비어 있습니다 —
   STVL 3D 복셀이 없으면 못 봅니다.

치수 20 x 14 m. 남쪽 벽을 y = -4.0 에 두어 `docking.yaml` 의
`home_dock.pose [1.0, -3.60]` 을 그대로 씁니다 (아래에서 대조 검증).

사람은 `gz-sim-velocity-control-system` 이 `/model/<이름>/cmd_vel` 을 구독하고
ROS 쪽 `people_sim.py` 가 복도를 따라 왕복시킵니다. 액터 메시(.dae)를 받아
오지 않으므로 네트워크 없이 동작합니다.
"""

import math
import os
import re
import sys

TEX = 'model://room_materials/materials/textures'

X0, X1 = -10.0, 10.0
Y0, Y1 = -4.0, 10.0              # Y0 은 도크 좌표 때문에 고정
WALL_H = 2.4
WALL_T = 0.12
SEG = 2.5

COR_A = (-0.9, 0.9)              # 남쪽 복도 (E-W)
COR_B = (5.1, 6.9)               # 북쪽 복도 (E-W)
PART_X = (-6.0, -2.0, 2.0, 6.0)  # 방 칸막이 x
DOOR = 0.9

out = []
_tex = 0


def w(s=''):
    out.append(s)


def tex():
    global _tex
    t = _tex
    _tex += 1
    return '%s/office_%03d.png' % (TEX, t)


def vis(name, pose, size, texture):
    w(f"""      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material><diffuse>1 1 1 1</diffuse><pbr><metal>
          <albedo_map>{texture}</albedo_map><roughness>0.92</roughness>
        </metal></pbr></material>
      </visual>""")


def col(name, pose, size):
    w(f"""      <collision name="{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>""")


def slab(name, cx, cy, sx, sy, h=WALL_H):
    w(f'    <model name="{name}">')
    w('      <static>true</static>')
    w(f'      <pose>{cx:.4f} {cy:.4f} {h/2:.4f} 0 0 0</pose>')
    w('      <link name="link">')
    col('c', '0 0 0 0 0 0', f'{sx:.4f} {sy:.4f} {h:.4f}')
    vis('v', '0 0 0 0 0 0', f'{sx:.4f} {sy:.4f} {h:.4f}', tex())
    w('      </link>')
    w('    </model>')


def furniture(name, cx, cy, sx, sy, sz, z0, texture):
    """가구 한 점. 벽이 밋밋하므로 **특징점은 여기서 나옵니다.**"""
    w(f'    <model name="{name}">')
    w('      <static>true</static>')
    w(f'      <pose>{cx:.3f} {cy:.3f} {z0 + sz/2:.3f} 0 0 0</pose>')
    w('      <link name="link">')
    col('c', '0 0 0 0 0 0', f'{sx:.3f} {sy:.3f} {sz:.3f}')
    vis('v', '0 0 0 0 0 0', f'{sx:.3f} {sy:.3f} {sz:.3f}', f'{TEX}/{texture}')
    w('      </link>')
    w('    </model>')


def person(name, cx, cy, rgb):
    """보행자 한 명. 속도는 people_sim.py 가 ROS 로 줍니다.

    **중력을 끕니다.** 반지름 0.22 m 에 높이 1.7 m 인 원기둥을 세워 두면
    물리적으로 넘어지는 것이 당연합니다(실제로 넘어졌습니다). 사람은
    스스로 균형을 잡는 존재이므로, 여기서는 중력을 빼서 그 균형을 대신합니다.
    velocity-control 이 매 스텝 속도를 지정하므로 로봇과 부딪혀도 밀려나지
    않고 제 갈 길을 갑니다 — 실제 사람이 로봇을 신경 쓰지 않는 것과 같아
    시험으로서 오히려 맞습니다.

    관성은 크게 둡니다. 작으면 접촉 토크에 팽이처럼 돕니다.
    """
    w(f'    <model name="{name}">')
    w(f'      <pose>{cx:.2f} {cy:.2f} 0.85 0 0 0</pose>')
    w('      <link name="link">')
    w('        <gravity>false</gravity>')
    w('        <inertial><mass>70</mass><inertia>'
      '<ixx>200</ixx><iyy>200</iyy><izz>200</izz>'
      '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')
    w('        <collision name="c"><geometry>'
      '<cylinder><radius>0.22</radius><length>1.70</length></cylinder>'
      '</geometry></collision>')
    w('        <visual name="v"><geometry>'
      '<cylinder><radius>0.22</radius><length>1.70</length></cylinder></geometry>')
    w(f'          <material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse>'
      '<specular>0.2 0.2 0.2 1</specular></material>')
    w('        </visual>')
    w('      </link>')
    w('      <plugin filename="gz-sim-velocity-control-system"'
      ' name="gz::sim::systems::VelocityControl">')
    w(f'        <topic>/model/{name}/cmd_vel</topic>')
    w('      </plugin>')
    w('    </model>')


def frange(a, b, s):
    v = a
    while v < b - 1e-6:
        yield v
        v += s


# ====================================================================
w('<?xml version="1.0" ?>')
w('<!-- generate_office.py 로 생성됨. 생성기를 고치세요. -->')
w('<sdf version="1.9">')
w('  <world name="office">')
w("""
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.7 0.7 0.7 1</ambient>
      <background>0.8 0.85 0.9 1</background>
      <shadows>true</shadows>
    </scene>
    <!-- 사무실 조명: 그림자를 약하게 둡니다. 밋밋한 벽에 강한 그림자가
         지면 그 경계가 유일한 특징점이 되어, 조명이 바뀌면 지도가 흔들립니다. -->
    <light type="directional" name="ceil1">
      <cast_shadows>true</cast_shadows><pose>0 0 6 0 0 0</pose>
      <diffuse>0.55 0.55 0.55 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <direction>-0.3 0.2 -0.93</direction>
    </light>
    <light type="directional" name="ceil2">
      <cast_shadows>false</cast_shadows><pose>0 0 6 0 0 0</pose>
      <diffuse>0.4 0.4 0.42 1</diffuse><specular>0 0 0 1</specular>
      <direction>0.4 -0.3 -0.87</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>
""")

w('    <model name="floor">')
w('      <static>true</static>')
w(f'      <pose>{(X0+X1)/2:.2f} {(Y0+Y1)/2:.2f} -0.005 0 0 0</pose>')
w('      <link name="link"><visual name="v">')
w(f'        <geometry><box><size>{X1-X0} {Y1-Y0} 0.01</size></box></geometry>')
w('        <material><diffuse>1 1 1 1</diffuse><pbr><metal>')
w(f'          <albedo_map>{TEX}/floor.png</albedo_map><roughness>0.95</roughness>')
w('        </metal></pbr></material>')
w('      </visual></link>')
w('    </model>')

# ---- 외벽 ----
n = 0
for x in frange(X0, X1, SEG):
    ln = min(SEG, X1 - x)
    slab(f'w_s{n}', x + ln / 2, Y0 - WALL_T / 2, ln, WALL_T); n += 1
    slab(f'w_n{n}', x + ln / 2, Y1 + WALL_T / 2, ln, WALL_T); n += 1
for y in frange(Y0, Y1, SEG):
    ln = min(SEG, Y1 - y)
    slab(f'w_w{n}', X0 - WALL_T / 2, y + ln / 2, WALL_T, ln); n += 1
    slab(f'w_e{n}', X1 + WALL_T / 2, y + ln / 2, WALL_T, ln); n += 1
n_wall = n

# ---- 복도 벽 (방과 복도를 가르는 긴 벽, 방마다 문 하나) ----
def corridor_wall(tag, y, xs):
    """y 에 벽을 놓되 각 방 구간 가운데에 문을 냅니다."""
    edges = [X0] + list(xs) + [X1]
    k = 0
    for a, b in zip(edges[:-1], edges[1:]):
        dc = (a + b) / 2.0
        left = dc - DOOR / 2 - a
        right = b - (dc + DOOR / 2)
        if left > 0.05:
            slab(f'{tag}_{k}a', a + left / 2, y, left, WALL_T)
        if right > 0.05:
            slab(f'{tag}_{k}b', b - right / 2, y, right, WALL_T)
        k += 1


corridor_wall('cA_n', COR_A[1], PART_X)      # 복도A 북쪽 = 방밴드1 남쪽
corridor_wall('cB_s', COR_B[0], PART_X)      # 복도B 남쪽 = 방밴드1 북쪽
corridor_wall('cB_n', COR_B[1], PART_X)      # 복도B 북쪽 = 방밴드2 남쪽

# ---- 방 칸막이 (세로) ----
for i, px in enumerate(PART_X):
    slab(f'p1_{i}', px, (COR_A[1] + COR_B[0]) / 2, WALL_T, COR_B[0] - COR_A[1])
    slab(f'p2_{i}', px, (COR_B[1] + Y1) / 2, WALL_T, Y1 - COR_B[1])

# ---- 가구 ----
# 벽이 밋밋하므로 여기가 특징점의 주 공급원입니다.
POSTERS = ['poster%d.png' % i for i in range(1, 9)]
nf = 0
room_bands = [(COR_A[1], COR_B[0]), (COR_B[1], Y1)]
edges = [X0] + list(PART_X) + [X1]
for band_i, (by0, by1) in enumerate(room_bands):
    for r, (a, b) in enumerate(zip(edges[:-1], edges[1:])):
        cx = (a + b) / 2.0
        cy = (by0 + by1) / 2.0
        t = POSTERS[(band_i * 5 + r) % len(POSTERS)]
        # 책상 (상판 0.75 — 라이다 평면 0.49 보다 높아 2D 로도 보임)
        furniture(f'desk_{nf}', cx - 0.9, cy + 0.4, 1.40, 0.70, 0.06, 0.72, t)
        furniture(f'desk_{nf}_l1', cx - 1.5, cy + 0.4, 0.06, 0.66, 0.72, 0.0, 'crate.png')
        furniture(f'desk_{nf}_l2', cx - 0.3, cy + 0.4, 0.06, 0.66, 0.72, 0.0, 'crate.png')
        # 의자 (좌판 0.45 — 라이다 평면 아래. 카메라만 봅니다)
        furniture(f'chair_{nf}', cx - 0.9, cy - 0.35, 0.45, 0.45, 0.05, 0.43, 'crate.png')
        furniture(f'chair_{nf}_b', cx - 0.9, cy - 0.55, 0.45, 0.06, 0.50, 0.43, t)
        # 선반 (벽 쪽, 높음)
        furniture(f'shelf_{nf}', cx + 1.3, by1 - 0.35, 0.90, 0.40, 1.80, 0.0, t)
        nf += 1

# ---- 움직이는 사람 3명 ----
# 복도에서 원을 그립니다. 반지름 = v/wz.
#
# **로봇 스폰 지점 (0, -2.0) 에서 멀리 둡니다.** 처음에 (0, -2.4) 에 두었더니
# 70 kg 짜리가 로봇과 겹쳐 생성되어, Gazebo 가 초기 관통을 해소하면서
# 둘 다 튕겨 나갔습니다. 정적 물체와 달리 동적 물체는 겹치면 폭발합니다.
ROBOT_SPAWN = (0.0, -2.0)
# 시작 위치. 실제 경로는 people_sim.py 의 ROUTES 와 맞춰야 합니다.
_people = [
    ('person_0', -8.0, 0.0, '0.85 0.30 0.25 1'),     # 복도 A 서쪽 끝
    ('person_1', 8.0, 6.0, '0.25 0.45 0.85 1'),      # 복도 B 동쪽 끝
    ('person_2', -8.0, -1.6, '0.25 0.70 0.35 1'),    # 로비 서쪽 끝
]
for _p in _people:
    d0 = math.dist((_p[1], _p[2]), ROBOT_SPAWN)
    if d0 < 1.5:
        sys.stderr.write('ERROR: %s 가 로봇 스폰에서 %.2f m 뿐입니다 (>=1.5 필요)\n'
                         % (_p[0], d0))
        sys.exit(1)
    person(*_p)

# ====================================================================
# 충전 도킹 스테이션 (generate_room.py / docking.yaml / URDF 와 같은 값)
# 도킹 방향. docking.yaml 의 reverse_dock 과 **반드시 같아야 합니다** —
# 동판 위치가 이 값에 따라 앞뒤로 바뀝니다. 기하 근거는 generate_room.py.
REVERSE_DOCK = True
DOCK_X, DOCK_ROBOT_Y = 1.0, (-3.67 if REVERSE_DOCK else -3.60)
DOCK_WALL_Y = Y0 + WALL_T / 2.0
DOCK_PANEL_T, DOCK_PANEL_W, DOCK_PANEL_H = 0.05, 0.52, 0.45
DOCK_PLATE, DOCK_MARKER_Z = 0.14, 0.31
DOCK_MARKER_DX, DOCK_MARKER_IDS = (-0.16, 0.0, 0.16), (1, 0, 2)
DOCK_PLATE_LAT, DOCK_PLATE_W, DOCK_PLATE_L = 0.055, 0.075, 0.10
DOCK_PLATE_Z, DOCK_PLATE_BACK = 0.04, 0.10
_pf = DOCK_WALL_Y + DOCK_PANEL_T
_pt = 0.008
_pcy = _pf - 0.002 + _pt / 2.0

w('    <model name="dock_station">')
w('      <static>true</static>')
w('      <link name="link">')
vis('dock_panel', f'{DOCK_X} {DOCK_WALL_Y + DOCK_PANEL_T/2} {DOCK_PANEL_H/2} 0 0 0',
    f'{DOCK_PANEL_W} {DOCK_PANEL_T} {DOCK_PANEL_H}', f'{TEX}/dock_body.png')
col('dock_panel_c', f'{DOCK_X} {DOCK_WALL_Y + DOCK_PANEL_T/2} {DOCK_PANEL_H/2} 0 0 0',
    f'{DOCK_PANEL_W} {DOCK_PANEL_T} {DOCK_PANEL_H}')
for _dx, _mid in zip(DOCK_MARKER_DX, DOCK_MARKER_IDS):
    vis(f'dock_marker_{_mid}', f'{DOCK_X + _dx} {_pcy} {DOCK_MARKER_Z} 0 0 0',
        f'{DOCK_PLATE} {_pt} {DOCK_PLATE}', f'{TEX}/dock_marker_{_mid}.png')
# 포고핀이 로봇 뒤쪽이고 후진으로 들어가므로 동판은 벽 쪽으로 들어갑니다.
# 포고핀이 로봇 뒤쪽이라 후진 도킹이면 동판이 벽 쪽으로 들어갑니다.
_py = DOCK_ROBOT_Y + (-DOCK_PLATE_BACK if REVERSE_DOCK else DOCK_PLATE_BACK)
for _k, _s in enumerate((-1, 1)):
    p = f'{DOCK_X + _s*DOCK_PLATE_LAT} {_py} {DOCK_PLATE_Z/2} 0 0 0'
    s = f'{DOCK_PLATE_W} {DOCK_PLATE_L} {DOCK_PLATE_Z}'
    w(f'      <visual name="dock_plate_{_k}"><pose>{p}</pose>'
      f'<geometry><box><size>{s}</size></box></geometry>'
      '<material><ambient>0.72 0.45 0.20 1</ambient>'
      '<diffuse>0.72 0.45 0.20 1</diffuse></material></visual>')
    col(f'dock_plate_{_k}_c', p, s)
w('      </link>')
w('    </model>')

w('  </world>')
w('</sdf>')

# ---- docking.yaml 대조 ----
_y = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      '..', '..', 'orinbot_navigation', 'config', 'docking.yaml'))
_chk = '확인 못 함'
try:
    m = re.search(r'home_dock:.*?pose:\s*\[([-0-9.]+),\s*([-0-9.]+)', open(_y).read(), re.S)
    if m:
        px, py = float(m.group(1)), float(m.group(2))
        if abs(px - DOCK_X) > 1e-6 or abs(py - DOCK_ROBOT_Y) > 1e-6:
            sys.stderr.write('ERROR: docking.yaml [%.3f, %.3f] != 월드 [%.3f, %.3f]\n'
                             % (px, py, DOCK_X, DOCK_ROBOT_Y))
            sys.exit(1)
        _chk = '일치 [%.2f, %.2f]' % (px, py)
except OSError as exc:                                    # noqa: BLE001
    _chk = '읽기 실패 (%s)' % exc

sys.stdout.write('\n'.join(out) + '\n')
e = sys.stderr.write
e('사무실 %.0f x %.0f m,  벽 높이 %.1f m,  벽은 저대비(office_*.png)\n'
  % (X1 - X0, Y1 - Y0, WALL_H))
e('복도 A y %.1f~%.1f (%.1f m), 복도 B y %.1f~%.1f (%.1f m), 문 %.2f m\n'
  % (COR_A[0], COR_A[1], COR_A[1] - COR_A[0],
     COR_B[0], COR_B[1], COR_B[1] - COR_B[0], DOOR))
e('방 %d 칸, 가구 %d 세트 (책상/의자/선반), 움직이는 사람 3명\n'
  % ((len(edges) - 1) * len(room_bands), nf))
e('필요 텍스처 %d 장 (office_000~%03d)\n' % (_tex, _tex - 1))
e('도크: docking.yaml 대조 %s\n' % _chk)
e('로봇 시작 권장: (0, -2.0) — 남쪽 로비, 도크 앞\n')
