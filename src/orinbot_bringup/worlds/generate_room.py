#!/usr/bin/env python3
"""room.sdf 생성기.

벽/바닥을 2 m 세그먼트로 쪼개서 텍스처가 늘어나지 않게 합니다
(512 px 텍스처를 8 m 벽 하나에 붙이면 64 px/m 라 흐려져서 특징점이
안 잡힙니다. 2 m 세그먼트면 256 px/m 로 충분히 선명합니다).
세그먼트가 60개 가까이 되어 손으로 쓰기에는 많아 스크립트로 만듭니다.

    python3 generate_room.py > room.sdf

월드 배치를 바꾸려면 이 파일의 상수/리스트를 고치고 다시 생성하세요.
생성 결과인 room.sdf 도 저장소에 함께 두므로, 평소에는 실행할 필요가
없습니다.
"""

import math
import sys

TEX = 'model://room_materials/materials/textures'

ROOM_X = 5.0          # 벽 중심까지의 거리 (전후)
ROOM_Y = 4.0          # 벽 중심까지의 거리 (좌우)
WALL_H = 1.2
WALL_T = 0.1
SEG = 2.0             # 세그먼트 길이

out = []


def w(s=''):
    out.append(s)


def pbr(texture, roughness=0.9):
    return f"""        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr>
            <metal>
              <albedo_map>{TEX}/{texture}</albedo_map>
              <metalness>0.0</metalness>
              <roughness>{roughness}</roughness>
            </metal>
          </pbr>
        </material>"""


def box_visual(name, pose, size, texture):
    w(f"""      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
{pbr(texture)}
      </visual>""")


def box_collision(name, pose, size):
    w(f"""      <collision name="{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>""")


def frange(a, b, step):
    v = a
    while v < b - 1e-9:
        yield v
        v += step


# ----------------------------------------------------------------------
w('<?xml version="1.0" ?>')
w('<!-- generate_room.py 로 생성됨. 직접 수정하지 말고 생성기를 고치세요. -->')
w('<sdf version="1.9">')
w('  <world name="room">')
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
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.82 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- 그림자로 인한 특징점 소실을 줄이려고 광원을 둘로 나눴습니다 -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.7 0.7 0.7 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 8 0 0 0</pose>
      <diffuse>0.35 0.35 0.4 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.5 -0.4 -0.8</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <!-- 시각 요소를 두지 않습니다.
             바닥 타일의 윗면이 정확히 z=0 이라 이 평면과 완전히 겹치고,
             그러면 z-파이팅이 생겨 카메라 영상에서 바닥이 얼룩덜룩하게
             흔들립니다 (컬러 영상에서 특히 뚜렷). 특징점도 오염됩니다.
             방은 벽으로 둘러싸여 있어 바깥은 보이지 않으므로
             충돌면만 남겨도 문제 없습니다. -->
      </link>
    </model>
""")

# ---------------- 바닥 타일 (시각용, 충돌 없음) ----------------
# 중요: 타일 윗면이 정확히 z=0 이어야 합니다.
# 로봇 바퀴는 ground_plane(z=0)에 닿는데 타일 중심을 +0.005 에 두면
# 윗면이 z=0.010 이 되어, 뎁스 카메라가 보는 "바닥"이 로봇이 굴러가는
# 면보다 1cm 높아집니다. 그러면 코스트맵의 min_obstacle_height 를
# 1cm 만큼 잘못 잡게 되고, 3cm 장애물이 2cm 로 보입니다 (실측 확인).
# 두께 0.01 짜리 박스이므로 중심은 -0.005 입니다.
w('    <model name="floor_tiles">')
w('      <static>true</static>')
w('      <link name="link">')
n = 0
for x in frange(-ROOM_X, ROOM_X, SEG):
    for y in frange(-ROOM_Y, ROOM_Y, SEG):
        cx, cy = x + SEG / 2, y + SEG / 2
        box_visual(f'tile_{n}', f'{cx} {cy} -0.005 0 0 0',
                   f'{SEG} {SEG} 0.01', 'floor.png')
        n += 1
w('      </link>')
w('    </model>')

# ---------------- 외벽 ----------------
w('    <model name="walls">')
w('      <static>true</static>')
w('      <link name="link">')
# 세그먼트마다 서로 다른 텍스처를 씁니다.
# 모든 벽이 똑같이 생기면 RTAB-Map 이 다른 장소를 같은 곳으로 착각해
# 잘못된 루프 클로저를 맺고 그래프가 망가집니다(실측: 자세 3m 이탈).
seg = 0
n = 0
# x = +-ROOM_X 벽 (y 방향으로 분할)
for sx in (ROOM_X, -ROOM_X):
    for y in frange(-ROOM_Y, ROOM_Y, SEG):
        cy = y + SEG / 2
        pose = f'{sx} {cy} {WALL_H/2} 0 0 0'
        size = f'{WALL_T} {SEG} {WALL_H}'
        box_visual(f'wall_x_{n}', pose, size, f'wall_{seg:02d}.png')
        box_collision(f'wall_x_col_{n}', pose, size)
        n += 1
        seg += 1
# y = +-ROOM_Y 벽 (x 방향으로 분할)
n = 0
for sy in (ROOM_Y, -ROOM_Y):
    for x in frange(-ROOM_X, ROOM_X, SEG):
        cx = x + SEG / 2
        pose = f'{cx} {sy} {WALL_H/2} 0 0 0'
        size = f'{SEG} {WALL_T} {WALL_H}'
        box_visual(f'wall_y_{n}', pose, size, f'wall_{seg:02d}.png')
        box_collision(f'wall_y_col_{n}', pose, size)
        n += 1
        seg += 1
w('      </link>')
w('    </model>')

# ---------------- 내부 칸막이 (루프 폐쇄 경로를 만들기 위함) ----------------
# 지그재그 경로가 생겨야 같은 장소로 되돌아오는 loop closure 를 시험할 수 있습니다.
PARTITIONS = [
    # (cx, cy, len_x, len_y)
    (-1.5, -2.5, WALL_T, 3.0),
    (1.5, 2.5, WALL_T, 3.0),
]
w('    <model name="partitions">')
w('      <static>true</static>')
w('      <link name="link">')
for i, (cx, cy, lx, ly) in enumerate(PARTITIONS):
    pose = f'{cx} {cy} {WALL_H/2} 0 0 0'
    size = f'{lx} {ly} {WALL_H}'
    box_visual(f'part_{i}', pose, size, f'wall_{seg + i:02d}.png')
    box_collision(f'part_col_{i}', pose, size)
w('      </link>')
w('    </model>')

# ---------------- 좁은 통로 뱅크 (통과 능력 검증용) ----------------
# 로봇은 폭 0.40 m 입니다. "충분히 들어갈 수 있는데 못 지나간다"는 현상을
# 재현·측정하려면 문 하나로는 부족해서, 폭을 바꿔가며 여러 개를 둡니다.
#
# 각 통로는 깊이 CORRIDOR_D 의 벽 사이를 지나가야 하므로 "얇은 문"이
# 아니라 짧은 복도입니다. 얇은 문은 한 순간만 정렬하면 되지만 복도는
# 그 길이 내내 정렬을 유지해야 해서 컨트롤러에 훨씬 어렵습니다.
#
# (x, y중심, 폭). x=-3.0 벽은 방을 둘로 가르는 칸막이라 항상 통과해야 하고,
# x=+3.2 벽은 시험 전용으로 더 좁은 폭들을 모아 둡니다.
DOOR_X = -3.0
DOOR_GAP = 0.80
CORRIDOR_D = 0.60          # 통로 깊이(=복도 길이)

# (라벨, x, 개구부 중심 y, 개구부 폭, 벽 깊이)
PASSAGES = [
    ('main',  DOOR_X,  0.0, DOOR_GAP, WALL_T),        # 기존 0.80 문 (얇음)
    ('c090',   3.2,    2.6, 0.90, CORRIDOR_D),
    ('c070',   3.2,    0.6, 0.70, CORRIDOR_D),
    ('c060',   3.2,   -3.0, 0.60, CORRIDOR_D),
    ('c055',   3.2,   -1.6, 0.55, CORRIDOR_D),
]


def passage_wall(label, x, gap_cy, gap_w, depth, y_lo, y_hi, tex_base):
    """y_lo~y_hi 구간에 gap 을 하나 남기고 벽을 세운다.

    개구부 양쪽 조각에 서로 다른 텍스처를 붙입니다. 같은 무늬를 쓰면
    떨어져 있는 두 벽면이 똑같이 보여 잘못된 루프 클로저가 생깁니다.
    """
    for k, (a, b) in enumerate(((y_lo, gap_cy - gap_w / 2),
                                (gap_cy + gap_w / 2, y_hi))):
        if b - a <= 0.01:
            continue
        cy, ln = (a + b) / 2, b - a
        pose = f'{x} {cy} {WALL_H/2} 0 0 0'
        size = f'{depth} {ln} {WALL_H}'
        box_visual(f'{label}_{k}', pose, size, f'wall_{tex_base + k:02d}.png')
        box_collision(f'{label}_col_{k}', pose, size)


w('    <model name="passages">')
w('      <static>true</static>')
w('      <link name="link">')
# x=-3.0 : 방을 가르는 벽, 개구부 하나
# 텍스처는 외벽(0~17)/칸막이(18~19) 와 겹치지 않는 번호를 씁니다
passage_wall('main', DOOR_X, 0.0, DOOR_GAP, WALL_T, -ROOM_Y, ROOM_Y, 26)
# x=+3.2 : 시험용. 개구부 3개를 남기고 나머지를 막는다.
# 개구부 구간을 y 순으로 정렬한 뒤 그 사이사이를 벽으로 채웁니다.
_gaps = sorted((cy - gw / 2, cy + gw / 2) for _, _x, cy, gw, _d in PASSAGES[1:])
_walls, _y = [], -ROOM_Y
for _a, _b in _gaps:
    if _a - _y > 0.01:
        _walls.append((_y, _a))
    _y = _b
if ROOM_Y - _y > 0.01:
    _walls.append((_y, ROOM_Y))
for _i, (_a, _b) in enumerate(_walls):
    _cy, _ln = (_a + _b) / 2, _b - _a
    _pose = f'3.2 {_cy} {WALL_H/2} 0 0 0'
    _size = f'{CORRIDOR_D} {_ln} {WALL_H}'
    box_visual(f'corr_{_i}', _pose, _size, f'wall_{21 + _i:02d}.png')
    box_collision(f'corr_col_{_i}', _pose, _size)
w('      </link>')
w('    </model>')

# ---------------- 포스터 (벽마다 다른 그림 -> 장소 식별에 유리) ----------------
# 카메라 높이(약 0.17 m)에서 잘 보이도록 낮게 붙입니다.
POSTERS = [
    # (x, y, yaw, texture)
    (ROOM_X - 0.06, -2.0, 0, 'poster1.png'),
    (ROOM_X - 0.06, 2.0, 0, 'poster2.png'),
    (-ROOM_X + 0.06, -1.0, 0, 'poster3.png'),
    (-ROOM_X + 0.06, 2.5, 0, 'poster4.png'),
    (-3.0, ROOM_Y - 0.06, 1.5708, 'poster5.png'),
    (3.0, ROOM_Y - 0.06, 1.5708, 'poster6.png'),
    (-2.0, -ROOM_Y + 0.06, 1.5708, 'poster7.png'),
    (2.5, -ROOM_Y + 0.06, 1.5708, 'poster8.png'),
]
w('    <model name="posters">')
w('      <static>true</static>')
w('      <link name="link">')
for i, (x, y, yaw, tex) in enumerate(POSTERS):
    box_visual(f'poster_{i}', f'{x} {y} 0.55 0 0 {yaw}', '0.02 0.8 0.6', tex)
w('      </link>')
w('    </model>')

# ---------------- 장애물 ----------------
# Nav2 경로계획 시험용 + 시각 랜드마크
CRATES = [
    # x=3.2 는 통로 시험용 벽이 지나가므로 비워 둡니다 (겹치면 충돌 형상이 깨짐)
    (4.5, 3.4, 0.25, 0.5),
    (-3.4, 1.6, 0.3, 0.6),
    (0.6, 3.0, 0.2, 0.4),
    (-0.5, -3.2, 0.25, 0.5),
]
for i, (x, y, h, s) in enumerate(CRATES):
    w(f"""    <model name="crate_{i}">
      <static>true</static>
      <pose>{x} {y} {h} 0 0 {0.3 * i}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{s} {s} {2*h}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{s} {s} {2*h}</size></box></geometry>
{pbr('crate.png')}
        </visual>
      </link>
    </model>""")

# ---------------- 낮은 장애물 (라이다 사각지대 검증용) ----------------
# 라이다 스캔 평면은 지면 0.49 m 입니다. 그보다 낮은 물체는 라이다가
# 전혀 보지 못하므로, 30도 아래를 보는 D435i 가 담당해야 합니다.
LOW_OBSTACLES = [
    # (x, y, 높이[m], 가로/세로[m], 색)
    (1.8, -0.6, 0.03, 0.30, (0.9, 0.6, 0.1)),   # 3cm 문턱
    (-1.0, 1.2, 0.05, 0.25, (0.9, 0.3, 0.6)),   # 5cm 상자
    (2.6, 1.4, 0.10, 0.20, (0.2, 0.8, 0.8)),    # 10cm 상자
]
for i, (x, y, h, sz, c) in enumerate(LOW_OBSTACLES):
    w(f"""    <model name="low_obstacle_{i}">
      <static>true</static>
      <pose>{x} {y} {h/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sz} {sz} {h}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sz} {sz} {h}</size></box></geometry>
          <material>
            <ambient>{c[0]} {c[1]} {c[2]} 1</ambient>
            <diffuse>{c[0]} {c[1]} {c[2]} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""")

# ---------------- 다층 선반 (센서 담당 구역 분리 검증용) ----------------
# 선반은 이 로봇의 두 센서가 "서로 다른 것을 본다"를 가장 잘 드러내는 구조입니다.
#
#   라이다 스캔 평면 : 지면 0.4908 m 의 얇은 원판. 이 높이를 지나는 것만 봅니다.
#   D435i           : 0.36 m 에서 15도 아래. 낮은 것과 가까운 것을 봅니다.
#
# 그래서 선반 판의 높이를 어디에 두느냐로 세 가지 상황을 만듭니다.
#
#   shelf_0 (판이 라이다 평면에 걸림) : 2D 라이다만으로도 벽처럼 보입니다.
#   shelf_1 (판이 라이다 평면을 비켜감): 라이다에는 얇은 기둥 4개만 찍혀
#       "거의 뚫린 곳"으로 보이지만, 실제로는 0.9 m 높이 판이 튀어나와 있어
#       로봇 몸통(높이 0.46 m)은 못 지나갑니다. STVL 3D 복셀 레이어가
#       없으면 Nav2 가 여기로 경로를 냅니다.
#   shelf_2 (옆판이 막힌 형태)        : 라이다는 옆판을 보고, 카메라는 선반
#       안쪽의 물건들을 봅니다. 특징점이 풍부해 루프 클로저에 유리합니다.
#
# (이름, x, y, yaw, 폭, 깊이, [판 중심 높이...], 다리굵기, 옆판)
SHELVES = [
    ('shelf_0', -4.55, -2.0, 0.0, 1.40, 0.35, [0.06, 0.49, 0.95, 1.40], 0.06, False),
    ('shelf_1',  2.40,  3.60, 1.5708, 1.40, 0.35, [0.06, 0.90, 1.45], 0.05, False),
    ('shelf_2',  4.55, -0.60, 0.0, 1.20, 0.35, [0.06, 0.55, 1.05, 1.45], 0.06, True),
]
BOARD_T = 0.04
# 선반에 올려 둘 물건 (선반별로 다른 색 -> 장소 식별에 유리)
SHELF_ITEMS = {
    'shelf_0': [(0, -0.45, 0.16, (0.85, 0.25, 0.2)), (1, 0.30, 0.13, (0.2, 0.5, 0.85)),
                (2, -0.15, 0.18, (0.9, 0.75, 0.2))],
    'shelf_1': [(0, 0.40, 0.15, (0.25, 0.7, 0.35)), (1, -0.35, 0.20, (0.7, 0.3, 0.75))],
    'shelf_2': [(1, -0.30, 0.14, (0.95, 0.55, 0.15)), (2, 0.25, 0.17, (0.15, 0.65, 0.7)),
                (3, 0.00, 0.12, (0.6, 0.6, 0.65))],
}

for name, sx, sy, yaw, wid, dep, boards, leg, sides in SHELVES:
    top = boards[-1] + BOARD_T / 2.0
    w(f"""    <model name="{name}">
      <static>true</static>
      <pose>{sx} {sy} 0 0 0 {yaw}</pose>
      <link name="link">""")
    # 다리 4개 (선반 폭/깊이 방향 모서리)
    for k, (dx, dy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        lx = dx * (wid / 2.0 - leg / 2.0)
        ly = dy * (dep / 2.0 - leg / 2.0)
        pose = f'{lx} {ly} {top/2.0} 0 0 0'
        size = f'{leg} {leg} {top}'
        box_visual(f'{name}_leg_{k}', pose, size, 'crate.png')
        box_collision(f'{name}_legc_{k}', pose, size)
    # 선반 판
    for k, bz in enumerate(boards):
        pose = f'0 0 {bz} 0 0 0'
        size = f'{wid} {dep} {BOARD_T}'
        box_visual(f'{name}_board_{k}', pose, size, 'crate.png')
        box_collision(f'{name}_boardc_{k}', pose, size)
    # 뒷판은 항상 둡니다 (벽에 붙여 두므로 뒤로 새는 것을 막습니다)
    back_pose = f'0 {-(dep/2.0 - 0.01)} {top/2.0} 0 0 0'
    back_size = f'{wid} 0.02 {top}'
    box_visual(f'{name}_back', back_pose, back_size, 'crate.png')
    box_collision(f'{name}_backc', back_pose, back_size)
    if sides:
        for k, dx in enumerate((1, -1)):
            pose = f'{dx * (wid/2.0 - 0.01)} 0 {top/2.0} 0 0 0'
            size = f'0.02 {dep} {top}'
            box_visual(f'{name}_side_{k}', pose, size, 'crate.png')
            box_collision(f'{name}_sidec_{k}', pose, size)
    # 올려 둔 물건 (충돌 포함 — 카메라가 봐야 할 3D 장애물입니다)
    for k, (bi, off, isz, c) in enumerate(SHELF_ITEMS.get(name, [])):
        iz = boards[bi] + BOARD_T / 2.0 + isz / 2.0
        w(f"""        <collision name="{name}_item_c{k}">
          <pose>{off} 0 {iz} 0 0 0</pose>
          <geometry><box><size>{isz} {isz} {isz}</size></box></geometry>
        </collision>
        <visual name="{name}_item_{k}">
          <pose>{off} 0 {iz} 0 0 0</pose>
          <geometry><box><size>{isz} {isz} {isz}</size></box></geometry>
          <material>
            <ambient>{c[0]} {c[1]} {c[2]} 1</ambient>
            <diffuse>{c[0]} {c[1]} {c[2]} 1</diffuse>
          </material>
        </visual>""")
    w('      </link>')
    w('    </model>')

PILLARS = [
    (2.0, 0.5, 0.25, 0.9, (0.8, 0.3, 0.2)),
    (-2.2, -0.8, 0.22, 0.8, (0.2, 0.6, 0.35)),
    (0.0, 1.8, 0.2, 0.7, (0.85, 0.7, 0.2)),
]
for i, (x, y, r, h, c) in enumerate(PILLARS):
    w(f"""    <model name="pillar_{i}">
      <static>true</static>
      <pose>{x} {y} {h/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material>
            <ambient>{c[0]} {c[1]} {c[2]} 1</ambient>
            <diffuse>{c[0]} {c[1]} {c[2]} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>""")

# ---------------- 충전 도킹 스테이션 ----------------
# 남쪽 벽에 붙입니다. 반경 1.5 m 안에 아무것도 없어 진입로가 넓고,
# 좁은 통로 뱅크(x=3.2)를 거치지 않고 갈 수 있는 자리입니다.
#
# 높이를 정한 근거 (카메라가 마커를 놓치는 거리를 최소화):
#   카메라는 지면 0.36 m 에서 15도 아래를 봅니다. 컬러 화각은
#   69.4 x 42.8 도이므로 시야는 수평 기준 +6.4도 ~ -36.4도 입니다.
#   마커(한 변 s=0.12) 중심을 높이 h 에 두면 마커가 화면 안에 완전히
#   들어오는 최소 거리는
#       위쪽 잘림: (h + s/2 - 0.36) / tan(6.4도)
#       아래쪽 잘림: (0.36 - h + s/2) / tan(36.4도)
#   둘 중 큰 값입니다. 두 식이 같아지는 h = 0.316 에서 최소가 되어
#   0.14 m 까지 보입니다. h 를 0.20 으로 낮추면 0.30 m 에서 이미
#   화면 아래로 잘려 나가므로, "낮은 도크가 자연스럽다"는 직관대로
#   두면 안 됩니다.
# 마커를 놓치는 마지막 구간은 docking_server 가 직전 검출값으로
# 밀고 들어갑니다 (docking.yaml 의 external_detection_timeout).
DOCK_X = 1.0
DOCK_WALL_Y = -ROOM_Y + WALL_T / 2.0    # 벽 안쪽면 = -3.95
DOCK_PANEL_T = 0.05                     # 백패널 두께
DOCK_PANEL_W = 0.52
DOCK_PANEL_H = 0.45
DOCK_PLATE = 0.14                       # 마커판 한 변 (텍스처 전체)
DOCK_MARKER = 0.10                      # 검은 사각형 한 변
DOCK_MARKER_Z = 0.31                    # 마커 중심 높이 (위 계산)
# 마커 3장의 좌우 배치 (도크 중심 기준, 월드 x). 왼쪽/가운데/오른쪽에
# 각각 id 1 / 0 / 2 를 붙입니다. 검출기(dock_marker_board.py)가 이 배치를
# 그대로 알고 있어야 하나의 보드로 풀 수 있습니다.
DOCK_MARKER_DX = (-0.16, 0.0, 0.16)
DOCK_MARKER_IDS = (1, 0, 2)

# --- 충전 접점 (바닥 동판 2장) ---
# 실물 구조: 스테이션 바닥에 +/- 동판 두 장, 로봇 배면(하부)에 6핀
# 포고핀 블록 두 개(좌우). 로봇이 그 위를 타고 들어가면 포고핀이
# 눌리며 접촉합니다.
#
# **동판을 바닥에 평평하게 깔 수 없습니다.** 로봇 접지고는 60 mm 인데
# (섀시 바닥 0.06 m), 코스트맵의 min_obstacle_height 가 0.03 이라 로봇은
# 30 mm 미만 물체를 장애물로 보지 않고 그냥 밟고 지나갑니다. 배면 브래킷을
# 30 mm 아래로 내리면 월드의 3 cm 문턱 같은 것에 그대로 부딪힙니다.
# 그래서 브래킷은 30~60 mm 사이(여기서는 45 mm)에 두고, 동판을 40 mm
# 높이로 올려 그 5 mm 를 포고핀 스트로크로 메웁니다.
#
# 좌우 위치: 구동륜이 ±0.17, 캐스터가 ±0.14 에 있으므로 바퀴가 지나가지
# 않는 띠는 |y| < 0.11 입니다. 동판은 그 안에 들어가야 합니다.
# 바퀴가 동판 위로 올라가면 로봇 전체가 같이 들려서 브래킷과의 간격이
# 그대로 유지되어 버립니다.
#
# 동판 규격 75 x 100 mm 는 **실물 부품 치수**입니다 (좌우 75, 앞뒤 100).
#
# 접촉 허용치는 동판에서 **브래킷이 아니라 핀 배열**을 뺀 값입니다.
# 실물 커넥터는 6핀 2x3 배열 / 피치 2.54 mm / male DIP 이므로,
# 전기적으로 닿아야 하는 면적은 브래킷(30 mm 각)이 아니라
#     가로 (3-1)*2.54 + 핀지름 = 6.1 mm
#     세로 (2-1)*2.54 + 핀지름 = 3.5 mm
# 뿐입니다. 브래킷은 이 커넥터를 붙드는 구조물일 뿐이라 동판 밖으로
# 걸쳐 있어도 무방합니다 (동판 상면 40 mm, 브래킷 하단 45 mm 로
# 애초에 닿지 않습니다).
# 따라서 (동판 - 핀배열)/2 로
#     세로 ±48 mm, 가로 ±34 mm.
#
# 30 mm 브래킷을 접촉면으로 잡았을 때(±35 / ±22.5 mm)보다 오히려
# 넓습니다. 접촉 판정을 브래킷 기준으로 두면 실제로는 닿아 있는데
# "충전 안 됨"이 되어, 있지도 않은 정렬 정확도를 요구하게 됩니다.
#
# 단락 걱정은 없습니다: 커넥터가 반대편 동판(안쪽 가장자리 17.5 mm)에
# 닿으려면 72.5 mm 를 밀려야 하는데 그 전에 자기 동판을 벗어납니다.
#
# 좌우 배치 확인: 중심 오프셋 ±0.055 에 반폭 0.0375 이므로 동판은
# |y| = 0.0175 ~ 0.0925 를 차지합니다. 바퀴가 지나가지 않는 띠
# |y| < 0.11 안입니다.
DOCK_PLATE_LAT = 0.055                  # 동판 중심의 좌우 오프셋
DOCK_PLATE_W = 0.075                    # 좌우 폭 (실물 규격)
DOCK_PLATE_L = 0.10                     # 앞뒤 길이 (실물 규격)
# 포고핀 커넥터 (6핀 2x3, 피치 2.54 mm, male DIP). 3핀 방향을 로봇
# 좌우로 놓습니다. 접촉 허용치는 여기서 유도됩니다.
DOCK_PIN_PITCH = 0.00254
DOCK_PIN_DIA = 0.001                    # 플런저 지름 (2.54 피치 표준)
DOCK_PIN_W = 2 * DOCK_PIN_PITCH + DOCK_PIN_DIA   # 좌우 (3핀)
DOCK_PIN_L = 1 * DOCK_PIN_PITCH + DOCK_PIN_DIA   # 앞뒤 (2핀)
DOCK_PLATE_Z = 0.04                     # 동판 윗면 높이
DOCK_PLATE_BACK = 0.10                  # 로봇 중심에서 뒤로 (= 브래킷 위치)
DOCK_BRACKET_Z = 0.045                  # 로봇 배면 브래킷 하단 (urdf 와 맞출 것)
DOCK_COPPER = (0.72, 0.45, 0.20)

# --- 가이드 벽은 두지 않습니다 ---
# 실물 스테이션에 가이드 벽이 없습니다. 그래서 각도를 잡아 줄 물리적
# 수단이 없고, 최종 자세는 전적으로 마커 인식 정확도가 결정합니다.
# 마커를 한 장이 아니라 세 장(보드)으로 둔 이유가 이것입니다.
#
# 코스트맵 쪽 이유도 있습니다. 로봇 내접 반경(0.20 m)의 2배보다 좁은
# 통로는 안쪽 전체가 INSCRIBED_INFLATED_OBSTACLE 로 칠해져 "충돌"로
# 읽힙니다. 즉 가이드 벽을 두면 그 구간에서 코스트맵 충돌 검사를
# 원리적으로 쓸 수 없습니다.

_panel_front = DOCK_WALL_Y + DOCK_PANEL_T               # -3.90

# 도킹 방향. docking.yaml 의 reverse_dock 과 **반드시 같아야 합니다** —
# 아래에서 동판 위치가 이 값에 따라 앞뒤로 바뀝니다.
#
# 후진 도킹: 정렬은 도크를 마주 본 채로 하고, 회전점에서 180도 돌아 뒤로
# 들어갑니다. 충전 내내 카메라가 벽이 아니라 방을 보므로 시각 오도메트리가
# 살아 있습니다 (벽 0.3 m 앞에서는 특징점이 없어 오도메트리가 깨지고, 그
# 오차가 포즈 그래프에 들어가 지도를 망칩니다).
#
# 도킹 완료 시 로봇 중심 (docking.yaml 의 home_dock.pose 와 같은 값):
#
#   패널 앞면 -3.900 / 마커면 -3.894 / 캐스터 스윕 반지름 0.198
#   도킹   y=-3.670 (마커면 0.224 m) — 섀시 뒷면이 패널에서 0.03 m
#   회전점 y=-3.344 (마커면 0.550 m) — 후진 거리 0.326 m
#
# 회전점을 정하는 것은 섀시가 아니라 **캐스터입니다.** 캐스터 구 반지름이
# 0.030 m 뿐이라 높이 0.040 m 인 동판 턱을 넘지 못하고 걸립니다.
# 위 값에서 캐스터 스윕(-3.542)과 동판 앞 모서리(-3.720) 사이가 178 mm.
REVERSE_DOCK = True
DOCK_ROBOT_Y = -3.67 if REVERSE_DOCK else -3.60
DOCK_CAM_REACH = 0.2125                 # orinbot.urdf.xacro: base_length/2 + 0.0125
# 마커판은 패널 앞면보다 2 mm 파묻습니다. 면을 정확히 맞추면 두 면이
# 같은 평면에 놓여 z-파이팅으로 카메라 영상에서 깜빡이고, 그 상태로는
# 마커 검출이 프레임마다 들쭉날쭉해집니다 (바닥 타일과 같은 이유).
_plate_t = 0.008
_plate_cy = _panel_front - 0.002 + _plate_t / 2.0

w(f"""    <model name="dock_station">
      <static>true</static>
      <link name="link">""")
# 백패널
box_visual('dock_panel',
           f'{DOCK_X} {DOCK_WALL_Y + DOCK_PANEL_T/2.0} {DOCK_PANEL_H/2.0} 0 0 0',
           f'{DOCK_PANEL_W} {DOCK_PANEL_T} {DOCK_PANEL_H}', 'dock_body.png')
box_collision('dock_panel_c',
              f'{DOCK_X} {DOCK_WALL_Y + DOCK_PANEL_T/2.0} {DOCK_PANEL_H/2.0} 0 0 0',
              f'{DOCK_PANEL_W} {DOCK_PANEL_T} {DOCK_PANEL_H}')
# 마커판 (충돌 없음 — 패널 안에 파묻혀 있어 의미가 없습니다)
for _dx, _mid in zip(DOCK_MARKER_DX, DOCK_MARKER_IDS):
    box_visual(f'dock_marker_{_mid}',
               f'{DOCK_X + _dx} {_plate_cy} {DOCK_MARKER_Z} 0 0 0',
               f'{DOCK_PLATE} {_plate_t} {DOCK_PLATE}',
               f'dock_marker_{_mid}.png')



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


# 충전 동판 2장 (바닥). 로봇 배면 브래킷이 이 위에 얹힙니다.
# 전기적 접촉은 흉내내지 않습니다 — battery_sim.py 가 기하로 판단합니다.
# 로봇이 도크를 마주 보므로 로봇 앞뒤축 = 월드 y, 로봇 좌우축 = 월드 x 입니다.
# 포고핀이 로봇 뒤쪽(x=-0.10)이고 후진으로 들어가므로 동판은 도킹 자세에서
# 로봇 중심보다 벽 쪽으로 0.10 m 들어갑니다.
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

# 손으로 옮겨 적어야 하는 값과 그 여유를 찍어 줍니다.
_marker_front = _plate_cy + _plate_t / 2.0
_cam_front = DOCK_ROBOT_Y - DOCK_CAM_REACH
sys.stderr.write(
    'docking.yaml home_dock.pose: [%.3f, %.3f, -1.5708]\n'
    '  카메라 최전단 y=%.4f, 마커판까지 여유 %+.1f mm\n'
    '  동판 상면 %.0f mm / 브래킷 하단 %.0f mm -> 포고 필요 스트로크 %.0f mm\n'
    '  핀 배열 %.1f x %.1f mm -> 접촉 허용치 세로 ±%.1f mm, 가로 ±%.1f mm\n'
    % (DOCK_X, DOCK_ROBOT_Y, _cam_front, (_cam_front - _marker_front) * 1000,
       DOCK_PLATE_Z * 1000, DOCK_BRACKET_Z * 1000,
       (DOCK_BRACKET_Z - DOCK_PLATE_Z) * 1000,
       DOCK_PIN_W * 1000, DOCK_PIN_L * 1000,
       (DOCK_PLATE_L - DOCK_PIN_L) / 2.0 * 1000,
       (DOCK_PLATE_W - DOCK_PIN_W) / 2.0 * 1000))

# 도크 자체는 라이다 스캔 평면(0.4908 m)보다 낮아 2D 라이다에 잡히지
# 않습니다. 하지만 벽면에서 0.115 m 밖에 안 나오는데 벽은 1.2 m 라
# 라이다에 잡히고 팽창 반경이 0.40 m 이므로, 경로가 도크를 뚫고 지나갈
# 일은 없습니다. 도크 형상 자체는 카메라(STVL / RTAB-Map 격자)가 봅니다.

w('  </world>')
w('</sdf>')

sys.stdout.write('\n'.join(out) + '\n')
