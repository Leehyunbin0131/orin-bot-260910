#!/usr/bin/env python3
"""도킹을 여러 초기 조건으로 **동시에** 시험하고 표로 모은다.

    python3 tools/dock_bench.py                 # 기본 케이스 묶음
    python3 tools/dock_bench.py --jobs 4        # 동시 인스턴스 수
    python3 tools/dock_bench.py --repeat 3      # 케이스당 반복

왜 이렇게 하나
--------------
도킹 파라미터 하나 바꿀 때마다 스택을 3분씩 올렸다 내리면 하루에 몇 번
못 돕니다. 아이작심처럼 한 프로세스 안에서 환경을 벡터화하는 것은
Gazebo 로 안 되지만, **독립 인스턴스를 격리해 N개 띄우는 것**은 됩니다.

격리는 두 겹입니다:
    ROS_DOMAIN_ID   ROS 2 디스커버리
    GZ_PARTITION    gz-transport
실측: 두 인스턴스를 띄웠을 때 각 도메인이 /clock 발행자를 1개씩만 봅니다
(섞였다면 2개). 인스턴스당 약 1 코어 / 1 GB.

시험대는 SLAM 과 Nav2 를 띄우지 않습니다 (dock_bench.launch.py 참고).
그래서 `_goto_entry` 대신 **로봇을 정렬 지점 근처에 바로 스폰**하고,
케이스마다 그 스폰 자세를 흔들어 초기 오차를 만듭니다.

판정 기준은 동판 접촉 허용치입니다: 세로 ±48 mm / 가로 ±34 mm / 각도 ±6.3도.
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DOCK = (1.0, -3.67, 1.5708)          # 도킹 완료 자세 (후진 도킹이라 벽을 등짐)
TOL_LON, TOL_LAT, TOL_YAW = 0.048, 0.034, math.radians(6.3)
# 정렬을 시작할 기준 자세. 마커면(-3.894)에서 약 0.79 m.
BASE = (1.0, -3.10, -1.5708)

# (이름, 스폰 dx[m], dy[m], dyaw[도], 파라미터 덮어쓰기)
CASES = [
    ('기준',            0.00,  0.00,  0.0, {}),
    ('가로 +5cm',       0.05,  0.00,  0.0, {}),
    ('가로 -5cm',      -0.05,  0.00,  0.0, {}),
    ('각도 +8도',       0.00,  0.00,  8.0, {}),
    ('각도 -8도',       0.00,  0.00, -8.0, {}),
    ('멀리 +10cm',      0.00,  0.10,  0.0, {}),
    ('회전점 0.70',     0.00,  0.00,  0.0, {'rotate_distance': 0.70}),
    ('예산 15mm',       0.00,  0.00,  0.0, {'contact_lateral_budget': 0.015}),
]


def env_for(idx):
    e = dict(os.environ)
    e['ROS_DOMAIN_ID'] = str(40 + idx)
    e['GZ_PARTITION'] = 'bench%d' % idx
    e['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    return e


def overlay(base_yaml, overrides, workdir):
    """staged_dock 파라미터만 덮어쓴 임시 yaml 을 만든다."""
    if not overrides:
        return base_yaml
    txt = open(base_yaml, encoding='utf-8').read()
    for k, v in overrides.items():
        # "    key: value" 형태만 바꿉니다 (주석 줄은 건드리지 않음).
        out, done = [], False
        for line in txt.split('\n'):
            s = line.strip()
            if not done and s.startswith(k + ':') and not s.startswith('#'):
                out.append('%s%s: %s' % (line[:len(line) - len(line.lstrip())], k, v))
                done = True
            else:
                out.append(line)
        if not done:
            raise SystemExit('파라미터 %s 를 %s 에서 찾지 못했습니다' % (k, base_yaml))
        txt = '\n'.join(out)
    path = os.path.join(workdir, 'docking.yaml')
    open(path, 'w', encoding='utf-8').write(txt)
    return path


# 프로브는 별도 프로세스로 돌립니다 — 인스턴스마다 ROS_DOMAIN_ID 가 달라
# 한 파이썬 프로세스에서 여러 도메인을 동시에 볼 수 없기 때문입니다.
# 도크 좌표는 아래에서 한 줄로 앞에 붙입니다 (본문에 % 서식이 많아
# 문자열 포매팅을 쓰면 충돌합니다).
PROBE_BODY = r'''
import json, math, sys, time, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import DockRobot
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
rclpy.init(); n = Node('bench')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
s = {'o': None, 'm': 0}
n.create_subscription(Odometry, '/ground_truth/odom',
                      lambda m: s.__setitem__('o', m), 10)
n.create_subscription(PoseStamped, '/detected_dock_pose',
                      lambda m: s.__setitem__('m', s['m'] + 1), 10)
ac = ActionClient(n, DockRobot, 'dock_robot')
out = {'ok': False, 'why': ''}
try:
    if not ac.wait_for_server(timeout_sec=120):
        out['why'] = 'dock_robot 액션 서버 없음'; raise SystemExit
    t0 = time.time()
    while s['m'] == 0 and time.time() - t0 < 60:
        rclpy.spin_once(n, timeout_sec=0.2)
    if s['m'] == 0:
        out['why'] = '마커 미검출 (스폰 위치에서 도크가 안 보임)'; raise SystemExit
    g = DockRobot.Goal(); g.use_dock_id = True; g.dock_id = 'home_dock'
    g.navigate_to_staging_pose = False      # Nav2 가 없으므로 스테이징 생략
    t0 = time.time()
    f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
    gh = f.result()
    if gh is None or not gh.accepted:
        out['why'] = '목표 거절'; raise SystemExit
    rf = gh.get_result_async()
    while not rf.done() and time.time() - t0 < 300:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not rf.done():
        out['why'] = '시간 초과'; raise SystemExit
    res = rf.result()
    out['secs'] = time.time() - t0
    for _ in range(40): rclpy.spin_once(n, timeout_sec=0.05)
    p = s['o'].pose.pose.position; q = s['o'].pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    dx, dy = p.x-DX, p.y-DY
    c, sn = math.cos(DYAW), math.sin(DYAW)
    out['lon'] = dx*c + dy*sn
    out['lat'] = -dx*sn + dy*c
    out['yaw'] = math.atan2(math.sin(yaw-DYAW), math.cos(yaw-DYAW))
    out['status'] = res.status
    out['ok'] = res.status == 4
    if not out['ok']:
        out['why'] = 'error_code %d' % res.result.error_code
except SystemExit:
    pass
finally:
    print('BENCH' + json.dumps(out))
    rclpy.shutdown()
'''

PROBE = 'DX, DY, DYAW = %.6f, %.6f, %.6f\n' % DOCK + PROBE_BODY


def run_case(idx, name, dx, dy, dyaw, overrides, repeat):
    env = env_for(idx)
    work = tempfile.mkdtemp(prefix='dockbench%d_' % idx)
    results = []
    try:
        yml = overlay(os.path.join(WS, 'src/orinbot_navigation/config/docking.yaml'),
                      overrides, work)
        for _ in range(repeat):
            x, y = BASE[0] + dx, BASE[1] + dy
            yaw = BASE[2] + math.radians(dyaw)
            log = open(os.path.join(work, 'launch.log'), 'w')
            proc = subprocess.Popen(
                ['ros2', 'launch', 'orinbot_navigation', 'dock_bench.launch.py',
                 'params_file:=' + yml,
                 'x:=%.4f' % x, 'y:=%.4f' % y, 'yaw:=%.4f' % yaw],
                cwd=WS, env=env, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            try:
                probe = subprocess.run(
                    [sys.executable, '-c', PROBE], cwd=WS, env=env,
                    capture_output=True, text=True, timeout=480)
                line = next((l for l in probe.stdout.split('\n')
                             if l.startswith('BENCH')), None)
                results.append(json.loads(line[5:]) if line
                               else {'ok': False, 'why': '프로브 출력 없음'})
            except subprocess.TimeoutExpired:
                results.append({'ok': False, 'why': '프로브 시간 초과'})
            finally:
                log.close()
                try:
                    os.killpg(proc.pid, 9)
                except Exception:                             # noqa: BLE001
                    pass
                proc.wait(timeout=30)
                time.sleep(3)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return name, overrides, results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--jobs', type=int, default=4, help='동시 인스턴스 수')
    ap.add_argument('--repeat', type=int, default=1, help='케이스당 반복')
    a = ap.parse_args()

    print('케이스 %d개 x %d회, 동시 %d개' % (len(CASES), a.repeat, a.jobs))
    print('인스턴스당 약 1 코어 / 1 GB — nproc %d, 여유 메모리를 확인하세요\n'
          % os.cpu_count())
    t0 = time.time()
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        futs = [ex.submit(run_case, i, *c, a.repeat) for i, c in enumerate(CASES)]
        rows = [f.result() for f in futs]

    print('%-14s %7s %9s %9s %8s %7s  %s'
          % ('케이스', '성공', '세로mm', '가로mm', '각도도', '초', '비고'))
    print('-' * 78)
    for name, ov, rs in rows:
        good = [r for r in rs if r.get('ok')]
        if not good:
            why = rs[0].get('why', '?') if rs else '?'
            print('%-14s %7s %9s %9s %8s %7s  %s'
                  % (name, '0/%d' % len(rs), '-', '-', '-', '-', why))
            continue
        f = lambda k: sum(r[k] for r in good) / len(good)      # noqa: E731
        print('%-14s %7s %9.1f %9.1f %8.2f %7.0f  %s'
              % (name, '%d/%d' % (len(good), len(rs)),
                 f('lon') * 1000, f('lat') * 1000,
                 math.degrees(f('yaw')), f('secs'),
                 ' '.join('%s=%s' % kv for kv in ov.items())))
    print('\n허용치: 세로 ±%.0f mm / 가로 ±%.0f mm / 각도 ±%.1f도'
          % (TOL_LON * 1000, TOL_LAT * 1000, math.degrees(TOL_YAW)))
    print('총 %.0f 초' % (time.time() - t0))


if __name__ == '__main__':
    main()
