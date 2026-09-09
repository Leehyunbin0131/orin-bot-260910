#!/usr/bin/env python3
"""Room editor, local speech UI, and gated NavigateToPose integration for the real robot."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped,TwistStamped
from nav_msgs.msg import Odometry,Path as NavPath
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
import yaml
from guide_core import SavedMap,atomic_json,resolve_room
from guide_stt import LocalSTT
from drive_rectangle import bounded_twist
from mapping_service import MappingService
from camera_preview import CameraPreview, serve_camera


class GuideService(Node):
    fresh_stamp=MappingService.fresh_stamp
    def input_problem(self):
        issue=MappingService.input_problem(self)
        # A newly arrived scan can precede the next wheel TF by one cycle.
        if issue.startswith('TF odom'):
            try:
                transform=self.tf.lookup_transform('odom','laser',Time())
                age=(self.get_clock().now().nanoseconds-(transform.header.stamp.sec*10**9+transform.header.stamp.nanosec))/1e9
                if -.1<=age<=.2:return ''
            except TransformException:pass
        return issue

    def __init__(self):
        super().__init__('guide_service')
        self.share=Path(get_package_share_directory('orinbot_hardware'))
        self.drive=self.declare_parameter('drive',False).value
        self.maps_root=Path(self.declare_parameter('maps_directory',str(Path.home()/'ros2_ws/maps')).value).expanduser().resolve()
        self.settings_path=Path(self.declare_parameter('settings_file',str(Path.home()/'.local/share/orinbot/guide.json')).value).expanduser()
        self.maps_root.mkdir(parents=True,exist_ok=True)
        self.lock=threading.RLock();self.operation=threading.Lock();self.closing=threading.Event()
        self.selected=None;self.map_id=None;self.process=None;self.process_log=None
        self.state='idle';self.message='저장된 맵을 선택하세요.';self.goal=None;self.generation=0;self.intent=False
        self.final_state='cancelled';self.stable=None;self.stop_started=0.;self.last_voice='';self.target_room=None
        self.scan=self.odom=self.amcl=None;self.last_scan=self.last_odom=0.;self.initialized=False
        self.last_initial=0;self.last_nav=0.;self.nav_command=(0.,0.);self.path=[];self.feedback={}
        self.initial_started=0.;self.last_amcl=0.;self.amcl_updates=0
        self.nav_start_future=None;self.nav_start_error=''
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.group=ReentrantCallbackGroup()
        self.camera_preview=CameraPreview(self)
        self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose',callback_group=self.group)
        self.lifecycle={};self.lifecycle_pending={};self.navigation_fault=False
        self.nomotion=self.create_client(Empty,'/request_nomotion_update',callback_group=self.group);self.nomotion_future=None
        self.lifecycle_clients={name:self.create_client(GetState,'/'+name+'/get_state',callback_group=self.group) for name in ('map_server','amcl','planner_server','controller_server','bt_navigator')}
        self.nav_start=self.create_client(ManageLifecycleNodes,'/guide_lifecycle/manage_nodes',callback_group=self.group)
        self.create_timer(.5,self.poll_lifecycle)
        self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.velocity_pub=self.create_publisher(TwistStamped,'/cmd_vel',10) if self.drive else None
        self.create_subscription(LaserScan,'/scan',self.scan_callback,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl_callback,10)
        self.create_subscription(TwistStamped,'/guide/nav_cmd_vel',self.nav_callback,10)
        self.create_subscription(NavPath,'/plan',self.path_callback,10)
        settings=yaml.safe_load((self.share/'config/hardware.yaml').read_text())
        self.radius,self.track=settings['wheel_radius'],settings['wheel_separation'];self.wheel_limit=math.floor(60/.229)*.0239691227
        self.stt=LocalSTT();self.create_timer(.05,self.tick)
        self.http=start_http(self,self.declare_parameter('gui_port',8080).value)
        self.get_logger().info(f'Guide GUI: http://127.0.0.1:{self.http.server_port}; drive={self.drive}')
        self.log_dir=Path.home()/'ros2_ws/log/guide'/datetime.now().strftime('%Y%m%d_%H%M%S_%f');self.log_dir.mkdir(parents=True)
        self.events=(self.log_dir/'events.jsonl').open('w')
        if self.settings_path.is_file():
            try:
                saved=json.loads(self.settings_path.read_text());self.load_map(saved['map_id'])
            except Exception as exc:self.message=f'맵 자동 로드 실패: {exc}'

    def event(self,kind,**values):
        self.events.write(json.dumps({'time':time.time(),'event':kind,**values},ensure_ascii=False,allow_nan=False)+'\n');self.events.flush()

    def catalog(self):
        return [str(p.relative_to(self.maps_root)) for p in sorted(self.maps_root.rglob('map.yaml'))
                if not (p.parent/'.incomplete').exists() and p.resolve().is_relative_to(self.maps_root)]

    def scan_callback(self,msg):self.scan,self.last_scan=msg,time.monotonic()
    def odom_callback(self,msg):self.odom,self.last_odom=msg,time.monotonic()
    def path_callback(self,msg):self.path=[[p.pose.position.x,p.pose.position.y] for p in msg.poses] if msg.header.frame_id=='map' else []
    def amcl_callback(self,msg):
        with self.lock:
            stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            if msg.header.frame_id!='map' or not self.last_initial or stamp<self.last_initial:return
            first=not self.initialized
            self.amcl=msg;self.last_amcl=time.monotonic();self.amcl_updates+=1;self.initialized=True
            if first:
                self.event('localization_received',elapsed=time.monotonic()-self.initial_started)
    def nav_callback(self,msg):
        linear,angular=msg.twist.linear.x,msg.twist.angular.z
        if all(math.isfinite(v) for v in (linear,angular)) and self.fresh_stamp(msg.header.stamp):
            self.nav_command=(linear,angular);self.last_nav=time.monotonic()

    def send(self,v=0.,w=0.):
        if not self.velocity_pub:return
        linear_limit=self.wheel_limit*self.radius
        angular_limit=2*linear_limit/self.track
        v,w=bounded_twist(max(0.,min(linear_limit,v)),max(-angular_limit,min(angular_limit,w)),self.radius,self.track,self.wheel_limit)
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_footprint'
        msg.twist.linear.x,msg.twist.angular.z=v,w;self.velocity_pub.publish(msg)

    def stationary(self):
        return self.odom and time.monotonic()-self.last_odom<.5 and abs(self.odom.twist.twist.linear.x)<.001 and abs(self.odom.twist.twist.angular.z)<.005

    def poll_lifecycle(self):
        if not self.process:return
        # Force a fresh post-initialization scan update even when the robot is stationary.
        if self.last_initial and not self.initialized and self.nomotion.service_is_ready():
            if self.nomotion_future is None or self.nomotion_future.done():
                self.nomotion_future=self.nomotion.call_async(Empty.Request())
        for name,client in self.lifecycle_clients.items():
            previous=self.lifecycle_pending.get(name)
            if previous is not None and not previous.done():continue
            if not client.service_is_ready():
                self.lifecycle[name]=(0,time.monotonic());continue
            future=client.call_async(GetState.Request());self.lifecycle_pending[name]=future
            def done(result,name=name):
                try:self.lifecycle[name]=(result.result().current_state.id,time.monotonic())
                except Exception:self.lifecycle[name]=(0,time.monotonic())
            future.add_done_callback(done)
        self.start_navigation_if_localized()

    def start_navigation_if_localized(self):
        with self.lock:
            if (not self.drive or not self.process or self.process.poll() is not None
                    or self.navigation_fault or self.nav_start_future is not None or self.nav_start_error):return
            if self.localization_problem() or not self.nav_start.service_is_ready():return
            request=ManageLifecycleNodes.Request();request.command=ManageLifecycleNodes.Request.STARTUP
            self.nav_start_future=self.nav_start.call_async(request)
            def started(future):
                with self.lock:
                    if future is not self.nav_start_future:return  # A different map was loaded.
                    try:
                        if not future.result().success:raise RuntimeError('Nav2 활성화 실패')
                        self.event('navigation_stack_started')
                    except Exception as exc:
                        self.nav_start_error=f'주행 서버를 시작하지 못했습니다: {exc}. 맵을 다시 불러와 주세요.'
            self.nav_start_future.add_done_callback(started)

    def localization_problem(self):
        for name in ('map_server','amcl'):
            state,updated=self.lifecycle.get(name,(0,0))
            if state!=3 or time.monotonic()-updated>2:return '맵과 라이다 위치 추정기를 준비하고 있습니다.'
        issue=self.input_problem()
        if issue:return issue
        if not self.last_initial:return '지도에서 로봇의 현재 위치와 방향을 지정하세요. 위치를 지정할 때까지 정지 상태로 기다립니다.'
        if not self.initialized or self.amcl is None:
            elapsed=int(time.monotonic()-self.initial_started)
            hint=' 10초 넘게 결과가 없습니다. 라이다 수신 상태와 지정한 위치·방향을 확인하세요.' if elapsed>=10 else ''
            return f'초기 위치 지정 후 라이다 위치 추정 결과 대기 중 · {elapsed}초.{hint}'
        if any(not math.isfinite(self.amcl.pose.covariance[i]) or not 0<=self.amcl.pose.covariance[i]<=.25 for i in (0,7,35)):
            return '위치 추정 오차가 큽니다. 현재 위치와 로봇 전방 방향을 확인하고 다시 지정하세요.'
        try:
            t=self.tf.lookup_transform('map','base_footprint',Time())
            if not self.fresh_stamp(t.header.stamp):return '로봇 위치 정보가 오래됐습니다.'
        except TransformException:return '라이다 결과는 수신했지만 맵 기준 로봇 위치 연결을 기다리고 있습니다.'
        return ''

    def ready_problem(self):
        if not self.drive:return '호실 편집 모드입니다. 실제 주행은 drive:=true로 실행하세요.'
        if self.selected is None:return '맵을 선택하세요.'
        if self.navigation_fault:return '주행 오류 후에는 맵을 다시 불러오고 현재 위치를 지정하세요.'
        if not self.process or self.process.poll() is not None:return 'Nav2가 실행되지 않았습니다.'
        issue=self.localization_problem()
        if issue:return issue
        if self.nav_start_error:return self.nav_start_error
        waiting=[name for name in ('planner_server','controller_server','bt_navigator')
                 if self.lifecycle.get(name,(0,0))[0]!=3 or time.monotonic()-self.lifecycle.get(name,(0,0))[1]>2]
        if waiting:return '라이다 위치 추정 확인 · 주행 서버 준비 중: '+', '.join(waiting)
        if self.count_publishers('/cmd_vel')!=1:return '다른 주행 프로그램이 켜져 있습니다. 종료하세요.'
        if not self.action.server_is_ready():return 'Nav2 준비 중입니다.'
        return ''

    def stop_nav_process(self):
        p=self.process
        if p:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try:p.wait(timeout=5)
                except subprocess.TimeoutExpired:pass
            try:os.killpg(p.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:p.wait(timeout=2)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=2)
        self.process=None
        if self.process_log:self.process_log.close();self.process_log=None

    def load_map(self,map_id):
        if self.closing.is_set():raise ValueError('프로그램 종료 중입니다.')
        if map_id not in self.catalog():raise ValueError('저장된 지도 목록에서 선택하세요.')
        if not self.operation.acquire(blocking=False):raise ValueError('다른 작업이 진행 중입니다.')
        try:
            with self.lock:
                if self.state in ('navigating','starting_goal','stopping'):raise ValueError('이동을 중단하고 정지를 확인한 뒤 맵을 변경하세요.')
                if self.drive and self.odom is not None and not self.stationary():raise ValueError('로봇을 정지하세요.')
                candidate=SavedMap(self.maps_root/map_id)
                if self.drive and not self.process and (self.count_publishers('/map') or any(n in ('amcl','slam_toolbox','bt_navigator') for n,_ in self.get_node_names_and_namespaces())):
                    raise ValueError('다른 SLAM·Nav2가 실행 중입니다. 먼저 종료하세요.')
                self.stop_nav_process();self.lifecycle.clear();self.navigation_fault=False;self.generation+=1;self.selected=candidate;self.map_id=map_id;self.initialized=False;self.last_initial=0;self.amcl=None;self.path=[]
                self.initial_started=0.;self.last_amcl=0.;self.amcl_updates=0
                self.nav_start_future=None;self.nav_start_error=''
                atomic_json(self.settings_path,{'map_id':map_id})
                self.state='ready';self.message='지도를 불러왔습니다. 호실을 등록하거나 로봇 위치를 지정하세요.'
                if self.drive:
                    self.process_log=(self.log_dir/'nav2.log').open('a')
                    self.process=subprocess.Popen(['ros2','launch','orinbot_hardware','guide_nav.launch.py',f'map:={candidate.path}'],stdout=self.process_log,stderr=subprocess.STDOUT,start_new_session=True)
                self.event('map_loaded',map_id=map_id)
        finally:self.operation.release()

    def set_initial(self,x,y,yaw):
        with self.lock:
            if not self.drive or not self.selected:raise ValueError('주행 모드에서 맵을 먼저 선택하세요.')
            if self.state in ('navigating','starting_goal','stopping'):raise ValueError('주행을 중단한 뒤 위치를 지정하세요.')
            if self.input_problem() or not self.stationary():raise ValueError('센서 준비와 로봇 정지를 확인하세요.')
            self.selected.validate_pose(x,y,yaw)
            if self.lifecycle.get('amcl',(0,0))[0]!=3 or not self.initial_pub.get_subscription_count():raise ValueError('AMCL 준비 중입니다. 잠시 후 다시 지정하세요.')
            msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
            msg.pose.pose.position.x=float(x);msg.pose.pose.position.y=float(y)
            msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
            msg.pose.covariance[0]=msg.pose.covariance[7]=.04;msg.pose.covariance[35]=.04
            self.last_initial=self.get_clock().now().nanoseconds;self.initialized=False;self.amcl=None
            self.initial_started=time.monotonic();self.last_amcl=0.;self.amcl_updates=0;self.initial_pub.publish(msg)
            self.event('initial_pose_requested',x=x,y=y,yaw=yaw)
            self.message='현재 위치를 지정했습니다. 라이다 위치 추정을 기다립니다.'

    def navigate(self,room_name):
        with self.lock:
            if self.closing.is_set():raise ValueError('프로그램 종료 중입니다.')
            if self.state in ('navigating','starting_goal','stopping'):raise ValueError('이미 안내 중입니다. 먼저 중단하거나 도착을 기다려 주세요.')
            issue=self.ready_problem()
            if issue:raise ValueError(issue)
            if not self.stationary():raise ValueError('로봇이 정지한 뒤 요청하세요.')
            room=next((r for r in self.selected.rooms if r['name']==room_name),None)
            if room is None:raise ValueError('등록되지 않은 호실입니다.')
            self.selected.validate_pose(room['x'],room['y'],room['yaw'])
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=self.get_clock().now().to_msg()
            goal.pose.pose.position.x=room['x'];goal.pose.pose.position.y=room['y']
            goal.pose.pose.orientation.z=math.sin(room['yaw']/2);goal.pose.pose.orientation.w=math.cos(room['yaw']/2)
            self.generation+=1;generation=self.generation;self.state='starting_goal';self.intent=False;self.target_room=room['name'];self.goal=None
            self.deadline=time.monotonic()+900;self.message=f'{room_name} 안내를 요청했습니다.';self.feedback={};self.last_nav=0
            future=self.action.send_goal_async(goal,feedback_callback=self.on_feedback)
            def accepted(done):
                with self.lock:
                    try:handle=done.result()
                    except Exception as exc:
                        if generation==self.generation:self.halt('failed',str(exc))
                        return
                    if generation!=self.generation:
                        if handle.accepted:handle.cancel_goal_async()
                        return
                    if not handle.accepted:self.halt('failed','Nav2가 목적지를 거절했습니다.');return
                    self.goal=handle;self.intent=True;self.state='navigating';self.message=f'{room_name}로 안내 중입니다.'
                    handle.get_result_async().add_done_callback(lambda result:self.on_result(result,generation))
            future.add_done_callback(accepted);self.event('goal_requested',room=room)
            return room

    def on_feedback(self,feedback):
        self.feedback={'distance_remaining':float(feedback.feedback.distance_remaining)}

    def on_result(self,future,generation):
        with self.lock:
            if generation!=self.generation:return
            try:
                result=future.result();status=result.status;detail=result.result.error_msg
            except Exception as exc:status=GoalStatus.STATUS_ABORTED;detail=str(exc)
            outcome='arrived' if status==GoalStatus.STATUS_SUCCEEDED else 'cancelled' if status==GoalStatus.STATUS_CANCELED else 'failed'
            message='목적지 도착. 정지를 확인합니다.' if outcome=='arrived' else '안내가 종료되었습니다. 정지를 확인합니다.'
            if outcome=='failed':message=self.input_problem() or detail or 'Nav2가 주행을 완료하지 못했습니다.'
            self.halt(outcome,message)

    def halt(self,outcome='cancelled',message='안내 중단 요청. 정지를 확인합니다.'):
        with self.lock:
            self.intent=False;self.generation+=1
            if outcome=='failed':self.navigation_fault=True
            if self.goal:
                self.goal.cancel_goal_async();self.goal=None
            self.send();self.state='stopping' if self.drive else outcome;self.final_state=outcome;self.message=message
            self.stop_started=time.monotonic();self.stable=None

    def tick(self):
        with self.lock:
            if self.state in ('navigating','starting_goal'):
                issue=self.ready_problem()
                if issue or time.monotonic()>self.deadline:
                    self.halt('failed',issue or '안내 시간 제한을 초과했습니다.');return
            if self.intent and self.state=='navigating':
                if time.monotonic()-self.last_nav<.25:self.send(*self.nav_command)
                else:self.send()
            elif self.drive:self.send()
            if self.state=='stopping':
                if self.stationary():
                    self.stable=self.stable or time.monotonic()
                    if time.monotonic()-self.stable>.4:
                        self.state=self.final_state
                        if self.state=='arrived':self.message=f'{self.target_room}에 도착했습니다.'
                        elif self.state=='cancelled':self.message='안내를 중단했고 정지를 확인했습니다.'
                        self.event('navigation_finished',state=self.state,message=self.message)
                else:self.stable=None
                if time.monotonic()-self.stop_started>6:
                    self.state='failed';self.message='정지 명령을 보냈으나 정지 피드백을 확인하지 못했습니다.'

    def snapshot(self):
        with self.lock:
            robot=None
            try:
                t=self.tf.lookup_transform('map','base_footprint',Time()).transform;q=t.rotation
                robot=[t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
            except TransformException:pass
            issue=self.ready_problem()
            message=self.message
            if self.drive and self.state=='ready':message=issue or '위치 추정과 주행 준비가 완료됐습니다. 목적지를 요청하면 출발합니다.'
            now=time.monotonic()
            def age(last):return round(max(0.,now-last),1) if last else None
            uncertainty=None
            if self.amcl is not None:
                values=[self.amcl.pose.covariance[i] for i in (0,7,35)]
                if all(math.isfinite(v) and v>=0 for v in values):
                    uncertainty={'x_m':math.sqrt(values[0]),'y_m':math.sqrt(values[1]),'yaw_deg':math.degrees(math.sqrt(values[2]))}
            localization={'initial_set':bool(self.last_initial),'received':self.initialized,
                          'updates':self.amcl_updates,'scan_age_s':age(self.last_scan),
                          'pose_age_s':age(self.last_amcl),'waiting_s':age(self.initial_started) if not self.initialized else None,
                          'uncertainty':uncertainty}
            return {'state':self.state,'message':message,'drive':self.drive,'ready':not issue,'readiness_issue':issue,
                'localization':localization,
                'maps':self.catalog(),'map_id':self.map_id,'map':self.selected.metadata() if self.selected else None,
                'robot':robot,'path':self.path,'stt_ready':self.stt.ready(),'last_voice':self.last_voice,
                'target_room':self.target_room,'feedback':self.feedback,'stationary':bool(self.stationary()),'epoch':self.generation}

    def command_text(self,text):
        with self.lock:
            if self.selected is None:raise ValueError('맵을 먼저 선택하세요.')
            result=resolve_room(text,self.selected.rooms);self.last_voice=text
            if result['intent']=='stop':self.halt();return result
            self.navigate(result['room']['name']);return result


def start_http(node,port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def reply(self,status,data,kind='application/json'):
            if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            if serve_camera(self,node,urlparse(self.path).path):return
            if self.path=='/':return self.reply(200,(node.share/'config/guide_gui.html').read_bytes(),'text/html; charset=utf-8')
            if self.path=='/api/status':return self.reply(200,node.snapshot())
            if self.path.startswith('/api/map.png') and node.selected:return self.reply(200,node.selected.png,'image/png')
            self.reply(404,{'error':'Not found'})
        def do_POST(self):
            origin=self.headers.get('Origin')
            if self.headers.get('X-Orinbot-UI')!='1' or (origin and urlparse(origin).netloc!=self.headers.get('Host')):
                return self.reply(403,{'success':False,'message':'Origin rejected'})
            try:
                length=int(self.headers.get('Content-Length',0));limit=700000 if self.path=='/api/speech' else 8192
                if not 0<=length<=limit:raise ValueError('요청 크기가 너무 큽니다.')
                self.connection.settimeout(10);raw=self.rfile.read(length)
                if len(raw)!=length:raise ValueError('요청 데이터가 불완전합니다.')
                if self.path=='/api/speech':
                    epoch=self.headers.get('X-Guide-Epoch')
                    if node.drive and epoch is None:raise ValueError('화면을 새로고침하고 음성을 다시 요청하세요.')
                    generation=int(epoch) if epoch is not None else node.generation
                    if generation!=node.generation:raise ValueError('녹음 중 상태가 바뀌었습니다. 다시 요청해 주세요.')
                    text=node.stt.transcribe(raw);node.last_voice=text
                    with node.lock:
                        if node.closing.is_set() or generation!=node.generation:raise ValueError('음성 처리 중 상태가 바뀌었습니다. 다시 요청해 주세요.')
                        result=node.command_text(text)
                    return self.reply(200,{'success':True,**result})
                body=json.loads(raw or b'{}')
                if self.path=='/api/load':node.load_map(body['map_id'])
                elif self.path=='/api/room':
                    with node.lock:
                        if node.selected is None:raise ValueError('맵을 선택하세요.')
                        if node.state in ('navigating','starting_goal','stopping'):raise ValueError('안내가 종료된 뒤 호실을 편집하세요.')
                        node.selected.save_room(body['name'],float(body['x']),float(body['y']),float(body['yaw']),body.get('aliases',[]))
                elif self.path=='/api/delete-room':
                    with node.lock:
                        if node.state in ('navigating','starting_goal','stopping'):raise ValueError('안내가 종료된 뒤 삭제하세요.')
                        if node.selected:node.selected.persist([r for r in node.selected.rooms if r['name']!=body['name']])
                elif self.path=='/api/initial':node.set_initial(float(body['x']),float(body['y']),float(body['yaw']))
                elif self.path=='/api/navigate':node.navigate(body['name'])
                elif self.path=='/api/text':return self.reply(200,{'success':True,**node.command_text(body['text'])})
                elif self.path=='/api/stop':node.halt()
                else:return self.reply(404,{'success':False,'message':'Not found'})
                self.reply(200,{'success':True})
            except Exception as exc:self.reply(400,{'success':False,'message':str(exc),'text':node.last_voice})
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler);server.daemon_threads=True
    threading.Thread(target=server.serve_forever,daemon=True).start();return server


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=GuideService();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    def interrupt(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN)
        node.closing.set();node.halt();node.http.shutdown();node.http.server_close()
        deadline=time.monotonic()+1.5
        while time.monotonic()<deadline:node.send();time.sleep(.05)
        executor.shutdown(timeout_sec=10);node.stop_nav_process();node.events.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
