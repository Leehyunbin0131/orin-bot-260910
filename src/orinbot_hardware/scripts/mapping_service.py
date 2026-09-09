#!/usr/bin/env python3
"""Start SLAM on request and save both occupancy map and pose graph without driving."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import struct
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from tf2_ros import Buffer, TransformListener, TransformException
import yaml


def verify_saved_map(directory):
    """A successful response requires both usable occupancy files and the pose graph."""
    required = ['map.yaml', 'map.posegraph', 'map.data']
    for name in required:
        p = directory/name
        if not p.is_file() or not p.stat().st_size:
            raise RuntimeError(f'Saved file missing or empty: {name}')
    config = yaml.safe_load((directory/'map.yaml').read_text())
    image = Path(config['image'])
    if not image.is_absolute(): image = directory/image
    if not image.is_file() or not image.stat().st_size:
        raise RuntimeError('Saved map image missing or empty')
    if not math.isfinite(float(config['resolution'])) or float(config['resolution']) <= 0:
        raise RuntimeError('Invalid saved map resolution')
    return [str(directory/name) for name in required]+[str(image)]


def map_png(grid):
    """Dependency-free grayscale PNG; ROS occupancy rows start at the bottom."""
    def chunk(kind, payload):
        return struct.pack('!I',len(payload))+kind+payload+struct.pack('!I',zlib.crc32(kind+payload)&0xffffffff)
    w,h = grid.info.width,grid.info.height
    pixels = bytes(205 if v < 0 else round(254*(1-min(100,v)/100)) for v in grid.data)
    rows = b''.join(b'\0'+pixels[y*w:(y+1)*w] for y in range(h-1,-1,-1))
    return (b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!IIBBBBB',w,h,8,0,0,0,0))
            +chunk(b'IDAT',zlib.compress(rows,3))+chunk(b'IEND',b''))


def start_gui(node, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass
        def reply(self, code, content, kind='application/json'):
            self.send_response(code); self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(content))); self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff'); self.end_headers()
            try: self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError): pass
        def do_GET(self):
            path=urlparse(self.path).path
            if path=='/': return self.reply(200,(node.share/'config/mapping_gui.html').read_bytes(),'text/html; charset=utf-8')
            if path=='/api/status':
                data=json.loads(node.status(None,Trigger.Response()).message)
                grid=node.map
                data['stationary']=bool(node.stationary())
                data['map']=None;data['robot']=None
                if grid:
                    q=grid.info.origin.orientation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    data['map']={'sequence':node.map_sequence,'width':grid.info.width,'height':grid.info.height,
                        'resolution':grid.info.resolution,'origin':[grid.info.origin.position.x,grid.info.origin.position.y,yaw]}
                    try:
                        t=node.tf.lookup_transform('map','base_footprint',Time()).transform;q=t.rotation
                        data['robot']=[t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
                    except TransformException: pass
                return self.reply(200,json.dumps(data).encode())
            if path=='/api/map.png':
                grid=node.map
                if grid: return self.reply(200,map_png(grid),'image/png')
                return self.reply(404,b'{"error":"No map yet"}')
            self.reply(404,b'{"error":"Not found"}')
        def do_POST(self):
            # Local-only server, no CORS; custom header blocks cross-origin form submissions.
            origin=self.headers.get('Origin')
            if self.headers.get('X-Orinbot-UI')!='1' or (origin and urlparse(origin).netloc!=self.headers.get('Host')):
                return self.reply(403,b'{"success":false,"message":"Origin rejected"}')
            action={'/api/start':node.start,'/api/save':node.save}.get(self.path)
            if action is None: return self.reply(404,b'{"error":"Not found"}')
            result=action(None,Trigger.Response())
            self.reply(200,json.dumps({'success':result.success,'message':result.message}).encode())
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    server.daemon_threads=True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    return server


class MappingService(Node):
    def __init__(self):
        super().__init__('mapping_service')
        self.share = Path(get_package_share_directory('orinbot_hardware'))
        self.root = Path.home()/'ros2_ws'
        self.maps_root = Path(self.declare_parameter('maps_directory', str(self.root/'maps')).value).expanduser().resolve()
        self.state, self.message = 'idle', 'Call /mapping/start to create a new map'
        self.process, self.process_log = None, None
        self.operation = threading.Lock()
        self.closing = threading.Event()
        self.last_scan, self.last_odom, self.last_map = 0., 0., 0.
        self.scan = self.odom = self.map = None
        self.map_sequence = 0
        self.session = self.last_saved = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.group = ReentrantCallbackGroup()
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/mapping/start', self.start, callback_group=self.group)
        self.create_service(Trigger, '/mapping/save', self.save, callback_group=self.group)
        self.create_service(Trigger, '/mapping/status', self.status, callback_group=self.group)
        self.save_client = self.create_client(SaveMap, '/slam_toolbox/save_map', callback_group=self.group)
        self.graph_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map', callback_group=self.group)
        self.create_timer(1., self.monitor)
        self.get_logger().info('Ready: /mapping/start, /mapping/save, /mapping/status. No automatic motion.')
        port = self.declare_parameter('gui_port', 8080).value
        self.gui = start_gui(self, port) if port else None
        if self.gui: self.get_logger().info(f'Mapping GUI: http://127.0.0.1:{port}')

    def scan_callback(self, msg):
        self.scan, self.last_scan = msg, time.monotonic()

    def odom_callback(self, msg):
        self.odom, self.last_odom = msg, time.monotonic()

    def map_callback(self, msg):
        if (msg.header.frame_id == 'map' and msg.info.width > 0 and msg.info.height > 0
                and len(msg.data) == msg.info.width*msg.info.height and any(v >= 0 for v in msg.data)):
            self.map, self.last_map = msg, time.monotonic()
            self.map_sequence += 1

    def fresh_stamp(self, stamp):
        age = (self.get_clock().now().nanoseconds-(stamp.sec*10**9+stamp.nanosec))/1e9
        return -0.1 <= age <= 1.

    def input_problem(self):
        now = time.monotonic()
        if not self.scan or now-self.last_scan > 1 or not self.fresh_stamp(self.scan.header.stamp):
            return 'Fresh /scan is missing'
        if not self.odom or now-self.last_odom > .5 or not self.fresh_stamp(self.odom.header.stamp):
            return 'Fresh /odom is missing'
        if self.scan.header.frame_id != 'laser' or self.odom.header.frame_id != 'odom' or self.odom.child_frame_id != 'base_footprint':
            return 'Unexpected sensor or odometry frame'
        if self.count_publishers('/scan') != 1 or self.count_publishers('/odom') != 1:
            return 'Expected one scan publisher and one odometry publisher'
        if not any(math.isfinite(v) and self.scan.range_min <= v <= self.scan.range_max for v in self.scan.ranges):
            return 'Scan contains no valid measured ranges'
        q = self.odom.pose.pose.orientation
        values = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, q.x,q.y,q.z,q.w,
                  self.odom.twist.twist.linear.x,self.odom.twist.twist.angular.z)
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[2:6])-1) > .01:
            return 'Invalid odometry values'
        try:
            self.tf.lookup_transform('odom','laser',Time.from_msg(self.scan.header.stamp))
        except TransformException:
            return 'TF odom -> base_footprint -> laser is not ready at scan timestamp'
        return ''

    def stationary(self):
        return self.odom and abs(self.odom.twist.twist.linear.x) < .001 and abs(self.odom.twist.twist.angular.z) < .005

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def monitor(self):
        if self.state == 'mapping' and not self.alive():
            self.state, self.message = 'failed', 'SLAM process exited; check session slam.log'

    def wait_until(self, predicate, timeout):
        deadline = time.monotonic()+timeout
        while not self.closing.is_set() and time.monotonic() < deadline:
            if predicate(): return
            self.closing.wait(.1)
        raise RuntimeError('Operation cancelled or timed out')

    def stop_child(self):
        p = self.process
        if p is not None:
            # Own process group only. Launch handles the initial SIGINT for its children.
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
                try: p.wait(timeout=5)
                except subprocess.TimeoutExpired: pass
            try: os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError: pass
            try: p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                try: os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError: pass
                p.wait(timeout=2)
        self.process = None
        if self.process_log: self.process_log.close(); self.process_log = None

    def start(self, request, response):
        if not self.operation.acquire(blocking=False):
            response.success, response.message = False, 'Another mapping operation is in progress'; return response
        started = False
        try:
            if self.closing.is_set(): raise RuntimeError('Service is shutting down')
            if self.alive(): raise RuntimeError('Mapping is already running; existing map is preserved')
            issue = self.input_problem()
            if issue: raise RuntimeError(issue)
            if not self.stationary(): raise RuntimeError('Stop with keyboard k before starting mapping')
            if self.count_publishers('/map') or any(n == 'slam_toolbox' for n, _ in self.get_node_names_and_namespaces()):
                raise RuntimeError('Another map or SLAM node is active; stop it first')
            self.stop_child()
            self.session = self.root/'log/mapping'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            self.session.mkdir(parents=True)
            self.map, self.last_map = None, 0.
            self.state, self.message = 'starting', 'Waiting for the first occupancy map'
            self.process_log = (self.session/'slam.log').open('w')
            self.process = subprocess.Popen(['ros2','launch','slam_toolbox','online_async_launch.py',
                'use_sim_time:=false','autostart:=true',f'slam_params_file:={self.share / "config/slam.yaml"}'],
                stdout=self.process_log,stderr=subprocess.STDOUT,start_new_session=True)
            started = True
            def ready():
                if not self.alive(): raise RuntimeError('SLAM exited during startup')
                return self.map is not None and self.tf.can_transform('map','odom',Time())
            self.wait_until(ready,25)
            self.state, self.message = 'mapping', 'Mapping started; drive with the keyboard'
            response.success, response.message = True, self.message
        except Exception as exc:
            if started:
                self.stop_child(); self.state, self.message = 'failed', str(exc)
            response.success, response.message = False, str(exc)
        finally:
            self.operation.release()
        return response

    def invoke(self, client, request):
        if not client.wait_for_service(timeout_sec=3): raise RuntimeError('SLAM save service is unavailable')
        future = client.call_async(request)
        try:
            self.wait_until(future.done,20)
            result = future.result()
            if result is None or result.result != 0: raise RuntimeError('SLAM save service reported failure')
        except Exception:
            future.cancel()
            raise

    def save(self, request, response):
        if not self.operation.acquire(blocking=False):
            response.success, response.message = False, 'Another mapping operation is in progress'; return response
        directory = None
        try:
            if not self.alive() or self.state != 'mapping': raise RuntimeError('Start mapping first')
            issue = self.input_problem()
            if issue: raise RuntimeError(issue)
            if not self.stationary(): raise RuntimeError('Stop with keyboard k before saving')
            # Wait for a fresh raster after the user has stopped, then save both products.
            before = time.monotonic()
            self.wait_until(lambda: self.last_map > before,5)
            if not self.stationary(): raise RuntimeError('Robot moved while preparing the save; stop and retry')
            directory = self.maps_root/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            directory.mkdir(parents=True,exist_ok=False)
            (directory/'.incomplete').touch()
            prefix = str(directory/'map')
            req = SaveMap.Request(); req.name.data = prefix
            self.invoke(self.save_client,req)
            self.invoke(self.graph_client,SerializePoseGraph.Request(filename=prefix))
            files = verify_saved_map(directory)
            metadata = {'saved_at':datetime.now().isoformat(),'session':str(self.session),
                        'files':files,'hardware':yaml.safe_load((self.share/'config/hardware.yaml').read_text()),
                        'slam':yaml.safe_load((self.share/'config/slam.yaml').read_text())}
            (directory/'metadata.json').write_text(json.dumps(metadata,indent=2))
            (directory/'.incomplete').unlink()
            self.last_saved = str(directory)
            response.success, response.message = True, f'Saved map + pose graph: {directory}; mapping continues'
        except Exception as exc:
            response.success, response.message = False, str(exc)+(f'; partial output: {directory}' if directory else '')
        finally:
            self.operation.release()
        return response

    def status(self, request, response):
        issue = self.input_problem()
        response.success = True
        response.message = json.dumps({'state':self.state,'message':self.message,'inputs_ready':not issue,
            'input_issue':issue,'operation_busy':self.operation.locked(),'slam_running':self.alive(),
            'map_size':[self.map.info.width,self.map.info.height] if self.map else None,
            'last_saved':self.last_saved,'session_log':str(self.session) if self.session else None},ensure_ascii=False)
        return response


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = MappingService()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    def interrupt(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt); signal.signal(signal.SIGTERM,interrupt)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN)
        node.closing.set()
        if node.gui: node.gui.shutdown(); node.gui.server_close()
        executor.shutdown(timeout_sec=30)
        with node.operation:
            node.stop_child()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
