#!/usr/bin/env python3
"""
Web Monitor Node - ROS2状态监控Web服务
提供WebSocket实时推送和REST API控制
┌─────────────────────────────────────────┐
│                 主进程                   │
├─────────────────┬───────────────────────┤
│     主线程       │     ROS2 Thread       │
│  (asyncio)      │     (daemon)          │
│                 │                       │
│  - uvicorn      │  - MultiThreadedExec  │
│  - FastAPI      │  - imu_callback       │
│  - WebSocket    │  - joint_callback     │
│  - REST API     │  - broadcast_timer    │
└─────────────────┴───────────────────────┘
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState, Imu, Joy
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger

import os
import json
import asyncio
import threading
import signal
import socket
import subprocess
from typing import List, Dict, Any
from dataclasses import dataclass, asdict
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse
import uvicorn

from .state import get_cpu_stats, get_memory_usage

# MARK: State
@dataclass
class MonitorState:
    """监控状态数据结构"""
    domain_id: int = 0
    controller_state: str = "PD模式"
    
    # 系统状态
    cpu_usage: float = 0.0
    cpu_load_1m: float = 0.0
    memory_usage: float = 0.0
    
    # IMU数据
    imu_orientation: List[float] = None
    imu_angular_vel: List[float] = None
    imu_linear_acc: List[float] = None
    
    # 关节数据
    joint_names: List[str] = None
    joint_positions: List[float] = None
    joint_velocities: List[float] = None
    joint_efforts: List[float] = None
    
    # 关节限位
    joint_limits_lower: List[float] = None
    joint_limits_upper: List[float] = None
    
    # 时间戳
    imu_timestamp: float = 0.0
    joint_timestamp: float = 0.0
    
    def __post_init__(self):
        if self.imu_orientation is None:
            self.imu_orientation = [0.0, 0.0, 0.0, 1.0]
        if self.imu_angular_vel is None:
            self.imu_angular_vel = [0.0, 0.0, 0.0]
        if self.imu_linear_acc is None:
            self.imu_linear_acc = [0.0, 0.0, 0.0]
        if self.joint_names is None:
            self.joint_names = []
        if self.joint_positions is None:
            self.joint_positions = []
        if self.joint_velocities is None:
            self.joint_velocities = []
        if self.joint_efforts is None:
            self.joint_efforts = []
        if self.joint_limits_lower is None:
            self.joint_limits_lower = []
        if self.joint_limits_upper is None:
            self.joint_limits_upper = []
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

# MARK: Socket管理器
class ConnectionManager:
    """WebSocket连接管理器"""
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
    
    async def broadcast(self, message: str):
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except:
                disconnected.append(connection)
        for conn in disconnected:
            self.disconnect(conn)


# MARK: 节点
class WebMonitorNode(Node):
    """ROS2 Web监控节点"""
    
    def __init__(self, manager: ConnectionManager):
        super().__init__('web_monitor_node')
        
        self.manager = manager
        self.state = MonitorState()
        self.state.domain_id = int(os.environ.get('ROS_DOMAIN_ID', 0))
        self.loop = None
        
        # 订阅话题
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.state_sub = self.create_subscription(
            String, '/controller_state', self.state_callback, 10)
        
        # 发布话题
        # self.joy_pub = self.create_publisher(Joy, '/wb_joy_topic', 10)   # 保留
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.cmd_pub = self.create_publisher(String, '/web_command', 10)
        
        # 服务客户端 - 获取关节限位
        self.joint_limits_client = self.create_client(Trigger, '/get_joint_limits')
        self._joint_limits_fetched = False
        
        # 定时广播状态 (10Hz)
        self.broadcast_timer = self.create_timer(0.1, self.broadcast_state)
        
        self.get_logger().info(f'Web Monitor Node started, Domain ID: {self.state.domain_id}')
    
    def set_loop(self, loop):
        self.loop = loop
    
    def imu_callback(self, msg: Imu):
        self.state.imu_orientation = [
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        ]
        self.state.imu_angular_vel = [
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ]
        self.state.imu_linear_acc = [
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ]
        self.state.imu_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    
    def joint_callback(self, msg: JointState):
        self.state.joint_names = list(msg.name)
        self.state.joint_positions = list(msg.position)
        self.state.joint_velocities = list(msg.velocity) if msg.velocity else []
        self.state.joint_efforts = list(msg.effort) if msg.effort else []
        self.state.joint_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 如果还没获取关节限位，尝试获取
        if not self._joint_limits_fetched and len(self.state.joint_limits_lower) == 0:
            self._fetch_joint_limits()
    
    def state_callback(self, msg: String):
        self.state.controller_state = msg.data
    
    def _fetch_joint_limits(self):
        """异步获取关节限位"""
        if not self.joint_limits_client.service_is_ready():
            self.get_logger().debug('Joint limits service not available')
            return
        
        try:
            future = self.joint_limits_client.call_async(Trigger.Request())
            future.add_done_callback(self._joint_limits_callback)
        except Exception as e:
            self.get_logger().warn(f'Failed to call joint limits service: {e}')
    
    def _joint_limits_callback(self, future):
        """处理关节限位服务响应"""
        try:
            response = future.result()
            if response.success:
                limits = json.loads(response.message)
                self.state.joint_limits_lower = limits.get('lower', [])
                self.state.joint_limits_upper = limits.get('upper', [])
                self._joint_limits_fetched = True
                self.get_logger().info(f'Joint limits fetched: {len(self.state.joint_limits_lower)} joints')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse joint limits: {e}')
    
    def broadcast_state(self):
        if self.loop and self.manager.active_connections:
            # 更新系统状态
            cpu_stats = get_cpu_stats()
            self.state.cpu_usage = cpu_stats.usage_percent
            self.state.cpu_load_1m = cpu_stats.load_avg_1m
            mem_stats = get_memory_usage()
            self.state.memory_usage = mem_stats['usage_percent']
            
            message = json.dumps(self.state.to_dict())
            asyncio.run_coroutine_threadsafe(
                self.manager.broadcast(message), self.loop
            )
    
    def publish_joy(self, axes: List[float], buttons: List[int]):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 确保axes是float类型
        axes_float = [float(x) for x in axes]
        axes_padded = axes_float + [0.0] * max(0, 6 - len(axes_float))
        msg.axes = axes_padded[:6]
        # 确保buttons是int类型
        buttons_int = [int(b) for b in buttons]
        buttons_padded = buttons_int + [0] * max(0, 12 - len(buttons_int))
        msg.buttons = buttons_padded[:12]
        self.joy_pub.publish(msg)
        self.get_logger().debug(f'Published Joy: axes={msg.axes[:4]}')

    
    # 发布指令
    def publish_command(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Published command: {cmd}')


# 全局变量
manager = ConnectionManager()
node: WebMonitorNode = None
app = FastAPI(title="ROS2 Web Monitor")

# 静态文件目录 - 使用包资源路径
def get_static_dir():
    try:
        with pkg_resources.files('mjsim.web_monitor').joinpath('static') as p:
            return Path(p)
    except:
        return Path(__file__).parent / "static"

STATIC_DIR = get_static_dir()


# MARK: 路由
@app.get("/", response_class=HTMLResponse)
async def get_index():
    index_path = STATIC_DIR / "index.html"
    if index_path.exists():
        return FileResponse(index_path)
    return HTMLResponse("<h1>Web Monitor</h1><p>index.html not found</p>")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            try:
                cmd = json.loads(data)
                handle_command(cmd)
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        manager.disconnect(websocket)


def handle_command(cmd: dict):
    """处理前端发来的命令"""
    global node
    if not node:
        return
    
    cmd_type = cmd.get('type', '')
    
    if cmd_type == 'joy':
        axes = cmd.get('axes', [0.0, 0.0, 0.0, 0.0])
        buttons = cmd.get('buttons', [0] * 12)
        node.publish_joy(axes, buttons)
    
    elif cmd_type == 'command':
        command = cmd.get('command', '')
        node.publish_command(command)


@app.get("/api/state")
async def get_state():
    if node:
        return node.state.to_dict()
    return {"error": "Node not initialized"}


@app.post("/api/joy")
async def post_joy(data: dict):
    if node:
        axes = data.get('axes', [0.0, 0.0])
        buttons = data.get('buttons', [])
        node.publish_joy(axes, buttons)
        return {"status": "ok"}
    return {"error": "Node not initialized"}


@app.post("/api/command")
async def post_command(data: dict):
    if node:
        cmd = data.get('command', '')
        node.publish_command(cmd)
        return {"status": "ok"}
    return {"error": "Node not initialized"}


# MARK: 端口自检
def get_local_ip() -> str:
    """获取本机局域网IP地址"""
    try:
        # 创建UDP socket连接外部地址来获取本机IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def kill_process_on_port(port: int) -> bool:
    """
    检查端口占用并关闭占用进程（仅限自己的进程）
    
    Args:
        port: 要检查的端口号
        
    Returns:
        True if port is now free, False otherwise
    """
    try:
        # 使用 lsof 查找占用端口的进程
        result = subprocess.run(
            ['lsof', '-ti', f':{port}'],
            capture_output=True,
            text=True
        )
        
        if result.returncode != 0 or not result.stdout.strip():
            # 端口未被占用
            return True
        
        pids = result.stdout.strip().split('\n')
        current_pid = os.getpid()
        
        for pid_str in pids:
            try:
                pid = int(pid_str)
                if pid == current_pid:
                    continue
                
                # 检查是否是自己的进程（同一用户）
                proc_result = subprocess.run(
                    ['ps', '-o', 'comm=', '-p', str(pid)],
                    capture_output=True,
                    text=True
                )
                
                proc_name = proc_result.stdout.strip()
                
                # 只杀死 python/uvicorn 相关的进程
                if 'python' in proc_name.lower() or 'uvicorn' in proc_name.lower():
                    print(f"Killing process {pid} ({proc_name}) occupying port {port}")
                    os.kill(pid, signal.SIGTERM)
                    
                    # 等待进程结束
                    import time
                    for _ in range(10):  # 等待最多1秒
                        time.sleep(0.1)
                        try:
                            os.kill(pid, 0)  # 检查进程是否存在
                        except OSError:
                            break
                    else:
                        # 如果还在运行，强制杀死
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except OSError:
                            pass
                            
            except (ValueError, OSError) as e:
                print(f"Warning: Could not handle PID {pid_str}: {e}")
                continue
        
        return True
    
    except FileNotFoundError:
        # lsof 不可用，尝试使用 ss
        try:
            result = subprocess.run(
                ['ss', '-tlnp', f'sport = :{port}'],
                capture_output=True,
                text=True
            )
            if 'LISTEN' not in result.stdout:
                return True
            print(f"Warning: Port {port} is in use, but cannot identify process")
            return False
        except:
            return True
    except Exception as e:
        print(f"Warning: Error checking port {port}: {e}")
        return True


def run_ros2_node():
    """在独立线程中运行ROS2节点"""
    global node
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


# MARK: Main
def main(args=None):
    global node, manager
    
    rclpy.init(args=args)
    node = WebMonitorNode(manager)
    
    # 获取事件循环
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    node.set_loop(loop)
    
    # 挂载静态文件
    if STATIC_DIR.exists():
        app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
    
    # 启动ROS2线程
    ros_thread = threading.Thread(target=run_ros2_node, daemon=True)
    ros_thread.start()
    
    # 检查并释放端口
    port = 2025
    kill_process_on_port(port)
    
    # 启动Web服务
    print(f"\n{'='*50}")
    print(f"  Web Monitor starting...")
    local_ip = get_local_ip()
    print(f"  Access: http://{local_ip}:{port}")
    print(f"  ROS Domain ID: {node.state.domain_id}")
    print(f"{'='*50}\n")
    
    config = uvicorn.Config(app, host="0.0.0.0", port=port, loop="asyncio", log_level="warning")
    server = uvicorn.Server(config)
    
    try:
        loop.run_until_complete(server.serve())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
