#!/usr/bin/env python3
"""
Lightweight, deterministic in-process ROS 2 (`rclpy`, `tf2_ros`, standard
messages, and `ament_index_python`) compatibility layer for executing and
testing the exact AROLA ROS 2 nodes in environments without `/opt/ros/humble`
installed (such as Debian 12 Python 3.11 CI / sandbox containers).

When native ROS 2 Humble (`rclpy`) IS installed, `ensure_ros2_environment()`
is a complete no-op and returns `native_ros2=True`.
"""

from dataclasses import dataclass, field
import os
import sys
import types
from typing import Any, Callable, Dict, List, Optional, Tuple
import yaml


def _flatten_params(prefix: str, data: Dict[str, Any], out: Dict[str, Any]):
    for key, val in data.items():
        full_key = f'{prefix}.{key}' if prefix else str(key)
        if isinstance(val, dict):
            _flatten_params(full_key, val, out)
        else:
            out[full_key] = val


class SimBus:
    """Deterministic single-process pub/sub, timer, and TF2 simulation bus."""

    def __init__(self):
        self.time_ns: int = 0
        self.nodes: List[Any] = []
        self.topics: Dict[str, List[Callable[[Any], None]]] = {}
        self.topic_counts: Dict[str, int] = {}
        self.last_msg: Dict[str, Any] = {}
        # child_frame -> (parent_frame, transform_stamped, authority_node_name)
        self.tf_tree: Dict[str, Tuple[str, Any, str]] = {}
        self.tf_parent_conflicts: List[Tuple[str, str, str]] = []

    def reset(self) -> None:
        self.time_ns = 0
        self.nodes.clear()
        self.topics.clear()
        self.topic_counts.clear()
        self.last_msg.clear()
        self.tf_tree.clear()
        self.tf_parent_conflicts.clear()

    def publish(self, topic: str, msg: Any) -> None:
        norm = topic if topic.startswith('/') else f'/{topic}'
        self.topic_counts[norm] = self.topic_counts.get(norm, 0) + 1
        self.last_msg[norm] = msg
        for cb in list(self.topics.get(norm, [])):
            cb(msg)

    def subscribe(
        self, topic: str, callback: Callable[[Any], None]
    ) -> None:
        norm = topic if topic.startswith('/') else f'/{topic}'
        self.topics.setdefault(norm, []).append(callback)

    def set_transform(self, ts: Any, authority: str) -> None:
        parent = ts.header.frame_id.lstrip('/')
        child = ts.child_frame_id.lstrip('/')
        prev = self.tf_tree.get(child)
        if prev is not None and prev[0] != parent:
            conflict = (child, prev[0], parent)
            if conflict not in self.tf_parent_conflicts:
                self.tf_parent_conflicts.append(conflict)
        self.tf_tree[child] = (parent, ts, authority)

    def step_by_ns(self, delta_ns: int) -> None:
        target_ns = self.time_ns + delta_ns
        while True:
            earliest_ns: Optional[int] = None
            due_timer = None
            for node in list(self.nodes):
                for tmr in list(node._timers):
                    if not tmr.cancelled and tmr.next_call_ns <= target_ns:
                        if (
                            earliest_ns is None
                            or tmr.next_call_ns < earliest_ns
                        ):
                            earliest_ns = tmr.next_call_ns
                            due_timer = tmr
            if due_timer is None or earliest_ns is None:
                break
            self.time_ns = earliest_ns
            due_timer.next_call_ns += due_timer.period_ns
            due_timer.callback()
        self.time_ns = target_ns


GLOBAL_BUS = SimBus()
PENDING_NODE_PARAMS: Dict[str, Dict[str, Any]] = {}


def load_yaml_params_for_bus(yaml_path: str) -> Dict[str, Dict[str, Any]]:
    with open(yaml_path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    parsed: Dict[str, Dict[str, Any]] = {}
    for node_name, block in raw.items():
        if isinstance(block, dict) and 'ros__parameters' in block:
            flat: Dict[str, Any] = {}
            _flatten_params('', block['ros__parameters'] or {}, flat)
            parsed[node_name] = flat
    return parsed


def ensure_ros2_environment(workspace_root: Optional[str] = None) -> bool:
    """
    Return `True` if native ROS 2 `rclpy` is installed; otherwise register
    deterministic pure-Python ROS 2 modules in `sys.modules` (`False`).
    """
    try:
        import rclpy as _real_rclpy  # noqa: F401

        if not getattr(_real_rclpy, '__sim_compat__', False):
            return True
        return False
    except ImportError:
        pass

    ws_root = os.path.abspath(
        workspace_root
        or os.path.join(os.path.dirname(__file__), '..', '..')
    )

    # ---- Message types ----
    @dataclass
    class TimeMsg:
        sec: int = 0
        nanosec: int = 0

    @dataclass
    class Header:
        stamp: TimeMsg = field(default_factory=TimeMsg)
        frame_id: str = ''

    @dataclass
    class BoolMsg:
        data: bool = False

    @dataclass
    class StringMsg:
        data: str = ''

    @dataclass
    class Int32Msg:
        data: int = 0

    @dataclass
    class Float32Msg:
        data: float = 0.0

    @dataclass
    class ColorRGBA:
        r: float = 0.0
        g: float = 0.0
        b: float = 0.0
        a: float = 1.0

    @dataclass
    class Point:
        x: float = 0.0
        y: float = 0.0
        z: float = 0.0

    @dataclass
    class Vector3:
        x: float = 0.0
        y: float = 0.0
        z: float = 0.0

    @dataclass
    class Quaternion:
        x: float = 0.0
        y: float = 0.0
        z: float = 0.0
        w: float = 1.0

    @dataclass
    class Pose:
        position: Point = field(default_factory=Point)
        orientation: Quaternion = field(default_factory=Quaternion)

    @dataclass
    class PoseStamped:
        header: Header = field(default_factory=Header)
        pose: Pose = field(default_factory=Pose)

    @dataclass
    class PointStamped:
        header: Header = field(default_factory=Header)
        point: Point = field(default_factory=Point)

    @dataclass
    class PoseWithCovariance:
        pose: Pose = field(default_factory=Pose)
        covariance: List[float] = field(default_factory=lambda: [0.0] * 36)

    @dataclass
    class PoseWithCovarianceStamped:
        header: Header = field(default_factory=Header)
        pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)

    @dataclass
    class Twist:
        linear: Vector3 = field(default_factory=Vector3)
        angular: Vector3 = field(default_factory=Vector3)

    @dataclass
    class TwistWithCovariance:
        twist: Twist = field(default_factory=Twist)
        covariance: List[float] = field(default_factory=lambda: [0.0] * 36)

    @dataclass
    class Transform:
        translation: Vector3 = field(default_factory=Vector3)
        rotation: Quaternion = field(default_factory=Quaternion)

    @dataclass
    class TransformStamped:
        header: Header = field(default_factory=Header)
        child_frame_id: str = ''
        transform: Transform = field(default_factory=Transform)

    @dataclass
    class Odometry:
        header: Header = field(default_factory=Header)
        child_frame_id: str = ''
        pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)
        twist: TwistWithCovariance = field(
            default_factory=TwistWithCovariance
        )

    @dataclass
    class Path:
        header: Header = field(default_factory=Header)
        poses: List[PoseStamped] = field(default_factory=list)

    @dataclass
    class LaserScan:
        header: Header = field(default_factory=Header)
        angle_min: float = 0.0
        angle_max: float = 0.0
        angle_increment: float = 0.0
        time_increment: float = 0.0
        scan_time: float = 0.0
        range_min: float = 0.0
        range_max: float = 30.0
        ranges: List[float] = field(default_factory=list)
        intensities: List[float] = field(default_factory=list)

    @dataclass
    class Image:
        header: Header = field(default_factory=Header)
        height: int = 0
        width: int = 0
        encoding: str = 'rgb8'
        data: bytes = b''

    @dataclass
    class Joy:
        header: Header = field(default_factory=Header)
        axes: List[float] = field(default_factory=list)
        buttons: List[int] = field(default_factory=list)

    @dataclass
    class AckermannDrive:
        steering_angle: float = 0.0
        steering_angle_velocity: float = 0.0
        speed: float = 0.0
        acceleration: float = 0.0
        jerk: float = 0.0

    @dataclass
    class AckermannDriveStamped:
        header: Header = field(default_factory=Header)
        drive: AckermannDrive = field(default_factory=AckermannDrive)

    @dataclass
    class DurationMsg:
        sec: int = 0
        nanosec: int = 0

    @dataclass
    class Marker:
        ARROW: int = 0
        CUBE: int = 1
        SPHERE: int = 2
        CYLINDER: int = 3
        LINE_STRIP: int = 4
        LINE_LIST: int = 5
        TEXT_VIEW_FACING: int = 9
        ADD: int = 0
        DELETE: int = 2
        DELETEALL: int = 3
        header: Header = field(default_factory=Header)
        ns: str = ''
        id: int = 0
        type: int = 0
        action: int = 0
        pose: Pose = field(default_factory=Pose)
        scale: Vector3 = field(default_factory=Vector3)
        color: ColorRGBA = field(default_factory=ColorRGBA)
        lifetime: DurationMsg = field(default_factory=DurationMsg)
        text: str = ''
        points: List[Point] = field(default_factory=list)
        colors: List[ColorRGBA] = field(default_factory=list)

    @dataclass
    class MarkerArray:
        markers: List[Marker] = field(default_factory=list)

    # ---- rclpy core primitives ----
    class Duration:
        def __init__(self, seconds: float = 0.0, nanoseconds: int = 0):
            self.nanoseconds = int(seconds * 1e9) + int(nanoseconds)

    class Time:
        def __init__(self, seconds: int = 0, nanoseconds: int = 0):
            self.nanoseconds = int(seconds) * 10**9 + int(nanoseconds)

        def to_msg(self) -> TimeMsg:
            return TimeMsg(
                sec=int(self.nanoseconds // 10**9),
                nanosec=int(self.nanoseconds % 10**9),
            )

        @classmethod
        def from_msg(cls, msg: TimeMsg) -> 'Time':
            return cls(seconds=msg.sec, nanoseconds=msg.nanosec)

        def __sub__(self, other: 'Time') -> Duration:
            return Duration(
                nanoseconds=self.nanoseconds - other.nanoseconds
            )

    class Clock:
        def now(self) -> Time:
            return Time(nanoseconds=GLOBAL_BUS.time_ns)

    class Logger:
        def __init__(self, name: str):
            self.name = name
            self.records: List[Tuple[str, str]] = []

        def info(self, msg: str) -> None:
            self.records.append(('INFO', str(msg)))

        def warn(self, msg: str) -> None:
            self.records.append(('WARN', str(msg)))

        def warning(self, msg: str) -> None:
            self.warn(msg)

        def error(self, msg: str) -> None:
            self.records.append(('ERROR', str(msg)))

        def debug(self, msg: str) -> None:
            self.records.append(('DEBUG', str(msg)))

    class _ParamValue:
        def __init__(self, val: Any):
            self._val = val

        @property
        def string_value(self) -> str:
            return str(self._val)

        @property
        def double_value(self) -> float:
            return float(self._val)

        @property
        def integer_value(self) -> int:
            return int(self._val)

        @property
        def bool_value(self) -> bool:
            return bool(self._val)

    class Parameter:
        def __init__(self, name: str, value: Any = None):
            self.name = name
            self.value = value

        def get_parameter_value(self) -> _ParamValue:
            return _ParamValue(self.value)

    class Publisher:
        def __init__(self, msg_type: Any, topic: str):
            self.msg_type = msg_type
            self.topic = topic

        def publish(self, msg: Any) -> None:
            GLOBAL_BUS.publish(self.topic, msg)

    class Subscription:
        def __init__(self, msg_type: Any, topic: str, cb: Callable):
            self.msg_type = msg_type
            self.topic = topic
            self.callback = cb

    class Timer:
        def __init__(self, period_s: float, callback: Callable):
            self.period_ns = max(100_000, int(round(period_s * 1e9)))
            self.callback = callback
            self.next_call_ns = GLOBAL_BUS.time_ns + self.period_ns
            self.cancelled = False

        def cancel(self) -> None:
            self.cancelled = True

    class Node:
        def __init__(self, node_name: str, namespace: str = ''):
            self._name = node_name
            self._namespace = namespace
            self._logger = Logger(node_name)
            self._clock = Clock()
            self._params: Dict[str, Parameter] = {}
            self._overrides = dict(PENDING_NODE_PARAMS.get(node_name, {}))
            self._timers: List[Timer] = []
            self._publishers: List[Publisher] = []
            self._subscriptions: List[Subscription] = []
            GLOBAL_BUS.nodes.append(self)

        def get_name(self) -> str:
            return self._name

        def get_logger(self) -> Logger:
            return self._logger

        def get_clock(self) -> Clock:
            return self._clock

        def declare_parameter(self, name: str, default_value: Any = None):
            val = self._overrides.get(name, default_value)
            param = Parameter(name, val)
            self._params[name] = param
            return param

        def get_parameter(self, name: str) -> Parameter:
            if name in self._params:
                return self._params[name]
            return Parameter(name, self._overrides.get(name))

        def set_parameters(self, params: List[Parameter]) -> None:
            for p in params:
                self._params[p.name] = p

        def create_publisher(
            self, msg_type: Any, topic: str, qos: Any = 10
        ):
            pub = Publisher(msg_type, topic)
            self._publishers.append(pub)
            return pub

        def create_subscription(
            self,
            msg_type: Any,
            topic: str,
            callback: Callable,
            qos: Any = 10,
        ):
            sub = Subscription(msg_type, topic, callback)
            self._subscriptions.append(sub)
            GLOBAL_BUS.subscribe(topic, callback)
            return sub

        def create_timer(
            self, period_s: float, callback: Callable
        ) -> Timer:
            tmr = Timer(period_s, callback)
            self._timers.append(tmr)
            return tmr

        def destroy_node(self) -> None:
            for tmr in self._timers:
                tmr.cancel()
            if self in GLOBAL_BUS.nodes:
                GLOBAL_BUS.nodes.remove(self)

    # ---- TF2 multi-hop rigid transform lookup (2D SE(2) exact) ----
    import math as _math

    def _ts_to_se2(ts: TransformStamped) -> Tuple[float, float, float]:
        tx = ts.transform.translation.x
        ty = ts.transform.translation.y
        q = ts.transform.rotation
        yaw = _math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return tx, ty, yaw

    def _compose_se2(
        a: Tuple[float, float, float], b: Tuple[float, float, float]
    ) -> Tuple[float, float, float]:
        ax, ay, ayaw = a
        bx, by, byaw = b
        cx = ax + _math.cos(ayaw) * bx - _math.sin(ayaw) * by
        cy = ay + _math.sin(ayaw) * bx + _math.cos(ayaw) * by
        return cx, cy, ayaw + byaw

    class Buffer:
        def lookup_transform(
            self,
            target_frame: str,
            source_frame: str,
            time: Any = None,
            timeout: Any = None,
        ) -> TransformStamped:
            tgt = target_frame.lstrip('/')
            src = source_frame.lstrip('/')
            chain: List[TransformStamped] = []
            curr = src
            visited = set()
            while curr != tgt:
                if curr in visited or curr not in GLOBAL_BUS.tf_tree:
                    raise RuntimeError(
                        f'TF lookup failed from {source_frame} to '
                        f'{target_frame} (missing link at {curr}).'
                    )
                visited.add(curr)
                parent, ts, _ = GLOBAL_BUS.tf_tree[curr]
                chain.append(ts)
                curr = parent

            pose = (0.0, 0.0, 0.0)
            for ts in reversed(chain):
                pose = _compose_se2(pose, _ts_to_se2(ts))

            out = TransformStamped()
            out.header.frame_id = tgt
            out.header.stamp = Clock().now().to_msg()
            out.child_frame_id = src
            out.transform.translation.x = pose[0]
            out.transform.translation.y = pose[1]
            out.transform.rotation.z = _math.sin(0.5 * pose[2])
            out.transform.rotation.w = _math.cos(0.5 * pose[2])
            return out

    class TransformListener:
        def __init__(self, buffer: Buffer, node: Node):
            self.buffer = buffer
            self.node = node

    class TransformBroadcaster:
        def __init__(self, node: Node):
            self.node = node

        def sendTransform(self, transform_or_list: Any) -> None:
            items = (
                transform_or_list
                if isinstance(transform_or_list, list)
                else [transform_or_list]
            )
            for item in items:
                GLOBAL_BUS.set_transform(item, self.node.get_name())

    # ---- Register modules cleanly in sys.modules ----
    def _mkmod(name: str, **attrs: Any) -> types.ModuleType:
        mod = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(mod, k, v)
        sys.modules[name] = mod
        return mod

    rclpy_node = _mkmod('rclpy.node', Node=Node)
    rclpy_time = _mkmod('rclpy.time', Time=Time, Duration=Duration)
    rclpy_duration = _mkmod('rclpy.duration', Duration=Duration)
    rclpy_param = _mkmod('rclpy.parameter', Parameter=Parameter)
    rclpy_exec = _mkmod(
        'rclpy.executors', ExternalShutdownException=RuntimeError
    )
    _mkmod(
        'rclpy',
        __sim_compat__=True,
        init=lambda args=None: None,
        shutdown=lambda: None,
        try_shutdown=lambda: None,
        spin=lambda node: None,
        node=rclpy_node,
        time=rclpy_time,
        duration=rclpy_duration,
        parameter=rclpy_param,
        executors=rclpy_exec,
        Time=Time,
        Duration=Duration,
    )

    _mkmod('builtin_interfaces')
    _mkmod('builtin_interfaces.msg', Time=TimeMsg, Duration=DurationMsg)
    _mkmod('std_msgs')
    _mkmod(
        'std_msgs.msg',
        Header=Header,
        Bool=BoolMsg,
        String=StringMsg,
        Int32=Int32Msg,
        Float32=Float32Msg,
        ColorRGBA=ColorRGBA,
    )
    _mkmod('geometry_msgs')
    _mkmod(
        'geometry_msgs.msg',
        Point=Point,
        PointStamped=PointStamped,
        Vector3=Vector3,
        Quaternion=Quaternion,
        Pose=Pose,
        PoseStamped=PoseStamped,
        PoseWithCovariance=PoseWithCovariance,
        PoseWithCovarianceStamped=PoseWithCovarianceStamped,
        Twist=Twist,
        TwistWithCovariance=TwistWithCovariance,
        Transform=Transform,
        TransformStamped=TransformStamped,
    )
    _mkmod('nav_msgs')
    _mkmod('nav_msgs.msg', Odometry=Odometry, Path=Path)
    _mkmod('sensor_msgs')
    _mkmod('sensor_msgs.msg', LaserScan=LaserScan, Image=Image, Joy=Joy)
    _mkmod('ackermann_msgs')
    _mkmod(
        'ackermann_msgs.msg',
        AckermannDrive=AckermannDrive,
        AckermannDriveStamped=AckermannDriveStamped,
    )
    _mkmod('visualization_msgs')
    _mkmod('visualization_msgs.msg', Marker=Marker, MarkerArray=MarkerArray)
    _mkmod(
        'tf2_ros',
        Buffer=Buffer,
        TransformListener=TransformListener,
        TransformBroadcaster=TransformBroadcaster,
    )

    def _get_pkg_share(pkg_name: str) -> str:
        installed = os.path.join(
            ws_root, 'install', pkg_name, 'share', pkg_name
        )
        if os.path.isdir(installed):
            return installed
        fallback_map = {
            'core': os.path.join(ws_root, 'src', 'core'),
            'simple_planner': os.path.join(
                ws_root, 'src', 'planning', 'simple_planner'
            ),
            'pure_pursuit': os.path.join(
                ws_root, 'src', 'control', 'pure_pursuit'
            ),
            'race_monitor': os.path.join(
                ws_root, 'src', 'monitor', 'race_monitor'
            ),
            'watchdog': os.path.join(
                ws_root, 'src', 'watchdog', 'watchdog'
            ),
            'f1tenth_gym_ros': os.path.join(ws_root, 'f1tenth_gym_ros'),
        }
        if pkg_name in fallback_map and os.path.isdir(
            fallback_map[pkg_name]
        ):
            return fallback_map[pkg_name]
        raise PackageNotFoundError(pkg_name)

    class PackageNotFoundError(KeyError):
        pass

    ament_pkgs = _mkmod(
        'ament_index_python.packages',
        get_package_share_directory=_get_pkg_share,
        PackageNotFoundError=PackageNotFoundError,
    )
    _mkmod(
        'ament_index_python',
        packages=ament_pkgs,
        get_package_share_directory=_get_pkg_share,
    )

    # Launch file AST / description test stubs when ROS launch pkg absent
    class _LaunchEntity:
        def __init__(self, *args: Any, **kwargs: Any):
            self.args = args
            self.kwargs = kwargs

    class LaunchDescription:
        def __init__(self, entities: Optional[List[Any]] = None):
            self.entities = list(entities or [])

        def add_action(self, action: Any) -> None:
            self.entities.append(action)

    _mkmod('launch', LaunchDescription=LaunchDescription)
    _mkmod(
        'launch.actions',
        DeclareLaunchArgument=_LaunchEntity,
        LogInfo=_LaunchEntity,
        IncludeLaunchDescription=_LaunchEntity,
        OpaqueFunction=_LaunchEntity,
    )
    _mkmod('launch.conditions', IfCondition=_LaunchEntity)
    _mkmod(
        'launch.substitutions',
        LaunchConfiguration=_LaunchEntity,
        PathJoinSubstitution=_LaunchEntity,
        Command=_LaunchEntity,
    )
    _mkmod(
        'launch.launch_description_sources',
        PythonLaunchDescriptionSource=_LaunchEntity,
    )
    _mkmod('launch_ros')
    _mkmod('launch_ros.actions', Node=_LaunchEntity)
    _mkmod('launch_ros.substitutions', FindPackageShare=_LaunchEntity)

    return False
