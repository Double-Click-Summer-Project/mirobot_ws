"""ROS 2 node that bridges safe joint/XYZ commands to Mirobot serial G-code."""

from __future__ import annotations

import math
import termios
import threading
import time
from typing import Optional

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
import serial
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .protocol import (
    CommandValidationError,
    make_joint_command,
    make_pump_command,
    make_xyz_command,
    ordered_joint_positions,
    parse_status,
)


class MirobotDriver(Node):
    """Own the serial port and expose joint, XYZ, and pump interfaces."""

    def __init__(self) -> None:
        super().__init__("driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("status_query_period_s", 0.25)
        self.declare_parameter("status_timeout_s", 2.0)
        self.declare_parameter("min_send_period_s", 0.05)
        self.declare_parameter("homing_confirmation_delay_s", 0.5)
        self.declare_parameter(
            "homing_idle_confirmation_delay_s",
            3.0,
        )
        self.declare_parameter("homing_timeout_s", 60.0)
        self.declare_parameter("motion_completion_timeout_s", 120.0)
        self.declare_parameter("joint_completion_tolerance_deg", 0.5)
        self.declare_parameter("xyz_completion_tolerance_mm", 1.0)
        self.declare_parameter("pump_on_pwm", 1000)
        self.declare_parameter("pump_off_pwm", 0)
        self.declare_parameter("auto_home", False)
        self.declare_parameter("enable_motion_after_auto_home", False)

        self.declare_parameter(
            "joint_names",
            [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
            ],
        )
        self.declare_parameter(
            "joint_lower_limits_deg",
            [-100.0, -30.0, -170.0, -350.0, -205.0, -360.0],
        )
        self.declare_parameter(
            "joint_upper_limits_deg",
            [160.0, 70.0, 60.0, 350.0, 36.0, 360.0],
        )
        self.declare_parameter("joint_feedrate", 2000)

        self.declare_parameter(
            "workspace_lower_mm",
            [140.0, -270.0, 40.0],
        )
        self.declare_parameter(
            "workspace_upper_mm",
            [290.0, 270.0, 300.0],
        )
        self.declare_parameter("xyz_feedrate", 2000)
        self.declare_parameter("cartesian_motion_mode", "G0")

        self.port = str(self.get_parameter("port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.reconnect_period = self._positive_float_parameter(
            "reconnect_period_s"
        )
        self.status_query_period = self._positive_float_parameter(
            "status_query_period_s"
        )
        self.status_timeout = self._positive_float_parameter(
            "status_timeout_s"
        )
        self.min_send_period = self._nonnegative_float_parameter(
            "min_send_period_s"
        )
        self.homing_confirmation_delay = self._positive_float_parameter(
            "homing_confirmation_delay_s"
        )
        self.homing_idle_confirmation_delay = (
            self._positive_float_parameter(
                "homing_idle_confirmation_delay_s"
            )
        )
        self.homing_timeout = self._positive_float_parameter(
            "homing_timeout_s"
        )
        self.motion_completion_timeout = self._positive_float_parameter(
            "motion_completion_timeout_s"
        )
        self.joint_completion_tolerance = self._positive_float_parameter(
            "joint_completion_tolerance_deg"
        )
        self.xyz_completion_tolerance = self._positive_float_parameter(
            "xyz_completion_tolerance_mm"
        )
        if self.homing_timeout <= max(
            self.homing_confirmation_delay,
            self.homing_idle_confirmation_delay,
        ):
            raise ValueError(
                "homing_timeout_s must exceed both homing confirmation "
                "delays"
            )

        self.joint_names = list(self.get_parameter("joint_names").value)
        self.joint_lower_limits = list(
            self.get_parameter("joint_lower_limits_deg").value
        )
        self.joint_upper_limits = list(
            self.get_parameter("joint_upper_limits_deg").value
        )
        self.joint_feedrate = int(
            self.get_parameter("joint_feedrate").value
        )
        self.workspace_lower = list(
            self.get_parameter("workspace_lower_mm").value
        )
        self.workspace_upper = list(
            self.get_parameter("workspace_upper_mm").value
        )
        self.xyz_feedrate = int(self.get_parameter("xyz_feedrate").value)
        self.cartesian_motion_mode = str(
            self.get_parameter("cartesian_motion_mode").value
        )
        self.pump_on_pwm = int(self.get_parameter("pump_on_pwm").value)
        self.pump_off_pwm = int(self.get_parameter("pump_off_pwm").value)
        self.auto_home = bool(
            self.get_parameter("auto_home").value
        )
        self.enable_motion_after_auto_home = bool(
            self.get_parameter(
                "enable_motion_after_auto_home"
            ).value
        )
        try:
            make_pump_command(self.pump_on_pwm)
            make_pump_command(self.pump_off_pwm)
        except CommandValidationError as exc:
            raise ValueError(f"invalid pump PWM parameter: {exc}") from exc

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._connected_publisher = self.create_publisher(
            Bool,
            "connected",
            latched_qos,
        )
        self._motion_enabled_publisher = self.create_publisher(
            Bool,
            "motion_enabled",
            latched_qos,
        )
        self._busy_publisher = self.create_publisher(
            Bool,
            "is_busy",
            state_qos,
        )
        self._status_publisher = self.create_publisher(
            String,
            "status",
            state_qos,
        )
        self._joint_state_publisher = self.create_publisher(
            JointState,
            "joint_states",
            state_qos,
        )
        self._xyz_publisher = self.create_publisher(
            Point,
            "xyz",
            state_qos,
        )

        self.create_subscription(
            JointState,
            "joint_command",
            self._on_joint_command,
            command_qos,
        )
        self.create_subscription(
            Point,
            "xyz_command",
            self._on_xyz_command,
            command_qos,
        )
        self.create_service(Trigger, "home", self._on_home)
        self.create_service(
            SetBool,
            "set_motion_enabled",
            self._on_set_motion_enabled,
        )
        self.create_service(SetBool, "set_pump", self._on_set_pump)

        self._serial: Optional[serial.Serial] = None
        self._serial_lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._last_send_time = 0.0
        self._last_status_time = 0.0
        self._last_state: Optional[str] = None
        self._busy = False
        self._motion_enabled = False
        self._homed = self.dry_run
        self._homing_reserved = False
        self._homing_requested = False
        self._homing_started = False
        self._homing_sent_time = 0.0
        self._command_reserved = False
        self._pending_joint_target: Optional[tuple[float, ...]] = None
        self._pending_xyz_target: Optional[tuple[float, ...]] = None
        self._pending_motion_time = 0.0
        self._connection_epoch = 0
        self._stale_warning_active = False
        self._auto_home_state = (
            "waiting_home_request"
            if self.auto_home
            else "disabled"
        )

        if self.dry_run:
            self.get_logger().warning(
                "dry_run=true: the serial port will not be opened"
            )
        else:
            self._reader_thread = threading.Thread(
                target=self._reader_loop,
                name="mirobot_serial_reader",
                daemon=True,
            )
            self._reader_thread.start()
            self._try_connect()
            self.create_timer(
                self.reconnect_period,
                self._reconnect_timer,
            )
            self.create_timer(
                self.status_query_period,
                self._status_query_timer,
            )

        self.create_timer(0.10, self._state_publish_timer)
        if self.auto_home:
            self.create_timer(0.10, self._auto_home_timer)
            self.get_logger().warning(
                "Automatic homing enabled: the robot will move as soon "
                "as the serial connection is ready"
            )
        self.get_logger().info(
            "Mirobot driver ready: "
            f"port={self.port}, baud={self.baud_rate}, "
            f"joint_topic={self.resolve_topic_name('joint_command')}, "
            f"xyz_topic={self.resolve_topic_name('xyz_command')}"
        )

    def _positive_float_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a positive finite number")
        return value

    def _nonnegative_float_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be a non-negative finite number")
        return value

    def _is_connected(self) -> bool:
        if self.dry_run:
            return False
        with self._serial_lock:
            return self._serial is not None and self._serial.is_open

    def _try_connect(self) -> None:
        if self.dry_run or self._is_connected():
            return

        candidate = None
        try:
            candidate = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.10,
                write_timeout=1.0,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                exclusive=True,
            )
            candidate.reset_input_buffer()
            candidate.reset_output_buffer()
        except (OSError, serial.SerialException, termios.error) as exc:
            if candidate is not None:
                try:
                    candidate.close()
                except (OSError, serial.SerialException, termios.error):
                    pass
            self.get_logger().error(
                f"Cannot open {self.port} at {self.baud_rate}: {exc}"
            )
            return

        with self._serial_lock:
            if self._serial is None:
                self._serial = candidate
                self._last_send_time = 0.0
                with self._state_lock:
                    self._connection_epoch += 1
                    self._last_status_time = 0.0
                    self._last_state = None
                    self._busy = False
                    self._motion_enabled = False
                    self._homed = False
                    self._homing_reserved = False
                    self._homing_requested = False
                    self._homing_started = False
                    self._homing_sent_time = 0.0
                    self._command_reserved = False
                    self._pending_joint_target = None
                    self._pending_xyz_target = None
                    self._pending_motion_time = 0.0
                    self._stale_warning_active = False
                    self._auto_home_state = (
                        "waiting_home_request"
                        if self.auto_home
                        else "disabled"
                    )
            else:
                candidate.close()
                return

        self.get_logger().info(
            f"Serial connected: {self.port} @ {self.baud_rate}"
        )

    def _drop_serial(
        self,
        expected: serial.Serial,
        reason: Exception,
    ) -> None:
        dropped = False
        with self._serial_lock:
            if self._serial is expected:
                self._serial = None
                dropped = True
                try:
                    expected.close()
                except (OSError, serial.SerialException, termios.error):
                    pass
                with self._state_lock:
                    self._connection_epoch += 1
                    self._motion_enabled = False
                    self._busy = False
                    self._last_state = None
                    self._last_status_time = 0.0
                    self._homed = False
                    self._homing_reserved = False
                    self._homing_requested = False
                    self._homing_started = False
                    self._homing_sent_time = 0.0
                    self._command_reserved = False
                    self._pending_joint_target = None
                    self._pending_xyz_target = None
                    self._pending_motion_time = 0.0
                    self._auto_home_state = (
                        "waiting_home_request"
                        if self.auto_home
                        else "disabled"
                    )
        if not dropped:
            return

        self.get_logger().error(
            f"Serial disconnected ({reason}); motion commands disabled"
        )

    def _reconnect_timer(self) -> None:
        if not self._is_connected():
            self._try_connect()

    def _send_line(
        self,
        line: str,
        *,
        log_command: bool = False,
        require_motion_reservation: bool = False,
        require_homing_reservation: bool = False,
    ) -> bool:
        command = str(line).strip()
        if not command:
            return False
        if require_motion_reservation and require_homing_reservation:
            raise ValueError("a command cannot use two reservations")

        if self.dry_run:
            if (
                require_motion_reservation
                or require_homing_reservation
            ):
                with self._state_lock:
                    now = time.monotonic()
                    if require_motion_reservation:
                        if not self._motion_reservation_is_valid_locked(now):
                            return False
                        self._finish_motion_send_locked(now)
                    elif not self._homing_reserved:
                        return False
                    else:
                        self._finish_home_send_locked(now)
            if log_command:
                self.get_logger().info(f"[dry-run] TX: {command}")
            return True

        failure = None
        active_serial = None
        payload = (command + "\r\n").encode("ascii")
        with self._write_lock:
            delay = (
                self.min_send_period
                - (time.monotonic() - self._last_send_time)
            )
            if delay > 0.0:
                time.sleep(delay)

            with self._serial_lock:
                active_serial = self._serial
                if active_serial is None or not active_serial.is_open:
                    return False

                if (
                    require_motion_reservation
                    or require_homing_reservation
                ):
                    with self._state_lock:
                        now = time.monotonic()
                        if (
                            require_motion_reservation
                            and not self._motion_reservation_is_valid_locked(
                                now
                            )
                        ):
                            return False
                        if (
                            require_homing_reservation
                            and not self._homing_reserved
                        ):
                            return False
                        try:
                            written = active_serial.write(payload)
                            if written != len(payload):
                                raise serial.SerialTimeoutException(
                                    "incomplete serial write"
                                )
                            sent_time = time.monotonic()
                            self._last_send_time = sent_time
                            if require_motion_reservation:
                                self._finish_motion_send_locked(sent_time)
                            else:
                                self._finish_home_send_locked(sent_time)
                        except (
                            OSError,
                            serial.SerialException,
                            termios.error,
                        ) as exc:
                            failure = exc
                else:
                    try:
                        written = active_serial.write(payload)
                        if written != len(payload):
                            raise serial.SerialTimeoutException(
                                "incomplete serial write"
                            )
                        self._last_send_time = time.monotonic()
                    except (
                        OSError,
                        serial.SerialException,
                        termios.error,
                    ) as exc:
                        failure = exc

        if failure is not None and active_serial is not None:
            self._drop_serial(active_serial, failure)
            return False

        if log_command:
            self.get_logger().info(f"TX: {command}")
        return True

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            raw = b""
            failure = None
            source_epoch = 0
            line = ""
            with self._serial_lock:
                active_serial = self._serial
                if active_serial is not None:
                    source_epoch = self._connection_epoch

            if active_serial is None:
                self._stop_event.wait(0.05)
                continue

            # pyserial supports one reader and one writer concurrently.
            # Do not hold the serial lifetime lock during this blocking
            # read: doing so can starve homing commands and status queries.
            try:
                raw = active_serial.readline()
            except (
                OSError,
                serial.SerialException,
                termios.error,
            ) as exc:
                failure = exc

            if failure is not None:
                self._drop_serial(active_serial, failure)
                continue

            if raw:
                line = raw.decode(
                    "utf-8",
                    errors="replace",
                ).strip()
                if line:
                    self._handle_serial_line(line, source_epoch)

    def _handle_serial_line(
        self,
        line: str,
        source_epoch: Optional[int] = None,
    ) -> None:
        if source_epoch is not None:
            with self._state_lock:
                if source_epoch != self._connection_epoch:
                    return

        status = parse_status(line)
        if status is None:
            lowered = line.casefold()
            if any(
                token in lowered
                for token in (
                    "error",
                    "alarm",
                    "lock",
                    "using reset pos",
                )
            ):
                self._set_fault_state(line, source_epoch)
                self.get_logger().error(f"RX: {line}")
            else:
                self.get_logger().debug(f"RX: {line}")
            return

        status_message = String()
        status_message.data = status.raw
        self._status_publisher.publish(status_message)
        state_lower = status.state.casefold()

        if not status.complete:
            if any(
                token in state_lower
                for token in ("error", "alarm", "lock")
            ):
                self._set_fault_state(status.raw, source_epoch)
            self.get_logger().warning(
                "Ignoring incomplete Mirobot status frame"
            )
            return

        now = time.monotonic()
        home_completed = False
        motion_completed = False
        fault_detected = any(
            token in state_lower
            for token in ("error", "alarm", "lock")
        )

        with self._state_lock:
            if (
                source_epoch is not None
                and source_epoch != self._connection_epoch
            ):
                return
            self._last_status_time = now
            self._last_state = status.state
            self._stale_warning_active = False

            if (
                self._homing_requested
                and now - self._homing_sent_time
                >= self.homing_confirmation_delay
                and state_lower != "idle"
                and not fault_detected
            ):
                self._homing_started = True

            if fault_detected:
                self._motion_enabled = False
                self._busy = True
                self._homed = False
                self._homing_reserved = False
                self._homing_requested = False
                self._homing_started = False
                self._homing_sent_time = 0.0
                self._command_reserved = False
                self._pending_joint_target = None
                self._pending_xyz_target = None
                self._pending_motion_time = 0.0
            elif state_lower == "idle":
                if self._homing_reserved:
                    self._busy = True
                elif self._homing_requested:
                    idle_confirmation_ready = (
                        self._homing_started
                        or now - self._homing_sent_time
                        >= self.homing_idle_confirmation_delay
                    )
                    if idle_confirmation_ready:
                        self._homed = True
                        self._homing_requested = False
                        self._homing_started = False
                        self._homing_sent_time = 0.0
                        self._busy = False
                        home_completed = True
                    else:
                        self._busy = True
                elif self._command_reserved:
                    self._busy = True
                elif (
                    self._pending_joint_target is not None
                    and status.joint_degrees is not None
                ):
                    if self._target_reached(
                        status.joint_degrees,
                        self._pending_joint_target,
                        self.joint_completion_tolerance,
                    ):
                        self._pending_joint_target = None
                        self._pending_motion_time = 0.0
                        self._busy = False
                        motion_completed = True
                    else:
                        self._busy = True
                elif (
                    self._pending_xyz_target is not None
                    and status.xyz_mm is not None
                ):
                    if self._target_reached(
                        status.xyz_mm,
                        self._pending_xyz_target,
                        self.xyz_completion_tolerance,
                    ):
                        self._pending_xyz_target = None
                        self._pending_motion_time = 0.0
                        self._busy = False
                        motion_completed = True
                    else:
                        self._busy = True
                else:
                    self._busy = False
            else:
                self._busy = True

        if fault_detected:
            self.get_logger().error(
                "Mirobot fault detected; motion commands disabled"
            )
        elif home_completed:
            self.get_logger().info(
                "Homing completion confirmed by post-command Idle status"
            )
        elif motion_completed:
            self.get_logger().info(
                "Motion completion confirmed by Idle target feedback"
            )

        if status.joint_degrees is not None:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = list(self.joint_names)
            joint_state.position = [
                math.radians(value)
                for value in status.joint_degrees
            ]
            self._joint_state_publisher.publish(joint_state)

        if status.xyz_mm is not None:
            point = Point()
            point.x = status.xyz_mm[0] / 1000.0
            point.y = status.xyz_mm[1] / 1000.0
            point.z = status.xyz_mm[2] / 1000.0
            self._xyz_publisher.publish(point)

    def _set_fault_state(
        self,
        raw: str,
        source_epoch: Optional[int] = None,
    ) -> None:
        with self._state_lock:
            if (
                source_epoch is not None
                and source_epoch != self._connection_epoch
            ):
                return
            self._motion_enabled = False
            self._busy = True
            self._last_state = None
            self._last_status_time = 0.0
            self._homed = False
            self._homing_reserved = False
            self._homing_requested = False
            self._homing_started = False
            self._homing_sent_time = 0.0
            self._command_reserved = False
            self._pending_joint_target = None
            self._pending_xyz_target = None
            self._pending_motion_time = 0.0
            self._stale_warning_active = False
        self.get_logger().error(
            f"Fault/reset response detected; motion disabled: {raw}"
        )

    @staticmethod
    def _target_reached(
        actual: tuple[float, ...],
        target: tuple[float, ...],
        tolerance: float,
    ) -> bool:
        return len(actual) == len(target) and all(
            abs(actual_value - target_value) <= tolerance
            for actual_value, target_value in zip(actual, target)
        )

    def _status_query_timer(self) -> None:
        if self._is_connected():
            self._send_line("?")

    def _status_is_fresh_locked(self, now: float) -> bool:
        return (
            self._last_status_time > 0.0
            and now - self._last_status_time <= self.status_timeout
        )

    def _state_publish_timer(self) -> None:
        now = time.monotonic()
        log_stale = False
        log_homing_timeout = False
        log_motion_timeout = False
        with self._state_lock:
            status_is_fresh = self._status_is_fresh_locked(now)
            idle_and_fresh = (
                status_is_fresh
                and self._last_state is not None
                and self._last_state.casefold() == "idle"
            )
            if (
                not self.dry_run
                and self._homing_requested
                and self._homing_sent_time > 0.0
                and now - self._homing_sent_time > self.homing_timeout
            ):
                self._motion_enabled = False
                self._homed = False
                self._homing_requested = False
                self._homing_started = False
                self._homing_sent_time = 0.0
                self._busy = not idle_and_fresh
                log_homing_timeout = True

            if (
                not self.dry_run
                and self._pending_motion_time > 0.0
                and now - self._pending_motion_time
                > self.motion_completion_timeout
            ):
                self._motion_enabled = False
                self._command_reserved = False
                self._pending_joint_target = None
                self._pending_xyz_target = None
                self._pending_motion_time = 0.0
                self._busy = not idle_and_fresh
                log_motion_timeout = True

            if (
                not self.dry_run
                and self._motion_enabled
                and not status_is_fresh
            ):
                self._motion_enabled = False
                if not self._stale_warning_active:
                    log_stale = True
                self._stale_warning_active = True
            enabled_value = self._motion_enabled
            busy_value = self._busy

        if log_stale:
            self.get_logger().error(
                "Mirobot status timed out; motion commands disabled"
            )
        if log_homing_timeout:
            self.get_logger().error(
                "Homing confirmation timed out; motion remains disabled"
            )
        if log_motion_timeout:
            self.get_logger().error(
                "Motion completion timed out; motion commands disabled"
            )

        connected = Bool()
        connected.data = self._is_connected()
        self._connected_publisher.publish(connected)

        enabled = Bool()
        enabled.data = enabled_value
        self._motion_enabled_publisher.publish(enabled)

        busy = Bool()
        busy.data = busy_value
        self._busy_publisher.publish(busy)

    def _auto_home_is_active_locked(self) -> bool:
        return self._auto_home_state in {
            "waiting_home_request",
            "waiting_home_completion",
        }

    def _external_motion_blocked_by_auto_home(self) -> bool:
        with self._state_lock:
            blocked = self._auto_home_is_active_locked()
        if blocked:
            self.get_logger().warning(
                "Motion command rejected: automatic homing is in progress"
            )
        return blocked

    def _auto_home_timer(self) -> None:
        with self._state_lock:
            stage = self._auto_home_state

        if stage == "waiting_home_request":
            if not self.dry_run and not self._is_connected():
                return

            response = self._on_home(
                Trigger.Request(),
                Trigger.Response(),
            )
            if (
                response.success
                or response.message == "homing is already in progress"
            ):
                with self._state_lock:
                    if self._auto_home_state == stage:
                        self._auto_home_state = (
                            "waiting_home_completion"
                        )
                self.get_logger().warning(
                    "Automatic homing started"
                )
                return

            with self._state_lock:
                if self._auto_home_state == stage:
                    self._auto_home_state = "failed"
            self.get_logger().error(
                "Automatic homing failed to start: "
                f"{response.message}"
            )
            return

        if stage != "waiting_home_completion":
            return

        failed = False
        completed = False
        with self._state_lock:
            if (
                self._homed
                and not self._homing_reserved
                and not self._homing_requested
            ):
                self._auto_home_state = "complete"
                self._motion_enabled = (
                    self.enable_motion_after_auto_home
                )
                completed = True
            elif (
                not self._homing_reserved
                and not self._homing_requested
                and not self._homed
            ):
                self._auto_home_state = "failed"
                failed = True

        if failed:
            self.get_logger().error(
                "Automatic homing stopped because completion was not "
                "confirmed"
            )
        elif completed:
            suffix = (
                "motion commands remain enabled"
                if self.enable_motion_after_auto_home
                else "motion commands remain disabled"
            )
            self.get_logger().info(
                f"Automatic homing complete; {suffix}"
            )

    def _motion_command_allowed(self) -> bool:
        if not self.dry_run and not self._is_connected():
            with self._state_lock:
                self._motion_enabled = False
            self.get_logger().error(
                "Motion command rejected: serial port is disconnected"
            )
            return False

        rejection = None
        now = time.monotonic()
        with self._state_lock:
            if not self._motion_enabled:
                rejection = "call set_motion_enabled first"
            elif not self.dry_run and not self._homed:
                rejection = "homing has not completed for this connection"
            elif (
                not self.dry_run
                and not self._status_is_fresh_locked(now)
            ):
                self._motion_enabled = False
                rejection = "Mirobot status is stale"
            elif (
                not self.dry_run
                and (
                    self._last_state is None
                    or self._last_state.casefold() != "idle"
                )
            ):
                rejection = (
                    f"Mirobot state is {self._last_state}, not Idle"
                )
            elif self._busy or self._command_reserved:
                rejection = "the previous motion has not completed"
            else:
                # Reserve the single in-flight motion slot.
                self._command_reserved = True

        if rejection is not None:
            self.get_logger().warning(
                f"Motion command rejected: {rejection}"
            )
            return False
        return True

    def _cancel_motion_reservation(self) -> None:
        with self._state_lock:
            self._command_reserved = False

    def _motion_reservation_is_valid_locked(self, now: float) -> bool:
        if self.dry_run:
            return self._motion_enabled and self._command_reserved
        return (
            self._motion_enabled
            and self._homed
            and self._command_reserved
            and self._status_is_fresh_locked(now)
            and self._last_state is not None
            and self._last_state.casefold() == "idle"
        )

    def _stage_motion_target(
        self,
        *,
        joint_target: Optional[tuple[float, ...]] = None,
        xyz_target: Optional[tuple[float, ...]] = None,
    ) -> bool:
        if (joint_target is None) == (xyz_target is None):
            raise ValueError("exactly one motion target must be provided")

        with self._state_lock:
            if not self._motion_reservation_is_valid_locked(
                time.monotonic()
            ):
                self._command_reserved = False
                return False

            if not self.dry_run:
                self._pending_joint_target = joint_target
                self._pending_xyz_target = xyz_target
                self._pending_motion_time = 0.0
                self._busy = True
            return True

    def _finish_motion_send_locked(self, sent_time: float) -> None:
        self._command_reserved = False
        if self.dry_run:
            self._pending_joint_target = None
            self._pending_xyz_target = None
            self._pending_motion_time = 0.0
            self._busy = False
        else:
            self._pending_motion_time = sent_time

    def _finish_home_send_locked(self, sent_time: float) -> None:
        self._homing_reserved = False
        self._homing_started = False
        if self.dry_run:
            self._homed = True
            self._homing_requested = False
            self._homing_sent_time = 0.0
            self._busy = False
        else:
            self._homed = False
            self._homing_requested = True
            self._homing_sent_time = sent_time
            self._busy = True

    def _abort_staged_motion(self) -> None:
        with self._state_lock:
            self._command_reserved = False
            self._pending_joint_target = None
            self._pending_xyz_target = None
            self._pending_motion_time = 0.0

    def _on_joint_command(self, message: JointState) -> None:
        if self._external_motion_blocked_by_auto_home():
            return
        self._send_joint_command(message)

    def _send_joint_command(self, message: JointState) -> bool:
        if not self._motion_command_allowed():
            return False

        try:
            command = make_joint_command(
                message.name,
                message.position,
                self.joint_names,
                self.joint_lower_limits,
                self.joint_upper_limits,
                self.joint_feedrate,
            )
            ordered_positions = ordered_joint_positions(
                message.name,
                message.position,
                self.joint_names,
            )
            target = tuple(
                math.degrees(value) for value in ordered_positions
            )
        except CommandValidationError as exc:
            self._cancel_motion_reservation()
            self.get_logger().error(f"Joint command rejected: {exc}")
            return False

        if not self._stage_motion_target(joint_target=target):
            self.get_logger().error(
                "Joint command cancelled because robot state changed"
            )
            return False

        if not self._send_line(
            command,
            log_command=True,
            require_motion_reservation=True,
        ):
            self._abort_staged_motion()
            self.get_logger().error(
                "Joint command was not sent because state changed "
                "or serial is unavailable"
            )
            return False
        return True

    def _on_xyz_command(self, message: Point) -> None:
        if self._external_motion_blocked_by_auto_home():
            return
        if not self._motion_command_allowed():
            return

        try:
            command = make_xyz_command(
                (message.x, message.y, message.z),
                self.workspace_lower,
                self.workspace_upper,
                self.xyz_feedrate,
                self.cartesian_motion_mode,
            )
            target = (
                message.x * 1000.0,
                message.y * 1000.0,
                message.z * 1000.0,
            )
        except CommandValidationError as exc:
            self._cancel_motion_reservation()
            self.get_logger().error(f"XYZ command rejected: {exc}")
            return

        if not self._stage_motion_target(xyz_target=target):
            self.get_logger().error(
                "XYZ command cancelled because robot state changed"
            )
            return

        if not self._send_line(
            command,
            log_command=True,
            require_motion_reservation=True,
        ):
            self._abort_staged_motion()
            self.get_logger().error(
                "XYZ command was not sent because state changed "
                "or serial is unavailable"
            )

    def _on_home(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        connected = self._is_connected()
        if not self.dry_run and not connected:
            response.success = False
            response.message = "serial port is disconnected"
            return response

        with self._state_lock:
            state_lower = (
                self._last_state.casefold()
                if self._last_state is not None
                else None
            )
            if self._homing_reserved or self._homing_requested:
                response.success = False
                response.message = "homing is already in progress"
                return response
            if self._command_reserved:
                response.success = False
                response.message = "a motion command is being sent"
                return response
            if (
                not self.dry_run
                and self._busy
                and state_lower not in {None, "alarm"}
            ):
                response.success = False
                response.message = (
                    f"cannot home while Mirobot state is {self._last_state}"
                )
                return response

            self._motion_enabled = False
            self._homed = False
            self._homing_reserved = True
            self._homing_requested = False
            self._homing_started = False
            self._homing_sent_time = 0.0
            self._command_reserved = False
            self._pending_joint_target = None
            self._pending_xyz_target = None
            self._pending_motion_time = 0.0
            self._busy = not self.dry_run

        if self._send_line(
            "$H",
            log_command=True,
            require_homing_reservation=True,
        ):
            response.success = True
            if self.dry_run:
                response.message = "dry-run homing command logged"
            else:
                response.message = (
                    "homing command sent; "
                    "wait for post-command Idle confirmation"
                )
        else:
            with self._state_lock:
                self._homing_reserved = False
                self._homing_requested = False
                self._homing_started = False
                self._homing_sent_time = 0.0
                self._busy = (
                    self._last_state is not None
                    and self._last_state.casefold() != "idle"
                )
            response.success = False
            response.message = "failed to send homing command"
        return response

    def _on_set_motion_enabled(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if not request.data:
            with self._state_lock:
                self._motion_enabled = False
                auto_home_cancelled = self._auto_home_is_active_locked()
                if auto_home_cancelled:
                    self._auto_home_state = "cancelled"
            response.success = True
            response.message = (
                "future motion commands disabled; "
                "current motion is not stopped"
            )
            if auto_home_cancelled:
                response.message += (
                    "; automatic homing completion tracking was cancelled"
                )
            return response

        rejection = None
        already_enabled = False
        now = time.monotonic()
        connected = self._is_connected()
        with self._state_lock:
            if self._auto_home_is_active_locked():
                self._motion_enabled = False
                rejection = (
                    "automatic homing is in progress"
                )
            elif self.dry_run:
                already_enabled = self._motion_enabled
                self._motion_enabled = True
            elif not connected:
                self._motion_enabled = False
                rejection = "serial port is disconnected"
            elif not self._homed:
                self._motion_enabled = False
                rejection = "homing has not completed for this connection"
            elif not self._status_is_fresh_locked(now):
                self._motion_enabled = False
                rejection = "Mirobot status is missing or stale"
            elif self._last_state is None:
                self._motion_enabled = False
                rejection = "no complete Mirobot status received yet"
            elif self._motion_enabled:
                already_enabled = True
            elif self._last_state.casefold() != "idle":
                rejection = (
                    f"Mirobot state is {self._last_state}, not Idle"
                )
            elif self._busy or self._command_reserved:
                rejection = "Mirobot is still busy"
            else:
                self._motion_enabled = True

        if rejection is not None:
            response.success = False
            response.message = rejection
            return response

        response.success = True
        response.message = (
            "motion commands already enabled"
            if already_enabled
            else "motion commands enabled"
        )
        return response

    def _on_set_pump(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if not self.dry_run and not self._is_connected():
            response.success = False
            response.message = "serial port is disconnected"
            return response

        pwm = self.pump_on_pwm if request.data else self.pump_off_pwm
        command = make_pump_command(pwm)
        if not self._send_line(command, log_command=True):
            response.success = False
            response.message = "failed to send pump command"
            return response

        state = "on" if request.data else "off"
        response.success = True
        response.message = (
            f"pump {state} command sent (PWM={pwm}); "
            "device acknowledgement is not available"
        )
        return response

    def destroy_node(self) -> None:
        with self._state_lock:
            self._motion_enabled = False
            self._command_reserved = False
        self._stop_event.set()

        with self._write_lock:
            with self._serial_lock:
                active_serial = self._serial
                self._serial = None
                if active_serial is not None:
                    try:
                        active_serial.close()
                    except (
                        OSError,
                        serial.SerialException,
                        termios.error,
                    ):
                        pass
                with self._state_lock:
                    self._connection_epoch += 1
                    self._homing_reserved = False
                    self._homing_requested = False
                    self._homing_started = False
                    self._homing_sent_time = 0.0
                    self._pending_joint_target = None
                    self._pending_xyz_target = None
                    self._pending_motion_time = 0.0
                    self._busy = False

        if (
            self._reader_thread is not None
            and self._reader_thread.is_alive()
        ):
            self._reader_thread.join(timeout=0.5)

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MirobotDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
