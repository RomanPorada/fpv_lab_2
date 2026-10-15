import math

from geometry_msgs.msg import TwistStamped

from .flight_test_node import FlightTestNode
from .handlers.mission_handlers import MISSION_HANDLERS


class MissionNode(FlightTestNode):
    """Політ до точки: P_target = (x0 - OFFSET_X, y0 - OFFSET_Y)."""

    # Варіант 41 (таблиця 2.3)
    OFFSET_X = 4.6
    OFFSET_Y = 0.2

    HOVER_DURATION = 5.0       # с, стабілізація після набору висоти
    TAKEOFF_TIMEOUT = 30.0     # с
    ALTITUDE_TOLERANCE = 0.2   # м
    POSITION_TOLERANCE = 0.2   # м, допустима похибка
    SETTLE_SPEED = 0.05        # м/с, "майже зупинився"
    SETTLE_TIMEOUT = 3.0       # с
    TARGET_HOLD = 3.0          # с утримання перед посадкою
    MAX_SPEED = 1.0            # м/с
    MIN_SPEED = 0.15           # м/с
    SLOWDOWN_GAIN = 0.8        # 1/с: швидкість = gain * відстань
    FLIGHT_TIMEOUT = 60.0      # с
    MAX_DEVIATION = 1.5        # м, захист від польоту не в той бік

    def __init__(self):
        super().__init__()
        self._handlers = MISSION_HANDLERS
        self.start_position = None
        self.target_position = None
        self.final_position = None
        self.altitude_reached = False
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            "/ap/v1/cmd_vel",
            10,
        )
        self.get_logger().info(
            f"Mission: offset dx={self.OFFSET_X}, dy={self.OFFSET_Y}"
        )

    def odometry_callback(self, message):
        super().odometry_callback(message)
        if self.start_position is None:
            p = message.pose.pose.position
            self.start_position = (p.x, p.y, p.z)
            self.target_position = (
                p.x - self.OFFSET_X,
                p.y - self.OFFSET_Y,
            )
            self.get_logger().info(
                f"Start position: x={p.x:.3f} y={p.y:.3f}"
            )
            self.get_logger().info(
                f"Target position: x={self.target_position[0]:.3f} "
                f"y={self.target_position[1]:.3f}"
            )

    def publish_velocity(self, vx, vy, vz=0.0):
        """Швидкість у системі "map" (ENU): x - схід, y - північ, z - вгору."""
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.twist.linear.x = float(vx)
        message.twist.linear.y = float(vy)
        message.twist.linear.z = float(vz)
        self.cmd_vel_publisher.publish(message)

    def record_final_position(self):
        p = self.odometry.pose.pose.position
        self.final_position = (p.x, p.y)

    def target_error(self):
        return math.hypot(
            self.final_position[0] - self.target_position[0],
            self.final_position[1] - self.target_position[1],
        )

    def log_summary(self):
        log = self.get_logger().info
        log("===== Mission summary (variant 41) =====")
        if self.start_position is not None:
            log(
                f"Start position:          x={self.start_position[0]:.3f} "
                f"y={self.start_position[1]:.3f}"
            )
        if self.target_position is not None:
            log(
                f"Target position:         x={self.target_position[0]:.3f} "
                f"y={self.target_position[1]:.3f}"
            )
        if self.final_position is not None:
            log(
                f"Position before landing: x={self.final_position[0]:.3f} "
                f"y={self.final_position[1]:.3f}"
            )
            log(f"Positioning error:       {self.target_error():.3f} m")
        if self.odometry is not None:
            p = self.odometry.pose.pose.position
            log(
                f"Position after landing:  x={p.x:.3f} y={p.y:.3f} "
                f"z={p.z:.3f}"
            )
        log("Landing completed successfully")
