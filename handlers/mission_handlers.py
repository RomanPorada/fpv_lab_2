import math
import time
from typing import TYPE_CHECKING

from ..flight_state import FlightState
from . import HANDLERS
from .handler import FlightStateHandler
from .waiting_for_landing import WaitingForLandingHandler

if TYPE_CHECKING:
    from ..mission_node import MissionNode


class MissionHoveringHandler(FlightStateHandler):
    """Чекає набору висоти, стабілізується, потім летить до цілі."""

    def handle(self, node: "MissionNode") -> None:
        if node.status is None or not node.status.armed:
            node.fail("Drone disarmed unexpectedly during flight")
            return
        if node.odometry is None or node.start_position is None:
            return

        now = time.monotonic()
        climbed = node.odometry.pose.pose.position.z - node.start_position[2]

        if not node.altitude_reached:
            if climbed >= node.TAKEOFF_ALTITUDE - node.ALTITUDE_TOLERANCE:
                node.altitude_reached = True
                node.hover_started_at = now
                node.get_logger().info(
                    f"Altitude reached: {climbed:.2f} m, "
                    f"hovering for {node.HOVER_DURATION:.1f} s"
                )
            elif now - node.hover_started_at > node.TAKEOFF_TIMEOUT:
                node.fail("Takeoff altitude was not reached in time")
            return

        if now - node.hover_started_at >= node.HOVER_DURATION:
            node.get_logger().info("Hover completed, flying to target")
            node.state = FlightState.FLYING_TO_TARGET


class FlyingToTargetHandler(FlightStateHandler):
    """Політ до цільової точки з плавним сповільненням і зупинкою."""

    def __init__(self):
        self._started_at = None
        self._initial_distance = None
        self._reached_since = None
        self._hold_until = None
        self._last_log = 0.0

    def _land(self, node: "MissionNode", reason: str) -> None:
        node.get_logger().warning(reason)
        node.publish_velocity(0.0, 0.0)
        node.record_final_position()
        node.state = FlightState.LANDING

    def handle(self, node: "MissionNode") -> None:
        if node.status is None or not node.status.armed:
            node.fail("Drone disarmed unexpectedly during flight")
            return
        if node.odometry is None or node.target_position is None:
            return

        now = time.monotonic()

        # Утримання в цільовій точці перед посадкою (час для скріншота)
        if self._hold_until is not None:
            node.publish_velocity(0.0, 0.0)
            if now >= self._hold_until:
                node.record_final_position()
                node.get_logger().info("Hold finished, starting landing")
                node.state = FlightState.LANDING
            return

        position = node.odometry.pose.pose.position
        dx = node.target_position[0] - position.x
        dy = node.target_position[1] - position.y
        distance = math.hypot(dx, dy)

        if self._started_at is None:
            self._started_at = now
            self._initial_distance = distance
            node.get_logger().info(
                f"Flying to target, distance {distance:.2f} m"
            )

        if now - self._last_log >= 1.0:
            self._last_log = now
            node.get_logger().info(
                f"Position x={position.x:.2f} y={position.y:.2f}, "
                f"distance to target {distance:.2f} m"
            )

        if distance > self._initial_distance + node.MAX_DEVIATION:
            self._land(node, "Drone is moving away from the target, landing")
            return
        if now - self._started_at > node.FLIGHT_TIMEOUT:
            self._land(node, "Target was not reached in time, landing")
            return

        if distance <= node.POSITION_TOLERANCE:
            node.publish_velocity(0.0, 0.0)
            if self._reached_since is None:
                self._reached_since = now
            linear = node.odometry.twist.twist.linear
            speed_now = math.hypot(linear.x, linear.y)
            settled = (
                speed_now <= node.SETTLE_SPEED
                or now - self._reached_since > node.SETTLE_TIMEOUT
            )
            if settled:
                node.get_logger().info(
                    f"Target reached, error {distance:.3f} m, "
                    f"holding {node.TARGET_HOLD:.1f} s"
                )
                self._hold_until = now + node.TARGET_HOLD
            return

        self._reached_since = None
        speed = min(
            node.MAX_SPEED,
            max(node.MIN_SPEED, node.SLOWDOWN_GAIN * distance),
        )
        node.publish_velocity(speed * dx / distance, speed * dy / distance)


class MissionWaitingForLandingHandler(WaitingForLandingHandler):
    def handle(self, node: "MissionNode") -> None:
        super().handle(node)
        if node.state == FlightState.FINISHED:
            node.log_summary()


MISSION_HANDLERS = {
    **HANDLERS,
    FlightState.HOVERING: MissionHoveringHandler(),
    FlightState.FLYING_TO_TARGET: FlyingToTargetHandler(),
    FlightState.WAITING_FOR_LANDING: MissionWaitingForLandingHandler(),
}
