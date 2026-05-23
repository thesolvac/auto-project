"""Navigation state machine (Layer 5).

Orchestrates the full closed loop in simulation: read telemetry, fuse pose
(odometry + AprilTags), plan with A*, follow with pure pursuit, and recover from
obstacles and slip. Depends only on the abstract I/O interfaces, the planning
algorithms, and the pose filter — never on hardware or the simulator directly.

States and transitions (see docs/architecture.md for the committed diagram):

    IDLE        -> PLANNING        (a goal is set)
    PLANNING    -> EXECUTING       (a path was found)
                -> EMERGENCY       (no path exists)
    EXECUTING   -> REACHED         (goal within tolerance)
                -> AVOIDING        (ultrasonic < threshold)
                -> RELOCALIZING    (ERR SLIP)
    AVOIDING    -> PLANNING        (obstacle added to the map, replan)
    RELOCALIZING-> PLANNING        (a fresh AprilTag fix arrived)
"""

from __future__ import annotations

import math
from enum import StrEnum

from autoproject.algorithms.astar import astar
from autoproject.algorithms.occupancy_grid import OccupancyGrid
from autoproject.algorithms.path_smoother import smooth_path
from autoproject.algorithms.pure_pursuit import PurePursuit
from autoproject.comms.interfaces import IRobotComms
from autoproject.localization.pose_fusion import PoseFilter
from autoproject.simulation.geometry import normalize_angle
from autoproject.vision.interfaces import IAprilTagDetector


class NavState(StrEnum):
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    AVOIDING = "avoiding"
    RELOCALIZING = "relocalizing"
    EMERGENCY = "emergency"
    REACHED = "reached"


_TERMINAL = {NavState.REACHED, NavState.EMERGENCY}


class Navigator:
    """Closed-loop navigation state machine driven one tick at a time."""

    def __init__(
        self,
        comms: IRobotComms,
        detector: IAprilTagDetector,
        base_grid: OccupancyGrid,
        pose_filter: PoseFilter,
        pursuit: PurePursuit,
        *,
        goal: tuple[float, float, float],
        tag_map: dict[int, tuple[float, float]],
        inflate_radius_m: float,
        wheel_radius_m: float = 0.040,
        counts_per_rev: int = 4096,
        goal_pos_tol_m: float = 0.05,
        goal_heading_tol_rad: float = math.radians(5.0),
        obstacle_size_m: float = 0.25,
        rotate_speed_mps: float = 0.08,
        relocalize_timeout_ticks: int = 100,
        slip_cooldown_ticks: int = 150,
    ) -> None:
        self.comms = comms
        self.detector = detector
        self.pose_filter = pose_filter
        self.pursuit = pursuit
        self.goal = goal
        self.tag_map = tag_map
        self.goal_pos_tol_m = goal_pos_tol_m
        self.goal_heading_tol_rad = goal_heading_tol_rad
        self.obstacle_size_m = obstacle_size_m
        self.rotate_speed_mps = rotate_speed_mps
        self.relocalize_timeout_ticks = relocalize_timeout_ticks
        self.slip_cooldown_ticks = slip_cooldown_ticks

        self._base_grid = base_grid
        self._inflate_radius_m = inflate_radius_m
        self.grid = base_grid.inflate(inflate_radius_m)
        self.metres_per_count = (2.0 * math.pi * wheel_radius_m) / counts_per_rev

        self.state = NavState.IDLE
        self.path: list[tuple[float, float]] = []
        self._prev_left: int | None = None
        self._prev_right: int | None = None
        self._obstacle_event: tuple[str, float] | None = None
        self._pending_obstacle: tuple[float, float] | None = None
        self._slip_event = False
        self._got_fix = False
        self._reloc_wait = 0
        self._slip_cooldown = 0

        comms.on_obstacle(self._on_obstacle)
        comms.on_slip(self._on_slip)

    # --- event callbacks (set flags consumed by the state machine) ---
    def _on_obstacle(self, sensor: str, distance_m: float) -> None:
        self._obstacle_event = (sensor, distance_m)

    def _on_slip(self) -> None:
        self._slip_event = True

    # --- main loop ---
    def tick(self) -> NavState:
        """Advance one control cycle and return the resulting state."""
        # SimRobotComms.step advances the world with the last command and fires
        # obstacle/slip callbacks; RealRobotComms would surface the same events.
        telemetry = (
            self.comms.step()
            if hasattr(self.comms, "step")
            else self.comms.get_telemetry()
        )
        self._localize(telemetry)
        self._run_state()
        return self.state

    def run(self, max_ticks: int = 5000) -> NavState:
        """Tick until a terminal state or ``max_ticks``; returns the final state."""
        if self.state == NavState.IDLE:
            self.state = NavState.PLANNING
        for _ in range(max_ticks):
            if self.tick() in _TERMINAL:
                break
        return self.state

    # --- localization ---
    def _localize(self, telemetry) -> None:
        self._got_fix = False
        if telemetry is None:
            return
        if self._prev_left is None:
            self._prev_left = telemetry.enc_left_counts
            self._prev_right = telemetry.enc_right_counts
        d_left = (telemetry.enc_left_counts - self._prev_left) * self.metres_per_count
        d_right = (
            telemetry.enc_right_counts - self._prev_right
        ) * self.metres_per_count
        self._prev_left = telemetry.enc_left_counts
        self._prev_right = telemetry.enc_right_counts
        self.pose_filter.predict(d_left, d_right)

        for det in self.detector.detect():
            known = det.tag_id in self.tag_map
            if known and det.range_m is not None and det.bearing_rad is not None:
                self.pose_filter.update(
                    self.tag_map[det.tag_id], det.range_m, det.bearing_rad
                )
                self._got_fix = True

    # --- state machine ---
    def _run_state(self) -> None:
        if self.state == NavState.IDLE:
            return
        if self.state == NavState.PLANNING:
            self._plan()
        elif self.state == NavState.EXECUTING:
            self._execute()
        elif self.state == NavState.AVOIDING:
            self._avoid()
        elif self.state == NavState.RELOCALIZING:
            self._relocalize()
        # REACHED / EMERGENCY are terminal: ensure the robot is stopped.
        elif self.state in _TERMINAL:
            self.comms.stop()

    def _plan(self) -> None:
        pose = self.pose_filter.pose
        raw = astar(self.grid, (pose[0], pose[1]), (self.goal[0], self.goal[1]))
        if raw is None:
            self.comms.stop()
            self.state = NavState.EMERGENCY
            return
        self.path = smooth_path(self.grid, raw)
        self.pursuit.reset()
        self.state = NavState.EXECUTING

    def _execute(self) -> None:
        if self._slip_cooldown > 0:
            self._slip_cooldown -= 1
            self._slip_event = False  # ignore slip while recovering from the last one
        if self._slip_event:
            self._slip_event = False
            self.comms.stop()
            self._reloc_wait = 0
            self.state = NavState.RELOCALIZING
            return

        event = self._obstacle_event
        self._obstacle_event = None
        if event is not None and event[0] == "front":
            ox, oy = self._obstacle_world_pos(event[1])
            # Only react to *unknown* obstacles; known map features (which the
            # planned path already clears) are expected and ignored.
            if self._base_grid.is_free(ox, oy):
                self._pending_obstacle = (ox, oy)
                self.comms.stop()
                self.state = NavState.AVOIDING
                return

        pose = self.pose_filter.pose
        dist = math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])
        if dist < self.goal_pos_tol_m:
            heading_err = normalize_angle(self.goal[2] - pose[2])
            if abs(heading_err) < self.goal_heading_tol_rad:
                self.comms.stop()
                self.state = NavState.REACHED
                return
            # In position; rotate in place toward the goal heading.
            turn = self.rotate_speed_mps if heading_err > 0 else -self.rotate_speed_mps
            self.comms.move(-turn, turn)
            return

        v_left, v_right = self.pursuit.compute(pose, self.path)
        self.comms.move(v_left, v_right)

    def _avoid(self) -> None:
        # Add the detected obstacle to the map, then replan around it.
        if self._pending_obstacle is not None:
            ox, oy = self._pending_obstacle
            half = self.obstacle_size_m / 2.0
            self._base_grid.mark_rect(ox - half, oy - half, ox + half, oy + half)
            self.grid = self._base_grid.inflate(self._inflate_radius_m)
            self._pending_obstacle = None
        self.state = NavState.PLANNING

    def _obstacle_world_pos(self, distance_m: float) -> tuple[float, float]:
        """World position of an obstacle ``distance_m`` ahead along the heading."""
        pose = self.pose_filter.pose
        reach = distance_m + self.obstacle_size_m / 2.0
        return pose[0] + reach * math.cos(pose[2]), pose[1] + reach * math.sin(pose[2])

    def _relocalize(self) -> None:
        self.comms.stop()
        self._reloc_wait += 1
        # Replan once a fresh AprilTag fix corrects the pose. If none arrives
        # within the timeout, replan anyway from the odometry estimate so the
        # robot never deadlocks waiting for a tag it cannot see.
        if self._got_fix or self._reloc_wait >= self.relocalize_timeout_ticks:
            self._slip_cooldown = self.slip_cooldown_ticks
            self._reloc_wait = 0
            self.state = NavState.PLANNING
