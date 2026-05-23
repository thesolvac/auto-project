"""Drive the simulated robot and print telemetry (Phase 2 acceptance demo).

Run with:  python -m autoproject.examples.sim_drive

Builds a World from the demo scenario, gets a SimRobotComms via the factory
(``mode: sim``), commands a gentle arc, and prints telemetry as the simulation
advances — proving the I/O abstraction runs end-to-end with no hardware.
"""

from __future__ import annotations

import logging

from autoproject.factory import build_robot_comms, load_runtime
from autoproject.simulation.world import World
from autoproject.utils.config import CONFIG_DIR

logger = logging.getLogger(__name__)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    world = World.from_scenario(CONFIG_DIR / "sim_scenarios" / "demo_room.yaml")
    comms = build_robot_comms(load_runtime(), world=world)
    comms.on_obstacle(lambda sensor, dist: logger.info("  ! OBSTACLE %s @ %.2f m", sensor, dist))
    comms.on_slip(lambda: logger.info("  ! SLIP"))

    comms.move(0.10, 0.12)  # gentle left-curving arc
    for i in range(40):
        tel = comms.step()
        if i % 8 == 0:
            pose = world.pose
            logger.info(
                "t=%4.2fs  encL=%5d encR=%5d  front=%.2f rear=%.2f  pose=(%.2f, %.2f, %.2f)",
                tel.timestamp_s,
                tel.enc_left_counts,
                tel.enc_right_counts,
                tel.dist_front_m,
                tel.dist_rear_m,
                pose.x,
                pose.y,
                pose.theta,
            )
    comms.stop()
    logger.info("done; final pose=(%.2f, %.2f, %.2f)", world.pose.x, world.pose.y, world.pose.theta)


if __name__ == "__main__":
    main()
