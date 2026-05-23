"""autoproject — autonomous indoor navigation robot (simulation-first).

Layered architecture (built bottom-up, one phase at a time):

    Layer 0  simulation   — World model, physics, sensor noise, ground truth
    Layer 2  comms        — IRobotComms (+ Real/Sim)
    Layer 2  vision       — ICamera, IAprilTagDetector (+ Real/Sim)
    Layer 3  algorithms   — occupancy grid, A*, smoothing, pure pursuit
    Layer 4  localization  — wheel odometry, complementary/EKF fusion
    Layer 5  navigation   — state machine planner/executor/recovery
    Layer 6  ui           — Flask + WebSocket visualization & logging

`factory.py` wires Real* vs Sim* implementations per config/runtime.yaml.
"""

__version__ = "0.0.0"
