# parameters_config.py

import numpy as np
from dataclasses import dataclass, field
from pal.products.qcar import IS_PHYSICAL_QCAR
from hal.products.mats import SDCSRoadMap

# General Configuration
@dataclass
class GeneralConfig:
    IS_PHYSICAL_QCAR: bool = False
    enableSteeringControl: bool = True
    fps = 30
    tf = 10
    startDelay = 2
    data_sampling_rate = 10

# Speed Controller Configuration
@dataclass
class SpeedControllerConfig:
    maxThrottle = 0.3
    v_ref: float = 0.5
    K_p: float = 0.1
    K_i: float = 1.0

# Steering Controller Configuration
@dataclass
class SteeringControllerConfig:
    enableSteeringControl: bool = True
    K_stanley: float = 1.0              # Proportional gain.
    cyclic: bool = True                 # Waypoints looping mode.
    nodeSequence: list = (0, 20, 0)
    controllerUpdateRate = 100
    maxSteeringAngle = np.pi / 6        # Max steering angle, e.g., 30 degrees in radians.

# Region Setup
roadmap = SDCSRoadMap(leftHandTraffic=False)
waypointSequence = roadmap.generate_path(SteeringControllerConfig.nodeSequence)
initialPose = roadmap.get_node_pose(SteeringControllerConfig.nodeSequence[0]).squeeze()

# Main Configuration Wrapper
@dataclass
class Config:
    general: GeneralConfig = field(default_factory=GeneralConfig)
    speed_controller: SpeedControllerConfig = field(default_factory=SpeedControllerConfig)
    steering_controller: SteeringControllerConfig = field(default_factory=SteeringControllerConfig)



