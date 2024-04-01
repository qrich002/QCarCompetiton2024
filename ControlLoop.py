# ControlLoop.py
import time

from pal.products.qcar import QCar, QCarGPS
from hal.products.qcar import QCarEKF

from parameters_config import Config, GeneralConfig, SpeedControllerConfig, SteeringControllerConfig, initialPose, waypointSequence

from SpeedController import SpeedController
from SteeringController import SteeringController


def ControlLoop(config: Config, KILL_THREAD):
    
    speedController = SpeedController(config.speed_controller)
    steeringController = SteeringController(config, initialPose, waypointSequence)
    qcar = QCar(readMode=1, frequency=config.steering_controller.controllerUpdateRate)

    if config.general.enableSteeringControl:
        ekf = QCarEKF(x_0=initialPose)  
        gps = QCarGPS(initialPose=initialPose)  
    else:
        gps = memoryview(b'')

# Initial setup
    throttle = 0
    steering_angle = 0
    t0 = time.time()  # Record the start time
    t = 0
    data_sampling_counter = 0  # Initialize data sampling counter

    # Main control loop
    while t < config.general.tf + config.general.startDelay and not KILL_THREAD.is_set():
        # Update loop timing
        previous_time = t
        t = time.time() - t0
        delta_time = t - previous_time

        # Read from sensors and update state estimates
        sensor_data = qcar.read()

        """
        if sensor_data is None or "velocity" not in sensor_data or "image" not in sensor_data:
            print("Warning: Sensor data is incomplete or unavailable. Skipping iteration.")
            time.sleep(1.0) 
            continue  # Skip the rest of the loop iteration
        """

        if t >= config.general.startDelay:
            throttle = speedController.update(config.speed_controller.v_ref, config.speed_controller.maxThrottle, delta_time)   # sensor_data["velocity"]
            
            """"
            if config.general.enableSteeringControl:
                image = sensor_data["image"]  
                steering_angle = steeringController.adjust_steering(image, delta_time)
                if steering_angle is None:
                    print("Steering angle is None. Defaulting to 0.")
                    steering_angle = 0 
            """
            steering_angle = 0

            qcar.write(throttle, steering_angle)

        data_sampling_counter += 1
        if data_sampling_counter >= config.general.data_sampling_rate and t > config.general.startDelay:
            data_sampling_counter = 0  # Reset counter
            # Add any visualization or logging update logic here

        if KILL_THREAD.is_set():
            break