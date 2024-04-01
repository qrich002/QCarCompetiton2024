# SteeringController.py

import cv2
import numpy as np

from parameters_config import Config, SteeringControllerConfig, initialPose, waypointSequence

class SteeringController:
    def __init__(self, config: SteeringControllerConfig, initialPose, waypointSequence):
        self.config = config
        self.currentAngle = 0.0  # Initialize the current steering angle to straight.
        self.maxSteeringAngle = config.steering_controller.maxSteeringAngle
        self.K_stanley = config.steering_controller.K_stanley
        self.cyclic = config.steering_controller.cyclic

def update_steering(self, steering_angle):
        
        print(f"Updating Steering Angle: {steering_angle} radians.")
        self.currentAngle = steering_angle

def compute_steering_angle(self, track_center, image_width):
        """
        Computes the steering angle based on the horizontal offset of the track's center from the image center.
        
        Parameters:
        - track_center (int): The horizontal pixel coordinate of the track's center in the image.
        - image_width (int): The width of the image in pixels.
        
        Returns:
        - (float): The computed steering angle in radians, clamped to maxSteeringAngle.
        """
        center_offset = track_center - (image_width / 2)
        steering_angle = self.K_stanley * center_offset  # Simple proportional control
        return np.clip(steering_angle, -self.maxSteeringAngle, self.maxSteeringAngle)  # Clamp angle


def process_image_and_adjust_steering(self, image):
        """
        Processes the input image to detect the track's center and adjust the steering angle accordingly.
        
        Parameters:
        - image (numpy.ndarray): The input image from the vehicle's camera.
        
        Returns:
        - (float): The updated steering angle based on the detected track's position.
        """
        # Image preprocessing to detect the track
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 120, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Assume largest contour is the track
            largest_contour = max(contours, key=cv2.contourArea)
            contour_moments = cv2.moments(largest_contour)
            if contour_moments["m00"] != 0:  # Avoid division by zero
                track_center = int(contour_moments["m10"] / contour_moments["m00"])
                steering_angle = self.compute_steering_angle(track_center, image.shape[1])
                self.update_steering(steering_angle)
                return steering_angle
        
        print("Track not detected, maintaining current steering angle.")
        return self.currentAngle


