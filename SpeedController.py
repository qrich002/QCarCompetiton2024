#SpeedControl.py

from parameters_config import Config, SpeedControllerConfig

# ==============  Speed Control  ====================
class SpeedController:
    def __init__(self, config: SpeedControllerConfig):
        self.config = config
        self.maxThrottle = config.maxThrottle
        self.kp = config.K_p
        self.ki = config.K_i
        self.ei = 0   # Reset the integral error

    def update(self, v_ref, velocity, delta_time):
        '''
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
        '''
        return .05
