import numpy as np

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt=1):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class PositionControl:
    def __init__(self, odometry, wheelbase, max_speed):
        self.odom = odometry
        self.wheelbase = wheelbase
        self.max_speed = max_speed
        self.pid_linear = PID(Kp=1.0, Ki=0.0, Kd=0.1)
        self.pid_angular = PID(Kp=1.0, Ki=0.0, Kd=0.1)

    def compute_control(self, target_x, target_y, dt):
        current_x, current_y, current_theta = self.odom.get_pose()

        error_x = target_x - current_x
        error_y = target_y - current_y

        distance_error = np.sqrt(error_x**2 + error_y**2)
        angle_to_target = np.arctan2(error_y, error_x)
        angle_error = (angle_to_target - current_theta + np.pi) % (2 * np.pi) - np.pi

        linear_speed = self.pid_linear.compute(distance_error,)
        angular_speed = self.pid_angular.compute(angle_error,)

        # Limit speeds to max speed
        linear_speed = np.clip(linear_speed, -self.max_speed, self.max_speed)
        angular_speed = np.clip(angular_speed, -self.max_speed, self.max_speed)

        return linear_speed, angular_speed

    def set_wheel_speeds(self, linear_speed, angular_speed):
        v_r = linear_speed + (angular_speed * self.wheelbase / 2)
        v_l = linear_speed - (angular_speed * self.wheelbase / 2)
        return v_l, v_r