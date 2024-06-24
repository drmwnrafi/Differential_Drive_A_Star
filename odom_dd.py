import numpy as np

class odometry():
    def __init__(self, wheelbase, radius_left, radius_right):
        self.wheelbase = wheelbase
        self.radius_left = radius_left
        self.radius_right = radius_right
        self.prev_angle_left = None
        self.prev_angle_right = None
        self.dist_left = 0
        self.dist_right = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        
    def difference_angle(first, second):
        return (second - first + 180) % 360 - 180

    def distance(self, pos_l, pos_r):
        if self.prev_angle_left is not None:
            d_theta_L = self.difference_angle(self.prev_angle_left, pos_l)
            self.dist_left += (np.pi * 2 * self.radius_left * (d_theta_L / 360))
            self.prev_angle_left = pos_l
        else:
            self.prev_angle_left = pos_l

        if self.prev_angle_right is not None:
            d_theta_R = self.difference_angle(self.prev_angle_right, pos_r)
            self.dist_right += (np.pi * 2 * self.radius_right * (d_theta_R / 360))
            self.prev_angle_right = pos_r
        else:
            self.prev_angle_right = pos_r

    def run(self, pos_l, pos_r):
        self.distance(pos_l, pos_r)
        d_distance = (self.dist_left + self.dist_right) / 2
        d_theta = (self.dist_right - self.dist_left) / self.wheelbase

        odom_dx = d_distance * np.cos(self.theta + d_theta / 2)
        odom_dy = d_distance * np.sin(self.theta + d_theta / 2) 
        self.x += odom_dx
        self.y += odom_dy
        self.theta = (self.theta + d_theta) % (2 * np.pi)

    def get_pose(self):
        return self.x, self.y, self.theta
