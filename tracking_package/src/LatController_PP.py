import math
import numpy as np
# D3无人车轴距1.6米，转向角45度
# 使用Pure Pursuit算法对无人车进行横向控制
class LatControllerPP:
    def __init__(self, min_lookahead_distance, lookahead_speed_kappa):
        self.path = []  # 路径，包含一系列的路径点，每个点是(x, y)坐标
        self.lookahead_distance = 0  # 预瞄距离
        self.position_x = 0
        self.position_y = 0
        self.heading = 0
        self.min_lookahead_distance = min_lookahead_distance
        self.lookahead_speed_kappa = lookahead_speed_kappa
        self.target_point = [0,0]

        self.acc_max = 0
        self.speed = 0
        self.param_compute()
        self.backward = False

    def set_position(self, x, y, yaw, speed):
        self.position_x = x
        self.position_y = y
        self.heading = yaw
        self.lookahead_distance = self.min_lookahead_distance + (abs(speed)*self.lookahead_speed_kappa)

    def set_path(self, path):
        self.path = path

    def diff_angle(self, heading, angle):
        error = angle - heading
        if error < -math.pi:
            return error + 2 * math.pi  # 小于-pi加2pi
        elif error >= math.pi:
            return error - 2 * math.pi  # 大于pi减2pi
        else:
            return error

    # pure pursuit算法中用于计算的车身位置为后轮中心位置，而D3中获取的位置为车辆中心位置，此函数用于纠正位置坐标
    def pos_correction(self):
        kappa_ = math.tan(self.heading)
        bias_ = self.position_y - kappa_ * self.position_x
        dx_ = 0.8 / math.sqrt((kappa_ * kappa_) + 1)
        if abs(self.heading) > math.pi / 2:
            dx_ = -dx_
        dx_ = -dx_
        self.position_x = self.position_x + dx_
        self.position_y = kappa_ * self.position_x + bias_

    def steer_calculate(self, angle):
        steer = math.pi / 4
        if abs(angle) < steer:
            steer = angle
        else:
            if angle < 0:
                steer = -steer

        if self.backward:
            steer = np.clip(steer / (math.pi / 8), -1, 1)
        else:
            steer = np.clip(steer / (math.pi / 10), -1, 1)
        return steer

    def param_compute(self):
        # 轴距
        l = 1.6
        # 最大线速度
        vmax = 16
        # 最小转弯半径r=l/tan(最大转向角)
        R_min = l
        # 最大侧向加速度
        self.acc_max = (vmax*vmax)/R_min

    def compute_speed(self, r):

        v = 17
        a = (v * v) / r
        if a > self.acc_max:
            v = math.sqrt(self.acc_max*r)
        d = math.hypot(self.path[len(self.path) - 1][0] - self.position_x,
                   self.path[len(self.path) - 1][1] - self.position_y)
        if d < 3:
            v = 0
        return v

    def find_target_point(self):
        closest_dist = float('inf')
        target_point = 0
        for i in range(0, len(self.path)):
            dist = math.hypot(self.position_x - self.path[i][0], self.position_y - self.path[i][1])
            if dist < closest_dist:
                closest_dist = dist
                target_point = i

        length = 0
        for j in range(target_point, len(self.path)-1):
            length += math.hypot(self.path[j+1][0] - self.path[j][0], self.path[j+1][1] - self.path[j][1])
            target_point = j
            if length > self.lookahead_distance:
                break
        self.target_point = self.path[target_point]

    def compute_onestep(self):
        if not self.path:
            return 0,0,[]
        else:
            # self.pos_correction()
            self.find_target_point()
            dx = self.target_point[0] - self.position_x
            dy = self.target_point[1] - self.position_y
            target_point_heading = math.atan2(dy, dx)
            alpha = self.diff_angle(self.heading, target_point_heading)
            R = 100000
            if alpha != 0:
                R = abs(math.hypot(dx, dy)/(2*math.sin(alpha)))
            control_v = self.compute_speed(R)

            # 倒退横向控制代码，自主判断是否倒退，当车身朝向与目标点朝向夹角大于90度时触发
            # 不确定是否使用，自主判断可能会带来控制混乱，可以由上层传入标志位决定是否倒退控制
            self.backward = False
            if abs(alpha) > math.pi/2:
                self.backward = True
                if self.heading < 0:
                    self.heading = self.heading + math.pi
                else:
                    self.heading = self.heading - math.pi
                alpha = self.diff_angle(self.heading, target_point_heading)
                steer_angle = math.atan(3.2 * math.sin(alpha) / math.hypot(dx, dy))
                steer = -self.steer_calculate(steer_angle)
                # 可视化**************************************************************************************
                if steer < 0:
                    xc = self.position_x - R * math.sin(self.heading)
                    yc = self.position_y + R * math.cos(self.heading)
                    theta = math.atan2(self.position_y - yc, self.position_x - xc)
                    theta2 = math.atan2(self.target_point[1] - yc, self.target_point[0] - xc)
                else:
                    xc = self.position_x + R * math.sin(self.heading)
                    yc = self.position_y - R * math.cos(self.heading)
                    theta2 = math.atan2(self.position_y - yc, self.position_x - xc)
                    theta = math.atan2(self.target_point[1] - yc, self.target_point[0] - xc)
                return steer, -control_v, [xc, yc, R * 2, theta * 57.29, theta2 * 57.29, self.target_point[0],
                                          self.target_point[1]]

            # 正常前进横向控制
            steer_angle = math.atan(3.2 * math.sin(alpha) / math.hypot(dx, dy))
            steer = self.steer_calculate(steer_angle)

            # 可视化*****************************************************************************
            if steer>0:
                xc = self.position_x - R*math.sin(self.heading)
                yc = self.position_y + R*math.cos(self.heading)
                theta = math.atan2(self.position_y - yc, self.position_x - xc)
                theta2 = math.atan2(self.target_point[1] - yc, self.target_point[0] - xc)
            else:
                xc = self.position_x + R * math.sin(self.heading)
                yc = self.position_y - R * math.cos(self.heading)
                theta2 = math.atan2(self.position_y-yc, self.position_x-xc)
                theta = math.atan2(self.target_point[1] - yc, self.target_point[0] - xc)
            return steer, control_v, [xc, yc, R*2,theta*57.29,theta2*57.29, self.target_point[0], self.target_point[1]]
