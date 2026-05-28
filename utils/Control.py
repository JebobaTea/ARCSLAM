from collections import deque
import board
import digitalio
import busio
import math
import numpy as np
import time
import adafruit_bno055


def dist_euclid(x1, y1, x2, y2):
    return math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

def normalize_rad(rad : float):
    return (rad + np.pi) % (2 * np.pi) - np.pi

def filter_waypoints(location, current_idx, waypoints) -> int:
    # waypoints no longer in roar_py format, so [x, y]
    # TODO: pray that this works
    for i in range(current_idx, len(waypoints)):
        # TODO: find appropriate threshold
        if dist_euclid(location[0], location[1], waypoints[i][0], waypoints[i][1])  < 0.1:
            return i
    return current_idx

class Vehicle:
    def __init__(self):
        self.maneuverable_waypoints = None
        self.nano_addy = 0x08
        self.i2c = board.I2C()
        self.location = None
        self.rotation = None
        self.long_ctrl = 0
        self.last_time = time.time()
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
        self.sensor.mode = adafruit_bno055.IMUPLUS_MODE
        self.current_waypoint_idx = 1
        self.location_buffer = deque(maxlen=10)
        self.lat_pid_controller = LatPIDController(config=self.get_lateral_pid_config())

    def replace_waypoints(self, new_waypoints, clip):
        # reverse pixelation operation
        for wpt in new_waypoints:
            rescaled_x = wpt[0] * math.pow(0.1, clip)
            rescaled_y = wpt[1] * math.pow(0.1, clip)
            wpt[0] = rescaled_x
            wpt[1] = rescaled_y
        self.maneuverable_waypoints = new_waypoints
        self.current_waypoint_idx = 1

    # TODO: find appropriate values
    def get_lateral_pid_config(self):
        conf = {
            "1": {
                "Kp": 0.7,
                "Kd": 0.1,
                "Ki": 0.05
            },
            "3": {
                "Kp": 0.7,
                "Kd": 0.1,
                "Ki": 0.07
            },
            "5": {
                "Kp": 0.65,
                "Kd": 0.1,
                "Ki": 0.08
            },
            "10": {
                "Kp": 0.57,
                "Kd": 0.13,
                "Ki": 0.09
            }
        }
        return conf

    def control_to_bytes(self, steer, target_ctrl, debug=False):
        d_throt = target_ctrl - self.long_ctrl
        throt_step = d_throt/10
        self.long_ctrl += throt_step
        throttle = self.long_ctrl * 256
        throttle = int(throttle)
        if throttle > 255:
            # insurance for if my math is off
            throttle = 255
        if throttle < -255:
            throttle = -255
        # should now be clipped to -255, 255
        throttle_strong = throttle
        throttle_weak = int(throttle * ((1 - abs(steer))))
        # byte 0: forward A, byte 1: backward A, byte 2: forward B, byte 3: backward B
        # steer < 0 --> left, steer > 0 --> right
        if debug:
            print(throttle)
        control = []
        if abs(steer) > 0.5:
            throttle_strong *= 1.2
        if abs(steer) > 0.7:
            throttle_strong *= 1.5
        throttle_strong = int(throttle_strong)
        if steer > 0:
            if throttle > 0:
                # for right turn, weaken right-side throttle by factor of 1 - steer
                # side A --> L convention
                control = [throttle_strong, 0x00, throttle_weak, 0x00, 0x00, 0x00, 0x00, 0x00]
            else:
                # induce weaker braking on the side that requires more power
                control = [0x00, -throttle_weak, 0x00, -throttle_strong, 0x00, 0x00, 0x00, 0x00]
        else:
            if throttle > 0:
                # left turn: weaken right-side throttle (side B)
                control = [throttle_weak, 0x00, throttle_strong, 0x00, 0x00, 0x00, 0x00, 0x00]
            else:
                control = [0x00, -throttle_strong, 0x00, -throttle_weak, 0x00, 0x00, 0x00, 0x00]
        return bytes(control)

    def send_signal(self, signal):
        while not self.i2c.try_lock():
            pass
        try:
            #reg = bytes([0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07])
            self.i2c.writeto(self.nano_addy, signal)
            # communication debug
            # buffer = bytearray(8)
            # self.i2c.readfrom_into(self.nano_addy, buffer)
            # print("Data: ", [b for b in buffer])
        finally:
            self.i2c.unlock()

    def drive(self, odom_transform, deft=0.1):
        new_time = time.time()
        self.location = odom_transform
        # constant time assumption does not hold
        self.lat_pid_controller._dt = new_time - self.last_time
        self.current_waypoint_idx = filter_waypoints(
            self.location,
            self.current_waypoint_idx,
            self.maneuverable_waypoints
        )
        self.rotation = self.sensor.euler
        rotcp = self.rotation[:]
        # rotation must be in RPY format in radians
        self.rotation = [x * 3.141592/180 for x in self.rotation]
        dt = self.last_time - new_time

        vehicle_speed = 0
        if len(self.location_buffer) > 1:
            d_trav = dist_euclid(self.location_buffer[0][0],
                                        self.location_buffer[0][1],
                                        self.location_buffer[1][0],
                                        self.location_buffer[1][1])
            vehicle_speed = d_trav / dt

        waypoint_to_follow = self.lat_pid_controller.get_waypoint_at_offset(self.maneuverable_waypoints,
                                                                            self.current_waypoint_idx, 3)
        steer, error = self.lat_pid_controller.run_in_series(
            self.location, self.rotation, vehicle_speed, waypoint_to_follow
        )
        throttle = deft
        #if vehicle_speed < 0.05:
           # throttle = 0.12
        if (vehicle_speed > 0.1):
            throttle = 0
        self.send_signal(self.control_to_bytes(steer, throttle))
        print("Waypoint to follow: " + str(waypoint_to_follow))
        print("Steer: " + str(steer))
        print("Speed: " + str(vehicle_speed))
        print("Signal: " + str(list(self.control_to_bytes(steer, throttle))))
        print("Rotation: " + str(rotcp))
        print("Error: " + str(error)) 
        self.location_buffer.append(odom_transform)
        self.last_time = new_time

    def kill(self):
        for i in range(30):
            self.send_signal(self.control_to_bytes(0, 0))
            time.sleep(0.05)
        self.send_signal(bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))

class LatPIDController():
    def __init__(self, config: dict, dt: float = 0.05):
        self.config = config
        self.steering_boundary = (-1.0, 1.0)
        self._error_buffer = deque(maxlen=10)
        self._dt = dt

    def run_in_series(self, vehicle_location, vehicle_rotation, current_speed, next_waypoint) -> float:
        v_begin = vehicle_location
        direction_vector = np.array([
            np.cos(normalize_rad(vehicle_rotation[2])),
            np.sin(normalize_rad(vehicle_rotation[2])),
            0])
        v_end = v_begin + direction_vector

        v_vec = np.array([(v_end[0] - v_begin[0]), (v_end[1] - v_begin[1]), 0])

        w_vec = np.array(
            [
                next_waypoint[0] - v_begin[0],
                next_waypoint[1] - v_begin[1],
                0,
            ]
        )

        v_vec_normed = v_vec / np.linalg.norm(v_vec)
        w_vec_normed = w_vec / np.linalg.norm(w_vec)
        error = np.arccos(min(max(v_vec_normed @ w_vec_normed.T, -1), 1))
        _cross = np.cross(v_vec_normed, w_vec_normed)

        if _cross[2] > 0:
            error *= -1
        self._error_buffer.append(error)
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / self._dt
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        k_p, k_d, k_i = self.find_k_values(current_speed=current_speed, config=self.config)

        lat_control = float(
            np.clip((k_p * error) + (k_d * _de) + (k_i * _ie), self.steering_boundary[0], self.steering_boundary[1])
        )

        return lat_control, error

    def find_k_values(self, current_speed: float, config: dict) -> np.array:
        k_p, k_d, k_i = 1, 0, 0
        for speed_upper_bound, kvalues in config.items():
            speed_upper_bound = float(speed_upper_bound)
            if current_speed < speed_upper_bound:
                k_p, k_d, k_i = kvalues["Kp"], kvalues["Kd"], kvalues["Ki"]
                break
        return np.array([k_p, k_d, k_i])

    def get_waypoint_at_offset(self, maneuverable_waypoints, current_index, offset):
        return maneuverable_waypoints[(current_index + offset) % len(maneuverable_waypoints)]
