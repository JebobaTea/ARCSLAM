import ydlidar
import numpy as np

def polar_to_2d(sc):
    cartesian_pts = []
    for pt in sc:
        dist = pt[1]
        rad = pt[0]
        x = dist * np.cos(rad) * -1 # for some reason, x values are inverted
        y = dist * np.sin(rad)
        cartesian_pts.append([x, y, 0])
    return np.array(cartesian_pts)

class YDScanner:
    def __init__(self, freq=12.0, rate=9, rep=1):
        self.scanfreq = freq
        self.samprate = rate
        self.repeat = rep

        if ydlidar.os_init():
            print("YDlidar SDK initialization success")
        else:
            print("Initialization failed, start praying")

        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"
        for key, value in ports.items():
            port = value

        self.laser = ydlidar.CYdLidar()
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, self.scanfreq)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, self.samprate)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)

    def get_raw_scan(self, n=1):
        #print("Starting scan, performing " + str(n) + " rotations")
        polar_pts = []
        for i in range(n):
            scan = ydlidar.LaserScan()
            r = self.laser.doProcessSimple(scan)
            if r:
                #print("Successful scan start for no. " + str(i))
                for point in scan.points:
                    # ignore invalid measurements
                    if (point.range < 0.005):
                        continue
                    polar_pts.append([point.angle, point.range])
            else:
                #print("Scan failure")
                return [[]]
        return np.array(polar_pts)

    def run_scan(self):
        scan_polar = self.get_raw_scan(n=self.repeat)
        if not scan_polar.any():
            print("Hit a snag, see information dump")
            return
        scan_2d = polar_to_2d(scan_polar)
        return scan_polar, scan_2d

    def activate(self):
        if not self.laser.initialize():
            self.laser.disconnecting()
            print("Lidar initialization failed")
            return False
        print("Lidar initialization success")
        if not self.laser.turnOn():
            self.laser.turnOff()
            self.laser.disconnecting()
            print("Lidar failed to turn on")
            return False
        print("Lidar on")
        return True

    def deactivate(self):
        self.laser.turnOff()
        self.laser.disconnecting()
        print("Lidar shutdown")
