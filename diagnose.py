from utils.Control import Vehicle
import time

vehicle = Vehicle()
print("Woke up peripherals")

vehicle.send_signal(bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
input("Breakpoint")

try:
    print("Forward only")
    for i in range(10):
        ctrl = vehicle.control_to_bytes(0, 0.1)
        print(list(ctrl))
        vehicle.send_signal(ctrl)
        time.sleep(0.1)

    #input("Breakpoint")
    print("Left")
    for i in range(25):
        ctrl = vehicle.control_to_bytes(-0.5, 0.1)
        print(list(ctrl))
        vehicle.send_signal(ctrl)
        time.sleep(0.1)

    #input("Breakpoint")
    print("Right")
    for i in range(10):
        ctrl = vehicle.control_to_bytes(0.5, 0.1)
        print(list(ctrl))
        vehicle.send_signal(ctrl)
        time.sleep(0.1)
finally:
    # vehicle.send_signal(bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
    vehicle.kill()
from utils.Control import Vehicle
import time

vehicle = Vehicle()
print("Woke up peripherals")

print("Forward only")
for i in range(20):
    vehicle.send_signal(vehicle.control_to_bytes(0, 0.1))
    time.sleep(0.1)

input("Breakpoint")
print("Left")
for i in range(20):
    vehicle.send_signal(vehicle.control_to_bytes(-0.5, 0.1))
    time.sleep(0.1)

input("Breakpoint")
print("Right")
for i in range(20):
    vehicle.send_signal(vehicle.control_to_bytes(0.5, 0.1))
    time.sleep(0.1)
