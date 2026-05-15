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