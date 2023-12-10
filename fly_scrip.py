import time
from math import degrees
from typing import List

from dronekit import VehicleMode, connect
from pymavlink import mavutil

vehicle = connect("tcp:127.0.0.1:5763", wait_ready=True)


def to_degrees(radians):
    return degrees(radians)


def takeoff(target_altitude):
    print("Taking off to {} meters...".format(target_altitude))
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Altitude: {} meters".format(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)


def set_alt_hold_and_yaw(yaw_angle):
    print("Setting AltHold mode...")
    vehicle.mode = VehicleMode("ALT_HOLD")
    time.sleep(2)

    print("Setting yaw angle to {} degrees...".format(yaw_angle))
    msg = vehicle.message_factory.command_long_encode(
        0, 0,                    # target_system, target_component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,                        # confirmation
        yaw_angle,                # param1 (yaw angle in degrees)
        0, 0, 0, 0, 0, 0           # param2,3,4,5,6,7 (not used)
    )
    vehicle.send_mavlink(msg)
    time.sleep(2)


def execute(
        point_a: List, point_b: List, target_altitude: int, rotation_in_degrees: int
):
    takeoff(target_altitude)

    vehicle.simple_goto(point_b)
    time.sleep(10)

    set_alt_hold_and_yaw(rotation_in_degrees)

    while vehicle.mode.name != "LOITER":
        time.sleep(1)

    vehicle.close()
    print("Mission complete!")
    return


if __name__ == "__main__":
    execute(
        point_a=[50.450739, 30.461242],
        point_b=[50.443326, 30.448078],
        target_altitude=100,
        rotation_in_degrees=350,
    )
