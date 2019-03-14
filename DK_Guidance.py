from dronekit import mavutil
import time
from math import pi, fabs



def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        #mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg);


def condition_yaw(vehicle, heading):

    heading = heading % 360

    print "Changing yaw to: ", heading;

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s (not supported by ArduPilot)
        1,          # param 3, direction -1 ccw, 1 cw
        0,          # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used


    # Loop till drone has changed orientation
    wait_for_yaw_change(vehicle, heading, msg);


def wait_for_yaw_change (vehicle, target_yaw_deg, msg):

    # send command to vehicle
    vehicle.send_mavlink(msg)
    # Blank movement must be issued after every command
    send_ned_velocity(vehicle, 0, 0, 0);

    target_yaw_rad = target_yaw_deg/180.0 * pi;
    if target_yaw_rad > pi:
        target_yaw_rad -= 2*pi;
    target_yaw_rad = round(target_yaw_rad, 3);

    while (fabs(round(vehicle.attitude.yaw, 2)  - target_yaw_rad) % 360) > 1:

        # send command to vehicle
        vehicle.send_mavlink(msg)
        # Blank movement must be issued after every command
        send_ned_velocity(vehicle, 0, 0, 0);

        print [target_yaw_deg, target_yaw_rad, round(vehicle.attitude.yaw, 2),
            (fabs(round(vehicle.attitude.yaw, 2)  - target_yaw_rad) % 360)]
            
        time.sleep(0.5);


def send_rtl(vehicle):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, #command
        0,  #confirmation
        0, 0, 0, 0, 0, 0, 0)    # param 1 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_land(vehicle, lat=0, lon=0):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_NAV_LAND, #command
        0,  #confirmation
        0, 0, 0, 0,    # param 1 ~ 4 not used
        lat,    # param 5, latitude (if lat, lon = 0 land at current position)
        lon,    # param 6, longiutde
        0)    # param 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
