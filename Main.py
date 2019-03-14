"""

Drone guidance and wind speed measuring script
* Not to be used together with mission planner (changes message frequency)
  ... if used with MP, 'msg_freq' must be matched with MP's update frequency!!!

"""

from DK_Guidance import *
from Interact import *
from MeasDef import Meas
from ReconstructDef import Reconstruct

import os, sys

#Callback method for new messages
def msg_handler_battery (self, name, msg):
    #print "*** Received message:"
    battery = msg.voltage_battery / 1000
    print "Battery voltage: ", battery


def main():

    target_altitude = 10;

    # Global measurement parameters
    yaw = 0             # Yaw while measuring [deg.]
    velocity = 0.1      # Up/down target velocity (disabled)
    throttle = 120      # Up/down target throtthle in loiter mode
    repetitions = 1     # Number of vertical measurement repetitions (up or down)
    scan_height = 10    # Scan height (from takeoff upwards) [m/s]
    missmatch = 0.95    # Relative allowed height missmatch

    msg_freq = 5        # Hz
    msg_period_ms = 1.0 / msg_freq * 1000

    # Executing code's full filepath
    pathname = os.path.dirname(sys.argv[0])
    fullpath = os.path.abspath(pathname)

    meas_obj = Meas(fullpath, msg_period_ms)


    print "PROGRAM START"


    # Initiate vehicle ---------------------------------------------------------
    [vehicle, initial_north, initial_east, initial_down] = \
        init (meas_obj, msg_freq)


    # Take off -----------------------------------------------------------------
    print("Taking off!")
    takeoff (vehicle, target_altitude)


    # Add message listeners ----------------------------------------------------
    vehicle.add_message_listener(
        'GLOBAL_POSITION_INT', meas_obj.msg_gps_handler)
    vehicle.add_message_listener(
        'ATTITUDE', meas_obj.msg_att_handler)


    # Conduct measurements -----------------------------------------------------
    measurement_procedure (
        vehicle, yaw, velocity, throttle, repetitions, scan_height, missmatch)


    # Remove message listeners -------------------------------------------------
    vehicle.remove_attribute_listener(
        'GLOBAL_POSITION_INT', meas_obj.msg_gps_handler)
    vehicle.remove_attribute_listener(
        'ATTITUDE', meas_obj.msg_att_handler)


    # Issue return to land (RTL) command ---------------------------------------
    yaw = 0
    return_to_land(vehicle, yaw)


    # Save measurement data ----------------------------------------------------
    meas_obj.save_meas()

    # Get rid of object reference, so memory can be (automatically) freed
    del meas_obj


    # Reconstruct raw measurements and get wind data (.csv + .png) -------------
    rec_obj = Reconstruct (fullpath)
    rec_obj.transform()
    rec_obj.draw_wind()

    # Get rid of object reference, so memory can be (automatically) freed
    del rec_obj


    print "PROGRAM END";


if __name__ == "__main__":
    main();
