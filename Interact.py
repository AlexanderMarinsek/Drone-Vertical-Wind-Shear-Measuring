import time
import datetime
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from math import sin, cos, sqrt, radians

from DK_Guidance import *


def init (meas_obj, msg_freq):
    # Wait for the default set of parameters
    vehicle = connect('udp:127.0.0.1:14550', wait_ready=False)
    vehicle.wait_ready(True, timeout=120)
    print "Connected"

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)


    # Set the yaw behavior when flying to waypoint
    print "WP_YAW_BEHAVIOR: %s" % vehicle.parameters['WP_YAW_BEHAVIOR']
    vehicle.parameters['WP_YAW_BEHAVIOR']=0   # Never change yaw
    #vehicle.parameters['WP_YAW_BEHAVIOR']=1   # Face next waypoint
    #vehicle.parameters['WP_YAW_BEHAVIOR']=2   # Face next waypoint except RTL
    print "WP_YAW_BEHAVIOR: %s" % vehicle.parameters['WP_YAW_BEHAVIOR']


    print "SR0_POSITION: %s" % vehicle.parameters['SR0_POSITION']
    vehicle.parameters['SR0_POSITION'] = msg_freq   # Never change yaw
    print "SR0_POSITION: %s" % vehicle.parameters['SR0_POSITION']

    print "SR0_EXTRA1: %s" % vehicle.parameters['SR0_EXTRA1']
    vehicle.parameters['SR0_EXTRA1'] = msg_freq   # Never change yaw
    print "SR0_EXTRA1: %s" % vehicle.parameters['SR0_EXTRA1']

    #while True:
    #    time.sleep(0.01)

    # Get initial position
    initial_north = vehicle.location.local_frame.north
    initial_east = vehicle.location.local_frame.east
    initial_down = vehicle.location.local_frame.down
    print "* Initial north: %f\n" \
    "\tinitial east: %f\n" \
    "\tinitial down: %f\n" % \
    (initial_north, initial_east, initial_down);


    return [vehicle, initial_north, initial_east, initial_down]



# Take off to target altitude
def takeoff (vehicle, target_altitude):
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Trigger just below target alt
        if vehicle.location.global_relative_frame.alt>=target_altitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# Measure (moving up-down or in star pattern - depends on yaw & phi relation)
def measurement_procedure (
    vehicle, yaw=0, velocity=1, throttle=150,
    repetitions=3, scan_height=10, missmatch=0.95):

    print "Started new measurement"

    vehicle.channels.overrides['3'] = 1500
    print "channel 3 value: ", vehicle.channels.overrides['3']

    vehicle.mode = VehicleMode("LOITER")
    while not vehicle.mode == "LOITER":
        print "mode: ", vehicle.mode
        vehicle.mode = VehicleMode("LOITER")
        time.sleep(1)
    print "mode: ", vehicle.mode

    #print "Sleeping, mode: ", vehicle.mode
    #time.sleep(5)

    start_down = vehicle.location.local_frame.down

    offset_down = vehicle.location.local_frame.down - start_down
    offset_down_prev = offset_down

    print "* start_down: ", start_down
    print "* offset_down: ", offset_down
    print "* vehicle.location.local_frame.down: ", vehicle.location.local_frame.down

    vehicle.channels.overrides['3'] = 1500 + throttle
    print "channel 3 value: ", vehicle.channels.overrides['3']

    start_time_ms = time.time()
    rest_period = 0.020
    #rest_period = 1

    delta_h = 0

    offset_timer = time.time()
    offset_timer_prev = time.time()

    #for i in range(0,repetitions*2):
    # First iteration is only half-way (from center to max radius)
    for i in range(0, repetitions):
        """
        # Set / correct yaw
        print "Setting yaw to: ", yaw
        condition_yaw(vehicle, yaw)
        print "Yaw set to: ", yaw
        """
        while True:

            # Need to constantly send throttle commands in simulator
            vehicle.channels.overrides['3'] = 1500 + throttle

            offset_down = vehicle.location.local_frame.down - start_down
            offset_timer = time.time()

            delta_h = abs(offset_down - offset_down_prev)
            delta_t = abs(offset_timer - offset_timer_prev)
            v_speed = delta_h / delta_t
            
            print "throttle: %d, start_down: %0.2f, offset_down: %0.2f," \
                " -scan_height: %d, delta_h: %0.2f, delta_t: %0.2f," \
                " v_speed: %0.2f" % \
                (throttle, start_down, offset_down,
                -scan_height, delta_h, delta_t,
                v_speed)

            if throttle > 0 and -offset_down >= scan_height:
                throttle = -throttle
                vehicle.channels.overrides['3'] = 1500 + throttle
                print "channel 3 value: ", vehicle.channels.overrides['3']
                break

            elif throttle < 0 and offset_down >= 0:
                throttle = -throttle
                vehicle.channels.overrides['3'] = 1500 + throttle
                print "channel 3 value: ", vehicle.channels.overrides['3']
                break
            offset_down_prev = offset_down
            offset_timer_prev = offset_timer

            if (time.time() - start_time_ms < rest_period):
                print "sleep: ", rest_period - (time.time() - start_time_ms)
                time.sleep(rest_period - (time.time() - start_time_ms))
                start_time_ms = time.time()

    vehicle.channels.overrides['3'] = 1500
    print "channel 3 value: ", vehicle.channels.overrides['3']
    #time.sleep(5)

    print "Finished measurement"


# Return to landing zone
def return_to_land (vehicle, yaw):
    """
    # Set / correct yaw
    print "Setting yaw to: ", yaw
    time.sleep(3);
    condition_yaw(vehicle, yaw)
    time.sleep(3);
    print "Yaw set to: ", yaw
    """

    print "RETURN TO LAND"

    vehicle.mode = VehicleMode("RTL")
    while not vehicle.mode == "RTL":
        print "mode: ", vehicle.mode
        vehicle.mode = VehicleMode("RTL")
        time.sleep(1)
    print "mode: ", vehicle.mode





"""
# Land and disarm vehicle in current position
def land_disarm(vehicle):
    print "Landing"
    send_land(vehicle);
    while True:
        #print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt <= 0 + 0.05:
            print("Landed")
            break
        time.sleep(0.02);

    print("Disarming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("AUTO")
    vehicle.armed = False

    while vehicle.armed:
        print(" Waiting for disarming...")
        time.sleep(1)
    print("Disarmed motors")
"""
