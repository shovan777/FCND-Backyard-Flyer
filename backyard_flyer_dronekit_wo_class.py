from dronekit import connect, VehicleMode
from pymavlink import mavutil
import exceptions
import socket
from enum import Enum
import numpy as np
import time

# flight state machine
class States(Enum):
    MANUAL = 0
    GUIDED = 1
    ARMING = 2
    TAKEOFF = 3
    WAYPOINT = 4
    LANDING = 5
    DISARMING = 6


class BackyardFlyer():
    def __init__(self, vehicle):
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.vehicle = vehicle

        # initial state
        self.mode = VehicleMode('STABILIZED')
        self.flight_state = States.MANUAL

        # register callbacks
        self.vehicle.add_attribute_listener('location.local_frame', self.local_location_callback)
        self.vehicle.add_attribute_listener('velocity', self.velocity_callback)
        self.vehicle.add_attribute_listener('armed', self.state_callback)
        self.vehicle.add_attribute_listener('mode', self.state_callback)

    def goto_position_target_local_ned(self, north, east, down):
        """	
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
        location in the North, East, Down frame.

        It is important to remember that in this frame, positive altitudes are entered as negative 
        "Down" values. So if down is "10", this will be 10 metres below the home altitude.

        Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        ignored. For more information see: 
        http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

        See the above link for information on the type_mask (0=enable, 1=ignore). 
        At time of writing, acceleration and yaw bits are ignored.

        """
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        vehicle.send_mavlink(msg)


    def local_location_callback(self, _, attr_name, value):
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * value[2]
            if altitude > 0.95 * target_position[2]:
                self.all_waypoints = [(0.0,0.0,3.0,0.0), (5.0,0.0,3,0), (5.0,5.0,3.0,0),(0.0,5.0,3.0,0.0)]
                self.waypoint_transition()
        if self.flight_state == States.WAYPOINT:
            north = value[0]
            east = value[1]
            if abs(north - self.target_position[0]) < 1.0 and abs(east - self.target_position[1]) < 1.0:
                print('I reached waypoint.')
                if self.check_state[0] == True:
                    self.landing_transition()
                else:
                    self.waypoint_transition()

    def velocity_callback(self, _, attr_name, value):
        if self.flight_state == States.LANDING:
            if ((self.vehicle.location.global_frame[2] - self.vehicle.home_location[2] < 0.1) and
                abs(self.vehicle.location.local_frame[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self, _, attr_name, value):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL and self.vehicle.is_armable:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            if self.vehicle.armed:
                self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            if not self.vehicle.armed:
                self.manual_transition()

    def calculate_box(self):
        return self.all_waypoints.pop()

    def arming_transition(self):
        self.vehicle.mode = VehicleMode('GUIDED')
        self.vehicle.armed = True
        
        self.vehicle.home_location = vehicle.location.global_frame
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print('takeoff transition')
        print(self.vehicle.location.local_frame)
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        self.vehicle.simple_takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print('waypoint transition')
        if len(self.all_waypoints) != 0:
            self.check_state[0] = False
            box_cord = self.calculate_box()
            self.target_position[:] = box_cord[:3]
            print(self.target_position)
            self.goto_position_target_local_ned(*box_cord)
            self.flight_state = States.WAYPOINT
        else:
            print('mission complete')
            self.check_state[0] = True
            # self.flight_state = States.WAYPOINT

    def landing_transition():
        self.vehicle.mode = VehicleMode('LAND')
        self.flight_state = States.LANDING
                
    def disarming_transition():
        print('disarm transition')
        print(self.vehicle.location.local_frame)
        self.vehicle.armed = False
        
    def manual_transition(self):
        self.vehicle.mode = VehicleMode('STABILIZED')
        self.vehicle.close()
        self.in_mission = False
        self.flight_state = States.MANUAL



#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
try:
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
# Bad TCP connection
except socket.error:
    print 'No server exists!'

# Bad TTY connection
except exceptions.OSError as e:
    print 'No serial exists!'

# API Error
except dronekit.APIException:
    print 'Timeout!'

# Other error
except:
    print 'Some other error!'

target_position = np.array([0.0, 0.0, 0.0])
all_waypoints = []
in_mission = True
check_state = {}

# initial state
mode = VehicleMode(States.MANUAL)
flight_state = States.MANUAL

def local_location_callback(self, attr_name, value):
    if flight_state == States.TAKEOFF:
        altitude = -1.0 * value[2]
        if altitude > 0.95 * target_position[2]:
            all_waypoints = [(0.0,0.0,3.0,0.0), (5.0,0.0,3,0), (5.0,5.0,3.0,0),(0.0,5.0,3.0,0.0)]
            waypoint_transition()
    if flight_state == States.WAYPOINT:
        north = value[0]
        east = value[1]
        if abs(north - target_position[0]) < 0.05 and abs(east - target_position[1]) < 0.05:
            print('I reached waypoint.')
            if check_state[0] == True:
                landing_transition()
            else:
                waypoint_transition()

def velocity_callback(self, attr_name, value):
    if flight_state == States.LANDING:
        if ((vehicle.location.global_frame[2] - vehicle.home_location[2] < 0.1) and
            abs(vehicle.location.local_frame[2]) < 0.01):
            disarming_transition()

def state_callback(self, attr_name, value):
    if not in_mission:
        return
    if flight_state == States.MANUAL and vehicle.is_armable:
        arming_transition()
    elif flight_state == States.ARMING:
        if armed:
            takeoff_transition()
    elif flight_state == States.DISARMING:
        if not armed:
            manual_transition()

# register callbacks
vehicle.add_attribute_listener('location.local_frame', local_location_callback)
vehicle.add_attribute_listener('velocity', velocity_callback)
vehicle.add_attribute_listener('armed', state_callback)
vehicle.add_attribute_listener('mode', state_callback)

def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)


# def local_location_callback(self, attr_name, value):
#     if flight_state == States.TAKEOFF:
#         altitude = -1.0 * value[2]
#         if altitude > 0.95 * target_position[2]:
#             all_waypoints = [(0.0,0.0,3.0,0.0), (5.0,0.0,3,0), (5.0,5.0,3.0,0),(0.0,5.0,3.0,0.0)]
#             waypoint_transition()
#     if flight_state == States.WAYPOINT:
#         north = value[0]
#         east = value[1]
#         if abs(north - target_position[0]) < 0.05 and abs(east - target_position[1]) < 0.05:
#             print('I reached waypoint.')
#             if check_state[0] == True:
#                 landing_transition()
#             else:
#                 waypoint_transition()

# def velocity_callback(self, attr_name, value):
#     if flight_state == States.LANDING:
#         if ((vehicle.location.global_frame[2] - vehicle.home_location[2] < 0.1) and
#             abs(vehicle.location.local_frame[2]) < 0.01):
#             disarming_transition()

# def state_callback(self, attr_name, value):
#     if not in_mission:
#         return
#     if flight_state == States.MANUAL and vehicle.is_armable:
#         arming_transition()
#     elif flight_state == States.ARMING:
#         if armed:
#             takeoff_transition()
#     elif flight_state == States.DISARMING:
#         if not armed:
#             manual_transition()

def calculate_box():
    return all_waypoints.pop()

def arming_transition():
    vehicle.mode = VehicleMode('GUIDED')
    vehicle.armed = True
    
    vehicle.home_location = vehicle.location.global_frame
    flight_state = States.ARMING

def takeoff_transition():
    print('takeoff transition')
    print(vehicle.location.local_frame)
    target_altitude = 3.0
    target_position[2] = target_altitude
    vehicle.simple_takeoff(target_altitude)
    flight_state = States.TAKEOFF

def waypoint_transition():
    print('waypoint transition')
    if len(all_waypoints) != 0:
        check_state[0] = False
        box_cord = calculate_box()
        target_position[:] = box_cord[:3]
        print(target_position)
        goto_position_target_local_ned(*box_cord)
        flight_state = States.WAYPOINT
    else:
        print('mission complete')
        check_state[0] = True
        flight_state = States.WAYPOINT

def landing_transition():
    vehicle.mode = VehicleMode('LAND')
    flight_state = States.LANDING
            
def disarming_transition():
    print('disarm transition')
    print(vehicle.location.local_frame)
    vehicle.armed = False
    
def manual_transition(self):
    vehicle.mode = VehicleMode('STABILIZED')
    vehicle.close()

