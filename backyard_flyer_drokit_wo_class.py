from dronekit import connect, Vehicle, VehicleMode
from enum import Enum
import numpy as numpy
from pymavlink import mavutil
import socket
import exceptions

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

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
print('Connecting to vehicle on: %s' % connection_string)
try:
    vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)

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

else:
    print 'connected sucessfully'


# initialization of backyard class
target_position = np.array([0.0, 0.0, 0.0])
all_waypoints = []
in_mission = True
check_state = {}

# initial state
flight_state = States.MANUAL
# go to manual mode
vehicle.mode = VehicleMode('STABILIZE')
while not (vehicle.mode.name == 'STABILIZE'):
    print('waiting for stabilized mode')
    print(vehicle.mode)

# register callbacks
vehicle.add_attribute_listener('location.local_frame', local_location_callback)
vehicle.add_attribute_listener('velocity', velocity_callback)
vehicle.add_attribute_listener('armed', state_callback)
vehicle.add_attribute_listener('mode', state_callback)

