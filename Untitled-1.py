
#%%
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
conn = MavlinkConnection('/dev/ttyACM0,57600', threaded=True, PX4=True)
drone = Drone(conn)
drone.start()


#%%
get_ipython().run_line_magic('pinfo2', 'MavlinkConnection')


#%%
drone.take_control()


#%%
drone.release_control()


#%%
a = []


#%%
def papa():
    return (1, 2, 3),(4,5,6)


#%%
a.append(*papa())


#%%
a


#%%
papa()


#%%
a


#%%
a.pop()


#%%
a


#%%
if !(len(a)): print(a)


#%%
from dronekit import connect, Vehicle, HasObservers


#%%
get_ipython().run_line_magic('pinfo2', 'connect')


#%%
from dronekit import connect, Vehicle, VehicleMode
from enum import Enum
import numpy as np


#%%



#%%
class States(Enum):
    MANUAL = 0
    GUIDED = 1
    ARMING = 2
    TAKEOFF = 3
    WAYPOINT = 4
    LANDING = 5
    DISARMING = 6


#%%
States.MANUAL.value
Vehicle.remo


#%%
class BackyardFlyer(Vehicle):
    def __init__(self, *args):
        super(BackyardFlyer, self).__init__(*args)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.mode = VehicleMode(States.MANUAL)
        self.flight_state = states.MANUAL
        
        # register callbacks
        self.add_attribute_listener('location.local_frame', self.local_location_callback)
        self.add_attribute_listener('velocity', self.velocity_callback)
        self.add_attribute_listener('armed', self.state_callback)
        self.add_attribute_listener('mode', self.state_callback)
        
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
            msg = self.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                0b0000111111111000, # type_mask (only positions enabled)
                north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
                0, 0, 0, # x, y, z velocity in m/s  (not used)
                0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
            # send command to vehicle
            self.send_mavlink(msg)

        
        def local_location_callback(self, attr_name, value):
            if self.flight_state == States.TAKEOFF:
                altitude = -1.0 * value[2]
                if altitude > 0.95 * self.target_position[2]:
                    self.all_waypoints = [(0.0,0.0,3.0,0.0), (5.0,0.0,3,0), (5.0,5.0,3.0,0),(0.0,5.0,3.0,0.0)]
                    self.waypoint_transition()
            if self.flight_state == States.WAYPOINT:
                north = value[0]
                east = value[1]
                if abs(north - self.target_position[0] < 0.05) and abs(east - self.target_position[1] < 0.05):
                    print('I reached waypoint.')
                    if self.check_state[0] == True:
                        self.landing_transition()
                    else:
                        self.waypoint_transition()
        
        def velocity_callback(self, attr_name, value):
            if self.flight_state == States.LANDING:
                if ((self.location.global_frame[2] - self.home_location[2] < 0.1) and
                   abs(self.location.local_frame[2]) < 0.01):
                    self.disarming_transition()
            
        def state_callback(self, attr_name, value):
            if not self.in_mission:
                return
            if self.flight_state == States.MANUAL and self.is_armable:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if not self.armed:
                    self.manual_transition()
                    
        def calculate_box(self):
            return self.all_waypoints.pop()
        
        def arming_transition(self):
            self.mode = VehicleMode('GUIDED')
            self.armed = True
            
            self.home_location = self.location.global_frame
            self.flight_state = States.ARMING
        
        def takeoff_transition(self):
            print('takeoff transition')
            print(self.location.local_frame)
            target_altitude = 3.0
            self.target_position[2] = target_altitude
            self.simple_takeoff(target_altitude)
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
                self.flight_state = States.WAYPOINT
                
        def landing_transition(self):
            self.mode = VehicleMode('LAND')
            self.flight_state = States.LANDING
            
        def disarming_transition(self):
            print('disarm transition')
            print(self.location.local_frame)
            self.armed = False
            
        def manual_transition(self):
            self.mode = VehicleMode('STABILIZED')
            self.close()
            
        def start(self):
            pass
        

        


#%%
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Demonstrates how to create attributes from MAVLink messages. ')
    parser.add_argument('--connect', 
                       help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None
    
    if not connection_string:
        import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
    vehicle = connect(connection_string, wait_ready=True, vehicle_class=BackyardFlyer)
    
    if sitl is not None:
    sitl.stop()


#%%
get_ipython().run_line_magic('pinfo2', 'Vehicle.close')


#%%



