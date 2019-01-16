from dronekit import connect, Vehicle

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Vehicle):
    def __init__(self, *args):
        super(BackyardFlyer, self).__init__(*args)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self._flightmode = Statesx









vehicle = connect(connection_string, wait_ready=True, vehicle_class=BackyardFlyer)
