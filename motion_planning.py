import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from planning_utils import read_data, calculate_start_goal_local_position
from planning_utils import create_grid,find_path_grid
from planning_utils import create_grid_and_edges_voronoi,create_graph_from_edges,find_path_graph_voronoi


GRID = False
VORONOI = True


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal_global_position=None):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.goal_global_position = goal_global_position

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 3:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        colliders_file = 'colliders.csv'
        # TODO: read lat0, lon0 from colliders into floating point values
        # Read in obstacle map
        lat0, lon0 = read_data(colliders_file)
        data = np.loadtxt(colliders_file, delimiter=',', dtype='Float64', skiprows=3)
        print(f'Home lat : {lat0}, lon : {lon0}')

        # # # TODO: set home position to (lat0, lon0, 0)
        self.set_home_position(lon0, lat0, 0)

        # Define a grid/Graph for a particular altitude and safety margin around obstacles
        if(GRID is True):
            grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        if (VORONOI is True):
            grid, edges, north_offset, east_offset = create_grid_and_edges_voronoi(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            G = create_graph_from_edges(edges)
            
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        # TODO: adapt to set goal as latitude / longitude position and convert
        # TODO: convert start position to current position rather than map center
        grid_start,grid_goal = calculate_start_goal_local_position(self.global_position, goal_global_position,
                                                                   self.global_home, north_offset, east_offset)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        print('Local Start and Goal: ', grid_start, grid_goal)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        # TODO: prune path to minimize number of waypoints
        if(GRID is True):
            path = find_path_grid(grid, grid_start, grid_goal)
        
        if (VORONOI is True):
             path = find_path_graph_voronoi(G, grid_start, grid_goal)
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_lon', type=str,default='-122.40195876', help="Goal longitude")
    parser.add_argument('--goal_lat', type=str,default='37.79673913', help="Goal latitude")
    parser.add_argument('--goal_alt', type=str,default='0.0', help="Goal altitude")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    goal_global_position = np.fromstring(f'{args.goal_lon},{args.goal_lat},{args.goal_alt}', dtype='Float64', sep=',')
    drone = MotionPlanning(conn, goal_global_position)
    time.sleep(1)

    drone.start()
