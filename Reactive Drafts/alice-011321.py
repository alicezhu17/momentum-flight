import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

import time # For the example only
import pygazebo
# What you import here depends on the message type you are subscribing to
import pygazebo.msg.v11.laserscan_stamped_pb2
import math

#i think this is the structure
#new code (portion not copied) located in calc_z_coord and lines 104-123

def rad_to_degrees(rad):
    #TODO
    return

def degrees_to_rad(deg):
    #TODO
    return

def calc_z_coord(data):
    dis = data.scan.ranges[140] #TODO which index is pointing down?
    deltaz = 0
    if abs(dis-AGL) < 1:
        deltaz = 3 #increase altitude by random number
    return deltaz


# copied from lidar_read.py
# This is the gazebo master from PX4 message `[Msg] Connected to gazebo master @ http://127.0.0.1:11345`
HOST, PORT = "127.0.0.1", 11345

class GazeboMessageSubscriber: 
    def __init__(self, host, port, timeout=30):
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout

    async def connect(self):
        connected = False
        for i in range(self.timeout):
            try:
                self.manager = await pygazebo.connect((self.host, self.port))
                connected = True
                break
            except Exception as e:
                print(e)
            await asyncio.sleep(1)

        if connected: 
            # info from gz topic -l, gz topic -i arg goes here
            self.gps_subscriber = self.manager.subscribe('/gazebo/default/iris_lmlidar/lmlidar/link/lmlidar/scan', 'gazebo.msgs.LaserScanStamped', self.LaserScanStamped_callback)

            await self.gps_subscriber.wait_for_connection()
            self.running = True
            while self.running:
                await asyncio.sleep(0.1)
        else:
            raise Exception("Timeout connecting to Gazebo.")

    def LaserScanStamped_callback(self, data):
        # What *_pb2 you use here depends on the message type you are subscribing to
        self.LaserScanStamped = pygazebo.msg.v11.laserscan_stamped_pb2.LaserScanStamped()
        self.LaserScanStamped.ParseFromString(data)
    
    async def get_LaserScanStamped(self):
        for i in range(self.timeout):
            try:
                return self.LaserScanStamped
            except Exception as e:
                # print(e)
                pass
            await asyncio.sleep(1)


async def run():
    #section copied from demo_mission.py
    #connects drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        home_alt = terrain_info.relative_altitude_m
        home_lat = terrain_info.latitude_deg
        home_lon = terrain_info.longitude_deg
        break
    
    #copied from mavsdk goto.py
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    #MAIN PART OF CODE
    max_alt = 5 
    dest_lat,dest_lon = None, None #TODO should be given
    x,y = home_lat, home_lon
    AGL = 10 #TODO  should be given

    gz_sub = GazeboMessageSubscriber(HOST, PORT)
    asyncio.ensure_future(gz_sub.connect())
    while abs(x-dest_lat) > 0.1 or abs(y-dest_lon) > 0.1: #if far, move
        data = await gz_sub.get_LaserScanStamped()
        x = data.scan.world_pose.position.x #TODO get current x, y, z
        y = data.scan.world_pose.position.y 
        z = data.scan.world_pose.position.z 
        deltaz = calc_z_coord(data)
        z += deltaz
        x += math.copysign(1, dest_lat-x)*5 #goes 5m in direc of dest i think
        y += math.copysign(1, dest_lat-x)*5
        await drone.action.goto_location(x,y,z)

    drone.action.goto_location(dest_lat,dest_lon,0) #land if close

    '''#not sure if this portion needed if we use goto function
    #rest copied from demo_mission.py
    #upload/start mission
    inject_pt_task = asyncio.ensure_future(inject_pt(drone, mission_items, home_alt, home_lat, home_lon))
    running_tasks = [inject_pt_task]
    
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))
    
    mission_plan = MissionPlan(mission_items)
    
    print("-- Arming")
    await drone.action.arm()

    print("awaiting")
    await asyncio.sleep(1)
    print("awaiting done")

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("awaiting")
    await asyncio.sleep(1)
    print("awaiting done")

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task
    '''

async def inject_pt(drone, mission_items, home_alt, home_lat, home_lon):
    pt_injected = False
    async for mission_progress in drone.mission.mission_progress():
        if(not mission_progress.current == -1):
            print(f"Mission progress: "
                f"{mission_progress.current}/"
                f"{mission_progress.total}")

            if(mission_progress.current == mission_progress.total and not pt_injected):
                mission_item_idx = mission_progress.current
                print("-- Pausing mission")
                await drone.mission.pause_mission()
                await drone.mission.clear_mission()

                print(f"-- Injecting waypoint at "
                f"{mission_item_idx}")

                mission_items.insert(mission_progress.current, MissionItem(home_lat,
                                     home_lon,
                                     home_alt,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
                mission_plan = MissionPlan(mission_items)
                await drone.mission.set_return_to_launch_after_mission(True)

                print("-- Uploading updated mission")
                await drone.mission.upload_mission(mission_plan)

                print("-- Resuming mission")
                await drone.mission.set_current_mission_item(mission_item_idx)
                await drone.mission.start_mission()

                pt_injected = True
        if(mission_progress.current == mission_progress.total):
            print("-- Landing")
            await drone.action.land()


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    print("Observing in air")

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())



