import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

import time # For the example only
import pygazebo
# What you import here depends on the message type you are subscribing to
import pygazebo.msg.v11.laserscan_stamped_pb2
import math

# approximation assumes 1 degree = 111000m
def deg_to_m(d):
    m = d*111000
    return m 

def m_to_deg(m):
    d = m/111000
    return d

def middle_range_min(data):
    '''Given lidar data as "data"
    Return range_min of middle sensors in meters
    
    Middle sensors are index 9, 29 etc
    data.scan.ranges[index] to access
    '''
    middle_range = [data.scan.ranges[i] for i in range(9, 180, 20)]
    return min(middle_range)

def fourth_range(data):
    return data.scan.ranges[109]

def back_range(data):
    back = [data.scan.ranges[i] for i in range(9, 60, 20)]
    return min(back)

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
    #connects drone from demo_mission.py
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
    
    #MAIN PART OF ALGORITHM
    #max_alt = 5, AGL = 3 meters
    home_latm, home_lonm = deg_to_m(home_lat), deg_to_m(home_lon)
    print(round(home_latm,3),round(home_lonm,3))

    dest_lat,dest_lon = 0.962, 39 #y=0.962 x=39 meters
    dest_latd,dest_lond = m_to_deg(dest_lat),m_to_deg(dest_lon)
    
    gz_sub = GazeboMessageSubscriber(HOST, PORT)
    asyncio.ensure_future(gz_sub.connect()) #connects with lidar
    data = await gz_sub.get_LaserScanStamped()

    print("-- Arming") #takeoff
    await drone.action.arm()
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(1)

    #to find average height and total time
    lastx = 1
    totz = 0
    zmoves = 0
    start_time = data.time.sec

    moves = 0
    x = home_lon
    destx = m_to_deg(35)
    while x<destx: #if far, move 
        moves += 1
        data = await gz_sub.get_LaserScanStamped() #gets lidar_data

        x = data.scan.world_pose.position.x #meters
        y = data.scan.world_pose.position.y 
        z = data.scan.world_pose.position.z 
        closest_obs = middle_range_min(data) #meters
        
        deltaxm, deltazm = 0, 0 #meters
        
        if abs(back_range(data)-fourth_range(data)) < 0.5:#if bottom lidars are close in reading, go diag
            deltazm = 1.5
            deltaxm = 2.5 
            await drone.action.set_maximum_speed(12) #set max hori velo
            print("drone diag at", round(x,3),round(y,3),round(z,3),"at",data.time.sec,"seconds")
        #elif close, go up (redundant)
        elif 3.5<closest_obs:#elif far, go down 
            deltazm = -1.5 
            await drone.action.set_maximum_speed(1) #set max descent velo
            print("drone down at", round(x,3),round(y,3),round(z,3),"at",data.time.sec,"seconds")
        else:#go horiz
            deltaxm = 2         
            await drone.action.set_maximum_speed(12) #set max hori velo   
            print("drone horiz at", round(x,3),round(y,3),round(z,3),"at",data.time.sec,"seconds")
            
        x += deltaxm #meters
        z += deltazm #meters
        x = m_to_deg(x)
        await drone.action.goto_location(dest_latd,x,z,90) #degrees, degrees, meters #lat=y,lon=x,height

        if lastx < deg_to_m(x):#finds total height for average height
            totz += z
            zmoves += 1
            lastx += 1


    await drone.action.goto_location(dest_latd,dest_lond,1,0) #land (39 1 1) when close,degrees
    data = await gz_sub.get_LaserScanStamped()
    print("drone down at", round(dest_lat,3),round(dest_lon,3),round(0,3))
    print("Total feet was",totz,"for",zmoves,"feet")
    print("Avg height was", round(totz/zmoves,3),"m")
    print("Total moves is", moves)
    print("Current time is", data.time.sec,"seconds")
    print("Total time is", data.time.sec-start_time,"seconds")
    print("Mission Complete")



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



