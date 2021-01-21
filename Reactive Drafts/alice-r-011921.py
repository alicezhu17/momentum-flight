import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

import time # For the example only
import pygazebo
# What you import here depends on the message type you are subscribing to
import pygazebo.msg.v11.laserscan_stamped_pb2
import math

# approximation assume 1 degree = 111000m
def deg_to_m(d):
    m = d*111000
    return m 

def m_to_deg(m):
    d = m/111000
    return d

def middle_range_min(data):
    '''Given lidar data as "data"
    Return range_min of middle sensors in meters
    
    Middle sensors should be index 9, 29 etc (check)
    data.scan.ranges[some index] to access
    '''
    middle_range = []
    for i in range(9, 180, 20):
        middle_range.append(data.scan.ranges[i])
    range_min = min(middle_range)
    return range_min


def rad_to_degrees(rad):
    return rad*180/math.pi

def degrees_to_rad(deg):
    return deg*math.pi/180

def calc_deltaz(data):
    '''Given lidar data as "data"
    Return positive change in z coordinate "deltaz"
    '''
    dis = dis_pointing_down(data) #TODO which index is pointing down? or lidar_data.range_min for closest distance
    deltaz = 0
    if abs(dis-AGL) < 1: #if distance is below a random number
        deltaz = 3 #increase altitude by random number
    return deltaz

def dis_pointing_down(data):
    '''Given lidar data as "data"
    Use orientation to calculate the index i of data.scan.ranges[i] that points down and
    Return the distance of data.scan.ranges[i] "dis"

    Note: I think data.scan.ranges[8x20=160] points down if orientation is normal
    '''
    i = 0 #TODO use orientation to calculate index i that points down
    dis = data.scan.ranges[i]
    return dis

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
    
    #MAIN PART OF CODE
    print(home_alt,home_lat,home_lon)
    
    max_alt = 5 
    dest_lat,dest_lon = home_lat + 0.0001, home_lon + 0.0001 #TODO given, assume in degrees, convert if nec
    x,y = home_lat, home_lon #degrees
    AGL = 4 #TODO should be given, meters

    gz_sub = GazeboMessageSubscriber(HOST, PORT)
    asyncio.ensure_future(gz_sub.connect()) #connects with lidar
    

    
    #data = await gz_sub.get_LaserScanStamped()

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(1)
    
    moves = 0
    landing_dis = m_to_deg(1)
    while abs(x-dest_lat) > landing_dis and abs(y-dest_lon) > landing_dis: #if far, move 
        moves += 1
        data = await gz_sub.get_LaserScanStamped() #gets lidar_data
        #print(data) #will print data when program is run

        x = m_to_deg(data.scan.world_pose.position.x) #converted to degrees
        y = m_to_deg(data.scan.world_pose.position.y) 
        z = m_to_deg(data.scan.world_pose.position.z) 
        closest_obs = middle_range_min(data) #meters
        
        deltaxm, deltaym, deltazm = 0, 0, 0 #meters
        if closest_obs<AGL:#if close, go up. if < 4
            deltazm = 4
            await drone.action.set_maximum_speed(3) #max ascent velo
            print("drone up at", round(x,5),round(y,5),round(z,5))
        elif 1.25*AGL<closest_obs:#if far, go down. if > 8
            deltazm = -1
            await drone.action.set_maximum_speed(1) #max descent velo
            print("drone down at", round(x,5),round(y,5),round(z,5))
        else:#can move between 4-8m so at most 4m, or 4/sqrt(2)=2.8 per side
            #landscape is at most 40*sqrt(2), to move at most 4m need to multiply by .07
            deltaxm = deg_to_m((dest_lat-home_lat)*.15) #experiment
            deltaym = deg_to_m((dest_lon-home_lon)*.15)          
            await drone.action.set_maximum_speed(12) #max hori velo   
            print("drone horiz at", round(x,5),round(y,5),round(z,5))
            
        deltax = m_to_deg(deltaxm) #degrees
        deltay = m_to_deg(deltaym) #degrees
        deltaz = deltazm #meters
        x += math.copysign(1, dest_lat-x)*deltax #goes deltax "units" in direc of dest 
        y += math.copysign(1, dest_lon-y)*deltay #degrees
        z += deltaz #meters
        await drone.action.goto_location(x,y,z,0) #degrees, degrees, meters
        

    await drone.action.goto_location(dest_lat,dest_lon,0,0) #land when close
    data = await gz_sub.get_LaserScanStamped()
    print("drone down at", round(dest_lat,5),round(dest_lon,5),round(0,5))
    print("Total moves is", moves)
    print("Current time is ", data.time.sec," seconds")
    print("Mission Complete")


    '''
    #pseudocode
    if close to obstacle: #closest_obs < some_random_number
        go up, set deltazm to something
    elif far:
        go down, set deltazm to something
    else:
        go straight, set deltaxm and deltaym to something
    
    '''

    '''
    # PSEUDOCODE
    if far from destination:
        move a lot
    elif kinda close:
        move a medium amount
    if close:
        move a tiny bit because the tolerance on Alices function above is .1m
    '''

    '''#don't think this portion needed if we use goto function
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



