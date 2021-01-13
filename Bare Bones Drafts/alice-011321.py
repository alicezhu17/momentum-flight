import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

#THIS IS a partially done rough draft, not very helpful at the moment

def rad_to_degrees(rad):
    #TODO
    return

def degrees_to_rad(deg):
    #TODO
    return

def barebones():
    '''
    #THIS SECTION IS COPIED FROM demo_mission.py
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
    '''

    max_alt = None #TODO, find 
    dest_lat,dest_lon = None #TODO, find

    mission_items = []
    # append mission items
    # (home_lat,home_lon,max_alt)
    # (dest_lat,dest_lon,max_alt)
    # (dest_lat,dest_lon,0)






