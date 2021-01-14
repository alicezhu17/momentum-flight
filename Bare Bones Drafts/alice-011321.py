import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

#new code (the only portion not copied from demo_mission.py) located in lines 45-81

#helper functions not needed for barebones
def rad_to_degrees(rad):
    #TODO
    return

def degrees_to_rad(deg):
    #TODO
    return

def calc_z_coord(data):
    #TODO
    return

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
    max_alt = 5 
    dest_lat,dest_lon = None, None #should be given

    mission_items = []
    # append (home_lat,home_lon,max_alt), 
    # (dest_lat,dest_lon,max_alt), (dest_lat,dest_lon,0)
    mission_items.append(MissionItem(home_lat,
                                     home_lon,
                                     max_alt,
                                     3, #max ascent velo
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(dest_lat,
                                     dest_lon,
                                     max_alt,
                                     12, #max horiz velo
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(dest_lat,
                                     dest_lon,
                                     0,
                                     1, #max descent velo
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    
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



