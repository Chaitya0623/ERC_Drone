from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal
import time
import cv2
import math
import os
import random
import threading
from geopy.distance import geodesic

def printGreen(skk): print("\033[92m {}\033[00m" .format(skk))
def printRed(skk): print("\033[91m {}\033[00m" .format(skk))

def local_to_gnss(start_point, relative_point):
    rel_dx, rel_dy, rel_alt = relative_point
    new_point = geodesic(meters=rel_dx).destination((start_point.lat, start_point.lon), bearing=0)
    new_point = geodesic(meters=rel_dy).destination((new_point.latitude, new_point.longitude), bearing=90)
    new_point_alt = start_point.alt + rel_alt
    
    return (new_point.latitude, new_point.longitude, new_point_alt)

def detect_aruco_markers():
    cap = cv2.VideoCapture(2)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters()

    arucos = {101: 'Start', 102: 'A', 103: 'B', 104: 'C'}
    if not os.path.exists("detected_images"):
        os.makedirs("detected_images")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            if ids[0][0] in arucos.keys():
                filename = os.path.join("detected_images", f"{ids[0][0]}_{random.randint(1,20)}.jpeg") # change number of images before task
                cv2.imwrite(filename, frame)
                print(f'Aruco Detected: {arucos.get(ids[0][0])}')

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

def arm_and_takeoff(target_altitude):
    print("Pre-arm checks...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode.name == 'GUIDED':
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)
        
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    printGreen('Vehicle Armed')
    time.sleep(4)

    printGreen(f"Gradually taking off to altitude: {target_altitude}")
    vehicle.simple_takeoff(target_altitude)
    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        print(vehicle.channels.overrides)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            printGreen(f"Reached target altitude: {target_altitude}")
            break
    
    time.sleep(4)

def disarm():
    print("Disarming...")
    vehicle.mode = VehicleMode("LOITER")

    while vehicle.armed:
        print(" Waiting for disarm...")
        time.sleep(1)

def gradual_takeoff(target_altitude, step_size=0.5):
    printGreen(f"Gradually taking off to altitude: {target_altitude}")
    
    current_altitude = vehicle.location.global_relative_frame.alt
    
    while current_altitude < target_altitude:
        next_altitude = min(current_altitude + step_size, target_altitude)        
        vehicle.simple_takeoff(next_altitude)
        time.sleep(1)
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Current altitude: {current_altitude}")

    printGreen("Reached target altitude")
    time.sleep(3)
    
def land():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.location.global_relative_frame.alt > 0:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        time.sleep(1)

    printGreen("Landed")

    while vehicle.armed:
        print("Waiting for disarm...")
        time.sleep(1)

    printGreen("Disarmed")

def wait_until_reached(target_location, threshold=0.5):
    while True:
        current_location = vehicle.location.global_frame
        dist_to_target = get_distance_metres(current_location, target_location)
        print(f"Distance to target: {dist_to_target}")
        
        if dist_to_target <= threshold:
            printGreen(f"Reached target location: {target_location}")
            break
        
        time.sleep(1)

def get_distance_metres(location1, location2):
    coords_1 = (location1.lat, location1.lon)
    coords_2 = (location2.lat, location2.lon)

    distance = geodesic(coords_1, coords_2).meters
    return distance

aruco_thread = threading.Thread(target=detect_aruco_markers)
aruco_thread.start()

try:
    connection_string = '/dev/ttyUSB0'
    vehicle = connect(connection_string, wait_ready=False, baud=57600)
    vehicle.mode = VehicleMode("ACRO")
    printGreen('Vehicle Connected')
    vehicle.groundspeed = 0.2 # change drone horizontal speed

    altitude = 2 # change altitude

    start_point = vehicle.location.global_frame
    print('Start Coordinates: ', start_point)

    relative_A = (1,0,altitude)
    relative_B = (0.3,0,altitude)
    relative_C = (0,0.3,altitude)

    coordinates_A = local_to_gnss(start_point, relative_A)
    print('Point A Coordinates: ', coordinates_A)
    coordinates_B = local_to_gnss(start_point, relative_B)
    print('Point B Coordinates: ', coordinates_B)
    coordinates_C = local_to_gnss(start_point, relative_C)
    print('Point C Coordinates: ', coordinates_C)
    
    arm_and_takeoff(altitude)
    time.sleep(3)

    lat_A, lon_A, alt_A = coordinates_A
    target_location_A = LocationGlobalRelative(lat_A, lon_A, alt_A)
    printGreen(f'Going to location: {target_location_A}')
    vehicle.simple_goto(target_location_A)
    wait_until_reached(target_location_A)
    time.sleep(3)
    printGreen('Reached Waypoint A')
    time.sleep(3)

    lat_B, lon_B, alt_B = coordinates_B
    target_location_B = LocationGlobalRelative(lat_B, lon_B, alt_B)
    printGreen(f'Going to location: {target_location_B}')
    vehicle.simple_goto(target_location_B)
    wait_until_reached(target_location_B)
    time.sleep(3)
    printGreen('Reached Waypoint B')
    time.sleep(3)
    land()
    disarm()
    printGreen("Disarmed")
    time.sleep(20)

    arm_and_takeoff(altitude)
    time.sleep(3)

    lat_C, lon_C, alt_C = coordinates_C
    target_location_C = LocationGlobalRelative(lat_C, lon_C, alt_C)
    printGreen(f'Going to location: {target_location_C}')
    vehicle.simple_goto(target_location_C)
    wait_until_reached(target_location_C)
    time.sleep(3)
    printGreen('Reached Waypoint C')
    time.sleep(3)

    lat_start, lon_start, alt_start = start_point
    target_location_start = LocationGlobalRelative(lat_start, lon_start, alt_start)
    printGreen(f'Going to location: {start_point}')
    vehicle.simple_goto(target_location_start)
    wait_until_reached(target_location_C)
    time.sleep(3)
    printGreen('Reached End Coordinate')
    time.sleep(3)

    land()
    printGreen('TASK COMPLETED')
    disarm()
    
except Exception as e:
    printRed(e)

except KeyboardInterrupt:
    printRed("Disarming and closing vehicle...")
    land()
    disarm()

vehicle.close()
print("Vehicle closed successfully")
aruco_thread.join()