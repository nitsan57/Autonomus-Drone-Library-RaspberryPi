import dronekit_sitl
import time
import math
import numpy as np
from picamera import PiCamera
from time import sleep
import os
# Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import read_json

DIRECTORY= "/home/pi/Documents/drone"
IMG_DIRECTORY= "/home/pi/Documents/drone/images"
FlYING_HEIGHT = 15

WIDTH_ANGLE = math.radians(62.2)
HIGHT_ANGLE = math.radians(48.8)
last_lon=-1
last_lat=-1
last_alt=-1
last_angle=-1


def init():
    camera = PiCamera()
    #vehicle = connect("udp:127.0.0.1:14551", wait_ready=True)
    vehicle = connect("udp:172.29.100.144:14551", wait_ready=True)
    print("Get some vehicle attribute values:")
    print(" GPS: %s" % vehicle.gps_0)
    print (" Battery: %s" % vehicle.battery)
    print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print (" System status: %s" % vehicle.system_status.state)
    print (" Mode: %s" % vehicle.mode.name)  # settable
    return vehicle, camera


def pixelSize(distance, image_h, image_w):
    h_pixel = (math.sin(HIGHT_ANGLE / 2) * 2 * distance) / math.cos(HIGHT_ANGLE/2)
    w_pixel = (math.sin(WIDTH_ANGLE / 2) * 2 * distance) / math.cos(WIDTH_ANGLE/2)
    return h_pixel / image_h, w_pixel / image_w


x, w = pixelSize(15, 1944, 2592)
CAMERA_RANGE_RADIUS=1944*x / 2


# Close vehicle object before exiting script

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation


def gotoPoint(vehicle, targetLocation):
    vehicle.simple_goto(targetLocation)
    currentLocation = vehicle.location.global_relative_frame
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=0.2: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)


def goto(vehicle, dNorth, dEast):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
    the target position. This allows it to be called with different position-setting commands.
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    gotoFunction = vehicle.simple_goto
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)



def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def smallest_dist(drone_location, way_points_list):
    print("before sort")
    for i in range(len(way_points_list)):
        way_points_list[i][1] = get_distance_metres(drone_location, way_points_list[i][0])
    return sorted(way_points_list, key=lambda x: x[1])

def take_pic_object(camera, vehicle, currentLocation, rotation, object_class):
    print("Take Picture!")
    camera.rotation = rotation
    camera.start_preview()
    time.sleep(5)
    lon = currentLocation.lon
    lat = currentLocation.lat
    angle = vehicle.attitude.yaw
    alt = currentLocation.alt
        
    camera.capture(IMG_DIRECTORY+'/'+str(lon)+','+str(lat)+','+str(alt)+','+str(angle)+'.jpg')
    camera.stop_preview()
    os.chdir(DIRECTORY)
    os.system("./detectTiny ./images/")
    object_list = read_json.text_parsing(IMG_DIRECTORY+'/'+str(lon)+','+str(lat)+','+str(alt)+','+str(angle)+'.txt')
    for object_value in object_list:
        if object_value[0] == object_class:
			print("there is a" + object_class)
			return object_value
    os.remove(IMG_DIRECTORY+'/'+str(lon)+','+str(lat)+','+str(alt)+','+str(angle)+'.jpg')
    os.remove(IMG_DIRECTORY+'/'+str(lon)+','+str(lat)+','+str(alt)+','+str(angle)+'.txt')
    return False




def findObject(p1, p2, p3, p4, object_class):
    """
    makes folder - images and put images on it
    """
    vehicle, camera = init()
    arm_and_takeoff(vehicle, FlYING_HEIGHT)
    print("Take off complete")
    print("Scannig Area Vehicle.simple_goto()")

    print("Set airsoeed to 2.5m/s.")
    vehicle.airspeed = 2.5


    print("flyyyyy")
    print("drone locationX:",  vehicle.location.global_frame.lon)
    print("drone locationY:", vehicle.location.global_frame.lat)
    drone_initial_location = vehicle.location.global_relative_frame
    p_list = np.array([[p1.lat, p1.lon], [p2.lat, p2.lon], [p3.lat, p3.lon], [p4.lat, p4.lon]])
    p_distance_list = [[p1, 0], [p2, 0], [p3, 0], [p4, 0]]
    p_distance_list = smallest_dist(drone_initial_location, p_distance_list)
    sortByX = sorted(p_list, key=lambda x: x[0])
    sortByY = sorted(p_list, key=lambda x: x[1])
    rectEastX = sortByY[3][1]
    rectWestX = sortByY[0][1]
    rectNorthY = sortByX[3][0]
    rectSouthY = sortByX[0][0]
    direction = [0, 0]
    compass = [0, 0]
    nearestP = p_distance_list[0][0]
    if nearestP.lon == rectEastX and nearestP.lat == rectSouthY:
        print("go north and west")
        compass = [-1, 1]
        direction = [-1, 1]
    if nearestP.lon == rectEastX and nearestP.lat == rectNorthY:
        print("go south and west")
        compass = [-1, -1]
        direction = [-1, -1]
    if nearestP.lon == rectWestX and nearestP.lat == rectNorthY:
        print("go south and east")
        compass = [1, -1]
        direction = [1, -1]
    if nearestP.lon == rectWestX and nearestP.lat == rectSouthY:
        print("go north and west")
        compass = [1, 1]
        direction = [1, 1]

    startingPoint = get_location_metres(nearestP, CAMERA_RANGE_RADIUS*direction[1], CAMERA_RANGE_RADIUS*direction[0])
    print("!!!!!!!!! "+ str(startingPoint.alt))
    startingPoint.alt = FlYING_HEIGHT
    gotoPoint(vehicle, startingPoint)
    currentLocation = vehicle.location.global_relative_frame
    vehicleX = currentLocation.lon
    vehicleY = currentLocation.lat
    time.sleep(2)
    pic_number = 1
    found_object = take_pic_object(camera, vehicle, currentLocation,0,object_class)
    if found_object != False:
        return found_object
    else:
        pic_number+=1

    print("take pic")

    # sortByX = sorted(p_list, key=lambda x: x[0])
    # sortByY = sorted(p_list, key=lambda x: x[1])
    rectEastX = sortByY[3][1]
    rectWestX = sortByY[0][1]
    rectNorthY = sortByX[3][0]
    rectSouthY = sortByX[0][0]
    # print("veicle.x:", vehicleX)
    # print("veicle.y:", vehicleY)
    # print("rectEastX:", rectEastX)
    # print("rectWestX:", rectWestX)
    # print("rectNorthY:", rectNorthY)
    # print("rectSouthY:", rectSouthY)
    rotate = 0
    while rectWestX < vehicleX < rectEastX and rectSouthY < vehicleY < rectNorthY:
        print("vehicleX in start of while:", vehicleX)
        while(rectWestX < get_location_metres(currentLocation, 0, compass[0]*2*CAMERA_RANGE_RADIUS+ 2).lon < rectEastX):
            time.sleep(2)
            goto(vehicle,0, compass[0] * CAMERA_RANGE_RADIUS * 2)
            currentLocation = vehicle.location.global_relative_frame
            found_object = take_pic_object(camera, vehicle, currentLocation, rotate, object_class)
            if found_object != False:
                print("found object!")
                return found_object
            else:
                pic_number+=1
            #print("take pic")
        print("finished lane")
        if rotate ==0:
            rotate = 90
        if rotate == 180:
            rotate = 270
        compass[0] *=-1
        time.sleep(2)
        goto(vehicle,compass[1] * CAMERA_RANGE_RADIUS * 2, 0)
        print("went up")
        #print("take pic")
        currentLocation = vehicle.location.global_relative_frame
        found_object = take_pic_object(camera, vehicle, currentLocation, rotate, object_class)
        if found_object != False:
            print("found object!")
            return found_object
        else:
            pic_number+=1
        if rotate ==90:
            rotate = 180
        if rotate == 270:
            rotate = 0
        vehicleX = vehicle.location.global_relative_frame.lon
        vehicleY = vehicle.location.global_relative_frame.lat
        print("vehicleX in end of up:",vehicleX)
    print("ended while")

    # goto_position_target_local_ned(150, 50, 15)

    # Hover for 1 seconds
    time.sleep(2)
    land(vehicle)
    while True:
        continue



def land(vehicle):
    print("Now let's land")
    vehicle.mode = VehicleMode("RTL")
    # Close vehicle object
    vehicle.close()
    # Shut down simulator
    # sitl.stop()
    print("Completed")


def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    #    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        vehicle.armed = True
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
   # if not os.path.exists("panorama"):
    #    os.makedirs("panorama")



# Initialize the takeoff sequence to 20m


# p1 = LocationGlobalRelative(31.7766, 35.20201, FlYING_HEIGHT)
# p2 = LocationGlobalRelative(31.77592, 35.20078, FlYING_HEIGHT)
# p3 = LocationGlobalRelative(31.7766, 35.20078, FlYING_HEIGHT)
# p4 = LocationGlobalRelative(31.775921,35.202014, FlYING_HEIGHT)

# circleToSquare();
#findObject(p1, p2, p3, p4,object_class)

#run cppp prigram

