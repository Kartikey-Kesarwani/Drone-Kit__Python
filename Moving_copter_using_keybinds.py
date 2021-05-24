from os import times
from dronekit import connect,VehicleMode,LocationGlobalRelative,LocationGlobal,mavutil
import time
import pygame
 
 #initilizing the pygame
pygame.init()
screen = pygame.display.set_mode((500,500))

vehicle=connect("udp:127.0.0.1:14550",wait_ready=True)
print("hello world")


#takeoff
def arm_and_takeoff(altitude):
    print("basic checkup")
    while not vehicle.is_armable:
        print("waiting to initialize")
        time.sleep(1)
    print("arming motors")
    vehicle.mode=VehicleMode("GUIDED")
    vehicle.armed=True

    while not vehicle.armed:
        print("waiting to be armed")
        time.sleep(1)
    print(vehicle.home_location)
    print("taking off")
    vehicle.simple_takeoff(altitude)

    while True:
        print("altitude: {val}".format(val=vehicle.location.global_relative_frame.alt))
        
        if vehicle.location.global_relative_frame.alt>=altitude*0.95:
            print("target altitude reached")
            break
        time.sleep(1)
arm_and_takeoff(50)


def set_ned_velocity(duration,x,y,z):
    msg=vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,0,0,
        x,y,z,
        0,0,0,
        0,0
    )
    for i in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
def condition_yaw(heading, relative=False):

    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


running=True
speed=15 #m/s
t= 5 #sec

#listening to events and reacting respectively
while running:
    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                set_ned_velocity(t,speed,0,0)
            if event.key == pygame.K_a:
                set_ned_velocity(t,0,-speed,0)
            if event.key == pygame.K_s:
                set_ned_velocity(t,-speed,0,0)
            if event.key == pygame.K_d:
                set_ned_velocity(t,0,speed,0)
            if event.key == pygame.K_UP:
                set_ned_velocity(t,0,0,-speed)
                print("altitude: {val}".format(val=vehicle.location.global_relative_frame.alt))
            if event.key == pygame.K_DOWN:
                set_ned_velocity(t,0,0,speed)
                print("altitude: {val}".format(val=vehicle.location.global_relative_frame.alt))
            if event.key == pygame.K_RIGHT:
                condition_yaw(90,relative=True)
            if event.key == pygame.K_LEFT:
                condition_yaw(-90,relative=True)

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_w or event.key == pygame.K_a or event.key == pygame.K_s or event.key == pygame.K_d or event.key == pygame.K_UP or event.key == pygame.K_DOWN:
                set_ned_velocity(t,0,0,0)
            # elif event.key == pygame.K_RIGHT or event.key == pygame.K_LEFT:
                # condition_yaw(0)
        if event.type == pygame.QUIT:
            # goto_position_target_global_int(vehicle.home_location)
            running = False
    pygame.display.update()
            

