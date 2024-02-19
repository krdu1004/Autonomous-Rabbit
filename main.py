from lane_keeping.controller import steering_angle_controller
from lane_keeping.line_detection import line_detection
from lane_keeping.PIDController import PIDController
from people_avoidance.emergency_braking import object_detection, init_object_detection
from camera import camera_init, get_camera_image
import cv2


#sl.Camera.reboot()?
# sl.rebootCamera()

# Camera init:
mat, camera, runtime, cam_status = camera_init()
viewer, objects, obj_runtime_param = init_object_detection(camera)
test_image = get_camera_image(mat, camera, runtime)
print("Camera Status: ", cam_status)
assert test_image is not None, "file could not be read, check with os.path.exists()"
height, width, depth = test_image.shape
set_width = 640       # Can be changed with 
show_image = False

# PIDs:
angle_setpoint = -90  # The desired angle you want to maintain
position_setpoint = set_width/2  # The desired position you want to reach
angle_pid = PIDController(kp=0.5, ki=0.1, kd=0.2, setpoint=angle_setpoint)
position_pid = PIDController(kp=0.1, ki=0.01, kd=0.02, setpoint=position_setpoint)


import time
times = []
# Main loop
s1 = time.time()
for i in range(200): # FOR TESTING, 10 iterations
    print("---------------")
    s2 = time.time()
# while True:
    image = get_camera_image(mat, camera, runtime)

    # print("grab: ", time.time()-s2)
    # print("Autonomous-Rabbit/bilder/video/00"+str(i+1)+".jpg")
    # image = cv2.imread("Autonomous-Rabbit/bilder/video/"+(5-len(str(i+1)))*str(0)+str(i+1)+".jpg")

    # assert image is not None, "file could not be read, check with os.path.exists()"

    # first avoid crashing
    emergency_brake, speed = object_detection(camera, mat, objects, viewer, obj_runtime_param, show_image)
    # print("Emergency status:", emergency_brake, "Speed:", speed)
    
    # print("emergency brake: ", time.time()-s2)
    
    if not emergency_brake:

        # Lane keeping:
        lane_lines = line_detection(image, set_width, cut_top=1/4)
        # print("lane detection: ", time.time()-s2)

        if lane_lines != None:
            steering_angle = steering_angle_controller(image, angle_pid, position_pid, lane_lines)
            # print("steering angle: ", time.time()-s2)
            # print(i, steering_angle)
        # Get speed:

    # Send speed and angle to robot:
    times.append(time.time()-s2)
    # cv2.waitKey(0)

times.pop(0)
print("Final time:", time.time()-s1)
mean = sum(times) / len(times) 
variance = sum([((x - mean) ** 2) for x in times]) / (len(times)-1) 
std = variance ** 0.5
print(times)
print(mean)
print(std)
camera.close()