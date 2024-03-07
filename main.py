from lane_keeping.controller import steering_angle_controller
from lane_keeping.line_detection import line_detection
from lane_keeping.PIDController import PIDController
from people_avoidance.object_detection import object_detection, init_object_detection
from people_avoidance.body_tracking import init_body_tracking, body_tracking
from camera import camera_init, get_camera_image
import matplotlib.pyplot as plt
import pyzed.sl as sl
import time
import cv2


#sl.Camera.reboot()?
# sl.rebootCamera()


# Tunable parameters:
set_width = 640      
show_image = False
fps = 30
resolution=sl.RESOLUTION.SVGA
detection_confidence = 40
detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST

# Camera init:
mat, camera, runtime, cam_status = camera_init(fps, resolution)

# viewer, objects, obj_runtime_param = init_object_detection(camera, detection_confidence, show_image, detection_model)
viewer, objects, obj_runtime_param = init_body_tracking(camera, detection_confidence, show_image, detection_model)

test_image = get_camera_image(mat, camera, runtime)
print("Camera Status: ", cam_status)


# PIDs:
angle_setpoint = -90  # The desired angle you want to maintain
position_setpoint = set_width/2  # The desired position you want to reach
angle_pid = PIDController(kp=0.5, ki=0.1, kd=0.2, setpoint=angle_setpoint)
position_pid = PIDController(kp=0.1, ki=0.01, kd=0.02, setpoint=position_setpoint)


times = []
get_image = []
rest = []
# Main loop
s1 = time.time()
for i in range(200): # FOR TESTING, 200 iterations
    print("---------------")
    s2 = time.time()
    s3 = time.time()
    image = get_camera_image(mat, camera, runtime)
    assert image is not None, "file could not be read, check with os.path.exists()"

    get_image.append(time.time()-s3)

    s4=time.time()

    # emergency_brake, speed = object_detection(camera, mat, objects, viewer, obj_runtime_param, show_image)
    emergency_brake, speed = body_tracking(camera, mat, objects, viewer, obj_runtime_param, show_image)

    # print("Emergency status:", emergency_brake, "Speed:", speed)
    
    if not emergency_brake:
        lane_lines = line_detection(image, set_width, cut_top=1/4)

        if lane_lines != None:
            steering_angle = steering_angle_controller(image, angle_pid, 
                                                       position_pid, lane_lines)
        # Get SPEED:
        

    # Send speed and angle to robot:
    times.append(time.time()-s2)
    rest.append(time.time()-s4)




# # timinig prints:
# times.pop(0)
# print("Final time:", time.time()-s1)
# mean = sum(times) / len(times) 
# variance = sum([((x - mean) ** 2) for x in times]) / (len(times)-1) 
# std = variance ** 0.5
# t=[x for x in range(len(times))]
# plt.plot(t,times)
# plt.show()
# print(mean)
# print(std)
# print()

# get_image.pop(0)
# mean = sum(get_image) / len(get_image) 
# variance = sum([((x - mean) ** 2) for x in get_image]) / (len(get_image)-1) 
# std = variance ** 0.5
# t=[x for x in range(len(get_image))]
# plt.plot(t,get_image)
# plt.show()
# print(mean)
# print(std)
# print()


# rest.pop(0)
# mean = sum(rest) / len(rest) 
# variance = sum([((x - mean) ** 2) for x in rest]) / (len(rest)-1) 
# std = variance ** 0.5
# t=[x for x in range(len(rest))]
# plt.plot(t,rest)
# plt.show()
# print(mean)
# print(std)
# print()


# Disable modules and close camera
mat.free(memory_type=sl.MEM.CPU)
camera.disable_object_detection()
camera.disable_positional_tracking()
camera.disable_body_tracking()
camera.close()