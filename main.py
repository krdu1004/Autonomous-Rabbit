from lane_keeping.controller import steering_angle_controller
from lane_keeping.line_detection import line_detection
from lane_keeping.PIDController import PIDController
from people_avoidance.emergency_braking import emergency_braking
from camera import camera_init, get_camera_image
import cv2


# Camera init:
mat, camera, runtime, cam_status = camera_init()
test_image = get_camera_image(mat, camera, runtime)
print("Camera Status: ", cam_status)
assert test_image is not None, "file could not be read, check with os.path.exists()"
height, width, depth = test_image.shape
set_width = width       # Can be changed with 

# PIDs:
angle_setpoint = -90  # The desired angle you want to maintain
position_setpoint = set_width/2  # The desired position you want to reach
angle_pid = PIDController(kp=0.5, ki=0.1, kd=0.2, setpoint=angle_setpoint)
position_pid = PIDController(kp=0.1, ki=0.01, kd=0.02, setpoint=position_setpoint)




# Main loop
for i in range(10): # FOR TESTING, 10 iterations
# while True:
    image = get_camera_image(mat, camera, runtime)
    assert image is not None, "file could not be read, check with os.path.exists()"

    # first avoid crashing
    emergency_brake, speed = emergency_braking(image)
    if not emergency_brake:

        # Lane keeping:
        a1_optimal, b1_optimal, a2_optimal, b2_optimal = line_detection(image, set_width, cut_top=1/4)
        steering_angle = steering_angle_controller(image, angle_pid, position_pid, a1_optimal, b1_optimal, a2_optimal, b2_optimal)

        # Get speed:
        print("speed")

    # Send speed and angle to robot:
    print("send")

    
    cv2.waitKey(0)
