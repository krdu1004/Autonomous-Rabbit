from lane_keeping.controller import steering_angle_controller
from lane_keeping.line_detection import line_detection
from lane_keeping.PIDController import PIDController
from people_avoidance.emergency_braking import emergency_braking


test_image = GET_IMAGE!
height, width, depth = test_image.shape
set_width = width       # Can be changed with 

# PIDs:
angle_setpoint = -90  # The desired angle you want to maintain
position_setpoint = set_width/2  # The desired position you want to reach
angle_pid = PIDController(kp=0.5, ki=0.1, kd=0.2, setpoint=angle_setpoint)
position_pid = PIDController(kp=0.1, ki=0.01, kd=0.02, setpoint=position_setpoint)


while True:
    image = GET_IMAGE!
    assert image is not None, "file could not be read, check with os.path.exists()"

    # first avoid crashing
    emergency_brake, speed = emergency_brake()

    # Lane keeping:
    a1_optimal, b1_optimal, a2_optimal, b2_optimal = line_detection(image, set_width, cut_top=1/4)
    steering_angle = steering_angle_controller(a1_optimal, b1_optimal, a2_optimal, b2_optimal)


    # Get speed:


    # Send speed and angle to robot:
