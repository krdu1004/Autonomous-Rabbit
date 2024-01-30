from lane_keeping.controller import steering_angle_controller
from lane_keeping.line_detection import line_detection
from lane_keeping.PIDController import PIDController



angle_pid = PIDController(kp=0.5, ki=0.1, kd=0.2, setpoint=angle_setpoint)
position_pid = PIDController(kp=0.1, ki=0.01, kd=0.02, setpoint=position_setpoint)


while True:
    image = GET_IMAGE!


    # first avoid crashing


    # Lane keeping:
    a1_optimal, b1_optimal, a2_optimal, b2_optimal = line_detection(image)
    steering_angle = steering_angle_controller(a1_optimal, b1_optimal, a2_optimal, b2_optimal)


    # Get speed:


    # Send speed and angle to robot:
