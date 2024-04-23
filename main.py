# ZED ID 4mm (SMALL LENS): 40329509
# ZED ID 2.2mm (BIG LENS): 42146143

from lane_keeping.controller import steering_angle_controller
from lane_keeping.line_detection import line_detection
from lane_keeping.PIDController import PIDController
from tracking.body_tracking import init_body_tracking
from emergency_braking import emergency_braking
from multi_camera import signal_handler, grab_run, multi_camera_init, single_camera, start_camera_threads, stop_camera_threads, zed_list, mat_list, depth_list, thread_list, runtime_list, timestamp_list, cam_status_list, stop_signal
#from main_GPS import GPS 
from speed_reference.GBController import GBController
from speed_reference.backcontroller import distance_controller, decide_control_method, check_within_bounds, cruise_controller

from camera import camera_init, get_camera_image
import matplotlib.pyplot as plt
import pyzed.sl as sl
import time
import cv2

# Import GPS functions
import gpsd_reader as GPSDReader
import sys
import gps_processor as GPSProsessor 

# Import stop gps
import stop_APM 

#sl.Camera.reboot()?
# sl.rebootCamera()
'''
distance_input = 800
sex_input = 'female'
finish_time_input = 240
optimal_speed_schedule = calculate_optimal_speed_pattern(distance = distance_input, sex = sex_input, finish_time):
'''
d_min = 2
d_max = 6

# Tunable parameters:
set_width = 640      
show_image = True
fps = 15
resolution=sl.RESOLUTION.SVGA
detection_confidence = 40
detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST

############################### Initialization of two cameras from serial number. Starting camera threads ###########################################
serial_number_front = 42146143
serial_number_back = 40329509

mat_list, zed_list, runtime_list, cam_status_list, name_list = multi_camera_init(fps, resolution)

front_cam_mat, front_cam, front_cam_runtime, front_cam_status = single_camera(serial_number_front, mat_list, zed_list, runtime_list, cam_status_list, name_list)
back_cam_mat, back_cam, back_cam_runtime, back_cam_status = single_camera(serial_number_back, mat_list, zed_list, runtime_list, cam_status_list, name_list)

start_camera_threads(zed_list, thread_list)

############################### Initialization of object detection for two cameras. Body tracking ###################################################
viewer_front, bodies_front, body_runtime_param_front = init_body_tracking(front_cam, detection_confidence, show_image, detection_model)
print("Front camera Status: ", front_cam_status)

viewer_back, bodies_back, body_runtime_param_back = init_body_tracking(back_cam, detection_confidence, show_image, detection_model)
print("Back camera Status: ", back_cam_status)


############################### Initialization of controllers ###############################################################
dc_pid = GBController(kp=1, ki=1, kd=1, setpoint=0.5, beta=1, gamma=1)

############################### Initialization of GPS variables #############################################################
fist_read_gnss = True
total_pos = 0
lat_prev = 0
lon_prev = 0
speed_APM = 0
gps_reader = GPSDReader()

if gps_reader.initialize() != 0:
    print("Failed to initialize GPSDReader. Exiting...")
    sys.exit(1)



################################# Input user #####################################################################################
final_position_input = 1000 # Total distance of the run
final_time_input = 60 #sec
vd_input = final_position_input/final_time_input 
user_input_list = [[0, vd_input], [0, final_position_input]]

# Elite - Lag elite_input_list fra funksjoner i pacing_schedule. Husk: Legg inn to.list() fra df nå. Skal være input fra app.

################################# while-loop ######################################################################################

for i in range(0, 400):
    try:
        print("---------------")
        image_front = get_camera_image(front_cam_mat, front_cam, front_cam_runtime)
        assert image_front is not None, "file could not be read, check with os.path.exists()"

        image_back = get_camera_image(back_cam_mat, back_cam, back_cam_runtime)
        assert image_back is not None, "file could not be read, check with os.path.exists()"

        front_cam.retrieve_bodies(bodies_front, body_runtime_param_front) # Retrieve the detected objects from front camera
        back_cam.retrieve_bodies(bodies_back, body_runtime_param_back) # Retrieve the detected objects from back camera

        runner_object = bodies_back.body_list[0] #Runner is the first element of the detected bodies. Assumed initialized with only runner visible for back camera
        #v_ref = distance_controller(v_APM= 0, runner_object=runner_object, distance_Controller=dc_pid, setpoint_dc = 3) #TEST

        ###################### Controller testing ############################################
        within_bounds, stable, d_ref = check_within_bounds(runner_object=runner_object, d_min = d_min, d_max = d_max)
        control_strategy, prev_error = decide_control_method(runner_object, d_max, d_min, within_bounds, stable, d_ref)

        if control_strategy == 1:
            v_ref = cruise_controller()
        elif control_strategy == 2:
            v_ref = distance_controller()

        print("Distance to runner: {}".format(runner_object.position[2]))
        print("V_ref: {}".format(v_ref))


        ################################################################# CODE camilla har lagt til (ikke endret noe skrevet før)
        fist_read_gnss, total_pos, lat_prev, lon_prev, speed_APM = GPSProsessor.gpsProcess(gps_reader, fist_read_gnss, total_pos, lat_prev, lon_prev)
        print("New Distance moved:", total_pos)
        print("New Speed:", speed_APM)
        time.sleep(0.1)

        # Check if the run is completed
        if(total_pos >= final_position_input):
           control_strategy, v_ref = stop_APM.run_is_finished(speed_APM, gps_reader, front_cam_mat, front_cam, back_cam_mat, back_cam, thread_list, stop_signal)

    except KeyboardInterrupt: #Clc + c
        print("KeyboardInterrupt detected. Stopping Threads...")
        # Disable modules and close front_cam
        front_cam_mat.free(memory_type=sl.MEM.CPU)
        front_cam.disable_object_detection()
        front_cam.disable_positional_tracking()
        front_cam.disable_body_tracking()

        back_cam_mat.free(memory_type=sl.MEM.CPU)
        back_cam.disable_object_detection()
        back_cam.disable_positional_tracking()
        back_cam.disable_body_tracking()
        stop_camera_threads(thread_list, stop_signal)
        gps_reader.stop_thread()

# viewer.update_view(front_cam_mat, bodies)
'''while viewer_front.is_available() and viewer_back.is_available():
    print("---------------")

    image_front = get_camera_image(front_cam_mat, front_cam, front_cam_runtime)
    assert image_front is not None, "file could not be read, check with os.path.exists()"


    front_cam.retrieve_bodies(bodies_front, body_runtime_param_front) # Retrieve the detected objects
    emergency_brake, speed = emergency_braking(bodies_front)


    viewer_front.update_view(front_cam_mat, bodies_front)
'''

    
    
# Disable modules and close front_cam
front_cam_mat.free(memory_type=sl.MEM.CPU)
front_cam.disable_object_detection()
front_cam.disable_positional_tracking()
front_cam.disable_body_tracking()

back_cam_mat.free(memory_type=sl.MEM.CPU)
back_cam.disable_object_detection()
back_cam.disable_positional_tracking()
back_cam.disable_body_tracking()
#front_cam.close()

stop_camera_threads(thread_list, stop_signal)
gps_reader.stop_thread()