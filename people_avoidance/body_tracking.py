



import pyzed.sl as sl
import ogl_viewer.viewer as gl
import numpy as np
from math import isnan




def init_body_tracking(camera, detection_confidence = 40, show_image = True, detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST):
    body_params = sl.BodyTrackingParameters()
    # Different model can be chosen, optimizing the runtime or the accuracy
    body_params.detection_model = detection_model
    body_params.enable_tracking = True
    body_params.image_sync = True
    body_params.enable_segmentation = False
    # Optimize the person joints position, requires more computations
    body_params.enable_body_fitting = False

    if body_params.enable_tracking:
        positional_tracking_param = sl.PositionalTrackingParameters()
        # positional_tracking_param.set_as_static = True
        positional_tracking_param.set_floor_as_origin = True
        camera.enable_positional_tracking(positional_tracking_param)

    print("Body tracking: Loading Module...")

    err = camera.enable_body_tracking(body_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Enable Body Tracking : "+repr(err)+". Exit program.")
        camera.close()
        exit()
    bodies = sl.Bodies()
    body_runtime_param = sl.BodyTrackingRuntimeParameters()
    # For outdoor scene or long range, the confidence should be lowered to avoid missing detections (~20-30)
    # For indoor scene or closer range, a higher confidence limits the risk of false positives and increase the precision (~50+)
    body_runtime_param.detection_confidence_threshold = detection_confidence
    

    # Viewer:
    if show_image:
        camera_info = camera.get_camera_information()
        viewer = gl.GLViewer()
        viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam, body_params.enable_tracking)
        print("init viewer")
    else:
        viewer = None

    return viewer, bodies, body_runtime_param


def body_tracking(camera, mat, bodies, viewer, body_runtime_param, show_image):
    emergency_brake = False
    speed_reduction = 0     # in % 
    camera.retrieve_bodies(bodies, body_runtime_param) # Retrieve the detected objects
    if show_image:
        print(viewer.is_available())
        viewer.update_view(mat, bodies)
    if True:
        for person in bodies.body_list:
            
            # Assuming r and v are numpy arrays representing position and velocity vectors
            pos = person.position
            vel = person.velocity
            if not isnan(vel[0]):
                r = np.array([pos[0], pos[1]])  # Position vector 2d plane
                v = np.array([vel[0], vel[1]])  # Velocity vector 2d plane

                # Calculate speed towards origin
                speed_towards_origin = -np.dot(r / np.linalg.norm(r), v)

                # Calculate distance to origin
                distance_to_origin = np.linalg.norm(r)

                closest_point = abs(np.cross(r, v)) / np.linalg.norm(v)

                if speed_towards_origin*2 > distance_to_origin and closest_point < 0.5:
                    print("Emergency brake")
                    emergency_brake = True
                    # Get a speed reduction between 0.25 and 1 based on speed and distance due to if.
                    speed_reduction = min(1, speed_towards_origin/(distance_to_origin*2))
        
    return emergency_brake, speed_reduction

# object_2Dbbox = object.bounding_box_2d; # Get the 2D bounding box of the object
# object_3Dbbox = object.bounding_box; # Get the 3D Bounding Box of the object


