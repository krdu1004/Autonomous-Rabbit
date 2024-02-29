



import pyzed.sl as sl
import ogl_viewer.viewer as gl




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
    camera.retrieve_bodies(bodies, body_runtime_param) # Retrieve the detected objects
    if show_image:
        print(viewer.is_available())
        viewer.update_view(mat, bodies)
    if False:
        for object in objects.object_list:
            print(object.id,object.position,object.velocity)
    
    emergency_brake = False
    speed = 0   #override/reduce speed based on 
    return emergency_brake, speed

# object_2Dbbox = object.bounding_box_2d; # Get the 2D bounding box of the object
# object_3Dbbox = object.bounding_box; # Get the 3D Bounding Box of the object


