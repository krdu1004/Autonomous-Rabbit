



import pyzed.sl as sl
import ogl_viewer.viewer as gl




def init_object_detection(camera, detection_confidence = 40, show_image = True, detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST):
    # Set initialization parameters
    detection_parameters = sl.ObjectDetectionParameters()
    detection_parameters.enable_tracking = True # Objects will keep the same ID between frames
    detection_parameters.enable_segmentation = True # Outputs 2D masks over detected objects

    # Set runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]
    obj_runtime_param.object_class_detection_confidence_threshold[sl.OBJECT_CLASS.PERSON] = detection_confidence

    # choose a detection model
    detection_parameters.detection_model = detection_model  

    if detection_parameters.enable_tracking :
        # Set positional tracking parameters
        positional_tracking_parameters = sl.PositionalTrackingParameters()
        # Enable positional tracking
        camera.enable_positional_tracking(positional_tracking_parameters)

    # Enable object detection with initialization parameters
    zed_error = camera.enable_object_detection(detection_parameters)
    if zed_error != sl.ERROR_CODE.SUCCESS:
        print("enable_object_detection", zed_error, "\nExit program.")
        camera.close()
        exit(-1)

    # Viewer:
    if show_image:
        camera_info = camera.get_camera_information()
        viewer = gl.GLViewer()
        viewer.init(camera_info.camera_configuration.calibration_parameters.left_cam, detection_parameters.enable_tracking)
        print("init viewer")
    else:
        viewer = None

    objects = sl.Objects() # Structure containing all the detected objects
    return viewer, objects, obj_runtime_param


def object_detection(camera, mat, objects, viewer, obj_runtime_param, show_image):
    camera.retrieve_objects(objects, obj_runtime_param) # Retrieve the detected objects
    if show_image:
        print(viewer.is_available())
        viewer.update_view(mat, objects)
    if False:
        for object in objects.object_list:
            print(object.id,object.position,object.velocity)
    
    emergency_brake = False
    speed = 0   #override/reduce speed based on 
    return emergency_brake, speed

# object_2Dbbox = object.bounding_box_2d; # Get the 2D bounding box of the object
# object_3Dbbox = object.bounding_box; # Get the 3D Bounding Box of the object


