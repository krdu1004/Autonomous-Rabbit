


# class TrackPointState:
#     OK = 0
#     PREDICTED = 1
#     OFF = 2

# class OBJECT_CLASS:
#     PERSON = 0
#     VEHICLE = 1
#     LAST = 2

# class TrackPoint:
#     def __init__(self, position, tracking_state, timestamp):
#         self.x = position[0]
#         self.y = position[1]
#         self.z = position[2]
#         self.tracking_state = tracking_state
#         self.timestamp = timestamp
        
#     def get_xyz(self):
#         return 1000*np.array([self.x, self.y, self.z])


# class Tracklet:
#     def __init__(self, obj, label, timestamp):
#         self.id = obj.id
#         self.label = label
#         self.object_type = obj.label
#         self.is_alive = True
#         self.last_detected_timestamp = timestamp
#         self.recovery_cpt = 0
#         self.recovery_length = 5
#         self.positions = [TrackPoint(obj.position, TrackPointState.OK, timestamp)]
#         self.positions_to_draw = [TrackPoint(obj.position, TrackPointState.OK, timestamp)]
#         self.tracking_state = obj.tracking_state
#     def generate_view(self, objects, image_left_ocv, img_scale, current_camera_pose, tracking_view, tracking_enabled):
#         for obj in objects.object_list:
#             pos = sl.Translation(obj.position)
#             temp = (pos *current_camera_pose.get_orientation()).get()
#             current_translation = current_camera_pose.get_translation().get()
#             obj.position = 1000*np.array([temp[0]+current_translation[0], temp[1]+current_translation[1], temp[2]+current_translation[2]])

#         if not self.has_background_ready:
#             self.generateBackground()
#         tracking_view[:] = self.background
#         self.drawScale(tracking_view)

#         if tracking_enabled:
#             current_timestamp = objects.timestamp.get_nanoseconds()
#             self.addToTracklets(objects)
#             self.detectUnchangedTrack(current_timestamp)
#             self.pruneOldPoints(current_timestamp)
#             self.render_2D(image_left_ocv, img_scale, objects.object_list, True, tracking_enabled)
#             self.drawTracklets(tracking_view, current_camera_pose)
#         else:
#             self.drawPosition(objects, tracking_view, current_camera_pose)



import pyzed.sl as sl
import ogl_viewer.viewer as gl
import numpy as np

# REMOVE:
debug = True
show_image = True
timing = True
init = sl.InitParameters()
camera = sl.Camera()
if not camera.is_opened():
    print("Opening ZED Camera...")
status = camera.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit(0)




mat = sl.Mat()

runtime = sl.RuntimeParameters()

init = sl.InitParameters()
fps=0
resolution=sl.RESOLUTION.SVGA
init.camera_resolution = resolution
#init.camera_resolution = sl.RESOLUTION.RESOLUTION_HD1080
init.camera_fps = fps  # The framerate is lowered to avoid any USB3 bandwidth issues
init.coordinate_units = sl.UNIT.METER
init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
camera = sl.Camera()
cam_status = camera.open(init)
print(cam_status)
detection_confidence = 40


# def emergency_braking(image, camera, detection_confidence = 40):
# Set initialization parameters
detection_parameters = sl.ObjectDetectionParameters()
detection_parameters.enable_tracking = True # Objects will keep the same ID between frames
detection_parameters.enable_segmentation = True # Outputs 2D masks over detected objects

# Set runtime parameters
obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.VEHICLE, sl.OBJECT_CLASS.PERSON]
obj_runtime_param.object_class_detection_confidence_threshold[sl.OBJECT_CLASS.PERSON] = detection_confidence
obj_runtime_param.object_class_detection_confidence_threshold[sl.OBJECT_CLASS.VEHICLE] = detection_confidence

# choose a detection model
detection_parameters.detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST  

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


objects = sl.Objects() # Structure containing all the detected objects
prev = 0
import time
# while viewer.is_available():

start=time.time()

for i in range(100):
    s=time.time()
    if camera.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        print(time.time()-s)
        camera.retrieve_image(mat, sl.VIEW.LEFT)
        print(time.time()-s)
        camera.retrieve_objects(objects, obj_runtime_param) # Retrieve the detected objects
        print(time.time()-s)
        
        if timing:
            timestamp = camera.get_timestamp(sl.TIME_REFERENCE.CURRENT)  # Get the timestamp at the time the image was captured
            print(timestamp.get_milliseconds()-prev)
            prev=timestamp.get_milliseconds()
        if show_image:
            viewer.update_view(mat, objects)
        if debug:
            for object in objects.object_list:
                print(object.id,object.position,object.velocity)
            
        print(time.time()-s)
        print()
print(time.time()-start)
print("FPS: ", 100/(time.time()-start))

# object_2Dbbox = object.bounding_box_2d; # Get the 2D bounding box of the object
# object_3Dbbox = object.bounding_box; # Get the 3D Bounding Box of the object


