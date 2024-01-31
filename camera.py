#!/usr/bin/env python3

import pyzed.sl as sl
import time
import numpy as np
import cv2

def camera_init(fps=30, resolution=sl.RESOLUTION.AUTO):
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
    init.camera_resolution = resolution
    #init.camera_resolution = sl.RESOLUTION.RESOLUTION_HD1080
    init.camera_fps = fps  # The framerate is lowered to avoid any USB3 bandwidth issues

    camera = sl.Camera()
    cam_status = camera.open(init)

    mat = sl.Mat()
    return mat, camera, runtime, cam_status

def get_camera_image(mat, camera, runtime):    # Obtain camera image and get timestamps
    t1 = time.process_time()
    err = camera.grab(runtime)
    # If grabbing image successfull, save to buffer
    if err == sl.ERROR_CODE.SUCCESS:
        # camera.retrieve_image(mat, sl.VIEW.SIDE_BY_SIDE)
        camera.retrieve_image(mat, sl.VIEW.LEFT)
        img = mat.get_data()

        img_name = 'images/filename_{}.jpg'.format(time.time())
        name = "kake"
        # Saving as numpy array
        cv2.imshow(name, img)
        t1 = time.process_time()
        countdown = 5
        # while img != sl.ERROR_CODE.SUCCESS or countdown > 0:
        while countdown > 0:
            print(img)
            img_write = mat.write(img_name)
            countdown -= 1
            if img_write == sl.ERROR_CODE.SUCCESS:
                break
        return img
        print("Saving file took {} ms".format((time.process_time()-t1)*1e3))
    return None