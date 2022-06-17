import time
import stretch_body.robot
import pyrealsense2 as rs
import numpy as np
from matplotlib import pyplot as plt
from scipy import ndimage
from stretch_body.hello_utils import ThreadServiceExit

robot=stretch_body.robot.Robot()
robot.startup()
#realsense code is from my notebook for capturing depth images of mannequin
#body (mostly originally copied from pyrealsense tutorials
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color)
pipe.start(config)
try:
    robot.head.move_to('head_tilt',-0.35)
    robot.push_command()

    #Depth Camera code
    time.sleep(5)
    for x in range(5):
        pipe.wait_for_frames()
    frameset=pipe.wait_for_frames()
    depth_frame = frameset.get_depth_frame()
    print("Frames Captured")
    colorizer = rs.colorizer()
    pipe.stop()
    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)
    aligned_depth_frame = frameset.get_depth_frame()
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
    rot = ndimage.rotate(colorized_depth, 270)
    plt.imshow(rot)
    plt.show()
except: (KeyboardInterrupt, SystemExit, ThreadServiceExit) 
pass

robot.stop()
