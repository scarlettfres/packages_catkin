#!/usr/bin/python
import roslib
import rospy   
roslib.load_manifest('sca_axis_cam')
import yaml
import argparse
import sensor_msgs.msg
from os import chdir
chdir("/home/sfress/catkin_ws/src/sca_axis_cam/camera_info/") # to put lauch_tf in the right folder

def parse_yaml(filename):
  stream = file(filename, 'r')
  calib_data = yaml.load(stream)
  cam_info = sensor_msgs.msg.CameraInfo()
  cam_info.header.frame_id = calib_data['camera_name']
  cam_info.width = calib_data['image_width']
  cam_info.height = calib_data['image_height']
  cam_info.K = calib_data['camera_matrix']['data']
  cam_info.D = calib_data['distortion_coefficients']['data']
  cam_info.R = calib_data['rectification_matrix']['data']
  cam_info.P = calib_data['projection_matrix']['data']
  cam_info.distortion_model = calib_data['distortion_model']
  return cam_info

if __name__ == "__main__":
  rospy.init_node('publish_info', anonymous=True)
  rate = rospy.Rate(15)

  # =====recup de parametreeeeees 
  arg_defaults = {
      'filename': 'jojo.yaml'
      }
  args = {}

  param_search = rospy.search_param('filename')
  # make sure argument was found (https://github.com/ros/ros_comm/issues/253)
  if param_search == None:
      filename = 'camera_fakeshop.yaml'
  else:
      filename = rospy.get_param(param_search, arg_defaults['filename'])

 # =====recup de parametreeeeees 
  
  #parser = argparse.ArgumentParser(description='Parses camera info yaml files and returns them as sensor_msgs.msg.CameraInfo.')
  #parser.add_argument('filename', help='input yaml file')
  #args = parser.parse_args()
  try:
    caminfo_pub = rospy.Publisher("/camera_info", sensor_msgs.msg.CameraInfo,queue_size=1)
    info = parse_yaml(filename)
    while not rospy.is_shutdown():  
      caminfo_pub.publish(info)
      rate.sleep()




    #print 'Read the following info from', args.filename, '\n', info
  except Exception, e:
    import traceback
    traceback.print_exc()