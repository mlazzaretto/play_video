#!/usr/bin/env python

import sys, os, glob
import rospy, rospkg
import yaml
import message_filters
from rospy_message_converter import message_converter
from std_msgs.msg import String
from hiros_skeleton_msgs.msg import SkeletonGroup
from play_video.msg import Action
from geometry_msgs.msg import Point

### Save 2D skeleton information detected by openpose on a file ###

def prepend_line(file_name, line):
  """ Insert given string as a new line at the beginning of a file """
  # define name of temporary dummy file
  dummy_file = file_name + '.bak'
  # open original file in read mode and dummy file in write mode
  with open(file_name, 'r') as read_obj, open(dummy_file, 'w') as write_obj:
      # Write given line to the dummy file
      write_obj.write(line + '\n')
      # Read lines from original file one by one and append them to the dummy file
      for line in read_obj:
          write_obj.write(line)
  # remove original file
  os.remove(file_name)
  # Rename dummy file as the original file
  os.rename(dummy_file, file_name)


class acquire_skeleton_data:

  def __init__(self):

    self.package_dir = rospkg.RosPack().get_path('play_video')
    self.output_dir = rospy.get_param('output_dir', self.package_dir + '/data/openpose_skeletons/')

    self.action_code = ''
    self.skeleton_file = ''

    rospy.init_node('op_writer', anonymous=True)
    self.action_sub = message_filters.Subscriber('/action_code', Action)
    self.skeleton_sub = message_filters.Subscriber('/hiros/opw/node_01/skeleton_group', SkeletonGroup)
    self.msgs = 0

    self.ts = message_filters.ApproximateTimeSynchronizer([self.action_sub, self.skeleton_sub], 10, 0.5, allow_headerless=False)
    self.ts.registerCallback(self.callback) 

    try:
      os.makedirs(self.output_dir)
    except OSError:
      print('Output directory already existing')
    print('Output dir: ',self.output_dir)


  def callback(self, action_msg, skeleton_msg):

    print("Callback **************************")

    if self.action_code == '':
      self.action_code = action_msg.action
      self.skeleton_file = rospy.get_param('skeleton_file', self.output_dir + self.action_code + '.skeleton')      

    if action_msg.action != self.action_code:
      prepend_line(self.skeleton_file, str(self.msgs))
      self.action_code = action_msg.action
      self.skeleton_file = rospy.get_param('skeleton_file', self.output_dir + self.action_code + '.skeleton') 
      self.msgs = 0

    skeletons = skeleton_msg.skeletons
    self.msgs = self.msgs +1

    with open(self.skeleton_file, 'a') as file:
      file.write('header:\n')
      line = 'seq:{}\nstamp.secs:{}\nstamp.nsecs:{}\n'.format(skeleton_msg.header.seq,skeleton_msg.header.stamp.secs,skeleton_msg.header.stamp.nsecs)
      file.write(line)

      #file.write('{}\n'.format(len(skeletons)))f
      file.write('{}\n'.format(str(len(skeletons))))
      for skeleton in skeletons:
        num_joints = skeleton.max_markers
        num_links = skeleton.max_links
        file.write("skeletons.id:{}\nskeletons.max_markers:{}\nskeleton.max_links:{}\n".format(skeleton.id, num_joints, num_links))
        file.write('skeletons.confidence:{}\n'.format(skeleton.confidence))
        id_count = 0
        file.write('skeletons.markers:{}\n'.format(len(skeleton.markers)))
        for joint in skeleton.markers:
          p = joint.center.pose.position
          file.write('id:{}\nconfidence:{}\ncenter.pose.position.x:{}\ncenter.pose.position.y:{}\n'.format(joint.id,joint.confidence, p.x, p.y))
        file.write('skeletons.links:{}\n'.format(len(skeleton.links)))
        for link in skeleton.links:
          p = link.center.pose.position
          file.write('id:{}\nparent_marker:{}\nchild_marker:{}\nconfidence:{}\ncenter.pose.position.x:{}\ncenter.pose.position.y:{}\n'.format(link.id,link.parent_marker,link.child_marker,link.confidence, p.x, p.y))
         

  # spin: keeps python from exiting until this node is stopped
  def spin(self):
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")

  def get_skeleton_msgs_num(self):
    return self.msgs

  def get_skeleton_file(self):
    return self.skeleton_file

  def get_out_path(self):
    return self.output_dir


def main(args):

  listener_node = acquire_skeleton_data()
  listener_node.spin()
  skeleton_file = listener_node.get_skeleton_file()
  with open(skeleton_file, 'a') as file:
    prepend_line(skeleton_file, str(listener_node.get_skeleton_msgs_num()))


if __name__ == '__main__':
   main(sys.argv)
