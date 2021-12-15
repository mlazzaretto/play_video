#!/usr/bin/env python

import sys, os, glob
import rospy, rospkg
import message_filters
from std_msgs.msg import String
from hiros_skeleton_msgs.msg import SkeletonGroup
from play_video.msg import Action
from geometry_msgs.msg import Point


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

### Subscribe to SkeletonGroup messages and save .skeleton files with joints information ###

class acquire_skeleton_data:

  def __init__(self):

    self.package_dir = rospkg.RosPack().get_path('play_video')
    self.output_dir = rospy.get_param('output_dir', self.package_dir + '/data/openpifpaf_skeletons/')

    self.action_code = ''
    self.skeleton_file = ''

    rospy.init_node('writer', anonymous=True)
    self.action_sub = message_filters.Subscriber('/action_code', Action)
    self.skeleton_sub = message_filters.Subscriber('/hiros/opw_3d/node_01/skeleton_group', SkeletonGroup)
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

      file.write('{}\n'.format(len(skeletons)))

      for skeleton in skeletons:
        num_joints = 65
        file.write("{}\n{}\n".format(skeleton.id, num_joints))
        id_count = 1
        for joint in skeleton.markers:

          while(joint.id != id_count):
            if id_count>23 and id_count<92:
              id_count = id_count+1
            else:
              file.write('{} {} {} {}\n'.format(id_count, 0.0, 0.0, 0.0))
              id_count = id_count + 1

          if id_count>23 and id_count<92:
            id_count = id_count+1
          else:
            p = joint.center.pose.position
            file.write('{} {} {} {}\n'.format(joint.id, p.x, p.y, p.z))
            id_count = id_count + 1

      while(id_count<=133):
        file.write('{} {} {} {}\n'.format(id_count, 0.0, 0.0, 0.0))
        id_count = id_count + 1



  '''
  def callback_skeleton(self, data):

    skeletons = data.skeletons

    for skeleton in skeletons:
      for joint in skeleton.markers:
        p = Point()
        p = joint.center.pose.position
        if p.x+p.y+p.z==0:
          return

      self.msgs = self.msgs +1

      with open(self.skeleton_file, 'a') as file:

        file.write('{}\n'.format(len(skeletons)))

        for skeleton in skeletons:
          num_joints = 67
          file.write("{}\n{}\n".format(skeleton.id, num_joints))
          id_count = 0
          for joint in skeleton.markers:
            while(joint.id != id_count):
              file.write('{} {} {} {}'.format(id_count, 0.0, 0.0, 0.0))
              if id_count != 25 and id_count != 420:
                id_count = id_count + 1
              elif id_count == 25:
                id_count = 400
              else:
                id_count = 500

            p = joint.center.pose.position
            file.write('{} {} {} {}'.format(joint.id, p.x, p.y, p.z))
            id_count = joint_id + 1

  def callback_act(self, data):
    if self.action_code == 'S':
      self.action_code = data.data
      self.skeleton_file = rospy.get_param('skeleton_file', self.output_dir + self.action_code + '.skeleton')      

    if data.data != self.action_code:
      prepend_line(self.skeleton_file, str(self.msgs))
      self.action_code = data.data
      self.skeleton_file = rospy.get_param('skeleton_file', self.output_dir + self.action_code + '.skeleton') 
      self.msgs = 0
  '''

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


if __name__ == '__main__':
   main(sys.argv)