#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import subprocess

class FreewayTurnInPlace:
    def __init__(self):
        self.front_obstacle_sub = rospy.Subscriber("freeway/front_obstace", Bool, self.front_obstacle_cb, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('freeway_spk')
        self.front_obstacle_flag = False
        self.audio_playing = False
        self.timer = rospy.Timer(rospy.Duration(10), self.play_standard_audio)
        self.loop_rate = rospy.Rate(10)  # 10 Hz

    def play_audio(self, file_path):
        if not self.audio_playing:
	        self.audio_playing = True
            subprocess.call(['mpg123', '-q' ,file_path])
            self.audio_playing = False
    
    def play_standard_audio(self, event=None):
        audio_file = self.pkg_path+'/scripts/source/gn/gangnam.mp3'
        self.play_audio(audio_file)
        rospy.loginfo("Played %s" %audio_file)
    
    def front_obstacle_cb(self, data):
        self.front_obstacle_flag = data.data
        rospy.loginfo("front_obstacle_flag : %s" %self.front_obstacle_flag)

    def main(self):
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            if not self.front_obstacle_flag:
                cmd_vel.angular.z = 0.3  # rad/s
                
            else:
                cmd_vel.angular.z = 0.0
                cmd_vel.linear.x = 0.0

            self.pub_cmd_vel.publish(cmd_vel)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('freeway_turn_in_place_node')
    FTIP = FreewayTurnInPlace()
    FTIP.main()
