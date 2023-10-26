#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import time
import threading
import os
#import sys, select, termios, tty
#from playsound import playsound
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Int32MultiArray
#from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult
from mbf_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult
from freeway_msgs.msg import DistanceTimeCalculator
from freeway_joyfw.msg import stm_am_msg, stm_fw_msg
from actionlib_msgs.msg import GoalID
import subprocess

class Freeway_spk:

    def __init__(self):
        self.rate = rospy.Rate(5) # ROS Rate at 5Hz
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_sub)
        rospy.Subscriber("freeway/goal", PoseStamped, self.freeway_goal_sub)
        rospy.Subscriber("freeway/manual_spk_call", Empty, self.manual_spk_call_sub)
        #   rospy.Subscriber("move_base/goal", MoveBaseActionGoal, mb_goal_sub)
        #   rospy.Subscriber("move_base_flex/move_base/cancel", GoalID, cancel_sub)
        rospy.Subscriber("freeway/goal_cancel", Empty, self.cancel_sub)
        rospy.Subscriber("freeaway/diagnostics", stm_fw_msg, self.diagnostics_sub)
        rospy.Subscriber("freeway/resume", Empty, self.resume_sub)
        #rospy.subscriber("freeway/goal_pause", Empty, self.goal_pause_sub)
        rospy.Subscriber("freeway/am_status", stm_am_msg, self.am_sub)
        rospy.Subscriber("move_base_flex/move_base/result", MoveBaseActionResult, self.result_sub)
        rospy.Subscriber("freeway/distancetimecalculator", DistanceTimeCalculator, self.distancetimecalculator_sub)
        rospy.Subscriber("freeway/ai_status", Int32MultiArray, self.ai_status_sub)

        self.s_command = -1
        self.r_command = -1
        self.pre_s_command = -1
        self.pre_r_command = -1

        self.dt_f=True
        self.goal_sub_flag = False
        self.dt_count=0
        self.ct=0.0
        self.auto_driving_timer_time = 7.0
        self.previous_arrival_time = 0.0
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('freeway_spk')
        self.audio_playing = False  # flag to indicate whether an audio file is currently playing
        self.latest_timestamp = 0.0
        self.play_lock = threading.Lock()
        #   while not rospy.is_shutdown():
        #     rate.sleep()
        self.request_audio(self.pkg_path+'/scripts/source/first_start.mp3')
        #playsound(pkg_path+'/scripts/source/first_start.mp3')

#    def play_audio(self, file_path):
#        # subprocess.call(['mpg123', '-q', file_path])
#        if self.audio_playing is True:
#            return
#
#        elif self.audio_playing is False:
#    	    self.audio_playing = True
#            subprocess.call(['mpg123', '-q' ,file_path])
#            self.audio_playing = False

#    def play_audio(self, file_path, timestamp):
#        if timestamp < self.latest_timestamp:
#            return

#        if not self.audio_playing:
#            self.audio_playing = True
#            subprocess.call(['mpg123', '-q', file_path])
#            self.audio_playing = False

#    def request_audio(self, file_path):
#        current_timestamp = time.time()
#        self.latest_timestamp = current_timestamp

#        rospy.sleep(0.5)  # Wait for 0.5 seconds

#        self.play_audio(file_path, current_timestamp)

    def play_audio(self, file_path):
        with self.play_lock:
            if self.audio_playing:
                return
            self.audio_playing = True

        subprocess.call(['mpg123', '-q', file_path])

        with self.play_lock:
            self.audio_playing = False

    def request_audio(self, file_path):
        current_timestamp = time.time()

        with self.play_lock:
            if current_timestamp - self.latest_timestamp < 0.5:
                # A request was made within the last 0.5 seconds, do not play.
                return
            self.latest_timestamp = current_timestamp

        self.play_audio(file_path)

    def manual_spk_call_sub(self, data):
        self.request_audio(self.pkg_path+'/scripts/source/first_start.mp3')

    def goal_sub(self, data):
        # rospy.sleep(0.2)
        #self.request_audio(self.pkg_path+'/scripts/source/goal_departure.mp3')
        #playsound(pkg_path+'/scripts/source/goal_departure.mp3')
        rospy.loginfo("Goal_Departure")

    def freeway_goal_sub(self, data):
        self.request_audio(self.pkg_path+'/scripts/source/goal_departure.mp3')
        rospy.loginfo("Freeway_Goal_Departure")

    def mb_goal_sub(self, data):
        # rospy.sleep(0.2)
        self.request_audio(self.pkg_path+'/scripts//source/goal_departure.mp3')
        #playsound(pkg_path+'/scripts/source/goal_departure.mp3')
        rospy.loginfo("Goal_Departure")

    def cancel_sub(self, data):
        #self.play_audio(pkg_path+'/scripts/source/goal_canceled.mp3')
        #playsound(pkg_path+'/scripts/source/goal_canceled.mp3')
        rospy.loginfo("Goal_Canceled")

    def diagnostics_sub(self, data):
        #self.play_audio(pkg_path+'/scripts/source/goal_canceled.mp3'])
        #playsound('source/goal_canceled.mp3')
        rospy.loginfo("Diagnostics_called")

    def distancetimecalculator_sub(self, data):
        if self.previous_arrival_time == 0.0 and data.arrival_time != 0.0:
            if data.arrival_time <=10:
                self.goal_sub_flag = False        
        if data.distance_remaining != 0.0:
            if rospy.Time.now().to_sec() - self.ct >= self.auto_driving_timer_time and self.dt_f == True and data.arrival_time > 7.0:
                #self.play_audio(self.pkg_path+'/scripts/source/auto_driving.mp3')
                #playsound(pkg_path+'/scripts/source/auto_driving.mp3')
                self.ct = rospy.Time.now().to_sec()
            elif data.arrival_time <= 7.0 and self.dt_count < 1 and self.goal_sub_flag != False:
                #self.play_audio(pkg_path+'/scripts/source/goal_close.mp3')
                #playsound(self.pkg_path+'/scripts/source/goal_close.mp3')
                self.dt_count = self.dt_count+1
                self.dt_f = False
            self.previous_arrival_time = data.arrival_time
        #rospy.loginfo("DistanceTimeCalculator_Called")
        else:
            self.ct = rospy.Time.now().to_sec()
            self.dt_count = 0
            self.dt_f = True
            self.goal_sub_flag = True
        self.previous_arrival_time = data.arrival_time

    def am_sub(self, data):
        if data.am_status2 == True:
            self.request_audio(self.pkg_path+'/scripts/source/auto_driving.mp3')
            #playsound(pkg_path+'/scripts/source/auto_driving.mp3')
            rospy.loginfo("Auto_driving")
        elif data.am_status2 == False:
            self.request_audio(self.pkg_path+'/scripts/source/manual_mode.mp3')
            #playsound(pkg_path+'/scripts/source/manual_mode.mp3')
            rospy.loginfo("Manual_mode")

    def result_sub(self, data):
        if data.status.status == 3:
            # rospy.sleep(1)
            self.request_audio(self.pkg_path+'/scripts/source/goal_arrived.mp3')
            #playsound(pkg_path+'/scripts/source/goal_arrived.mp3')
            rospy.loginfo("Goal_Arrived")
        elif data.status.status == 1:
            # rospy.sleep(1)
            rospy.loginfo("goal_moving")
        elif data.status.status == 4:
            self.request_audio(self.pkg_path+'/scripts/source/goal_cannotreach.mp3')
            #playsound(pkg_path+'/scripts/source/goal_cannotreach.mp3')
            rospy.loginfo("Goal_CannotReach")

    def resume_sub(self, data):
        #self.request_audio(self.pkg_path+'/scripts/source/etri/resume.mp3')
        rospy.loginfo("Goal_Resume called")

    def ai_status_sub(self, data):
        self.s_command = data.data[0]
        self.r_command = data.data[1]

        if self.r_command != self.pre_r_command:
            if self.r_command == 1:
                self.request_audio(self.pkg_path+'/scripts/source/etri/goal_departure.mp3')
                print("Goal departure called")
            elif self.r_command == 2:
                self.request_audio(self.pkg_path+'/scripts/source/etri/decel.mp3')
                print("Decel called")
            elif self.r_command == 3:
                self.request_audio(self.pkg_path+'/scripts/source/etri/accel.mp3')
                print("Accel called")
            elif self.r_command == 4:
                self.request_audio(self.pkg_path+'/scripts/source/etri/turn.mp3')
                print("Turn called")
            elif self.r_command == 5:
                self.request_audio(self.pkg_path+'/scripts/source/etri/stop.mp3')
                print("Stop called")
            elif self.r_command == 6:
                self.request_audio(self.pkg_path+'/scripts/source/etri/pause.mp3')
                print("Pause called")

            self.pre_r_command = self.r_command

        if  self.s_command != self.pre_s_command:
            if self.s_command == 0:
                pass
            elif self.s_command == 1:
                self.request_audio(self.pkg_path+'/scripts/source/etri/current_location.mp3')
                print("Current location called")
            elif self.s_command == 2:
                self.request_audio(self.pkg_path+'/scripts/source/etri/play_music.mp3')
                print("Play music called")
            elif self.s_command == 3:
                self.request_audio(self.pkg_path+'/scripts/source/etri/play_video.mp3')
                print("Play video called")
            elif self.s_command == 4:
                self.request_audio(self.pkg_path+'/scripts/source/etri/show_shopping.mp3')
                print("Show shopping called")
            elif self.s_command == 5:
                self.request_audio(self.pkg_path+'/scripts/source/etri/show_rest.mp3')
                print("Show rest called")
            elif self.s_command == 6:
                self.request_audio(self.pkg_path+'/scripts/source/etri/say_hello.mp3')
                print("Say hello called")
            elif self.s_command == 7:
                self.request_audio(self.pkg_path+'/scripts/source/etri/show_restaurant.mp3')
                print("Show restaurant called")
            elif self.s_command == 8:
                self.request_audio(self.pkg_path+'/scripts/source/etri/play_music.mp3')
                print("Play music called")
            elif self.s_command == 9:
                self.request_audio(self.pkg_path+'/scripts/source/etri/bye_bye.mp3')
                print("Bye Bye called")

            self.pre_s_command = self.s_command

def main():
    rospy.init_node('fw_spk_node', anonymous=True)
    Fs = Freeway_spk()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        pass
    
