#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
#import sys, select, termios, tty
#from playsound import playsound
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
#from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult
from mbf_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult
from freeway_msgs.msg import DistanceTimeCalculator
from freeway_joyfw.msg import stm_am_msg, stm_fw_msg
from actionlib_msgs.msg import GoalID
import subprocess

dt_f=True
goal_sub_flag = False
dt_count=0
ct=0.0
auto_driving_timer_time = 10.0
previous_arrival_time = 0.0
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('freeway_spk')
audio_playing = False  # flag to indicate whether an audio file is currently playing

def play_audio(file_path):
    # subprocess.call(['mpg123', '-q', file_path])
    global audio_playing
    if not audio_playing:
	audio_playing = True
        subprocess.call(['mpg123', '-q' ,file_path])
        audio_playing = False

def manual_spk_call_sub(data):
    play_audio(pkg_path+'/scripts/source/gn/first_start.mp3')

def goal_sub(data):
    #rospy.sleep(0.2)
    play_audio(pkg_path+'/scripts/source/gn/goal_departure.mp3')
    #playsound(pkg_path+'/scripts/source/goal_departure.mp3')
    rospy.loginfo("Goal_Departure")

def mb_goal_sub(data):
    #rospy.sleep(0.2)
    play_audio(pkg_path+'/scripts/source/gn/goal_departure.mp3')
    #playsound(pkg_path+'/scripts/source/goal_departure.mp3')
    rospy.loginfo("Goal_Departure")

def cancel_sub(data):
    #play_audio(pkg_path+'/scripts/source/goal_canceled.mp3')
    #playsound(pkg_path+'/scripts/source/goal_canceled.mp3')
    rospy.loginfo("Goal_Canceled")

def diagnostics_sub(data):
    #play_audio(pkg_path+'/scripts/source/goal_canceled.mp3'])
    #playsound('source/goal_canceled.mp3')
    rospy.loginfo("Diagnostics_called")

def distancetimecalculator_sub(data):
    global ct
    global dt_f
    global dt_count
    global auto_driving_timer_time
    global previous_arrival_time
    global goal_sub_flag

    if previous_arrival_time == 0.0 and data.arrival_time != 0.0:
        if data.arrival_time <=10:
            goal_sub_flag = False        
    if data.distance_remaining != 0.0:
        if rospy.Time.now().to_sec() - ct >= auto_driving_timer_time and dt_f == True and data.arrival_time > 7.0:
            play_audio(pkg_path+'/scripts/source/gn/auto_driving.mp3')
            #playsound(pkg_path+'/scripts/source/auto_driving.mp3')
            ct = rospy.Time.now().to_sec()
        elif data.arrival_time <= 7.0 and dt_count < 1 and goal_sub_flag != False:
            #play_audio(pkg_path+'/scripts/source/gn/goal_close.mp3')
            #playsound(pkg_path+'/scripts/source/goal_close.mp3')
            dt_count = dt_count+1
            dt_f = False
        previous_arrival_time = data.arrival_time
    #rospy.loginfo("DistanceTimeCalculator_Called")
    else:
        ct = rospy.Time.now().to_sec()
        dt_count = 0
        dt_f = True
        goal_sub_flag = True
    previous_arrival_time = data.arrival_time
    
def am_sub(data):
    if data.am_status2 == True:
        play_audio(pkg_path+'/scripts/source/gn/auto_driving.mp3')
        #playsound(pkg_path+'/scripts/source/auto_driving.mp3')
        rospy.loginfo("Auto_driving")
    elif data.am_status2 == False:
        play_audio(pkg_path+'/scripts/source/manual_mode.mp3')
        #playsound(pkg_path+'/scripts/source/manual_mode.mp3')
        rospy.loginfo("Manual_mode")

def result_sub(data):
    if data.status.status == 3:
        #rospy.sleep(1)
        play_audio(pkg_path+'/scripts/source/gn/goal_arrived.mp3')
        #playsound(pkg_path+'/scripts/source/goal_arrived.mp3')
        rospy.loginfo("Goal_Arrived")
    elif data.status.status == 1:
        #rospy.sleep(1)
        rospy.loginfo("goal_moving")
    elif data.status.status == 4:
        play_audio(pkg_path+'/scripts/source/goal_cannotreach.mp3')
        #playsound(pkg_path+'/scripts/source/goal_cannotreach.mp3')
        rospy.loginfo("Goal_CannotReach")


def main_f():

      rospy.init_node('fw_spk_node', anonymous=True)
  
      rate = rospy.Rate(5) # ROS Rate at 5Hz
      rospy.Subscriber("move_base_simple/goal", PoseStamped, goal_sub)
      rospy.Subscriber("freeway/manual_spk_call", Empty, manual_spk_call_sub)
    #   rospy.Subscriber("move_base/goal", MoveBaseActionGoal, mb_goal_sub)
    #   rospy.Subscriber("move_base_flex/move_base/cancel", GoalID, cancel_sub)
      rospy.Subscriber("freeway/goal_cancel", Empty, cancel_sub)
      rospy.Subscriber("freeaway/diagnostics", stm_fw_msg, diagnostics_sub)
      rospy.Subscriber("freeway/am_status", stm_am_msg, am_sub)
      rospy.Subscriber("move_base_flex/move_base/result", MoveBaseActionResult, result_sub)
      rospy.Subscriber("freeway/distancetimecalculator", DistanceTimeCalculator, distancetimecalculator_sub)

    #   while not rospy.is_shutdown():
    #     rate.sleep()
      play_audio(pkg_path+'/scripts/source/gn/first_start.mp3')
      #playsound(pkg_path+'/scripts/source/first_start.mp3')
      rospy.spin()
  
if __name__ == '__main__':
    try:
        main_f()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        pass
    
