#!/usr/bin/env python
'''
Brandon Marlowe
----------------
TurtleBot listens for voice commands given from user, and performs corresponding actions.
'''

import rospy
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
from std_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from math import *
import pyaudio
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from os import path
from os.path import expanduser

twist_msg = Twist()
mbs_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=15)
vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=15)

coord = [[-2.48, 5.03, 1.05], [1.07, 4.81, -0.751], [-2.19, 1.02, 0.681],
         [2.53, -0.81, 0.014]]

STOPPED = 0.0
STD_VEL = 0.275

# global variables for pose information
pose_x = 0.0
pose_y = 0.9
pose_w = 0.0
header_frame_id = ''
seq = 0

x_goal = 0.0
y_goal = 0.0

prev_vel = 0.0
prev_angle = 0.0
voice_cmd = ''

goal_set = False


def get_coord_index(cmd):
    numbers = ['ONE', 'TWO', 'THREE', 'FOUR']

    for index in range(len(numbers)):
        if numbers[index] in cmd:
            return index


def sign_of(val):
    return 1.0 if val > 0.0 else -1.0


def cmd_confirmation():
    global voice_cmd, prev_angle, prev_vel

    publish_twist_msg(STOPPED, STOPPED)
    SoundClient().say('Do you want me to {}?'.format(voice_cmd))

    while True:
        confirmation = get_voice_cmd()
        if confirmation == 'YES':
            SoundClient().say('On my way.')
            return True
        elif confirmation == 'NO':
            SoundClient().say('Okay. I will wait for a new request.')
            return False


# callback of TurtleBot current pose
def odom_callback(data):
    global pose_x, pose_y, pose_w, header_frame_id
    header_frame_id = data.header.frame_id
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y
    pose_w = data.pose.pose.orientation.w


# calculates the Euclidean distance of the turtlebot from the goal destination
def dist_from_goal():
    global x_goal, y_goal, pose_x, pose_y, goal_set
    rospy.Subscriber('/odom', Odometry, odom_callback)

    if sqrt(pow((x_goal - pose_x), 2) + pow((y_goal - pose_y), 2)) < 0.75:
        SoundClient().say('I have arrived at the destination')
        goal_set = False


def publish_twist_msg(lin_vel, ang_vel):
    global prev_vel, prev_angle

    twist_msg.linear.x = lin_vel
    twist_msg.angular.z = ang_vel
    vel_pub.publish(twist_msg)

    prev_vel = lin_vel
    prev_angle = ang_vel


def go_to_coord(x, y, w):

    global seq, x_goal, y_goal, w_goal

    x_goal = x
    y_goal = y
    w_goal = w

    ps_goal = PoseStamped()

    # placing goal destination in PoseStamped publisher
    ps_goal.header.frame_id = 'map'
    ps_goal.header.seq = seq
    ps_goal.header.stamp = rospy.Time.now()
    ps_goal.pose.position.x = x_goal
    ps_goal.pose.position.y = y_goal
    ps_goal.pose.position.z = 0.0
    ps_goal.pose.orientation.x = 0.0
    ps_goal.pose.orientation.y = 0.0
    ps_goal.pose.orientation.w = w_goal

    seq += 1

    mbs_pub.publish(ps_goal)


def get_control_signal():
    global goal_set, prev_vel, prev_angle, STOPPED, STD_VEL, voice_cmd

    if goal_set:
        if 'CANCEL' in get_voice_cmd():
            SoundClient().say('Action cancelled.')
            goal_set = False
            go_to_coord(pose_x, pose_y, pose_w)

        else:
            return

    voice_cmd = ''
    voice_cmd = get_voice_cmd()

    goal_set = False

    vel = prev_vel
    angle = prev_angle

    if 'TURN' in voice_cmd:
        if vel == STOPPED:
            vel = STD_VEL
        if 'RIGHT' in voice_cmd:
            voice_cmd = 'TURNING RIGHT'
            angle = -0.3
        elif 'LEFT' in voice_cmd:
            voice_cmd = 'TURNING LEFT'
            angle = 0.3

    elif 'ROTATE' in voice_cmd:
        vel = STOPPED
        if 'RIGHT' in voice_cmd:
            voice_cmd = 'ROTATING RIGHT'
            angle = -0.5
        if 'LEFT' in voice_cmd:
            voice_cmd = 'ROTATING LEFT'
            angle = 0.5

    elif 'GO TO POSITION' in voice_cmd:
        index = get_coord_index(voice_cmd)
        vel = STOPPED
        angle = STOPPED
        voice_cmd = 'GO TO POSITION {}'.format(index + 1)

        if cmd_confirmation():
            goal_set = True
            go_to_coord(coord[index][0], coord[index][1], coord[index][2])

    elif 'STOP' in voice_cmd:
        vel = STOPPED
        angle = STOPPED
        voice_cmd = 'STOPPING'

    elif 'MOVE' in voice_cmd:
        angle = STOPPED
        if 'FORWARD' in voice_cmd:
            voice_cmd = 'MOVING FORWARD'
            vel = STD_VEL
        elif 'BACK' in voice_cmd:
            voice_cmd = 'MOVING BACK'
            vel = -STD_VEL

    # if something changed, a valid command was understood
    if vel != prev_vel or angle != prev_angle:
        publish_twist_msg(vel, angle)
        SoundClient().say(voice_cmd)


def get_voice_cmd():

    PS_CORPUS_DIR = '/usr/share/pocketsphinx/model/hmm/'
    NEW_CORPUS_DIR = expanduser('~') + '/catkin_ws/src/MarloweB/corpus/'

    config = Decoder.default_config()
    config.set_string('-hmm', path.join(PS_CORPUS_DIR, 'en_US/hub4wsj_sc_8k/'))
    config.set_string('-lm', path.join(NEW_CORPUS_DIR, '2840.lm'))
    config.set_string('-dict', path.join(NEW_CORPUS_DIR, '2840.dic'))
    config.set_string('-logfn', '/dev/null')

    audio = pyaudio.PyAudio()
    stream = audio.open(
        format=pyaudio.paInt16,
        channels=1,
        rate=16000,
        input=True,
        frames_per_buffer=1024)

    decoder = Decoder(config)
    stream.start_stream()

    in_speech = False

    decoder.start_utt()

    while True:

        audio_buffer = stream.read(1024)

        if not goal_set:
            publish_twist_msg(prev_vel, prev_angle)
        if goal_set:
            dist_from_goal()

        if audio_buffer:
            decoder.process_raw(audio_buffer, False, False)

            if decoder.get_in_speech() != in_speech:
                in_speech = decoder.get_in_speech()

                if not in_speech:
                    decoder.end_utt()
                    cmd = decoder.hyp().hypstr
                    if cmd == None:
                        return ''
                    else:
                        return cmd
        else:
            break

    decoder.end_utt()


def main():

    while not rospy.is_shutdown():
        get_control_signal()


if __name__ == '__main__':

    rospy.init_node('voice_navigation_node', anonymous=False)

    try:
        main()

    except rospy.ROSException:
        pass
