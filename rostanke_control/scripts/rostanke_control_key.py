#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving :
    q   w   e  y
    a   s   d
    z   x   c  h
       
o/p : increase/decrease only linear speed by 10%
k/l : increase/decrease only angular speed by 10%
CTRL-C to quit

y up platform
h down platform

"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 2.5
turn = 3.5
joint1=0.0
joint2=0.0

def print_msg(msg):
    print ("                                                                               ", end='\r')
    print ("   "+ msg, end='\r')

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('rostanke_teleop')
    pub = rospy.Publisher('/rostanke/mobile_base_controller/cmd_vel', Twist, queue_size=1)
    pub_joint= rospy.Publisher('/rostanke/joint_states', JointState,queue_size=1)
    #r=rospy.Rate(5)
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    max_speed = 10.0
    max_turn = 10.0
    max_joint = 0.32
    push=0
    try:
        print (msg)
        print (vels(speed,turn))
        while(1):
            key = getKey()

            if key == 'w': #forward
                x=1.0
                th=0.0
                count = 0
                print_msg("forward")
            elif key == 'x': #backward
                x=-1.0
                th=0.0
                count = 0
                print_msg("backward")
            elif key == 'a': #left
                x=0.0
                th=1.0
                count = 0
                print_msg("left")
            elif key == 'd': #right
                x=0.0
                th=-1.0
                count = 0
                print_msg("right")
            elif key == 'q': #forward left
                x=1.0
                th=1.0
                count = 0
                print_msg("forward left")
            elif key == 'e': #forward right
                x=1.0
                th=-1.0
                count = 0
                print_msg("forward right")
            elif key == 'z': #backward left
                x=-1.0
                th=1.0
                count = 0
                print_msg("backward left")
            elif key == 'c': #backward right
                x=-1.0
                th=-1.0
                count = 0
                print_msg("backward right")
            elif key == 's': #stop
                x=0.0
                th=0.0
                count = 0
                print_msg("stop")   
            elif key == 'o': # increase only linear speed by 10%
                speed = speed * 1.1
                print_msg (vels(speed,turn))
                count = 0 
            elif key == 'p': #decrease only linear speed by 10%
                speed = speed * 0.9
                print_msg (vels(speed,turn))
                count = 0 
            elif key == 'k': #increase only angular speed by 10%
                turn = turn * 1.1
                print_msg (vels(speed,turn))
                count = 0 
            elif key == 'l': #decrease only angular speed by 10%
                turn = turn * 0.9
                print_msg (vels(speed,turn))
                count = 0
            elif key == 'y': #up platfrom 
                joint1=0.32
                joint2=-0.32
                print_msg ("platfrom up")
                count = 0
            elif key == 'h': #down platfrom
                joint1=0.0
                joint2=0.0
                print_msg ("platfrom down")
                count = 0
            elif key == '\x03':
                break 
            else:
                count = count + 1
                if count > 2:
                    x = 0
                    th = 0
                    print_msg("stop")

            
            target_speed = speed * x
            target_turn = turn * th

            if target_speed > max_speed:
                target_speed = max_speed
            elif target_speed < -max_speed:
                target_speed = -max_speed
            
            if target_turn > max_turn:
                target_turn = max_turn
            elif target_turn < -max_turn:
                target_turn = -max_turn
            
            twist = Twist()
            #joint_send = JointState()
            twist.linear.x = target_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_turn
            #print_msg("Numero de mensajes enviado: " + str(push))
            pub.publish(twist)
            #joint_send.name['plat00_joint','plat02_joint','front_left_base_wheel_joint','back_left_base_wheel_joint','front_right_base_wheel_joint','back_right_base_wheel_joint']
            #joint_send.position[joint1, joint2, 0.0, 0.0, 0.0, 0.0]
            #joint_send.name[0]="plat00_joint"; joint_send.position[0]=joint1
            #joint_send.name[1]="plat02_joint"; joint_send.position[1]=joint2
            
            #pub_joint.publish(joint_send)
            #push = push + 1
            #r.sleep()
            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print (e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)