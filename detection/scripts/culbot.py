#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool, Int32


import sys, select, termios, tty

msg = """
Control Your Culbot!
---------------------------
Moving around:			Arm Control:
   u    i    o			   1   2
   j    k    l			   3   4
   m    ,    .
1/2 : out/in ER linear actuator
5/6 : counter/clockwise
3/4 : down/up arm
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .1
turn = 1.5


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('turtlebot_teleop')
    # pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    arm_pub_ = rospy.Publisher('J1', Int32, queue_size=5)
    arm2_pub_ = rospy.Publisher('J3', Int32, queue_size=5)
    lin_pub_ = rospy.Publisher('J2', Int32, queue_size=5)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    
    arm_contr = 0
    lin_contr = 0
    arm2_contr = 0
    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                arm_contr = 0
                lin_contr = 0
            elif key =='1':
                arm_contr = 1
            elif key =='2':
                arm_contr = 2
            elif key =='3':
                lin_contr = 3
            elif key =='4':
                lin_contr = 4
            elif key =='5':
                arm2_contr = 5
            elif key =='6':
                arm2_contr = 6
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                    arm_contr = 0
                    lin_contr = 0
                    arm2_contr = 0
		    
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            
            j1 = Int32()
            j1.data = arm_contr

            j2 = Int32()
            j2.data = lin_contr

            j3 = Int32()
            j3.data = arm2_contr

            # pub.publish(twist)
            arm_pub_.publish(j1)
            lin_pub_.publish(j2)
            arm2_pub_.publish(j3)

            arm_contr = 0
            lin_contr = 0
            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        
        j1 = Int32()
        j1.data = 0

        j2 = Int32()
        j2.data = 0

        j3 = Int32()
        j3.data = 0

        # pub.publish(twist)
        arm_pub_.publish(j1)
        lin_pub_.publish(j2)
        arm2_pub_.publish(j3)

	

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)