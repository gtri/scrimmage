#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
import curses
from time import sleep

def keyboard_to_cmd_vel():
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(1)

    thrust = 1;
    pitch_acc = 1;
    roll_acc = 1;
    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('keyboard_to_cmd_vel', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        cmd_vel = Twist()
        
        c = stdscr.getch()
        if c == curses.KEY_LEFT:
            cmd_vel.angular.x = -roll_acc;
        elif c == curses.KEY_RIGHT:
            cmd_vel.angular.x = roll_acc;            
        elif c == curses.KEY_UP:
            cmd_vel.angular.y = pitch_acc;            
        elif c == curses.KEY_DOWN:
            cmd_vel.angular.y = -pitch_acc;
        elif c == curses.KEY_PPAGE:            
            cmd_vel.linear.x = thrust
        elif c == curses.KEY_NPAGE:
            cmd_vel.linear.x = -thrust
                        
        pub.publish(cmd_vel)
        sleep(0.10)

        cmd_vel = Twist()
        pub.publish(cmd_vel)
        
        rate.sleep()

    curses.endwin()    

if __name__ == '__main__':
    try:
        keyboard_to_cmd_vel()
    except rospy.ROSInterruptException:
        pass    
