#! /usr/bin/env python
# -*- coding: utf-8 -*-

import curses
import math
import rospy
import sys
from geometry_msgs.msg import Twist

def main(stdscr, persist):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('key_teleop', anonymous=True)
    rate = rospy.Rate(10) 
    twist = Twist()
    keycode = -1
    stdscr.addstr("Command\n")
    stdscr.addstr(" - UP/DOWN    : control linear x  (+= or -=)\n")
    stdscr.addstr(" - LEFT/RIGHT : control linear y  (+= or -=)\n")
    stdscr.addstr(" - e/r        : control angular z (+= or -=)\n")
    stdscr.addstr(" - any key    : reset of the twist\n")
    stdscr.addstr(" - ESC        : reset twist and exit\n")
    # We set the "wait for a key press" period to 100 ms. 
    if(persist): stdscr.timeout(100)
    while (not rospy.is_shutdown()) and (keycode != 27): # 27 is escape
        keycode = stdscr.getch()         # Wait for a key press for at most 100ms
        if   keycode == -1               : pass # No key has been pressed, we keep current twist.
        elif keycode == curses.KEY_UP    : twist.linear.x  = twist.linear.x  + .1
        elif keycode == curses.KEY_DOWN  : twist.linear.x  = twist.linear.x  - .1
        elif keycode == curses.KEY_LEFT  : twist.linear.y  = twist.linear.y  + .1
        elif keycode == curses.KEY_RIGHT : twist.linear.y  = twist.linear.y  - .1
        elif keycode == 101              : twist.angular.z = twist.angular.z + .2
        elif keycode == 114              : twist.angular.z = twist.angular.z - .2
        else                             : twist = Twist()
        pub.publish(twist)
        rate.sleep()

# Starts curses (terminal handling) and run our main function.
if __name__ == '__main__':
    persist = '--persist' in rospy.myargv(argv=sys.argv)
    try:
        curses.wrapper(lambda w: main(w,persist))
    except rospy.ROSInterruptException:
        pass
