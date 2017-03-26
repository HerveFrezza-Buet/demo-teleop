#! /usr/bin/env python
# -*- coding: utf-8 -*-

import curses
import math
import rospy
import sys
import dynamic_reconfigure.server
from std_msgs.msg           import String
from std_msgs.msg           import Empty
from geometry_msgs.msg      import Twist
from demo_teleop.cfg        import SafeDroneTeleopConfig
   
class Status:
    def __init__(self):
        self.status     = "landed" # "landing" "taking off" "automatic flight" "manual flight"
        self.last_time  = rospy.Time()
        self.last_input = rospy.Time()
        self.twist      = Twist()
        self.in_twist   = Twist()
        self.state_wait    = 3.0 # transition between states (taking_off, landing, etc..)
        self.watchdog_time = .25 # input crash security delay (no input twist => stop)
        self.linear     = 0.1 # initial value 
        self.angular    = 0.5 # initial value 
        self.delay      = 2.0 # initial value 
        self.status_pub = rospy.Publisher ('status', String, queue_size=1)
        self.r_pub      = rospy.Publisher ('reset', Empty, queue_size=1)
        self.t_pub      = rospy.Publisher ('takeoff', Empty, queue_size=1)
        self.l_pub      = rospy.Publisher ('land', Empty, queue_size=1)
        self.twist_pub  = rospy.Publisher ('cmd_vel_out', Twist, queue_size=1)
        self.twist_sub  = rospy.Subscriber('cmd_vel_in',  Twist, self.on_twist,  queue_size = 1)
        self.config_srv = dynamic_reconfigure.server.Server(SafeDroneTeleopConfig, self.on_reconf)

    def on_reconf(self, config, level):
        self.angular = config['angular']
        self.linear  = config['linear']
        self.delay   = config['delay']
        return config
        
    def on_twist(self, ros_data):
        self.in_twist = ros_data
        self.last_input = rospy.Time.now()
        
    def send_twist(self):
        if self.status == "manual flight":
            self.twist_pub.publish(self.twist)
        elif self.status == "automatic flight":
            time = rospy.Time.now()
            if(time - self.last_input).to_sec() > self.watchdog_time :
                self.in_twist   = Twist()
            self.twist_pub.publish(self.in_twist)
        
    def take_off(self):
        if self.status == "landed" :
            self.twist     = Twist()
            self.status    = "taking off"
            self.status_pub.publish(self.status)
            self.r_pub.publish(Empty())
            rospy.sleep(1.)
            self.t_pub.publish(Empty())
            self.last_time = rospy.Time.now()
        
    def land(self):
        self.last_time = rospy.Time.now()
        self.twist     = Twist()
        self.status    = "landing"
        self.l_pub.publish(Empty())

    def nop(self):
        if self.status == "manual flight" :
            time = rospy.Time.now()
            if (time - self.last_time).to_sec() > self.delay :
                self.status = "automatic flight" 
        elif self.status == "taking off":
            time = rospy.Time.now()
            if (time - self.last_time).to_sec() > self.state_wait :
                self.status = "manual flight"
                self.last_time = time
                self.twist = Twist()
        elif self.status == "landing":
            time = rospy.Time.now()
            if (time - self.last_time).to_sec() > self.state_wait :
                self.status = "landed"
        self.status_pub.publish(self.status)
        self.send_twist()
                
    def key_pressed(self):
        self.last_time = rospy.Time.now()
        if self.status != "manual flight":
            self.twist = Twist()
        if self.status == "automatic flight" :
            self.status = "manual flight"
        self.send_twist()
            
            
        
        

def main(stdscr):
    rospy.init_node('safe_drone_teleop', anonymous=True)
    log_pub = rospy.Publisher ('log', String, queue_size=1)
    rate = rospy.Rate(10) 
    keycode = -1
    status = Status()
    stdscr.addstr("Safe drone controller\n")
    stdscr.addstr("---------------------\n")
    stdscr.addstr("\n")
    stdscr.addstr("Command\n")
    stdscr.addstr(" - UP/DOWN      : control linear x\n")
    stdscr.addstr(" - LEFT/RIGHT   : control linear y\n")
    stdscr.addstr(" - e/r          : control angular z\n")
    stdscr.addstr(" - t/l          : take off / land\n")
    stdscr.addstr(" - PAGE UP/DOWN : elevation control\n")
    stdscr.addstr(" - any key      : reset of the twist\n")
    # We set the "wait for a key press" period to 100 ms. 
    stdscr.timeout(100)
    while (not rospy.is_shutdown()):
        keycode = stdscr.getch()         # Wait for a key press for at most 100ms
        if   keycode == -1 :
            status.nop() # No key has been pressed, we keep current twist.
        elif keycode == curses.KEY_UP :
            status.key_pressed()
            if xlevel == -1 :
                status.twist.linear.x  = 0
                xlevel = 0
            elif xlevel == 0:
                status.twist.linear.x  = status.linear
                xlevel = 1
        elif keycode == curses.KEY_DOWN :
            status.key_pressed()
            if xlevel == 0 :
                status.twist.linear.x  = -status.linear
                xlevel = -1
            elif xlevel == 1:
                status.twist.linear.x  = 0
                xlevel = 0
        elif keycode == curses.KEY_LEFT :
            status.key_pressed()
            if ylevel == -1 :
                status.twist.linear.y  = 0
                ylevel = 0
            elif ylevel == 0:
                status.twist.linear.y  = status.linear
                ylevel = 1
        elif keycode == curses.KEY_RIGHT :
            status.key_pressed()
            if ylevel == 0 :
                status.twist.linear.y  = -status.linear
                ylevel = -1
            elif ylevel == 1:
                status.twist.linear.y  = 0
                ylevel = 0
        elif keycode == curses.KEY_PPAGE :
            status.key_pressed()
            if zlevel == -1 :
                status.twist.linear.z  = 0
                zlevel = 0
            elif zlevel == 0:
                status.twist.linear.z  = status.linear
                zlevel = 1
        elif keycode == curses.KEY_NPAGE :
            status.key_pressed()
            if zlevel == 1 :
                status.twist.linear.z  = 0
                zlevel = 0
            elif zlevel == 0:
                status.twist.linear.z  = -status.linear
                zlevel = -1
        elif keycode == 101 : # e
            status.key_pressed()
            if alevel == -1 :
                status.twist.angular.z  = 0
                alevel = 0
            elif alevel == 0:
                status.twist.angular.z  = status.angular
                alevel = 1
        elif keycode == 114 : # r
            status.key_pressed()
            if alevel == 0 :
                status.twist.angular.z  = -status.angular
                alevel = -1
            elif alevel == 1:
                status.twist.angular.z  = 0
                alevel = 0
        elif keycode == 116 : # t
            status.take_off()
            xlevel = 0
            ylevel = 0
            zlevel  = 0
            alevel  = 0
        elif keycode == 108 : # l
            status.land()
            xlevel = 0
            ylevel = 0
            zlevel  = 0
            alevel  = 0
        else :
            status.key_pressed()
            status.twist = Twist()
            xlevel = 0
            ylevel = 0
            zlevel  = 0
            alevel  = 0
        if status.status == "automatic flight" :
            xlevel = 0
            ylevel = 0
            zlevel  = 0
            alevel  = 0
            status.twist = Twist()
        
            

# Starts curses (terminal handling) and run our main function.
if __name__ == '__main__':
    try:
        curses.wrapper(lambda w: main(w))
    except rospy.ROSInterruptException:
        pass
