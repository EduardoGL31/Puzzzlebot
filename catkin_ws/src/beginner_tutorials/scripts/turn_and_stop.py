#!/usr/bin/env python3 

import rospy  
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String  


#This class will suscribe to ghe /command topic and make the robot stop

# Then stops  

class Turn_and_stop():  

    def __init__(self):  

        # first thing, init a node!   
        rospy.init_node('turn_and_stop')
        rospy.on_shutdown(self.cleanup) # Call the cleanup function before finishing the node.  
        ###******* INIT PUBLISHERS *******###  
        # create the publisher to cmd_vel topic 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1) 
        my_twist = Twist() # create a twist message, fill in the details 
        self.string_msg = String()

        rospy.Subscriber("command",String,self.command_cb)
        my_twist.angular.z = 0.5
        self.cmd_vel_pub.publish(my_twist)
        
        rate = rospy.Rate(20) # The rate of the while loop will be 50Hz 

        rospy.loginfo("About to be moving forward!") 

        while not rospy.is_shutdown(): 
            if self.string_msg.data == "stop":
                my_twist.angular.z = 0

            self.cmd_vel_pub.publish(my_twist)
            rate.sleep()


    
    def command_cb(self, string_msg):
        self.string_msg = string_msg 



    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.  

        print("Stopping the robot") 

        print("Bye bye!!!")  

 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    Turn_and_stop()  
