#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class Pendullum(): 
    def __init__(self): 
        rospy.on_shutdown(self.cleanup) #This function will be called before killing the node.
        #########PUBLISHERS AND SUBSCRIBERS #################
        self.joint_states_pub = rospy.Publisher('joint_states', JointState, queue_size=1) 
        rospy.Subscriber("tau", Float32, self.message_cb) 
        ############ CONSTANTS AND VARIABLES ################ 

        #Declare Variables to be used
        self.joint_state = JointState()
        self.tau = Float32

        #SLM Parameters
        k = 0.01
        m = 0.75
        l = 0.36 
        a = l/2
        g = 9.8
        x1 = 0.0
        x2 = 0.0
        x1_dot = 0.0
        x2_dot = 0.0
        dt = 0.05
        J = 4/3 * m * (a * a)

        r = rospy.Rate(60) #60 Hz
        while not rospy.is_shutdown(): 
            if self.tau <= 0:
                print("Invalid time")
            else:
                 #SLM governing equation
                x1_dot += x2
                x2_dot = (1/(J + m * (a * a)) * (self.tau -  m * g * a * np.cos(x1) - k * x2 + self.tau))
                x2 += x2_dot*dt

            self.joint_states_pub.publish(self.joint_state) #publish the message
            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle. 
    
    def message_cb(self, msg): 
        ## This function receives the ROS message as the msg variable. 
        self.tau =  msg.data #msg.data is the string contained inside the ROS message 
        print("I received this message in the callback: " + self.tau)
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        print("I'm stopping") 
        zero_joint = JointState()
        self.joint_states_pub.publish(zero_joint)

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("pub_sub_with_classes", anonymous=True) 
    Pendullum()

# Setup Variables to be used

# Declare the input Message

# Declare the  process output message


#Define the callback functions


  #wrap to pi function
def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi),(2 * np.pi))
    if(result < 0):
        result += 2 * np.pi
    return result - np.pi


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("SLM_Sim")

    #Get Parameters   

    # Configure the Node
    loop_rate = rospy.Rate(rospy.get_param("~node_rate",100))


    # Setup the Subscribers


    #Setup de publishers

    print("The SLM sim is Running")
    try:
        #Run the node (YOUR CODE HERE)
        
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE
            #WRITE YOUR CODE HERE

            #Wait and repeat
            loop_rate.sleep()
    
    except rospy.ROSInterruptException:
        pass #Initialise and Setup node