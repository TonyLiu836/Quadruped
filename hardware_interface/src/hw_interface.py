import rospy
from PiPCA9685.PiPCA9685 import PCA9685
import time
import numpy as np

class hw_interface():
    def __init__(self,):
        self.jointPos = np.zeros(12)
        rospy.init_node('listener')
        self.pca = PCA9685()

        self.pca.set_pwm_freq(60)       #might need to change the freq to  match champ software

        self.pub = rospy.Publisher('')
        rospy.Subscriber()

    def anglesReceived(self, msg):
        print("angles received=", msg)
        newJointAngles = msg
        self.pca.set_pwm(channel, on, off) #use this to set position of each motor

    
    def run(self):
        r = rospy.Rate(10)
        
    
    

"""
def anglesReceived(msg):
    rospy.loginfo(rospy.get_caller_id() + "msg received: ", msg.data)


def listener():
    rospy.init_node('listener', annymous = True)

    rospy.subscriber("chatter", String, anglesReceived)

    rospy.spin()



if __name__=='__main__':
    listener()
"""