#! /usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist

class Multiplexer :
    def __init__(self) -> None:
       pub_freq = 4.
       self.rate = rospy.Rate(pub_freq)
       self.final_cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
       cmd_web_subscriber = rospy.Subscriber('/cmd_web', Twist, self.cmdWebCallBack, queue_size=10)
       cmd_local_subscriber = rospy.Subscriber('/cmd_local', Twist, self.cmdLocalCallBack, queue_size=10)

       self.cmd_web = Twist()
       self.cmd_local = Twist()
       self.final_cmd = Twist()
       self.last_web_cmd_time = 0.0
       self.last_local_cmd_time = 0.0
       self.switching_time = 5.0
       self.keppinglive = 1.0
       

    
    def cmdWebCallBack(self, twist_msg:Twist):
        self.cmd_web = twist_msg
        self.last_web_cmd_time = rospy.Time.now().to_sec() 
        
        self.final_cmd_publisher.publish(self.cmd_web)       

    def cmdLocalCallBack(self, twist_msg:Twist):
        self.cmd_local = twist_msg
        self.last_local_cmd_time = rospy.Time.now().to_sec() 
        
        t_now = rospy.Time.now().to_sec() 
        if(abs(t_now - self.last_web_cmd_time) > self.keppinglive):
            if (abs(t_now - self.last_local_cmd_time) > self.keppinglive):
                self.cmd_local.linear.x, self.cmd_local.linear.y  = [0.0, 0.0]               

            self.final_cmd_publisher.publish(self.cmd_local)        


    def getFinalCmd(self):
        t_now = rospy.get_time()
        if(abs(t_now - self.last_web_cmd_time) > self.keppinglive):
            print(f"getFinalCmd 1: {t_now, self.last_web_cmd_time}") 
            if (abs(t_now - self.last_local_cmd_time) > self.keppinglive):
                self.cmd_local.linear.x, self.cmd_local.linear.y  = [0.0, 0.0]
                print(f"getFinalCmd 2: {t_now, self.last_local_cmd_time}") 

            self.final_cmd =self.cmd_local
        else:
            print(f"getFinalCmd else 1: {t_now, self.last_local_cmd_time}") 
            self.final_cmd = self.cmd_web
           
        
        print(f"getFinalCmd: {self.final_cmd.linear.x , self.final_cmd.angular.z }")
        self.final_cmd_publisher.publish (self.final_cmd)
        

if __name__ == '__main__':
    rospy.init_node("multiplexer")
    print(f"multiplexer Node Initialized!")
    multiplexer = Multiplexer()
    rospy.spin()
