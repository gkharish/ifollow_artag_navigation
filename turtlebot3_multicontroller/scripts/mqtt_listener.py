#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist
import json
import sys

class MqttListener:
    def __init__(self) -> None:
        self.client = mqtt.Client()
        self.client.on_message = self.onMessage

        if self.client.connect("localhost", 1883, 60) != 0:
            print("Couldn't connect to MQTT Broker!")
            sys.exit(-1)

        self.client.subscribe('ros_mqtt/cmd_vel')

        self.pub = rospy.Publisher('/cmd_web', Twist, queue_size=10)

        self.listenMqttMsg()

    def onMessage(self, client, userdata, msg):
        #print(f'Msg: {msg.topic}')
        data = json.loads(msg.payload.decode())

        #publish ros msg
        self.mqtt2RosCmdVel(data)

    def mqtt2RosCmdVel(self, msg):
        
        cmd_vel = Twist()
        
        cmd_vel.linear.x = msg["linear"]["x"]
        cmd_vel.linear.y = msg['linear']['y']
        cmd_vel.linear.z = msg['linear']['z']

        cmd_vel.angular.x = msg['angular']['x']
        cmd_vel.angular.y = msg['angular']['y']
        cmd_vel.angular.z = msg['angular']['z']
        
        self.pub.publish(cmd_vel)


    def listenMqttMsg(self):
        try:
            print("Ctr-C to Exit")
            self.client.loop_forever()
        except:
            print(f"Disconnecting!")

        self.client.disconnect()
            
if __name__ == '__main__':
    rospy.init_node("mqtt_listener")
    print(f"mqtt_listener Node initialized!")
    mqtt_listener = MqttListener()

