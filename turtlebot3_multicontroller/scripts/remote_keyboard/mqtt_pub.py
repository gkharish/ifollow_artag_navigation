#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import sys
import  ros_data_types as ros_data
from read_keyboard import readKeyBoard, key_binding_dict, key_counter_dict, keypress_counters
from threading import Thread
import time

class MqttPublisher():
    def __init__(self) -> None:
        self.client = mqtt.Client()
        if self.client.connect("localhost", 1883, 60) != 0:
            print("Couldn't connect to MQTT Broker!")
            sys.exit(-1)

        self.max_angular_vel = 0.3
        self.step_angular_vel = 0.015

        self.max_linear_vel = 0.5
        self.step_linear_vel = 0.025

        self.pub_freq = 4.0

        self.cmd_vel_dict = ros_data.cmd_vel

        #For dsiplay purpose
        self.prev_linear_seed = 0.0
        self.prev_angular_seed = 0.0
        

    def setCmdVel(self, msg):
        key_linear_speed = keypress_counters['linear']*self.step_linear_vel
        key_angular_speed = keypress_counters['angular']*self.step_angular_vel
        msg["linear"]["x"] = key_linear_speed   #self.clamp(key_linear_speed, -1*self.max_linear_vel, self.max_linear_vel)
        msg['angular']['z'] = key_angular_speed #self.clamp(key_angular_speed, -1*self.max_angular_vel, self.max_angular_vel)
        
        # Display the speeds (Debug, TOBE removed)
        # if(self.prev_linear_seed != key_linear_speed or self.prev_angular_seed != key_angular_speed):
        #     new_line='\n'
        #     #print(f"{new_line} Current linear Speed: {key_linear_speed} and angular speed: {key_angular_speed} {new_line}")
        #     self.prev_linear_seed = key_linear_speed
        #     self.prev_angular_seed = key_angular_speed


    def publishLoop(self):
        print("Ctr-C to Exit")
        while(1):
            self.setCmdVel(self.cmd_vel_dict)
            self.client.publish("ros_mqtt/cmd_vel", json.dumps(self.cmd_vel_dict))
            time.sleep(1/self.pub_freq)
    
    def clamp(self, n, min, max):
        if n < min:
            return min
        elif n > max:
            return max
        else:
            return n
        
if __name__ == '__main__':

    print(f"Hello from mqtt_publisher Node!")

    mqtt_publisher = MqttPublisher()
    read_keyboard = readKeyBoard() 
    
    thread = Thread(target=read_keyboard.getKeyboardActivities)
    
    thread.start()

    mqtt_publisher.publishLoop()

    thread.join()
