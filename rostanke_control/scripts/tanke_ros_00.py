#!/usr/bin/env python3

import rospy
#from robots_control.srv import *
from robots_msg.msg import tanke_msg
from robots_msg.msg import cmd_robot_msg
#from sensor_msgs.msg import Joy
import socket
import select
import time



class robot_tanke:
    def __init__(self):
        # init ros
        rospy.init_node('tanke_node_00', anonymous=False)
        rospy.Subscriber('cmd_tanke_msg', cmd_robot_msg, self.send_cmd_to_esp8266)
        self.get_string_from_esp8266()
   
    
    def send_cmd_to_esp8266(self, data):
        message=bytearray([data.id,data.instruction,data.op1,data.op2,data.op3,data.op4,data.op5,data.id])
        sock.sendto(message, (UDP_IP, UDP_PORT))
        #time.sleep(0.05)


    
    def get_string_from_esp8266(self):
        global UDP_IP,UDP_PORT,id_robot
        r = rospy.Rate(15)
        pub = rospy.Publisher('pub_tanke_msg', tanke_msg, queue_size=1)
        r.sleep()
        msg = tanke_msg()
        while not rospy.is_shutdown():
            sock.setblocking(0)
            sock.connect((UDP_IP,UDP_PORT))
            ready=select.select([sock],[],[],0.05)
            if ready[0]:		
                data, addr = sock.recvfrom(1024)
                data = data.decode('ASCII')
                dat = data.split(',')         
                if dat[0] =='TANKE':
                    msg.type = dat[0]
                    msg.ip = dat[1]
                    msg.port = int(dat[2])
                    msg.id = int(dat[3])
                    msg.inst_before = int(dat[4])
                    msg.battery = float(dat[5])
                    msg.motor1.speed = int(dat[6])
                    msg.motor1.dir = int(dat[7])
                    msg.motor1.pos = 0     # could'nt read pos is a servo
                    msg.motor2.speed = int(dat[8])
                    msg.motor2.dir = int(dat[9])
                    msg.motor2.pos = 0     # could'nt read pos is a servo
                    msg.motor3.speed = int(dat[10])
                    msg.motor3.dir = int(dat[11])
                    msg.motor3.pos = 0     # could'nt read pos is a servo
                    msg.motor4.speed = int(dat[12])
                    msg.motor4.dir = int(dat[13])
                    msg.motor4.pos = 0     # could'nt read pos is a servo
                    msg.plat.action = dat[14]
                    msg.plat.up = bool(dat[15])
                    msg.plat.down = bool(dat[16])
                    msg.camera.type = dat[17]
                    msg.camera.id = int(dat[18])
                    msg.camera.targx = int(dat[19])
                    msg.camera.targy = int(dat[20])
                    msg.camera.targw = dat[21]
                    msg.camera.targh = int(dat[22])
                    msg.sensor_lidar.position = int(dat[23])
                    msg.sensor_lidar.range = int(dat[24])
                    msg.status=dat[25]
                    pub.publish(msg)
                    rospy.loginfo(msg.battery)
                    #r.sleep()
        
            
            
        
    
    

if __name__ == '__main__':
    UDP_IP = "192.168.0.108"   #esp8266 ip and port
    UDP_PORT = 8888
    id_robot=254
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM) 
    message=bytearray([id_robot,21,0,0,0,0,0,id_robot]) # to open the channell
    sock.sendto(message, (UDP_IP, UDP_PORT))
    try:
        robot_tanke()
    except rospy.ROSInterruptException: 
        pass
