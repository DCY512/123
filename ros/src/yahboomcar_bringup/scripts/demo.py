#!/usr/bin/env python3
# coding: utf-8
import rospy
import Speech_Lib
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
from Rosmaster_Lib import Rosmaster

car = Rosmaster()
spe = Speech_Lib.Speech()
rospy.init_node("driver_node", anonymous=False)
sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=100)
#sub_RGBLight = rospy.Subscriber("RGBLight", Int32, self.RGBLightcallback, queue_size=100)
#sub_Buzzer = rospy.Subscriber("Buzzer", Bool, self.Buzzercallback, queue_size=100)
sub_JoyState = rospy.Subscriber('/JoyState', Bool, self.JoyStateCallback)

def cmd_vel_callback(self, msg):
    # С���˶����ƣ������߻ص�����
    # Car motion control, subscriber callback function
    if not isinstance(msg, Twist): return
    # �·����ٶȺͽ��ٶ�
    # Issue linear vel and angular vel
    vx = msg.linear.x
    vy = msg.linear.y
    angular = msg.angular.z
    # С���˶�����,vel: ��1, angular: ��5
    # Trolley motion control,vesl=[-1, 1], angular=[-5, 5]
    # rospy.loginfo("cmd_velx: {}, cmd_vely: {}, cmd_ang: {}".format(vx, vy, angular))
    car.set_car_motion(vx, vy, angular)
def JoyStateCallback(self, msg):
    if not isinstance(msg, Bool): return
    Joy_active = msg.data
    pub_cmdVel.publish(Twist())

try:
    while True:
        speech_r = spe.speech_read()

        #print(speech_r)
        '''if speech_r != 999:
            print(speech_r)
            time.sleep(0.1)
            spe.void_write(1)'''
        if speech_r == 2 :
            vx = 0.0
            vy = 0.0
            angular = 0
            car.set_car_motion(vx, vy, angular)
            spe.void_write(speech_r)
        # ǰ��
        elif speech_r == 4 :
            vx = 0.5
            vy = 0.0
            angular = 0
            car.set_car_motion(vx, vy, angular)
            spe.void_write(speech_r)
        # ����
        elif speech_r == 5 :
            vx = -0.5
            vy = 0.0
            angular = 0
            car.set_car_motion(vx, vy, angular)
            spe.void_write(speech_r)
        # ��ת
        elif speech_r == 6 :
            vx = 0.2
            vy = 0.0
            angular = 0.5
            car.set_car_motion(vx, vy, angular)
            spe.void_write(speech_r)
        # ��ת    
        elif speech_r == 7 :
            vx = 0.2
            vy = 0.0
            angular = -0.5
            car.set_car_motion(vx, vy, angular) 
            spe.void_write(speech_r)
        # ����  
        elif speech_r == 8 :
            vx = 0.0
            vy = 0.0
            angular = 0.5
            car.set_car_motion(vx, vy, angular)
            spe.void_write(speech_r)
        # ����
        elif speech_r == 9 :
            vx = 0.0
            vy = 0.0
            angular = -0.5
            car.set_car_motion(vx, vy, angular)
            spe.void_write(speech_r)
        elif speech_r == 11 :
        	car.set_colorful_lamps(0xFF,255,0,0)
        	spe.void_write(speech_r) 
        	print("red")
        elif speech_r == 17 :
        	car.set_colorful_effect(3, 6, parm=1)
        	spe.void_write(speech_r)
        	print("qq")
        elif speech_r == 10 :
            car.set_colorful_effect(0, 6, parm=1)
            spe.void_write(speech_r)
        # ���̵�
        elif speech_r == 12 :
            car.set_colorful_lamps(0xFF,0,255,0)
            spe.void_write(speech_r)
        # ������
        elif speech_r == 13 :
            car.set_colorful_lamps(0xFF,0,0,255)
            spe.void_write(speech_r)
        # ���Ƶ�
        elif speech_r == 14 :
            car.set_colorful_lamps(0xFF,255,255,0)
            spe.void_write(speech_r)
        # ����ˮ��
        elif speech_r == 15 :
            car.set_colorful_effect(1, 6, parm=1)
            spe.void_write(speech_r)
        # �򿪽����
        elif speech_r == 16 :
            car.set_colorful_effect(4, 6, parm=1)
            spe.void_write(speech_r)
        # �򿪺�����
        elif speech_r == 17 :
            car.set_colorful_effect(3, 6, parm=1)
            spe.void_write(speech_r)
        # ��ʾ����
        elif speech_r == 18 :
            car.set_colorful_effect(1, 6, parm=1)
            spe.void_write(speech_r)
        # ����ģ�����
        elif speech_r == 32 :
            spe.void_write(speech_r)

        elif speech_r == 33 :
            spe.void_write(speech_r)

        elif speech_r == 34 :
            spe.void_write(speech_r)

        elif speech_r == 35 :
            spe.void_write(speech_r)

        elif speech_r == 36 :
            spe.void_write(speech_r)

        elif speech_r == 37 :
            spe.void_write(speech_r)


except KeyboardInterrupt:
    print("Close!")
