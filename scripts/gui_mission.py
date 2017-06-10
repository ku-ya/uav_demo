#!/usr/bin/env python
import time
import rospy
from uav_control.msg import trajectory
import pygame
import sys
#from trajectory_tracking_FOR_HADWARE import desired_pos, initialisation
from geometry_msgs.msg import PoseStamped
import numpy as np
import pdb
import roslib
import tf

uav_name = 'Maya'
pygame.init()
black = (0,0,0)
white = (255,255,255)
red = (255,0,0)
clock = pygame.time.Clock()

display_width, display_height = 400, 300
window = pygame.display.set_mode((display_width,display_height))
window.fill(white)
pygame.display.update()

br = tf.TransformBroadcaster()

def cmd_tf_pub(xd):
    br.sendTransform(tuple(xd),tf.transformations.quaternion_from_euler(0,0,0),
            rospy.Time.now(),
            'UAV_des',
            '/world')

def text_objects(text, font):
    textSurface = font.render(text, True, black)
    return textSurface, textSurface.get_rect()

def message_display(text):
    largeText = pygame.font.Font('freesansbold.ttf',40)
    TextSurf, TextRect = text_objects(text, largeText)
    TextRect.center = ((display_width/2),(display_height/2))
    window.blit(TextSurf, TextRect)
    pygame.display.update()

def window_update(msg):
    window.fill(white)
    message_display(msg)

mode = {'spin':['a',15],
        'Simon':['s',0],
        'quit':['q',0],
        'reset':['r',3],
        'quit':['q',0],
        'take off':['t',5],
        'land':['l',5],
        'hover':['h',0],
        'motor':['m',0],
        'warmup':['w',0],
        'kill':['k',0],
        'point to point':['p',15],
        'Est1':['b',18]
        }

mission =  {'mode':'init','t_mission':0,'motor':False,'warmup':False}
z_min = 0.35
z_hover = 1.5
v_up = 0.3
x_v = [0,0,0]
x_ship = [0,0,0]

def mocap_sub(msg):
    global x_v
    x = msg.pose.position
    x_v = np.array([x.x,x.y,x.z])

def mocap_sub_ship(msg):
    global x_ship
    x = msg.pose.position
    x_ship = np.array([x.x,x.y,x.z])

def motor_set(motor, warmup):
    rospy.set_param('/'+uav_name+'/uav/Motor', motor)
    rospy.set_param('/'+uav_name+'/uav/MotorWarmup', warmup)
def get_key():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            motor_set(False,False)
            sys.exit()
        if event.type == pygame.KEYDOWN:
            for mode_c in mode.keys():
               if eval('event.key == pygame.K_{}'.format(mode[mode_c][0])):
                   print('Flight mode: '+mode_c)
                   #window_update(mode_c)
                   mission['mode'] = mode_c
                   mission['t_mission'] = mode[mode_c][1]


pub = rospy.Publisher('xc', trajectory, queue_size= 10)
print('mode: t: takeoff, l: land, h: hover, s: Simon, m: motor, w: warmup, r: reset')
def mission_request():
    global x_v
    get_key()
    dt = 0.01
    t_init = time.time()
    cmd = trajectory()
    cmd.b1 = [1,0,0]
    cmd.header.frame_id = 'uav'
    t_total = mission['t_mission']
    t_cur = 0
    #print('motor: '+str(rospy.get_param('/'+uav_name+'/uav')))
    #print('motor warmup: '+str(rospy.get_param('/'+uav_name+'/uavWarmup')))
    print(mission['mode'])
    if mission['mode'] == 'kill':
        motor_set(False,False)
        print('Stopping motors')
        time.sleep(.1)

    elif mission['mode'] == 'warmup':
        motor_set(True,True)
        time.sleep(2)
        pub.publish(cmd)
        mission['warmup'] = False

    elif mission['mode'] == 'reset':
        rospy.set_param('/'+uav_name+'/uavWarmup', True)
        print('Motor warmup OFF')
        rospy.set_param('/'+uav_name+'/uavWarmup', False)
        print('Motor warmup OFF')
        pub.publish(cmd)

    elif mission['mode'] == 'quit':
        print('terminating mission')
        rospy.set_param('/'+uav_name+'/uav', False)
        rospy.set_param('/'+uav_name+'/uavWarmup', False)
        sys.exit()

    elif mission['mode'] == 'take off':
        print('Motor warmup ON')
        motor_set(True,True)
        rospy.sleep(2)
        motor_set(True,False)
        print('Taking off at {} sec'.format(time.time()-t_init))
        t_init = time.time()
        while t_cur <= t_total and mission['mode'] == 'take off':
            t_cur = time.time() - t_init
            time.sleep(dt)
            cmd.header.stamp = rospy.get_rostime()
            height = z_min+v_up*t_cur
            cmd.xc = [x_v[0],x_v[1],height if height < 1.5 else 1.5 ]
            print(cmd.xc)
            cmd.xc_dot = [0,0,v_up]
            cmd_tf_pub(cmd.xc)
            pub.publish(cmd)
            get_key()
        mission['mode'] = 'wait'
        print('Take off complete')

    elif mission['mode'] == 'land':
        print('Landing')
        z_hover = x_v[2]
        while t_cur <= t_total and mission['mode'] == 'land':
            t_cur = time.time() - t_init
            time.sleep(dt)
            cmd.header.stamp = rospy.get_rostime()
            height = z_hover - v_up*t_cur
            cmd.xc = [x_v[0],x_v[1],height if height > z_min else 0]
            cmd.xc_dot = [0,0,-v_up]
            pub.publish(cmd)
            cmd_tf_pub(cmd.xc)
            print(cmd.xc)
            get_key()
            if x_v[2] < z_min:
                motor_set(False, False)
                break
        mission['mode'] = 'wait'
        print('landing complete')
    elif mission['mode'] == 'spin':
        # TODO
        while t_cur <= t_total:
            t_cur = time.time() - t_init
            time.sleep(dt)
            cmd.header.stamp = rospy.get_rostime()
            theta = 2*np.pi/t_total*t_cur
            cmd.b1 = [np.cos(theta),np.sin(theta),0]
            cmd.xc = [0,0,1.5]
            cmd.xc_dot = [0,0,0]
            pub.publish(cmd)
            get_key()
            if x_v[2] < z_min:
                rospy.set_param('/'+uav_name+'/uav', False)
        mission['mode'] = 'wait'
        print('spin')
        pass
    elif mission['mode'] == 'point to point':
        # TODO
        while t_cur <= t_total and mission['mode'] == 'point to point':
            t_cur = time.time() - t_init
            time.sleep(dt)
            cmd.header.stamp = rospy.get_rostime()
            theta = 2*np.pi/t_total*t_cur
            #cmd.b1 = [np.cos(-np.pi/4*0),np.sin(-np.pi/4*0),0]
            cmd.xc = [0,np.sin(theta)*2,1.5]
            cmd.xc_dot = [0,0,0]
            pub.publish(cmd)
            get_key()
            if x_v[2] < z_min:
                rospy.set_param('/'+uav_name+'/uav', False)
        mission['mode'] = 'wait'
        print('Finish p2p')
        pass

    elif mission['mode'] == 'Est1':
        print('Motor warmup ON')
        motor_set(True,True)
        rospy.sleep(4)
        motor_set(True,False)
        t_init = time.time()
        # TODO
        x0=np.array([0,0,0])
        x1=np.array([0,0,1])
        x2=np.array([1,1,1.5])
        x3=np.array([0,1,1])
        x4=np.array([0,0,1.5])
        x5=np.array([1,1,1])
        t1=4
        dt2=6
        dt3=5
        dt4=4
        dt5=3
        t_total = 22
        while t_cur <= t_total and mission['mode'] == 'Est1':
            t_cur = time.time() - t_init
            time.sleep(dt)
            cmd.header.stamp = rospy.get_rostime()
            cmd.xc_2dot = [0,0,0]
            if t_cur<t1:
                cmd.xc = x0+(x1-x0)*t_cur/t1
                cmd.xc_dot = (x1-x0)/t1
            elif t_cur<(t1+dt2):
                cmd.xc = x1+(x2-x1)*(t_cur-t1)/dt2
                cmd.xc_dot = (x2-x1)/dt2
            elif t_cur<(t1+dt2+dt3):
                cmd.xc = x2+(x3-x2)*(t_cur-t1-dt2)/dt3
                cmd.xc_dot = (x3-x2)/dt3
            elif t_cur<(t1+dt2+dt3+dt4):
                cmd.xc = x3+(x4-x3)*(t_cur-t1-dt2-dt3)/dt4
                cmd.xc_dot = (x4-x3)/dt4
            elif t_cur<=(t1+dt2+dt3+dt4+dt5):
                cmd.xc = x4+(x5-x4)*(t_cur-t1-dt2-dt3-dt4)/dt5
                cmd.xc_dot = (x5-x4)/dt5
            cmd_tf_pub(cmd.xc)
            pub.publish(cmd)
            get_key()
            if x_v[2] < z_min:
                rospy.set_param('/'+uav_name+'/uav', False)
        mission['mode'] = 'wait'
        print('Finish p2p')
        pass


#    elif mission['mode'] == 'Simon':
#        print('Motor warmup ON')
#        motor_set(True,True)
#        rospy.sleep(2)
#        motor_set(True,False)
#        print('Simon')
#        t_init = time.time()
#        x0 = x_v #[0,0,0]
#        dictionnary = initialisation(x0[0],x0[1],x0[2])
#        while mission['mode'] == 'Simon':
#            t_cur = time.time() - t_init
#            time.sleep(dt)
#            cmd.header.stamp = rospy.get_rostime()
#            height = z_hover - v_up*t_cur
#            d_pos = desired_pos(t_cur,x_v,dictionnary, x_ship)
#            #cmd.b1 = d_pos[3]
#            cmd.xc = d_pos[0]
#            cmd.xc_dot = d_pos[1]
#            cmd.xc_2dot = d_pos[2]
#            pub.publish(cmd)
#            get_key()
#            if x_v[2] < z_min and t_cur > 5:
#                motor_set(False, False)
#        mission['mode'] = 'wait'
#        print('Simon mission complete')
#        rospy.set_param('/'+uav_name+'/uav', False)
#    elif mission['mode'] == 'hover':
#        mission['mode'] = 'wait'
#        pass
#
    else:
        pass
        #print('command not found: try again')

if __name__ == '__main__':
    try:
        rospy.init_node('command_station', anonymous=True)
        uav_pose = rospy.Subscriber('/vicon/'+uav_name+'/pose',PoseStamped, mocap_sub)
        ship_pose = rospy.Subscriber('/vicon/ship/pose',PoseStamped, mocap_sub_ship)
        while True:
            mission_request()
    except rospy.ROSInterruptException:
        pass
