import pyrealsense2 as rs
import numpy as np
import cv2
import time
import socket 
import random
#import pyttsx3

#dog connection
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        
host = socket.gethostname()
port = 13255        
s.connect((host, port))

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

#pipeline_wrapper = rs.pipeline_wrapper(pipeline)
#pipeline_profile = config.resolve(pipeline_wrapper)
#dsensor = pipeline_profile.get_device().first_depth_sensor()
#depth_unit=dsensor.get_depth_scale()

dmin=0.42

action='n'
t0= time.clock()
r=2

#spk=pyttsx3.init()
#voices=spk.getProperty('voices')
#v=0
#spk.setProperty('rate',150)
#spk.setProperty('volume',1.0)
#spk.setProperty('voice', 'english_rp+m3')
#spk.say("Hello, I am your new dog.      Do you need help?  I lo ove Inomotic.")
#spk.runAndWait()

try:
    while True:
        frames = pipeline.wait_for_frames()
        dframe = frames.get_depth_frame()
        if not dframe:
           continue
        hit=200
        dlmin=200
        drmin=200
	#central area
        for i in range(160,480):
           for j in range(160,480):
              d=dframe.get_distance(i,j)  #1 seul pixel suffit!!! attention aux mouches
              if d>0 and d<dmin:
                 hit=d
                 print("central")
                 break
           else:
              continue
           break
        #central area top (moins probable/dangereux)
        if hit>=dmin:
           for i in range(160,480):
              for j in range(160):
                 d=dframe.get_distance(i,j)  #1 seul pixel suffit!!! attention aux mouches
                 if d>0 and d<dmin:
                    hit=d
                    print("central top")
                    break
              else:
                 continue
              break
        #left area 
        for i in range(160):
           for j in range(480):
              d=dframe.get_distance(i,j)  #1 seul pixel suffit!!! attention aux mouches
              if d>0:
                 if d<dlmin:
                     dlmin=d
                 if d<dmin:
                     hit=d
                     print("left")
                     break
           else:
              continue
           break
        #right area
        for i in range(480,640):
           for j in range(480):
              d=dframe.get_distance(i,j)  #1 seul pixel suffit!!! attention aux mouches
              if d>0:
                 if d<drmin:
                     drmin=d
                 if d<dmin:
                     hit=d
                     print("right")
                     break
           else:
              continue
           break
        if hit>=dmin:
           if action=='n':
              s.send('f 0.4'.encode())
              print(s.recv(1024))
              action='f'
              print("Forward: distance to obstacle>=",hit)
           elif time.clock() - t0>5:
              
              #r=random.randrange(3)
              r=(r+1)%3
              
              if r==0:
                 s.send('f 0.0'.encode())
                 print("Stop: ",s.recv(1024))
                 time.sleep(0.1)
                 #v=(v+1)%2
                 #if v==0:
                 #   spk.setProperty('voice', 'english_rp+f3')
                 #else:
                 #   spk.setProperty('voice', 'english_rp+m3')

                 #spk.setProperty('voice',voices[v].id)
                 #spk.say("Hello, I am your new dog.      Do you need help?  I lo ove Inomotic.")
                 #spk.runAndWait()
                 s.send('p -0.9'.encode())
		 print("Pitch: ",s.recv(1024))
                 time.sleep(0.1)
                 for boucle in range(3): #j'ai avancé les lignes suivantes pour identation (Denis)
                     s.send('r 0.9'.encode())
                     print("Roll -: ",s.recv(1024))
                     time.sleep(0.8)
                     s.send('r -0.9'.encode())
                     print("Roll +: ",s.recv(1024))
                     time.sleep(0.8)
		 s.send('r 0'.encode())
                 print("Stop roll: ",s.recv(1024))
                 time.sleep(0.1)
                 s.send('p 0'.encode())
                 print("Stop Pitch: ",s.recv(1024))
                 time.sleep(0.4)
                 s.send('f 0.4'.encode())
                 print("Forward: ",s.recv(1024))
              elif r==1:
                 s.send('f 0.0'.encode())
                 print("Stop: ",s.recv(1024))
                 time.sleep(0.1)
	         s.send('y -0.2'.encode())
	         print("Yaw -: ",s.recv(1024))
	         time.sleep(1)
	         s.send('y -0.4'.encode())
	         print("Yaw +: ",s.recv(1024))
	         time.sleep(1)
	         s.send('y -0.2'.encode())
	         print("Yaw +: ",s.recv(1024))
	         time.sleep(1)
                 s.send('y 0.2'.encode())
	         print("Yaw -: ",s.recv(1024))
	         time.sleep(1)
	         s.send('y 0.4'.encode())
	         print("Yaw +: ",s.recv(1024))
	         time.sleep(1)
	         s.send('y 0.2'.encode())
	         print("Yaw +: ",s.recv(1024))
	         time.sleep(1)
                 s.send('y 0'.encode())
                 print("Stop yaw: ",s.recv(1024))
                 time.sleep(0.5)
                 s.send('f 0.4'.encode())
                 print("Forward: ",s.recv(1024))
              else:
                 s.send('f 0.0'.encode())
                 print("Stop: ",s.recv(1024))
                 time.sleep(0.1)
                 for boucle in range(2): #j'ai avancé les lignes suivantes pour identation (Denis)
                     s.send('h -0.8'.encode())
                     print("Down: ",s.recv(1024))
                     time.sleep(0.5)
                     s.send('h 0.8'.encode())
                     print("Up -: ",s.recv(1024))
                     time.sleep(0.5)
                 s.send('h 0'.encode())
                 print("Stop height +: ",s.recv(1024))
                 time.sleep(0.5)
                 s.send('f 0.4'.encode())
                 print("Forward: ",s.recv(1024))
              t0= time.clock()
              
        else:
           print("Danger: {} m".format(hit))
           if dlmin<drmin:
              s.send('f 0.0'.encode())
              print("Stop: ",s.recv(1024))
              time.sleep(1)
              #s.send('s -0.4'.encode())
              #print("Side step: ",s.recv(1024))
              #time.sleep(1)
              #s.send('s 0.0'.encode())
              #print("Stop side: ",s.recv(1024))
              s.send('t -0.35'.encode())
              print("Start right rotation: ",s.recv(1024))
              time.sleep(2)  #50de per second when val=1 => 25deg en 2 sec pour val=0.25
              s.send('t 0.0'.encode())
              print("Stop right rotation: ",s.recv(1024))
              action='n'
              time.sleep(1)
           else:
              s.send('f 0.0'.encode())
              print("Stop: ",s.recv(1024))
              time.sleep(1)
              #s.send('s 0.4'.encode())
              #print("Side step: ",s.recv(1024))
              #time.sleep(1)
              #s.send('s 0.0'.encode())
              #print("Stop side: ",s.recv(1024))
              s.send('t 0.35'.encode())
              print("Start left rotation: ",s.recv(1024))
              time.sleep(2)  
              s.send('t 0.0'.encode())
              print("Stop left rotation: ",s.recv(1024))
              action='n'
              time.sleep(1)

finally:
    # Stop streaming
    pipeline.stop()
    s.send('f 0.0'.encode())
    print("Forward: ",s.recv(1024))
    s.close() 
    #spk.stop()

