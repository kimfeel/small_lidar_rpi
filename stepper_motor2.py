#!/usr/bin/python

import RPi.GPIO as GPIO
import time 
import rospy
import roslib

#out1 = 13
#out2 = 11
#out3 = 15
#out4 = 16#12

out1 = 16#13
out2 = 11
out3 = 13#15
out4 = 15#16#12

GPIO.setmode(GPIO.BOARD)
GPIO.setup(out1,GPIO.OUT)
GPIO.setup(out2,GPIO.OUT)
GPIO.setup(out3,GPIO.OUT)
GPIO.setup(out4,GPIO.OUT)

print "First calibrate by giving some +ve and -ve values....."

def gpio_out(i):
   i = i % 8
   if i == 0:
      GPIO.output(out1,GPIO.HIGH)
      GPIO.output(out2,GPIO.LOW)
      GPIO.output(out3,GPIO.LOW)
      GPIO.output(out4,GPIO.LOW)
   elif i == 1:
      GPIO.output(out1,GPIO.HIGH)
      GPIO.output(out2,GPIO.HIGH)
      GPIO.output(out3,GPIO.LOW)
      GPIO.output(out4,GPIO.LOW)
   elif i == 2:
      GPIO.output(out1,GPIO.LOW)
      GPIO.output(out2,GPIO.HIGH)
      GPIO.output(out3,GPIO.LOW)
      GPIO.output(out4,GPIO.LOW)
   elif i == 3:
      GPIO.output(out1,GPIO.LOW)
      GPIO.output(out2,GPIO.HIGH)
      GPIO.output(out3,GPIO.HIGH)
      GPIO.output(out4,GPIO.LOW)
   elif i == 4:
      GPIO.output(out1,GPIO.LOW)
      GPIO.output(out2,GPIO.LOW)
      GPIO.output(out3,GPIO.HIGH)
      GPIO.output(out4,GPIO.LOW)
   elif i == 5:
      GPIO.output(out1,GPIO.LOW)
      GPIO.output(out2,GPIO.LOW)
      GPIO.output(out3,GPIO.HIGH)
      GPIO.output(out4,GPIO.HIGH)
   elif i == 6:
      GPIO.output(out1,GPIO.LOW)
      GPIO.output(out2,GPIO.LOW)
      GPIO.output(out3,GPIO.LOW)
      GPIO.output(out4,GPIO.HIGH)
   else:
      GPIO.output(out1,GPIO.HIGH)
      GPIO.output(out2,GPIO.LOW)
      GPIO.output(out3,GPIO.LOW)
      GPIO.output(out4,GPIO.HIGH)
'''
   i = i % 4
   if i == 0:
      GPIO.output(out1,GPIO.HIGH)
      GPIO.output(out2,GPIO.LOW)
      GPIO.output(out3,GPIO.LOW)
      GPIO.output(out4,GPIO.LOW)
   elif i == 1:
      GPIO.output(out1,GPIO.LOW)
      GPIO.output(out2,GPIO.HIGH)
      GPIO.output(out3,GPIO.LOW)
      GPIO.output(out4,GPIO.LOW)
   elif i == 2:
      GPIO.output(out1,GPIO.LOW)
      GPIO.output(out2,GPIO.LOW)
      GPIO.output(out3,GPIO.HIGH)
      GPIO.output(out4,GPIO.LOW)
   elif i == 3:
      GPIO.output(out1,GPIO.LOW)
      GPIO.output(out2,GPIO.LOW)
      GPIO.output(out3,GPIO.LOW)
      GPIO.output(out4,GPIO.HIGH)
'''

rospy.init_node("rotate_motor")
from std_msgs.msg import Int16
publisher = rospy.Publisher('steps',Int16)

try:
   #while(1):
   delay = 0.01
   gpio_out(0)
   time.sleep(5)
   x = -2048#input()
   x = x * -1
   start_time = time.time()

   for y in range(x, 0, -1):
      gpio_out(y)
      time.sleep(delay)
      elapsed_time = time.time() - start_time
      publisher.publish(y)
      #print y
      #time.sleep(0.002)
      
   print elapsed_time
  
except KeyboardInterrupt:
	GPIO.cleanup()


