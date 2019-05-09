'''                        roadTRIP v1.0                   '''
#########################      #####     #############Rey#####
##############################################################
'''


                                _..-------++._
                             _.-'/ |      _||  \"--._
                       __.--'`._/_\j_____/_||___\    `----.
                  _.--'_____    |  Rey     \     _____    /
                _j    /,---.\   |        =o |   /,---.\   |_
               [__]==// .-. \\==`===========/==// .-. \\=[__]
                 `-._|\ `-' /|___\_________/___|\ `-' /|_.'     
                       `---'                     `---'


'''
################################Working#######################
#Camera..................Line Detection#######################
#SuperSonicSensor........DistanceDetection####################
#L293d...................Navigation###########################
##############################################################




#Libraries
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import math

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
GPIO_TRIGGER = 6
GPIO_ECHO = 5
GPIO_left_t = 24
GPIO_right_t = 23
GPIO_front_t = 17
GPIO_backward_t = 27
#set variables
frame = cv2.VideoCapture(0) 
theta=0
minLineLength = 5
maxLineGap = 10
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_right_t, GPIO.OUT) 
GPIO.setup(GPIO_left_t, GPIO.OUT) 
GPIO.setup(GPIO_front_t, GPIO.OUT) 
GPIO.setup(GPIO_backward_t, GPIO.OUT)

#left/right/forward/backward turn
def right():
    GPIO.output(GPIO_right_t, GPIO.HIGH)
    GPIO.output(GPIO_left_t, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(GPIO_right_t, GPIO.LOW)
    
def front():
    GPIO.output(GPIO_front_t, GPIO.HIGH)
    GPIO.output(GPIO_backward_t, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(GPIO_front_t, GPIO.LOW)
    
def back():
    GPIO.output(GPIO_backward_t, GPIO.HIGH)
    GPIO.output(GPIO_front_t, GPIO.LOW)
    time.sleep(0.2)
    GPIO.output(GPIO_backward_t, GPIO.LOW)

def left():
    GPIO.output(GPIO_left_t, GPIO.HIGH)
    GPIO.output(GPIO_right_t, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(GPIO_left_t, GPIO.LOW)
    
#move left and right
def turn_left():
    left()
    left()
    front()
    right()

def turn_right():
    right()
    right()
    front()
    left()

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
'''

if __name__ == '__main__':
    try:
        while True:
            
            ret,image = frame.read()
            time.sleep(0)
    
    #for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #image = frame.array
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edged = cv2.Canny(blurred, 85, 85)
            lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
            if(lines !=None):
                for x in range(0, len(lines)):
                    for x1,y1,x2,y2 in lines[x]:
                        cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
                        theta=theta+math.atan2((y2-y1),(x2-x1))
                   
                        
   #print(theta)GPIO pins were connected to Rpi3 for steering steering control
    
        threshold=2
    
        dist = distance()
        print ("Measured Distance = %.1f cm" % dist)
        time.sleep(1)
    
        if(theta>threshold):
            turn_left()
            print("going left")
                
                
        if(theta<-threshold):
            turn_right()
            print("right")
    
        if(abs(theta)<threshold):
            front()
            print ("going straight")
        cv2.imshow("Frame",image)
        theta=0
        cv2.imshow("Frame",image)
        
        
          

    except KeyboardInterrupt:
        print("Car stopped by User")
        GPIO.cleanup()

        
'''   
while True:
    
    
    ret,image = frame.read()
    time.sleep(0)
    
#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edged = cv2.Canny(blurred, 85, 85)
    lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
    if(lines !=None):
        for x in range(0, len(lines)):
            for x1,y1,x2,y2 in lines[x]:
                cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
                theta=theta+math.atan2((y2-y1),(x2-x1))
   #print(theta)GPIO pins were connected to arduino for servo steering control
    
    threshold=2
    
    dist = distance()
    print ("Measured Distance = %.1f cm" % dist)
    time.sleep(1)
    
    if(theta>threshold):
        turn_left()
        print("left")
    if(theta<-threshold):
        turn_right()
        print("right")
    
    if(abs(theta)<threshold):
        
        front()
       #GPIO.output(8,False)
       #GPIO.output(7,False)
        print ("straight")
    
    theta=0
    cv2.imshow("Frame",image)
    key = cv2.waitKey(1) & 0xFF
    #rawCapture.truncate(0)
    if key == ord("q"):
        break
        
GPIO.cleanup()
cv2.destroyAllWindows()
print("Greetings")   
print("do not drink and drive")
print("Be Cool")
    
