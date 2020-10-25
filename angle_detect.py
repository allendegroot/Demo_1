# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from cv2 import aruco
import numpy
import math
import numpy
import math
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import smbus2
lcd_columns = 16
lcd_rows = 2
# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
bus = smbus2.SMBus(1)

#define constants
xwidth = 54 * (math.pi/180) #radians
ywidth = 41 * (math.pi/180) #radians

class angle_detection():
    def __init__(self):
        self.initialize_camera()
        self.reported_angle_rads = "No Aruco Found"
        self.reported_angle_degrees = "No Aruco Found"
        self.aruco_found = "No Aruco Found"
        self.refresh_speed = 300

    
    def initialize_camera(self):
        # initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera()
        self.rawCapture = PiRGBArray(self.camera)
        self.camera.iso = 100
        # Wait for the automatic gain control to settle
        time.sleep(2)
        # Now fix the values
        self.camera.shutter_speed = self.camera.exposure_speed
        self.camera.exposure_mode = 'off'
        g = self.camera.awb_gains
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = g
         
        # allow the camera to warmup
        time.sleep(0.1)
        self.camera.framerate = 32
    
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        
        self.parameters = aruco.DetectorParameters_create()
        
    def search_aruco(self):
        self.reported_angle_rads = "No Aruco Found"
        self.reported_angle_degrees = "No Aruco Found"
        self.aruco_found = "No Aruco Found"
        end_time = time.time() + self.refresh_speed;
        while(time.time() < end_time):
            self.camera.capture(self.rawCapture, format="bgr")
            image = self.rawCapture.array
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(image, self.aruco_dict, parameters=self.parameters)
            if(type(ids) == numpy.ndarray):
                self.aruco_found = "Found Aruco!"
                #Calculate Aruco center and pixel distances to center in x and y directions
                xcenter = int((corners[0][0][0][0]+corners[0][0][2][0])/2)
                ycenter = int((corners[0][0][0][1]+corners[0][0][2][1])/2)
                xdist = abs(xcenter - image.shape[1]/2)
                ydist = abs(ycenter - image.shape[0]/2)

                xangle = (xdist/image.shape[1]) * xwidth
                if (xcenter > 540):
                    xangle *= -1
                yangle = (ydist/image.shape[0]) * ywidth
                
                # Calculate the angle from teh z-axis to the center point
                # First calculate distance (in pixels to screen) on z-axis
                a1 = xdist/math.tan(xangle)
                a2 = ydist/math.tan(yangle)
                a = (a1 + a2)/2
                #Calculate distance in pixels to image center from screen center as a hypotenuse rather than x and y components
                d = math.sqrt(pow(xdist,2) + pow(ydist,2))
                #Calculat the angle to z-axis using these two numbersa nd the arctangent relationship.
                self.reported_angle_rads = math.atan(d/a)
                self.reported_angle_degrees = self.reported_angle_rads * (180/math.pi)
                # Start debug
                print(xangle)
                size = (int(image.shape[1] * .5), int(image.shape[0] * .5))
                lcd.text_direction = lcd.LEFT_TO_RIGHT;
                lcd.message =  "Beacon Detected" + "\n" + str(xangle)
                new_img = cv2.resize(image, size)
                cv2.imshow("Image", image)
                cv2.imshow("Resized", new_img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                newY = str(xangle)
                newX = str(yangle)
                
                # End debug
                break;
            self.rawCapture.truncate(0)
                
                
x = angle_detection()
x.search_aruco()
            
            
            
            