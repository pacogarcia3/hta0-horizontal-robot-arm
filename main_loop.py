import cv2
import numpy as np
import time
import os
import camera_realworldxyz
from picamera.array import PiRGBArray
from picamera import PiCamera
import commands_arduino





class main_loop:

    def __init__(self, serialport='/dev/ttyACM1',x_arm_adj=0,y_arm_adj=0):

        #Calibrate Camera XY vs. Arm coordinates XY
        #x_camera=x_arm    y_camera=y_arm
        self.x_arm_adj=2.62 #cam_x Zero Position vs. arm X zero position
        self.y_arm_adj=8.85 #cam_y Zero Position vs. arm Y zero positoin

        #initialize detection
        self.cameraXYZ=camera_realworldxyz.camera_realtimeXYZ()

        #initiate arduino
        #serialport='/dev/ttyACM1',LOW_Z=-20.5, HOVER_Z=-16.5,DROP_Z=-18.5, MID_Z=-15, HIGH_Z=-6
        self.arm_c=commands_arduino.arm_controller(serialport,-20.5,-16.5,-18.5,-15,-6)


    def truncate(self, n, decimals=0):
        #this function will help round numbers to prepare them to pass into arduino
        n=float(n)
        multiplier = 10 ** decimals
        return int(n * multiplier) / multiplier

    def capturefromPiCamera(self, imgdir,prefix="Cap",fullscreen=False,detectXYZ=True, calcXYZ=True, arm=True):

            #initiate counters
            img_counter=0
            bg_counter=0
            id_counter=0
            
            #initiate flags
            bg_capture=False
            move_object=False
            arm_move=False

            #initiate objects detected
            obj_detected=0
            obj_detected_prev=0

            #move arm to clear the camera vision
            if (arm==True): self.arm_c.move_clearcamera()


            #initiate camera and window
            camera = PiCamera()
            camera.resolution = (1280, 720)
            camera.framerate = 20
            rawCapture = PiRGBArray(camera, size=(1280, 720))
            time.sleep(0.2)
            win_name="Capture"       
            if (fullscreen==True):
                    cv2.namedWindow(win_name, cv2.WND_PROP_FULLSCREEN)
                    cv2.setWindowProperty(win_name,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
            else:
                    cv2.namedWindow(win_name)



            # capture frames from the camera
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

                    #listen for keystrokes
                    k = cv2.waitKey(1)
                    
                    image = frame.array
                    #if your image is flipped, used this command
                    #image=cv2.flip(image_cap, -1 )

                    # capture background
                    if bg_capture==False:
                            bg_counter+=1
                            #letting counter reach 52 provides warm up for camera, then capture background
                            if bg_counter==52:
                                self.cameraXYZ.load_background(image)
                                bg_capture=True
                                print("background captured")
                            else:
                                cv2.putText(image,"Warming Up",(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),2)

                    # with background captured, image detection will now work
                    if bg_capture==True:
                            #for the camera to detect the same number of objects for 20 frames, before triggering arm
                            if id_counter>20:
                                    cv2.putText(image,"Picking",(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                                    print("Trigger Arm")
                                    #arm flag ensures movement can happen
                                    self.pickanddrop(XYZ,arm)
                                    id_counter=0
                            #run detection when on
                            if detectXYZ==True:
                                    obj_detected_prev=obj_detected
                                    image, XYZ = self.cameraXYZ.detect_xyz(image,calcXYZ)
                                    obj_detected=len(XYZ)
                                    #start counter when same number of objects detected between frames
                                    if obj_detected>0 and obj_detected==obj_detected_prev:
                                            id_counter+=1
                                            cv2.putText(image,"Object Detected",(20,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)
                         
                    if k%256 == 27:
                            # ESC pressed
                            print("Exiting...")
                            break
                    elif k%256 == 32:
                            # SPACE pressed Captures an Image
                            img_name = imgdir + prefix + "_{}.jpg".format(img_counter)
                            cv2.imwrite(img_name, image)
                            print(prefix + "_{} written!".format(img_counter))
                            img_counter += 1


                    # show the frame
                    cv2.imshow(win_name, image)

                    # clear the stream in preparation for the next frame
                    rawCapture.truncate(0)

            if (arm==True): self.arm_c.move_home()

            cv2.destroyAllWindows()

            
    def pickanddrop(self, XYZ, arm=True):
        #set drop position
        arm_x_dest=40
        arm_y_dest=0

        for i in range(0,len(XYZ)):

            cam_x=self.truncate(XYZ[i][0],2) #camera X        
            cam_y=self.truncate(XYZ[i][1],2) #camera Y

            print("Grab Item "+str(i+1)+" at x: "+str(cam_x)+", y: "+str(cam_y))

            #adjust the reference of Camera vs. Arm
            arm_x=self.truncate(cam_x-self.x_arm_adj,1)
            arm_y=self.truncate(self.y_arm_adj-cam_y,1)
                    
            if (arm==True): self.arm_c.move_and_pickup(arm_x,arm_y)        
            if (arm==True): self.arm_c.transport_and_drop(arm_x_dest,arm_y_dest)

        if (arm==True): self.arm_c.move_clearcamera()

    def test_arm(self):
        self.arm_c.test_arm()

    def test_arm_XYZ(self,x,y,z):
        self.arm_c.move_toXYZ(x,y,z)
        
    def test_arm_clearcamera(self):
        self.arm_c.move_clearcamera()
        
    def test_arm_home(self):
        self.arm_c.move_home()
        
    def test_arm_home_plane(self):
        self.arm_c.move_home_plane()
