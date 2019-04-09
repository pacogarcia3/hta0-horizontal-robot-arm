import main_loop


#initiate variables
serial_port='/dev/ttyACM0'  #'/dev/ttyACM1'
imgdir="/home/pi/Desktop/Captures/"
imgprefix="CapF"

#initiate main loop
loop=main_loop.main_loop(serial_port)


def RunTests():

    #Arm Movement Testing
    #loop.test_arm()
    #loop.test_arm_XYZ(5,5,-9)
    loop.test_arm_home()
    #loop.test_arm_home_plane()
    #loop.test_arm_clearcamera()

def RunPickandPlace():
        #Run Pick and Place
        
        fullscreen=False
        detectXYZ=True
        calculateXYZ=True
        move_arm=True
        loop.capturefromPiCamera(imgdir,imgprefix,fullscreen,detectXYZ,calculateXYZ,move_arm)

def ImageDetection():
        #Work on Image Detection (press ESC when done, Space to capture image)
        loop.test_arm_clearcamera()

        fullscreen=False
        #set detect XYZ to False when you want to use this loop to capture pictures (press spacebar)
        detectXYZ=True
        #set calculateXYZ to enable real world XYZ to be calculated
        calculateXYZ=True
        move_arm=False
        loop.capturefromPiCamera(imgdir,imgprefix,fullscreen,detectXYZ,calculateXYZ,move_arm)

        loop.test_arm_home()



#RunTests()
#RunPickandPlace()
ImageDetection()
