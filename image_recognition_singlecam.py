import cv2
import numpy as np
import time


class image_recognition:
    
    def __init__(self,print_status=True, write_images=False,
                 image_Path="/home/pi/Desktop/Captures/",testing_Path="/home/pi/Desktop/Captures/",
                 preview_images=False,preview_autoclose=True,print_img_labels=True):
        
        self.IMGDIR=image_Path
        self.TESTDIR=testing_Path
        self.PREVIEW_IMAGES=preview_images
        self.PREVIEW_AUTOCLOSE=preview_autoclose
        self.PRINT_STATUS=print_status
        self.PRINT_IMG_LABELS=print_img_labels
        self.WRITE_IMAGES=write_images

        #valid contour parameters limits (in pixels)
        self.MIN_AREA=900 #30x30
        self.MAX_AREA=90000 #300x300
            #aspect ratio width/height
        self.MIN_ASPECTRATIO=1/5
        self.MAX_ASPECTRATIO=5

        #OstuSensitivity
        self.OtsuSensitivity=22

    def test_objectDetect(self,bgFile,targetFile):

        img=cv2.imread(self.TESTDIR+bgFile+".jpg")
        bg=cv2.imread(self.TESTDIR+targetFile+".jpg")

        self.run_detection(img,bg,True)

    def run_detection(self,img,bg,testRun=False):
           
        obj_count, contours_detected, contours_validindex=self.detectObjects(img,bg)

        obj_count, detected_points, img_output=self.detectionOutput(img,obj_count,contours_validindex,contours_detected)
       
        return obj_count, detected_points, img_output
    
    def detectObjects(self, image, bg_img,externalContours=True):

        img=image.copy()           
        background_img=bg_img.copy()


        # Process Image Difference
        diff=self.calculateDifference_Otsu(img,background_img)

        # ///////////// Find the Contours
        # use RETR_EXTERNAL for only outer contours... use RETR_TREE for all the hierarchy
        if externalContours==True:
            contours_detected, hierarchy = cv2.findContours(diff, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        else:
            contours_detected, hierarchy = cv2.findContours(diff, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        #calculate key variables
        height, width, channels = img.shape

        # /////// identify the VALID Contours
        contours_validindex= self.identify_validcontours(contours_detected,height,width)
        obj_count=len(contours_validindex)
        self.printStatus("valid contours "+ str(obj_count))

        return obj_count, contours_detected, contours_validindex

    def detectionOutput(self, image, obj_count, validcontours, diff_contours):

        img_output=image.copy()

        detected_points=[]

        if (len(validcontours)>0):
            for i in range(0,len(validcontours)):
                cnt=diff_contours[validcontours[i]]

                #get rectangle detected_points
                x,y,w,h=cv2.boundingRect(cnt)
                
                #get centroid
                M=cv2.moments(cnt)
                cx=int(M['m10']/M['m00'])
                cy=int(M['m01']/M['m00'])
                
                self.printStatus("point number "+str(i))
                self.printStatus(str(cx)+", "+str(y))
                self.printStatus("x: "+str(x)+" y: "+str(y)+" w: "+str(w)+" h: "+str(h))

                #draw retangle
                cv2.rectangle(img_output,(x,y),(x+w,y+h),(0,255,0),2)
                #draw center
                cv2.circle(img_output,(cx,cy),3,(0,255,0),2)

                if self.PRINT_IMG_LABELS ==True:
                    
                    #image,text,font,bottomleftconrner,fontscale,fontcolor,linetype
                    cv2.putText(img_output,"Point "+str(i),(x-w,y+h),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
                    cv2.putText(img_output,"cx,cy: "+str(self.truncate(cx,2))+","+str(self.truncate(cy,2)),(x-w,y+h+9),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)

                points=[x,y,w,h,cx,cy]
                detected_points.append(points)

        if (obj_count>1 or len(validcontours)==0):               
            self.previewImage("Multiple Objects Detected",img_output)
            one_object=False
        else:
            self.previewImage("One Objects Detected",img_output)
            one_object=True


        return obj_count, detected_points, img_output

    def truncate(self, n, decimals=0):
        n=float(n)
        multiplier = 10 ** decimals
        return int(n * multiplier) / multiplier

    def writeImage(self,filename,image,testdir=False):
        if self.WRITE_IMAGES==True:
            if testdir==False:
                cv2.imwrite(self.TESTDIR+filename,image)
            else:
                cv2.imwrite(self.IMGDIR+filename,image)

                
    def readImage(self,imgfile):

        img=cv2.imread(imgfile)

        return img


                
    def printStatus(self,text):

        if self.PRINT_STATUS==True:
            print(text)
            
    
    def previewImage(self, text, img):
        if self.PREVIEW_IMAGES==True:
            #show full screen
            cv2.namedWindow(text, cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(text,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

            cv2.imshow(text,img)
            if self.PREVIEW_AUTOCLOSE==True:
                cv2.waitKey(2000)
                cv2.destroyAllWindows()
    
            else:
                cv2.waitKey(0)
                cv2.destroyAllWindows()

    def calculateDifference_method1(self,img,background_img):
        
        # Object Recognition Tresholds
        diff_low_t=45
        diff_high_t=255

        self.previewImage("Original Image [Diff method1]",img)

       # In this approach, we are doing Gray>Difference>Blur>Treshold>Blur.

        # Background - Gray
        background_img_gray=cv2.cvtColor(background_img, cv2.COLOR_BGR2GRAY)
        self.previewImage("1 Background Gray",background_img_gray)

        # Image - Gray
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.previewImage("2 Image Gray",img_gray)
         
        # Calculate Difference
        diff_gray=cv2.absdiff(background_img_gray,img_gray)
        self.previewImage("3 Pre-Diff",diff_gray)

        # Diff Blur
        diff_gray_blur = cv2.GaussianBlur(diff_gray,(5,5),0)
        self.previewImage("4 Pre-Diff Blur",diff_gray_blur)

        #========= Threshold :: this is a manual calibratin point in this approach
        ret,diff_tresh = cv2.threshold(diff_gray_blur,diff_low_t,diff_high_t,cv2.THRESH_BINARY)
        self.previewImage("5 Image Treshold",diff_tresh)

        #Treshold Blur
        diff = cv2.GaussianBlur(diff_tresh,(5,5),0)
        self.previewImage("6 Image Treshold",diff)
        

        return diff

    def calculateDifference_method2(self,img,background_img):
        
        # Object Recognition Tresholds
        bg_low_t=0
        bg_high_t=255
        img_low_t=120
        img_high_t=255

        self.previewImage("Original Image [Diff method2]",img)

        # In this approach, we are doing Gray>Blur>Treshold>Difference.


        # Background - Gray
        background_img_gray=cv2.cvtColor(background_img, cv2.COLOR_BGR2GRAY)
        self.previewImage("1 Background Gray",background_img_gray)

        # Background - Blur
        background_img_blur = cv2.GaussianBlur(background_img_gray,(5,5),0)
        self.previewImage("2 Background Blur Gray",background_img_blur)       

        # Background - Treshold
        ret,background_img_tresh = cv2.threshold(background_img_blur,bg_low_t,bg_high_t,cv2.THRESH_BINARY_INV)
        self.previewImage("3 Background Treshold",background_img_tresh)

        # Image - Gray
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.previewImage("4 Image Gray",img_gray)

        # Image - Blur
        img_blur = cv2.GaussianBlur(img_gray,(5,5),0)
        self.previewImage("5 Image Blur Gray",img_blur,testingPreviews)
        
        # Image - Treshold
        #========= Threshold :: this is a manual calibratin point in this approach
        ret,img_tresh = cv2.threshold(img_blur,img_low_t,img_high_t,cv2.THRESH_BINARY_INV)
        self.previewImage("6 Image Treshold",img_tresh)

        # Calculate Difference
        diff=cv2.absdiff(background_img_tresh,img_tresh)
        self.previewImage("7 Diff",diff,testingPreviews)    

        return diff


    def calculateDifference_Otsu(self,img,background_img):
        
        # Object Recognition Tresholds
        diff_low_t=45
        diff_high_t=255


        self.previewImage("Original Image [Diff Otsu]",img)

        # Background - Gray
        background_img_gray=cv2.cvtColor(background_img, cv2.COLOR_BGR2GRAY)
        self.previewImage("1 Background Gray",background_img_gray)

        # Image - Gray
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.previewImage("2 Image Gray",img_gray)

        # Calculate Difference
        diff_gray=cv2.absdiff(background_img_gray,img_gray)
        self.previewImage("3 Pre-Diff",diff_gray)

        # Diff Blur
        diff_gray_blur = cv2.GaussianBlur(diff_gray,(5,5),0)
        self.previewImage("4 Pre-Diff Blur",diff_gray_blur)

            #========= Otsu automatically finds the right threhosld, calibration not needed.

        # find otsu's threshold value with OpenCV function
        ret, otsu_tresh = cv2.threshold(diff_gray_blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        self.previewImage("5 Otsu Treshold",otsu_tresh)

        self.printStatus("Otsu defined treshold value is %d" % ret)

        if (ret < self.OtsuSensitivity):
            #discard image
            #make the difference zero by subtracting backdroungs
            diff=cv2.absdiff(background_img_gray,background_img_gray)
        else:       
            #Treshold Blur
            diff = cv2.GaussianBlur(otsu_tresh,(5,5),0)
            self.previewImage("6 Image Treshold",diff)


        return diff


    def identify_validcontours(self,cnt,height,width):

        #helps discard noise on contour detection.
        #this is calibrated for Legos in this example

        contours_validindex=[]
        contour_index=-1

        for i in cnt:

            contour_index=contour_index+1
            ca=cv2.contourArea(i)

            # Calculate W/H Ratio
            x,y,w,h = cv2.boundingRect(i)

            aspect_ratio = float(w)/h

            # Flag as edge_noise if the object is at a Corner

            #height, width, channels = img.shape
            edge_noise=False
            if x==0:
                edge_noise=True
            if y==0:
                edge_noise=True
            if (x+w)==width:
                edge_noise=True
            if (y+h)==height:
                edge_noise=True
                       
            # DISCARD noise with measure if area not within parameters
            if ca>self.MIN_AREA and ca<self.MAX_AREA:

                # DISCARD as noise on ratio
                if aspect_ratio>=self.MIN_ASPECTRATIO and aspect_ratio<=self.MAX_ASPECTRATIO:
  
                    # DISCARD if at the Edge
                    if edge_noise==False:
                         contours_validindex.append(contour_index)

        return contours_validindex

    def square_Crop(self,cnt,crop_img,contour_img,height,width):

        #You need squares to use InceptionV3 :)

        x,y,w,h = cv2.boundingRect(cnt)

        #print(x,y,w,h)

        # EXPAND THE CONTOUR
        adjust=0.15
        y=int(y-((h*adjust)/2))
        if y<0:
            y=0
        x=int(x-((w*adjust)/2))
        if x<0:
            x=0
        w=int(w*(1+adjust))
        h=int(h*(1+adjust))

        # CHECK TO SEE IF EXPANDED CONTOUR IS IN BOUNDS
        if y<0: y=0
        if x<0: x=0
        if (x+w)>width: w=width-x
        if (y+h)>height: h=height-y        

        # SQUARE THE CONTOUR
        if w>h:
            #ensure contour is centered
            y=int(y-((w-h)/2))
            if y<0: y=0
            #make a square
            h=w
            if (y+h)>height: y=height-h
        if h>w:
            x=int(x-((h-w)/2))
            if x<0: x=0
            w=h
            if (x+w)>width: x=width-w


        #draw & crop
        crop_img = crop_img[y:y+h, x:x+w]

        cv2.rectangle(contour_img,(x,y),(x+w,y+h),(0,255,0),2)

        return crop_img, contour_img

    def square_rotatedCrop(self,cnt,crop_img,contour_img,height,width):

        #You need squares to use InceptionV3 :)

        #This rectangle will reflect the rotation of the image.
        rect = cv2.minAreaRect(cnt)
        
        img=crop_img.copy()

        r_cx=rect[0][0] #center x
        r_cy=rect[0][1] #centery y
        r_width=rect[1][0]
        r_height=rect[1][1]
    
        #EXPAND THE RECTANGLE ==> CHECK TO SEE IF NOT OUT OF BOUNDS
        adjust=0.15+0.05
        while True:
            #assume True in each iteration
            fits_inbounds=True
            #reduce adjustment each iteration
            adjust=adjust-0.05
            if adjust==0: break

            newW=int(r_width*(1+adjust))
            newH=int(r_height*(1+adjust))
            new_rect=(rect[0],[newW,newH],rect[2])
            nbox=cv2.boxPoints(new_rect)

            for i in range(nbox.shape[0]):
                #x or y smaller than zero
                for j in range (nbox[i].shape[0]):
                    if nbox[i][j]<0:
                        fits_inbounds=False
                #x greater than picture with
                if nbox[i][0]>width:
                    fits_inbounds=False
                #y greater than picture with
                    
                if nbox[i][1]>height:
                    fits_inbounds=False
                    
            if fits_inbounds==True:
                rect=new_rect
                break
            else:
                newW=int(r_width)
                newH=int(r_height)

        #Enable same Orientation when pieces are asymetrical (e.g. long side if image is horizontal)
        if r_height>r_width:
            #flip w, h
            rect=(rect[0],[r_height,r_width],rect[2]+90)

        #Make it a square
        if newW>newH:
            rect=(rect[0],[newW,newW],rect[2])
        if newH>newW:
            rect=(rect[0],[newH,newH],rect[2])

        #Draw the Box       
        boxdraw = cv2.boxPoints(rect)
        boxdraw = np.int0(boxdraw)
        cv2.drawContours(contour_img,[boxdraw],0,(0,0,255),3)

        # rotate img
        angle = rect[2]
        rows, cols = img.shape[0], img.shape[1]
        M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1)
        img_rot = cv2.warpAffine(img, M, (cols, rows))

        # rotate bounding box
        box = cv2.boxPoints(rect)      
        pts = np.int0(cv2.transform(np.array([box]), M))[0]
        pts[pts < 0] = 0

        #re-establish width and height on rotated image
        width, height, channel=img_rot.shape

        x=pts[1][1]
        xw=pts[0][1]
        w=xw-x
        y=pts[1][0]
        yh=pts[2][0]
        h=yh-y

        # CHECK TO SEE IF EXPANDED CONTOUR IS IN BOUNDS
        if y<0: y=0
        if x<0: x=0
        if (xw)>width: w=width-x
        if (yh)>height: h=height-y        

        # SQUARE THE CONTOUR
        if w>h:
            #make a square
            h=w
            if (y+h)>height: y=height-h
        if h>w:
            w=h
            if (x+w)>width: x=width-w

        # Crop the Image
        crop_img = img_rot[x:x+w,
                           y:y+h]
        
        return crop_img,contour_img,r_width,r_height


#imgdir="/home/pi/Desktop/Captures/"

#(self,print_status=True, write_images=False,
#                 image_Path="/home/pi/Desktop/Captures/",testing_Path="/home/pi/Desktop/Captures/",
#                 preview_images=False,preview_autoclose=True):
        
#imageRec=image_recognition(True,True,imgdir,imgdir,True,True)
        
#imageRec.test_objectDetect("undst_bg","undst_cam")
