import numpy as np
import cv2
import image_recognition_singlecam

class camera_realtimeXYZ:

    #camera variables
    cam_mtx=None
    dist=None
    newcam_mtx=None
    roi=None
    rvec1=None
    tvec1=None
    R_mtx=None
    Rt=None
    P_mtx=None

    #images
    img=None


    def __init__(self):

        imgdir="/home/pi/Desktop/Captures/"
        savedir="camera_data/"
        self.imageRec=image_recognition_singlecam.image_recognition(False,False,imgdir,imgdir,False,True,False)

        #self.imageRec=image_recognition_singlecam.image_recognition(True,False,imgdir,imgdir,True,True)

        self.cam_mtx=np.load(savedir+'cam_mtx.npy')
        self.dist=np.load(savedir+'dist.npy')
        self.newcam_mtx=np.load(savedir+'newcam_mtx.npy')
        self.roi=np.load(savedir+'roi.npy')
        self.rvec1=np.load(savedir+'rvec1.npy')
        self.tvec1=np.load(savedir+'tvec1.npy')
        self.R_mtx=np.load(savedir+'R_mtx.npy')
        self.Rt=np.load(savedir+'Rt.npy')
        self.P_mtx=np.load(savedir+'P_mtx.npy')

        s_arr=np.load(savedir+'s_arr.npy')
        self.scalingfactor=s_arr[0]

        self.inverse_newcam_mtx = np.linalg.inv(self.newcam_mtx)
        self.inverse_R_mtx = np.linalg.inv(self.R_mtx)
    
    def previewImage(self, text, img):
        #show full screen
        cv2.namedWindow(text, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(text,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

        cv2.imshow(text,img)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

    def undistort_image(self,image):
        image_undst = cv2.undistort(image, self.cam_mtx, self.dist, None, self.newcam_mtx)

        return image_undst

    def load_background(self,background):
        self.bg_undst=self.undistort_image(background)
        self.bg=background

    def detect_xyz(self,image,calcXYZ=True,calcarea=False):

        image_src=image.copy()
        
        #if calcXYZ==True:
        #    img= self.undistort_image(image_src)
        #    bg = self.bg_undst
        #else:
        img=image_src
        bg=self.bg
                    
        XYZ=[]
        #self.previewImage("capture image",img_undst)
        #self.previewImage("bg image",self.bg_undst)
        obj_count, detected_points, img_output=self.imageRec.run_detection(img,self.bg)

        if (obj_count>0):

            for i in range(0,obj_count):
                x=detected_points[i][0]
                y=detected_points[i][1]
                w=detected_points[i][2]
                h=detected_points[i][3]
                cx=detected_points[i][4]
                cy=detected_points[i][5]

                cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                
                #draw center
                cv2.circle(img,(cx,cy),3,(0,255,0),2)

                
                cv2.putText(img,"cx,cy: "+str(self.truncate(cx,2))+","+str(self.truncate(cy,2)),(x,y+h+28),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                if calcXYZ==True:
                    XYZ.append(self.calculate_XYZ(cx,cy))
                    cv2.putText(img,"X,Y: "+str(self.truncate(XYZ[i][0],2))+","+str(self.truncate(XYZ[i][1],2)),(x,y+h+14),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                if calcarea==True:
                    cv2.putText(img,"area: "+str(self.truncate(w*h,2)),(x,y-12),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

        return img, XYZ

    def calculate_XYZ(self,u,v):
                                      
        #Solve: From Image Pixels, find World Points

        uv_1=np.array([[u,v,1]], dtype=np.float32)
        uv_1=uv_1.T
        suv_1=self.scalingfactor*uv_1
        xyz_c=self.inverse_newcam_mtx.dot(suv_1)
        xyz_c=xyz_c-self.tvec1
        XYZ=self.inverse_R_mtx.dot(xyz_c)

        return XYZ


    def truncate(self, n, decimals=0):
        n=float(n)
        multiplier = 10 ** decimals
        return int(n * multiplier) / multiplier
