import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import ctypes
import wiringpi


camera = PiCamera()
camera.resolution = (640, 480) #640,480
camera.framerate = 60

rawCapture = PiRGBArray(camera, size=(640, 480))

time.sleep(0.1)

k=0
left_up =   np.empty(320, dtype=int, order='C')
left_down = np.empty(320, dtype=int, order='C')
right_up =  np.empty(320, dtype=int, order='C')
right_down =np.empty(320, dtype=int, order='C')


wiringpi.wiringPiSetup()
wiringpi.pinMode(25, 1)
wiringpi.pinMode(24, 1)
wiringpi.pinMode(23, 1)
wiringpi.pinMode(22, 1)  


def perspective(frame):
	'''
	x=(25,440,25,350,600,350,600,440)
		
	
	
        
	cv2.line(frame,x[0:2],x[2:4],(255,0,0),2)
	cv2.line(frame,x[2:4],x[4:6],(255,0,0),2)
	cv2.line(frame,x[4:6],x[6:8],(255,0,0),2)
	cv2.line(frame,x[6:8],x[0:2],(255,0,0),2)
	'''
        
	#cv2.line(frame,y[0:2],y[2:4],(0,255,0),2)
	#cv2.line(frame,y[2:4],y[4:6],(0,255,0),2)
	#cv2.line(frame,y[4:6],y[6:8],(0,255,0),2)
	#cv2.line(frame,y[6:8],y[0:2],(0,255,0),2)
	
	x1=np.float32([[25,440],[25,350],[600,350],[600,440]])
	x2=np.float32([[25,frame.shape[0]],[25,0],[600,0],[600,frame.shape[0]]])
	
	
	matrix= cv2.getPerspectiveTransform(x1,x2)
	frame2 = cv2.warpPerspective(frame, matrix, (640,480))#480,550#320,365#
	
	
	return frame2
	
def threshold(frame):	
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	frame1=cv2.inRange(frame, 0,50)#120,255
	frame3=cv2.Canny(frame1, 300, 700)#300,500
	frame4=cv2.add(frame1,frame3)
	#frame4= cv2.cvtColor(frame4, cv2.COLOR_GRAY2RGB)
	return frame4

def histogram(frame):
	left_c=(int)(frame.shape[1]/2)
	right_c=left_c
	
	
	
	for i in range(0,left_c):
		img1=frame[350,i]
		img2=frame[450,i]
		img1=(int)(img1/255)
		img2=(int)(img2/255)
		left_up[i]=img1
		left_down[i]=img2
		#cv2.line(frame,(i,350),(i,420),(0,0,0),1)
		
	for j in range(right_c,frame.shape[1]):
		img3=frame[350,j]
		img4=frame[450,j]
		img3=(int)(img3/255)
		img4=(int)(img4/255)
		right_up[j-right_c]=img3
		right_down[j-right_c]=img4	
		#cv2.line(frame,(j,350),(j,420),(0,0,0),1)
		
	
	return	frame
		

def lane_finder(frame):
	frame= cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
	L1 = np.argmax(left_up, axis=0)
	L2 = np.argmax(left_down, axis=0)
	#print(right_up)
	
	R1 = np.argmax(right_up, axis=0)+320
	R2 = np.argmax(right_down, axis=0)+320
	
	#print("L1 = ",L1)
	#print("L2 = ",L2)
	#print("R1 = ",R1)
	#print("R2 = ",R2)
	#print("Hello")

	C1=(int)((R1-L1)/2 +L1)
	C2=(int)((R2-L2)/2 +L2)
	
	
	
	cv2.line(frame,(L1,350),(R1,350),(0,255,0),2)
	cv2.line(frame,(L2,450),(R2,450),(0,255,0),2)
	cv2.line(frame,(C1,350),(C2,450),(0,255,255),2)
		
	center=314-C2 #137_is_center
		
			
	
	
	
	return frame,center
	
	
def Decision(result,k):
	if k==1:
		wiringpi.digitalWrite(24, 1)
		wiringpi.digitalWrite(23, 1)
		wiringpi.digitalWrite(22, 0)
		print("Stop Signal")
		
	else:
		if (result==0):
			wiringpi.digitalWrite(24, 0)
			wiringpi.digitalWrite(23, 0)
			wiringpi.digitalWrite(22, 0)
			print("Forward")
			
		
		elif(result>0 and result<=10):
			wiringpi.digitalWrite(24, 0)
			wiringpi.digitalWrite(23, 0)
			wiringpi.digitalWrite(22, 0)
			print("Left-1")
			
		elif(result>10 and result<=20):
			wiringpi.digitalWrite(24, 1)
			wiringpi.digitalWrite(23, 0)
			wiringpi.digitalWrite(22, 0)
			print("Left-2")
		    
		
		elif(result<0 and result>=-10):
			wiringpi.digitalWrite(24, 0)
			wiringpi.digitalWrite(23, 1)
			wiringpi.digitalWrite(22, 0)
			print("Right-1")
			
			
		
		elif(result<-10 and result>=-20):
			wiringpi.digitalWrite(24, 0)
			wiringpi.digitalWrite(23, 1)
			wiringpi.digitalWrite(22, 0)
			print("Right-2")
		
		else:
			wiringpi.digitalWrite(24, 1)
			wiringpi.digitalWrite(23, 1)
			wiringpi.digitalWrite(22, 0)
			print("Stop")	
		
	return


def Haar(stop_cascade,test):
	
	k=0
	
	gray = cv2.cvtColor(test, cv2.COLOR_BGR2GRAY)
	stops =stop_cascade.detectMultiScale(gray, 1.3, 5)
	
	for (x,y,w,h) in stops:
		cv2.rectangle(test,(x,y),(x+w,y+h),(255,0,0),2)
		k=1
	return k,test
    
				
	
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
		image1= frame.array
		frame =np.copy(image1)
		test= np.copy(image1)
		
		stop_cascade = cv2.CascadeClassifier("/home/pi/Haar_cascade/Stop Signs/StopSign_HAAR/Stopsign_HAAR_19Stages.xml")
		stop,test= Haar(stop_cascade,test)			
		
		frame1=perspective(frame)
				
		frame2=threshold(frame1)
				
		histogram(frame2)
				
		frame3,result=lane_finder(frame2)
		
		cv2.line(frame1,(0,350),(frame.shape[1],350),(0,255,0),2)
		cv2.line(frame1,(0,440),(frame.shape[1],440),(0,255,0),2)
		
		cv2.putText(test, 'Result = %d' %result, (20,40), cv2.FONT_HERSHEY_COMPLEX, 1,(0,0,255),2)
		
		Decision(result,stop);
		
		cv2.imshow("Frame 1", test )
		cv2.moveWindow("Frame 1",1000,50)
		
		frame1=cv2.resize(frame1,(640,480))
		cv2.imshow("Frame 2", frame1)
		cv2.moveWindow("Frame 2",1000,550)	
		
		frame3=cv2.resize(frame3,(640,480))
		cv2.imshow("Frame 3", frame3 )
		cv2.moveWindow("Frame 3",200,550)
		key = cv2.waitKey(1) & 0xFF   # clear the stream in preparation for the next frame
		rawCapture.truncate(0)
		
		if key == ord("q"): # if the `q` key was pressed, break from the loop
			break

cap.release()
cv2.destroyAllWindows()