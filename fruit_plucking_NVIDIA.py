##############################    tells esp32 to find tree and aproach it  #############################
import cv2
import os
import jetson.inference 
import jetson.utils
import time
import numpy as np
import serial

list_1=[] # list of center coordinates 
list_2=[] # list of distance from center
list_3=[] # list of widths 
bool_H=True
bool_V=True
bool_scenario=0 # 0- nothing detected, 1-tree detected, 2-orange detected
#i=0
selected=0  # selected index number of max width
margin=30   # of orange from center 
 # Serial data config starts
esp32= serial.Serial(
port='/dev/ttyUSB0',
baudrate=115200,
bytesize = serial.EIGHTBITS,
parity= serial.PARITY_NONE,stopbits = serial.STOPBITS_ONE,
timeout = 5,
xonxoff = False,
rtscts = False,
dsrdtr = False,
writeTimeout =2
)  # Serial data config ends

tree_found_count=0;
bool_tree_found=False
ID=0
top=0
left=0
right=0
bottom=0
item=0
width=0
width_t=0
center=(0,0)
center_t=(0,0)
  
timeStamp=time.time()
fpsFilt=0
net = jetson.inference.detectNet("ssd-mobilenet-v2", ['--model=/home/gaurav/jetson-inference/python/training/detection/ssd/models/gauravModel/ssd-mobilenet.onnx','--labels=/home/gaurav/jetson-inference/python/training/detection/ssd/models/gauravModel/labels.txt','--input-blob=input_0','--output-cvg=scores','--output-bbox=boxes'], threshold=0.5)
cam = jetson.utils.gstCamera(800,600, "/dev/video0")
display = jetson.utils.glDisplay()

fourcc = cv2.VideoWriter_fourcc(*'MJPG')
rec= cv2.VideoWriter('Fruit_plucking.avi',fourcc,10,(800,600))

def serial_send():
	if (bool_scenario==0):  # means nothing found
		esp32.write(("1|").encode())
		print("1")	
	if (bool_scenario==1):  # means orange found 
		esp32.write((str(round(width))+"|").encode())
		if bool_H==True or bool_V==True:
			print("orange_x= "+ str(center[0]) + "  orange_Y= "+ str(center[1]))
			#print("orange_Y= "+ str(center[1]))       
			esp32.write((str(center[0])+"|").encode())
			esp32.write((str(center[1]+800)+"|").encode())
		if bool_V==False and bool_H==False:  # means orange not detected
			print("0")
			esp32.write(("0|").encode())
	if (bool_scenario==2): # means tree found 
		esp32.write((str(round(width_t))+"|").encode())
		esp32.write((str(center_t[0]+3000)+"|").encode())
		print("tree_width= " + str(round(width_t)) + " tree_center= " + str(center_t[0]))
		#print("tree_center" + str(center[0]))
while display.IsOpen():
	timeStamp=time.time()
	img, width, height = cam.CaptureRGBA(zeroCopy = True)
	jetson.utils.cudaDeviceSynchronize()
	image=jetson.utils.cudaToNumpy(img,800,600,4)
	image2=cv2.cvtColor(image, cv2.COLOR_RGBA2BGR).astype(np.uint8)
	detections = net.Detect(img, width, height)
	bool_H=False
	bool_V=False
	bool_scenario=0
	width=0
	width_t=0
	center=(0,0)
	center_t=(0,0)
	for detect in detections:
		#i=i+1
		ID=detect.ClassID
		item=net.GetClassDesc(ID)
		if (int(ID)==1 and bool_tree_found==True):
			top=detect.Top
			left=detect.Left
			right=detect.Right
			bottom=detect.Bottom
			width=round((detect.Width))+1400
			Center=detect.Center
			center=(round(Center[0]),round(Center[1]))
			#print(center[0],center[1])
			################################### open cv
			blue=image2[center[1],center[0],0]
			green=image2[center[1],center[0],1]
			red=image2[center[1],center[0],2]
			print("red= " + str(red) + "  green= " + str(green) + "  blue= " + str(blue))
			if ((red/green>1.2 or red>green+blue) and (green/blue) >1.0):
				bool_H=True
				bool_V=True
				list_1.append(center)
				list_3.append(width)
				bool_scenario=1
				print("orange//////////////////////////////////////")
				cv2.circle(image2,(center[0],center[1]),10,(0,255,0),2)
				cv2.rectangle(image2,(int(left),int(top)),(int(right),int(bottom)),(0,255,0),2)
			###################################
		if (int(ID)==2 and bool_tree_found==False):
			bool_scenario=2
			top=detect.Top
			left=detect.Left
			right=detect.Right
			bottom=detect.Bottom
			width_t=round((detect.Width))+2200
			Center=detect.Center
			center_t=(round(Center[0]),round(Center[1]))
			cv2.rectangle(image2,(int(left),int(top)),(int(right),int(bottom)),(255,255,255),2)
		if (width_t>2780 and bool_tree_found==False):
			tree_found_count=tree_found_count+1
		if (width_t<2700 and bool_tree_found==False):
			tree_found_count=0 
		if (tree_found_count>30 and bool_tree_found==False):
			bool_tree_found=True
			print("///////////////////////////////////////////////////////////////////////////////////////")
	if (bool_scenario==1):	
		for i in range (len(list_1)):
			dist=round(np.sqrt(np.square(abs(400-list_1[i][0]))+np.square(abs(300-list_1[i][1]))))
			list_2.append(dist)
			selected=(list_3.index(max(list_3)))
			center=(list_1[selected])
			width=(list_3[selected])
	#print(center) 
	#print(round(width))
	list_1.clear()
	list_2.clear()
	list_3.clear()
	cv2.imshow('image',image2)
	cv2.moveWindow('image',0,0)
	serial_send() 
	if cv2.waitKey(1)==ord('q'):
		break
	fps=1/(time.time()-timeStamp)
	fpsFilt=0.9*fpsFilt + .1*fps
	rec.write(image2)
	#print(int(fps))
	#print("width= "+ str(round(right-left)))
cam.release()
rec.release()
cv2.destroyAllWindows()
