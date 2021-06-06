##############################    tells esp32 to pick and place oranges one by one using object detection model  #############################
import cv2
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
i=0
selected=0
margin=30
#a=100
esp32= serial.Serial(
port='/dev/ttyUSB0',
baudrate=38400,
bytesize = serial.EIGHTBITS,
parity= serial.PARITY_NONE,
stopbits = serial.STOPBITS_ONE,
timeout = 5,
xonxoff = False,
rtscts = False,
dsrdtr = False,
writeTimeout =2
)

ID=0
top=0
left=0
right=0
bottom=0
item=0
center=(0,0)
  
timeStamp=time.time()
fpsFilt=0
net = jetson.inference.detectNet("ssd-mobilenet-v2", ['--model=/home/gaurav/jetson-inference/python/training/detection/ssd/models/gauravModel/ssd-mobilenet.onnx','--labels=/home/gaurav/jetson-inference/python/training/detection/ssd/models/gauravModel/labels.txt','--input-blob=input_0','--output-cvg=scores','--output-bbox=boxes'], threshold=0.5)
cam = jetson.utils.gstCamera(800,600, "/dev/video0")
display = jetson.utils.glDisplay()

def serial_send():
	esp32.write((str(round(width))+"|").encode())
	#print(round(width))
	esp32.write((str(width)+"|").encode())
	if bool_H==True or bool_V==True:
		#print("x= "+ str(center[0]))
		#print("Y= "+ str(center[1]))       
		esp32.write((str(center[0])+"|").encode())
		esp32.write((str(center[1]+800)+"|").encode())
	if bool_V==False and bool_H==False:
		#print("000")
		esp32.write(("0|").encode())

while display.IsOpen():
	timeStamp=time.time()
	img, width, height = cam.CaptureRGBA(zeroCopy = True)
	jetson.utils.cudaDeviceSynchronize()
	image=jetson.utils.cudaToNumpy(img,800,600,4)
	image2=cv2.cvtColor(image, cv2.COLOR_RGBA2BGR).astype(np.uint8)
	detections = net.Detect(img, width, height)
	bool_H=False
	bool_V=False
	i=0
	for detect in detections:
		i=i+1
		ID=detect.ClassID
		item=net.GetClassDesc(ID)
		if (int(ID)==1):
			bool_H=True
			bool_V=True
			top=detect.Top
			left=detect.Left
			right=detect.Right
			bottom=detect.Bottom
			width=round((detect.Width))+1400
			Center=detect.Center
			#center=(round(center[0]*0.5+0.5*round(Center[0])),round(center[1]*0.5+0.5*round(Center[1])))
			center=(round(Center[0]),round(Center[1]))
			list_1.append(center)
			list_3.append(width)
			#print(item,top,left,right,bottom)
			#print(item,"top= "+ str(round(top)),"left= "+str(round(left)),"right=  "+str(round(right)),"bottom"+str(round(bottom)))
			cv2.rectangle(image2,(int(left),int(top)),(int(right),int(bottom)),(0,255,0),2)
	#cv2.line(image2,(400,0),(400,600),(0,255,0),5)
	#print(i) # number of objects detected
	for i in range (len(list_1)):
		#dist=round(np.sqrt(np.square(abs(400-list_1[i][0]))+np.square(abs(300-list_1[i][1]))))
		#list_2.append(dist)
		selected=(list_3.index(max(list_3)))
		center=(list_1[selected])
		width=(list_3[selected])
	#print(center) 
	print(round(width))
	list_1.clear()
	#list_2.clear()
	list_3.clear()
	cv2.imshow('image',image2)
	serial_send() 
	if cv2.waitKey(1)==ord('q'):
		break
	fps=1/(time.time()-timeStamp)
	fpsFilt=0.9*fpsFilt + .1*fps
	#print(int(fps))
	#print("width= "+ str(round(right-left)))
cam.release()
cv2.destroyAllWindows()
