from dynamixel_sdk import * # Uses Dynamixel SDK library
import time
from tkinter import *
import random
VERSION="0.1"
######### Variables to be set by user #########
DEVICENAME = '/dev/ttyUSB0' #set com port to be used
Version = "RIGHT" #Set as "RIGHT" or "LEFT"
###############################################
print("For dynamixel found port:" + str(DEVICENAME))
PROTOCOL_VERSION = 1.0 # See which protocol version is used in the Dynamixel
BAUDRATE = 1000000
portHandler = PortHandler(DEVICENAME)
dynamixel = PacketHandler(PROTOCOL_VERSION)
position=0
if Version=="RIGHT":
    MAINBOARD = 30
    WRIST= [31,32,33]
    FINGERS=[34, 35 ,36 ,37 ,38]
    IDS=[31, 32, 33, 34, 35 ,36 ,37 ,38]
elif Version=="LEFT":
    MAINBOARD = 40
    WRIST= [41,42,43]
    FINGERS=[44, 45 ,46 ,47 ,38]
    IDS=[41, 42, 44, 45 ,46 ,47 , 48]
else:
    print("Error. Invalid type selected! Close to terminate.")
    while(true):
        pass



HEIGHT=400
COMM_SUCCESS = 0 # CommunicationSuccess result value
COMM_TX_FAIL = -1001 # Communication TxFailed
Model = 0
Tuning_Lock = 23
Target_Position = 30
Present_Position = 36
Present_Temperature = 43
Current_mA = 68
Temperature_Offset = 110
SAVE_EEPROM = 114
Target_POT = 104
POT_Limit_CW = 106
POT_Limit_CCW = 108
Palm_IR_Sensor = 94
MIN_POS =0
MAX_POS =4095
Idlist = []
if portHandler.openPort():
    print("Succeeded to open the Dynamixel port")
else:
    print("Failed to open the Dynamixel port " + DEVICENAME)
    print("Press any key to terminate...")
    getch()
    quit()
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
def setpos(ID,position):
    position=dynamixel.write2ByteTxRx(portHandler, ID,Target_Position,position)
def getSlider1(event):
    position=slider1.get()
    setpos(WRIST[0],position)
def getSlider2(event):
    position=slider2.get()
    setpos(WRIST[1],position)
def getSlider3(event):
    position=slider3.get()
    setpos(WRIST[2],position)
def getSlider4(event):
    position=slider4.get()
    setpos(FINGERS[0],position)
def getSlider5(event):
    position=slider5.get()
    setpos(FINGERS[1],position)
def getSlider6(event):
    position=slider6.get()
    setpos(FINGERS[2],position)
def getSlider7(event):
    position=slider7.get()
    setpos(FINGERS[3],position)
def getSlider8(event):
    position=slider8.get()
    setpos(FINGERS[4],position)
def get2bytes(ID,EEPROM_POS):
    value=dynamixel.read2ByteTxRx(portHandler, ID,EEPROM_POS)
    return(value[0])
def setSlider(position):
    global slider1
    slider1.set(position)
    print("Setting slider to " + str(position))
    if position==MIN_POS: #setting to min also disable grasp deom
        demoflag=0
def getpos(ID):
    position=dynamixel.read2ByteTxRx(portHandler, ID,Present_Position)
    return(position[0])
master = Tk()
master.destroy()
master = Tk()
master.geometry("1200x600")
master.resizable(1,1)
master.title("RH8D demo")
slider1 = Scale(master, from_=MAX_POS,
to=MIN_POS,length=HEIGHT,label="Rot",tickinterval=MAX_POS,command=getSlider1)
slider1.grid(row=2,column=0, pady=0, padx = 0,sticky="n")
slider2 = Scale(master, from_=MAX_POS,
to=MIN_POS,length=HEIGHT,label="Adu",tickinterval=MAX_POS,command=getSlider2)
slider2.grid(row=2,column=1, pady=0, padx = 0,sticky="n")
slider3 = Scale(master, from_=MAX_POS,
to=MIN_POS,length=HEIGHT,label="Flex",tickinterval=MAX_POS,command=getSlider3)
slider3.grid(row=2,column=2, pady=0, padx = 0,sticky="n")
slider4 = Scale(master, from_=MAX_POS, to=MIN_POS,length=HEIGHT,label="ThumbAdu",tickinterval=MAX_POS,command=getSlider4)
slider4.grid(row=2,column=3, pady=0, padx = 0,sticky="n")
slider5 = Scale(master, from_=MAX_POS, to=MIN_POS,length=HEIGHT,label="ThumbFlex",tickinterval=MAX_POS,command=getSlider5)
slider5.grid(row=2,column=4, pady=0, padx = 0,sticky="n")
slider6 = Scale(master, from_=MAX_POS,
to=MIN_POS,length=HEIGHT,label="Index",tickinterval=MAX_POS,command=getSlider6)
slider6.grid(row=2,column=5, pady=0, padx = 0,sticky="n")
slider7 = Scale(master, from_=MAX_POS,
to=MIN_POS,length=HEIGHT,label="Middle",tickinterval=MAX_POS,command=getSlider7)
slider7.grid(row=2,column=6, pady=0, padx = 0,sticky="n")
slider8 = Scale(master, from_=MAX_POS,
to=MIN_POS,length=HEIGHT,label="Ring\Pinky",tickinterval=MAX_POS,command=getSlider8)
slider8.grid(row=2,column=7, pady=0, padx = 0,sticky="n")
currlabel = Label(master, text = "0")
currlabel.grid(row=6,column=0, pady=0, padx = 0)
position=getpos(WRIST[0])
slider1.set(position)
position=getpos(WRIST[1])
slider2.set(position)
position=getpos(WRIST[2])
slider3.set(position)
position=getpos(FINGERS[0])
slider4.set(position)
position=getpos(FINGERS[1])
slider5.set(position)
position=getpos(FINGERS[2])
slider6.set(position)
position=getpos(FINGERS[3])
slider7.set(position)
position=getpos(FINGERS[4])
slider8.set(position)
l = Label(master, text = "Current")
l.grid(row=7,column=0, pady=0, padx = 15)
while(1):
    master.update()
    largestcurrent=0
    for DXL_ID in IDS:
        curr=get2bytes(DXL_ID,Current_mA);
        if (curr>2048):
            curr=4095-curr
        if (curr)>(largestcurrent):
            largestcurrent=curr
    print(largestcurrent);
    currlabel.config(text=str(largestcurrent))
    if (abs(largestcurrent)>500):
        currlabel.config(bg="red")
    elif (abs(largestcurrent)>250):
        currlabel.config(bg="yellow")
    else:
        currlabel.config(bg="white")
