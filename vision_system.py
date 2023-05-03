from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import imutils
import cv2
import smbus
import time
import serial

bus = smbus.SMBus(1)
ser = serial.Serial(port='/dev/ttyACM0',baudrate = 9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=30)
vid = cv2.VideoCapture(0)

Z_POBIERANIA = -103
offset =  15

def setup(Addr):
	global address
	address = Addr

def read(chn):
    ok = False
    while not(ok):
        try:
            if chn == 0:
                bus.write_byte(address,0x40)
            if chn == 1:
                bus.write_byte(address,0x41)
            if chn == 2:
                bus.write_byte(address,0x42)
            if chn == 3:
                bus.write_byte(address,0x43)
            bus.read_byte(address)
            return bus.read_byte(address)
            ok = True
        except:
            pass

def read_avg(ch):
    sum = 0
    for i in range(3):
        sum = sum + read(ch)
        time.sleep(0.1)
    return ((sum/3)>100)

def tasmociag():
    ser.write('a\n'.encode())
    odp=ser.readline()
    #print(odp[0])
    if chr(odp[0])=='a':
        return True
    else:
        raise Exception('CONVEYOR error')

def zerowanie():
    ser.write('b\n'.encode())
    odp=ser.readline()
    if chr(odp[0])=='b':
        return True
    else:
        raise Exception('HOMING error')

def ruch(x,y,z):
    ciag = 1000000000+abs(x)*1000000+abs(y)*1000+abs(z-offset)
    znak = 1
    if x<0:
        znak = znak+1
    if y<0:
        znak = znak+2
    if z<0:
        znak = znak+4
    komunikat = "c"+str(ciag)+str(znak)+"\n"
    print(komunikat)
    ser.write(komunikat.encode())
    odp=ser.readline()
    if chr(odp[0])=='c':
        return True
    else:
        raise Exception('MOVING error')
        
def Wsun():
    ser.write('d\n'.encode())
    
def Wysun():
    ser.write('f\n'.encode())

def Magnes_ON():
    ser.write('g\n'.encode())
    
def Magnes_OFF():
    ser.write('h\n'.encode())
    
def Lampa_RED_OFF():
    ser.write('k\n'.encode())
    
def Lampa_RED_ON():
    ser.write('m\n'.encode())
    
def Lampa_BLUE():
    ser.write('i\n'.encode())

def Lampa_ORANGE():
    ser.write('j\n'.encode())

def Obrot_CW():
    ser.write('o\n'.encode())
    time.sleep(3)
    
def Obrot_CCW():
    ser.write('p\n'.encode())
    time.sleep(3)

def CLEAR():
    ser.write('r\n'.encode())
    ser.write('t\n'.encode())
    ser.write('v\n'.encode())

def BLUE():
    ser.write('s\n'.encode())
    
def GREEN():
    ser.write('u\n'.encode())
    
def RED():
    ser.write('q\n'.encode())

def WHITE():
    ser.write('w\n'.encode())

def Odloz_na_0():
    ruch(30,348,-60)
    ruch(30,348,-135)
    Ruch_odkladajacy()
    
def Odloz_na_1():
    ruch(-82,348,-60)
    ruch(-82,348,-135)
    Ruch_odkladajacy()
    
def Odloz_na_2():
    ruch(30,200,-60)
    ruch(30,245,-135)
    Ruch_odkladajacy()
    
def Odloz_na_3():
    ruch(-85,200,-60)
    ruch(-85,245,-135)
    Ruch_odkladajacy()

def Ruch_odkladajacy():
    Wysun()
    time.sleep(0.5)
    Magnes_OFF()
    time.sleep(0.5)
    Wsun()
    time.sleep(0.5)
    ruch(-30,295,0)

def Odrzuc():
    ruch(0,200,30)
    ruch(-240,170,20)
    ruch(-240,170,-20)
    Ruch_odkladajacy()
    
def Test_CAM(korekta=False):
    if korekta==True:
        x_start = 170
        x_l = 180
        y_start = 170
        y_l = 280
    else:
        x_start = 190
        x_l = 180
        y_start = 170
        y_l = 280
    
    wolne_0 = False
    wolne_1 = False
    wolne_2 = False
    wolne_3 = False
    OK = False
    
    zdjecie_zrobiono_poprawnie = False
    
    while zdjecie_zrobiono_poprawnie==False:
        try:
            for i in range(30):
                ret, image_INPUT = vid.read()
                time.sleep(0.1)
                
            (h, w) = image_INPUT.shape[:2]
            (cX, cY) = (w // 2, h // 2)
            M = cv2.getRotationMatrix2D((cX, cY), 2, 1.0)
            if korekta:
                image_INPUT = cv2.warpAffine(image_INPUT, M, (w, h))
                
            image = image_INPUT[x_start:x_start+x_l, y_start:y_start+y_l]
            
            cv2.imshow("Test", image)
            cv2.waitKey(1000)
            
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (7, 7), 0)
            edged = cv2.Canny(gray, 20, 100)
            edged = cv2.dilate(edged, None, iterations=1)
            edged = cv2.erode(edged, None, iterations=1)

            cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cnts = imutils.grab_contours(cnts)

            cntsSorted = sorted(cnts, key=lambda x: cv2.contourArea(x))
            orig = image.copy()

            x_together = []
            y_together = []

            for c in cntsSorted:
                for point in c:
                    if point[0][1]>20 and point[0][0]>20 and point[0][0]<y_l-20:
                        cv2.circle(image, (point[0][0], point[0][1]), 1, (0, 255, 255), -1)
                        x_together.append(point[0][0])
            
            x_min = min(x_together)
            x_max = max(x_together)
            cv2.circle(image, (x_min, 50), 3, (255, 0, 0), -1)
            cv2.circle(image, (x_max, 50), 3, (255, 0, 0), -1)
            print(x_min, x_max)
            time.sleep(0.5)
            
            for c in cntsSorted:
                for point in c:
                    if point[0][1]>30 and (point[0][0]<x_min+20 or point[0][0]>x_max-20):
                        cv2.circle(image, (point[0][0], point[0][1]), 2, (255, 0, 255), -1)
                        y_together.append(point[0][1])
                        
            cv2.imshow("Test", image)
            cv2.waitKey(1000)
            
            y_min = min(y_together)
            y_max = max(y_together)
            zdjecie_zrobiono_poprawnie = True
        except:
            zdjecie_zrobiono_poprawnie = False

    h = round(((y_max-y_min)/@@@)*100)/100.0 #@@@ - odwrotnosc wspolczynnika "pixels to metric ratio"

    cv2.line(orig, (x_max, y_max), (x_max, y_min),(255, 0, 255), 3)
    cv2.line(orig, (x_max, y_min), (x_min, y_min),(255, 0, 255), 3)
    cv2.line(orig, (x_min, y_min), (x_min, y_max),(255, 0, 255), 3)
    cv2.line(orig, (x_min, y_max), (x_max, y_max),(255, 0, 255), 3)

    cv2.circle(orig, (x_min, y_min), 5, (255, 0, 0), -1)
    cv2.circle(orig, (x_min, y_max), 5, (255, 0, 0), -1)
    cv2.circle(orig, (x_max, y_min), 5, (255, 0, 0), -1)
    cv2.circle(orig, (x_max, y_max), 5, (255, 0, 0), -1)

    if h<30:
        cv2.line(orig, (int(x_min+(x_max-x_min)/1.5), y_min), (int(x_min+(x_max-x_min)/1.5), y_max),(0, 255, 0), 4)
        cv2.putText(orig, "h",(int(x_min+(x_max-x_min)/1.5)+10, y_min+50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
    else:
        cv2.line(orig, (int(x_min+(x_max-x_min)/1.5), y_min), (int(x_min+(x_max-x_min)/1.5), y_max),(0, 0, 255), 4)
        cv2.putText(orig, "h",(int(x_min+(x_max-x_min)/1.5)+10, y_min+50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

    image_INPUT[x_start:x_start+x_l, y_start:y_start+y_l] = orig

    if h<29.5:
        cv2.putText(image_INPUT, "h="+str(h)+"mm", (150,380), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0),3)
        OK = True
    else:
        cv2.putText(image_INPUT, "h="+str(h)+"mm", (150,380), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,200),3)
        OK = False
        
    image_INPUT = cv2.resize(image_INPUT, (640, 480), interpolation= cv2.INTER_LINEAR)

    cv2.rectangle(image_INPUT,(440,0),(640,200),(100,100,100),-1)
    cv2.rectangle(image_INPUT,(439,0),(639,200),(0,255,0),3)
    
    if read_avg(0):
        cv2.circle(image_INPUT,(439+150,150), 40, (0,255,0), -1)
        wolne_0 = True
    else:
        cv2.circle(image_INPUT,(439+150,150), 40, (0,0,255), -1)
        
    if read_avg(1):
        cv2.circle(image_INPUT,(439+50,150), 40, (0,255,0), -1)
        wolne_1 = True
    else:
        cv2.circle(image_INPUT,(439+50,150), 40, (0,0,255), -1)

    if read_avg(2):
       cv2.circle(image_INPUT,(439+150,50), 40, (0,255,0), -1)
       wolne_2 = True
    else:    
        cv2.circle(image_INPUT,(439+150,50), 40, (0,0,255), -1)
        
    if read_avg(3):
        cv2.circle(image_INPUT,(439+50,50), 40, (0,255,0), -1)
        wolne_3 = True
    else:
        cv2.circle(image_INPUT,(439+50,50), 40, (0,0,255), -1)
        
    image_INPUT = cv2.resize(image_INPUT, (int(640*1.5), int(480*1.5)), interpolation= cv2.INTER_LINEAR)
    
    cv2.imshow("Robot 5DoF", image_INPUT)
    cv2.waitKey(1000)
    return wolne_0, wolne_1, wolne_2, wolne_3, OK
zerowanie()

while True:

    Wsun()
    Magnes_OFF()
    Lampa_BLUE()


    tasmociag()

    ruch(270,280,Z_POBIERANIA)
    Wysun()
    time.sleep(2)
    Magnes_ON()
    time.sleep(0.1)
    Wsun()
    time.sleep(1)
    ruch(270,280,22)
    Wysun()
    time.sleep(1)

    setup(0x48)

    '''
    #3   2
    #1   0
    '''
    Lampa_ORANGE()
    
    WHITE()
    wolne_0, wolne_1, wolne_2, wolne_3, OK = Test_CAM(korekta=False)
    if OK:
        Obrot_CW()
        wolne_0, wolne_1, wolne_2, wolne_3, OK = Test_CAM(korekta=True)
        Obrot_CCW()
    CLEAR()
    Lampa_BLUE()
    
    if OK:
        GREEN()
        time.sleep(1)
        odlozono = False
        while not(odlozono):
            if wolne_0:
                Odloz_na_0()
                odlozono = True
            elif wolne_1:
                Odloz_na_1()
                odlozono = True
            elif wolne_2:
                Odloz_na_2()
                odlozono = True
            elif wolne_3:
                Odloz_na_3()
                odlozono = True
            else:
                WHITE()
                Lampa_RED_ON()
                ser.write('y\n'.encode())
                time.sleep(0.1)
                ser.write('z\n'.encode())
                wolne_0, wolne_1, wolne_2, wolne_3, OK = Test_CAM(korekta=False)
                Lampa_RED_OFF()
                ser.write('y\n'.encode())
                time.sleep(0.1)
                ser.write('z\n'.encode())
                time.sleep(2)
        GREEN()
    else:
        RED()
        Odrzuc()
    CLEAR()
