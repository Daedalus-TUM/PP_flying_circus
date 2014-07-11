import math
import random
import os
import glob
import sys, traceback
import time
import threading 
import serial
import re
import struct
##
import cairo
##
import TeamFlyingCircus



class TeamFlyingCircus(threading.Thread):
    """ Das ist Eure Funktionsklasse, hier kommt alles rein, was speziell Euer Team betrifft. In den Eventhandler-Funktionen bitte keine komplizierten Berechnungen durchführen, sondern nur flags setzen, da diese von einem anderen Thread berechnet werden müssten und diesen evtl. blockieren würden(=>GUI hängt, etc.). Die run-Funktion wird in Eurem Thread ausgeführt, da kommen die Berechnungen rein. Zugriff auf die gui oder arduino habt Ihr über self.main.gui bzw. self.main.arduino """
    def __init__(self, main):
        threading.Thread.__init__(self) 
        self.main = main
        print("TeamFlyingCircus")
        self.run_ = True
        self.start_ = 0
        self.currentWP = 0
        self.buffer0 = 0
        self.buffer1 = 0
        self.buffer2 = 0
        self.distance = 0
        self.winkel_wegpkt = 0
        self.winkel_gondel = 0
        self.bufferlist_1_x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.bufferlist_1_y = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.bufferlist_2_x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.bufferlist_2_y = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.buffercount = 0
        self.buff = 3
        #threading.Thread.__init__(self) #p-initialisierung der vererbten Fähigkeit threading der Klasse Thread.
        #self.ttylock = threading.Lock()
        self.bob = serial.Serial("/dev/ttyUSB1", 115200, 8, 'N', 1, 0.0001)
        self.bob.flushInput() #flush input buffer, discarding all its contents
        self.bob.flushOutput()#flush output buffer, aborting current output
        self.strbuffer = b''
        self.lines = b''
        #Wegfindung: Definition der Wegpunkte. Zu Beachten: Alle Hindernisse müssen als Tore aufgebaut sein ((1,2);(3,4);(5,6)...)
        #wobei die kleinere Zahl den linken Torpfosten darstellt.
        """self.main.waypoints[0][0] = 
        self.main.waypoints[0][1] = 
        self.main.waypoints[1][0] = 
        self.main.waypoints[1][1] = 
        self.main.waypoints[2][0] = 
        self.main.waypoints[2][1] = 
        self.main.waypoints[3][0] = 
        self.main.waypoints[3][1] = 
        self.main.waypoints[4][0] = 
        self.main.waypoints[4][1] = 
        self.main.waypoints[5][0] = 
        self.main.waypoints[5][1] = 
        self.main.waypoints[6][0] = 
        self.main.waypoints[6][1] = 
        self.main.waypoints[7][0] = 
        self.main.waypoints[7][1] = 
        self.main.waypoints[8][0] = 
        self.main.waypoints[8][1] = 
        self.main.waypoints[9][0] = 
        self.main.waypoints[9][1] = 
        self.main.waypoints[10][0] = 
        self.main.waypoints[10][1] = 
        self.main.waypoints[11][0] = 
        self.main.waypoints[11][1] = 
        self.main.waypoints[12][0] = 
        self.main.waypoints[12][1] = 
        self.main.waypoints[13][0] = 
        self.main.waypoints[13][1] = 
        self.main.waypoints[14][0] = 
        self.main.waypoints[14][1] = """
        
    def run(self):
        #i=0
        while(self.run_):
            #i=i+1
            #print("TeamFlyingCircus run:",i)
            self.loop()
    def loop(self):
        if (self.start_):
            #wegpunktnavigation etc.
            #self.main.arduino.send("TeamFlyingCircus fliegt!\n")
            pass
        time.sleep(0.005) #schlafen ist gut, um die CPU nicht voll auszulasten
    def onStart(self):
        #Start-Button wurde gedrückt
        self.start_ = 1
        pass
    def onStop(self):
        #Stop-Button wurde gedrückt
        self.start_ = 0
        # reset etc.
        pass
    def onNewPos(self):
        self.main.doppel = self.main.doppel + 1
        while (self.bob.inWaiting() > 0):
          self.strbuffer = self.strbuffer + self.bob.read(self.bob.inWaiting())
          if b'X' in self.strbuffer:
            blocks = self.strbuffer.split(b'X')
            for block in blocks:
              received = block.split(b'Q')
            self.main.filterdPos[2] = int(received[3])
            self.main.doppel = int(received[4])
        
        if (self.main.doppel%2==0):# and (self.buffer2 + self.main.rawPos[2])/2 > 200):
          #filter over 20
          self.bufferlist_1_x[self.buffercount] = self.buffer0
          self.bufferlist_1_y[self.buffercount] = self.buffer1
          self.bufferlist_2_x[self.buffercount] = self.main.rawPos[0]
          self.bufferlist_2_y[self.buffercount] = self.main.rawPos[1]
          self.buffercount = self.buffercount + 1
          if(self.buffercount > self.buff):
            self.buffercount = 0

          filterd_1_x = sum(self.bufferlist_1_x)/self.buff
          filterd_1_y = sum(self.bufferlist_1_y)/self.buff
          filterd_2_x = sum(self.bufferlist_2_x)/self.buff
          filterd_2_y = sum(self.bufferlist_2_y)/self.buff
          
          #Gondelwinkel
          winkx = filterd_1_x - filterd_2_x
          winky = filterd_1_y - filterd_2_y
          if(winky!=0):
            if(math.atan(winkx/winky)!=0):
              self.winkel_gondel = 180/math.pi*(math.atan(winkx/winky))
          
          #create global x,y
          self.main.filterdPos[0] = (filterd_1_x + filterd_2_x)/2
          self.main.filterdPos[1] = (filterd_1_y + filterd_2_y)/2
          
          #create distance and wegpktern winkel to next waypoint
          buf0 = self.main.waypoints[self.currentWP][0] - self.main.filterdPos[0]
          buf1 = self.main.waypoints[self.currentWP][1] - self.main.filterdPos[1]
          
          self.distance = (buf0**2+buf1**2)**(0.5)/10
          self.distance = self.distance + 180
          if(buf1!=0):
            if(math.atan(buf0/buf1)!=0):
              self.winkel_wegpkt = 180 / math.pi * (math.atan(buf0/buf1))

          print(self.distance)
          print(self.winkel_wegpkt)
          print(self.winkel_gondel)
          print(self.main.filterdPos[2])
          print(" ")
          
          #formatting
          buf0 = self.distance
          buf1 = self.winkel_wegpkt
          buf2 = self.winkel_gondel
          
          if(buf0 < 0):
            buf0 += 65536
          if(buf1 < 0):
            buf1 += 65536
          if(buf2 < 0):
            buf2 += 65536
          
          bybuf0 = chr(int(buf0%256))
          bybuf2 = chr(int(buf1%256))
          bybuf1 = chr(int(buf0/256))
          bybuf3 = chr(int(buf1/256))
          bybuf4 = chr(int(buf2%256))
          bybuf5 = chr(int(buf2/256))
           
          if (self.distance < 30):
            self.currentWP = self.currentWP + 1
        
          #self.ttylock.acquire()
          self.bob.write(bytes(('{a}{b}{c}{d}{e}{f}'.format(a=bybuf0, b=bybuf2, c=bybuf1, d=bybuf3, e=bybuf4, f=bybuf5)).encode('UTF-8')))
          self.bob.flush()
          #self.ttylock.release()

        self.buffer0 = self.main.rawPos[0]
        self.buffer1 = self.main.rawPos[1]
        self.buffer2 = self.main.rawPos[2]
        pass

    def onButtonPressed(self, i):
        #welcher Button welche Nummer hat seht Ihr in der glade Datei oder im Eventhandler oder durch Testen
        #print("Button ", i)
        pass
    def onWaypointUpdate(self):
        #wegpunkt wurde in der gui geändert
        pass
    def onObstacleUpdate(self):
        pass
    def onExit(self):
        # Programm beenden
        self.run_=False
        pass
    def setkurswinkelflag(self):
        self.kurswinkelflag = True

