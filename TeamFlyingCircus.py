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
        buffer0 = self.main.rawPos[0]
        buffer1 = self.main.rawPos[1]
        buffer2 = self.main.rawPos[2]
        self.main.doppel = self.main.doppel + 1
        send1 = b' '
        if (self.main.doppel%2==0):
          self.main.filterdPos[0] = (buffer0 + self.main.rawPos[0])/2
          self.main.filterdPos[1] = (buffer1 + self.main.rawPos[1])/2
          self.main.filterdPos[2] = (buffer2 + self.main.rawPos[2])/2
          
          delx = self.main.rawPos[0] - buffer0
          dely = self.main.rawPos[1] - buffer1
          
          winkel = math.atan(dely/delx)
          cos = math.cos(winkel)
          sin = math.sin(winkel)
          
          buf0 = self.main.waypoints[0][0] - self.main.filterdPos[0]
          buf1 = self.main.waypoints[0][1] - self.main.filterdPos[1]
          
          local_x = cos*buf0 + sin*buf1
          local_y = cos*buf1 - sin*buf0
          
          print(local_x)
          print(local_y)
          send1 = struct.pack(h, local_x, local_y)
          
        self.main.arduino.send(string(send1))
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

