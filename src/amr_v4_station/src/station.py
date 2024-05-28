#!/usr/bin/env python3

import rclpy
import rclpy.executors
import sys
from std_msgs.msg import Bool
import RPi.GPIO as gpio
import pyodbc

sensor1Pin = 11
sensor2Pin = 13
light1Pin = 16
light2Pin = 18
light3Pin = 22

class Station:
    def __init__(self, name):
        self.name = name
        self.sensor1 = False
        self.sensor2 = False
        self.light = Light()
        self.inPlace = False
        self.stationClear = True
        self.stationNode = rclpy.create_node('station_node')
        self.stationPublisher = self.stationNode.create_publisher(Bool, 'station209A/stationStatus', 10)
        
        self.cnxn = pyodbc.connect('DRIVER={ODBC Driver 18 for SQL Server};SERVER=fisher-agc.database.windows.net;DATABASE=Fisher_AGC;UID=fisher_agc;PWD=tTp##L86?qM!4iG7')
        self.cursor = self.cnxn.cursor()

        gpio.setwarnings(False)
        gpio.setmode(gpio.BOARD)
        gpio.setup(sensor1Pin, gpio.IN, pull_up_down=gpio.PUD_DOWN)
        gpio.setup(sensor2Pin, gpio.IN, pull_up_down=gpio.PUD_DOWN)
        gpio.setup(light1Pin, gpio.OUT)
        gpio.setup(light2Pin, gpio.OUT)
        gpio.setup(light3Pin, gpio.OUT)

        
    def fetchStationClearFromDatabase(self):
        self.cursor.execute("SELECT cart_in_place FROM Station WHERE station_name = ?", self.name)
        result = self.cursor.fetchone()
        if result:
            self.stationClear = result[0]

    
    def checkSensors(self):
        self.sensor1 = gpio.input(sensor1Pin)
        self.sensor2 = gpio.input(sensor2Pin)
        self.inPlace = self.sensor1 and self.sensor2
    
    def sendStationStatus(self):
        self.cursor.execute("UPDATE Station SET cart_in_place = ? WHERE station_name = ?", (1 if self.inPlace else 0, self.name))
        self.cnxn.commit()
    
    def run(self):
        while rclpy.ok():
            self.light.changeColor(False, False)
            self.checkSensors()
            
            if self.inPlace:
                self.stationClear = False
                self.sendStationStatus()
                print('tote in place')
        
            while not self.stationClear:
                self.checkSensors()
                self.fetchStationClearFromDatabase()
                
                if self.inPlace:
                    self.light.changeColor(True, True)
                    print('tote in place')
                else:
                    self.light.changeColor(True, False)
                    print('tote moved')
                    
            self.sendStationStatus()
            self.fetchStationClearFromDatabase()
            print('station cleared')
            
class Light:
    def __init__(self):
        self.r = False
        self.g = False
        self.b = False
        
    def changeColor(self, s1, s2):
        if s1 and s2:
            self.r = False
            self.g = False
            self.b = True
        elif (s1 and not s2) or (not s1 and s2):
            self.r = True
            self.g = False
            self.b = False
        elif not s1 and not s2:
            self.r = False
            self.g = True
            self.b = False
        
        gpio.output(light1Pin, self.r)
        gpio.output(light2Pin, self.g)
        gpio.output(light3Pin, self.b)

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: station.py <station_name>")
        sys.exit(1)
    
    station_name = sys.argv[1]
    
    print('station name: ' + station_name)
    station = Station(station_name)
    station.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()