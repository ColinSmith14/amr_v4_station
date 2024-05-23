#!/usr/bin/env python3

import rclpy
import rclpy.executors
import sys
from std_msgs.msg import Bool
import pyodbc

sensor1Pin = 1
sensor2Pin = 2
light1Pin = 3
light2Pin = 4
light3Pin = 5

class Station:
    def __init__(self, name):
        self.name = name
        self.sensor1 = False
        self.sensor2 = False
        self.light = Light()
        self.inPlace = True
        self.stationClear = True
        self.stationNode = rclpy.create_node('station_node')
        self.stationPublisher = self.stationNode.create_publisher(Bool, 'station209A/stationStatus', 10)
        
        self.cnxn = pyodbc.connect('DRIVER={ODBC Driver 18 for SQL Server};SERVER=fisher-agc.database.windows.net;DATABASE=Fisher_AGC;UID=fisher_agc;PWD=tTp##L86?qM!4iG7')
        self.cursor = self.cnxn.cursor()
        
    def fetchStationClearFromDatabase(self):
        result = self.cursor.execute("SELECT cart_in_place FROM Station WHERE station_name = '" + self.name + "'")
        self.stationClear = result.fetchone()[0]
        print(self.stationClear)

    
    def checkSensors(self):
        # self.sensor1 = gpio.input(sensor1Pin)
        # self.sensor2 = gpio.input(sensor2Pin)
        self.inPlace = self.sensor1 and self.sensor2
    
    def sendStationStatus(self):
        if(not self.stationClear):
            self.cursor.execute("UPDATE Station SET cart_in_place = 0 WHERE station_name = '" + self.name + "'")
        else:
            self.cursor.execute("UPDATE Station SET cart_in_place = 1 WHERE station_name = '" + self.name + "'")
    
    
    def run(self):
        while rclpy.ok():
            self.light.changeColor(False, False)
            # self.checkSensors()
            
            if self.inPlace:
                self.stationClear = False
                self.sendStationStatus()
        
            while(not self.stationClear):
                # self.checkSensors()
                self.fetchStationClearFromDatabase()
                
                if(self.inPlace):
                    self.light.changeColor(True, True)
                    self.stationClear = True
                    self.inPlace = False
                else:
                    self.light.changeColor(True, False)
                    
            self.cursor.execute("UPDATE Station SET cart_in_place = 1 WHERE station_name = '" + self.name + "'")
            self.fetchStationClearFromDatabase()
            
                    
            
            


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
        
        # GPIO.output(light1Pin, self.r)
        # GPIO.output(light2Pin, self.g)
        # GPIO.output(light3Pin, self.b)
        # print("R: ", self.r, " G: ", self.g, " B: ", self.b)

def main(args = None):
    rclpy.init(args = args)
    
    # Temporary node to fetch the parameter
    node = rclpy.create_node('temporary_node_for_param')
    
    # Declare the parameter
    node.declare_parameter('station_name', 'station_default')
    
    # Get the parameter
    station_name = node.get_parameter('station_name').get_parameter_value().string_value
    
    # Destroy the temporary node
    node.destroy_node()
    print('station name: ' + station_name)
    station = Station(station_name)
    station.run()
    rclpy.shutdown()
