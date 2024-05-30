#!/usr/bin/env python3

import rclpy
import rclpy.executors
import sys
import time
import logging
import json
from std_msgs.msg import Bool
import RPi.GPIO as gpio
import pyodbc

# setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

with open('config.json', 'r') as config_file:
    config = json.load(config_file)
    
class Database:
    def __init__(self, config):
        self.config = config
        self.con = None
        self.cursor = None
        
    def connect(self):
        try:
            self.connection = pyodbc.connect(
                'DRIVER={};SERVER={};DATABASE={};UID={};PWD={}'.format(
                    self.config['driver'],
                    self.config['server'],
                    self.config['database'],
                    self.config['username'],
                    self.config['password']
                )
            )
            self.cursor = self.connection.cursor()
            logger.info("Database connection established.")
        except pyodbc.Error as e:
            logger.error(f"Database connection error: {e}")
            sys.exit(1)
    
    def disconnect(self):
        self.connection.close()
        logger.info("Database connection closed.")

class LightStack:
    def __init__(self, config):
        self.r = False
        self.g = False
        self.b = False
        self.count = 0
        self.config = config
        gpio.setup(self.config['light1'], gpio.OUT)
        gpio.setup(self.config['light2'], gpio.OUT)
        gpio.setup(self.config['light3'], gpio.OUT)
        
        
    
    def change_color(self, s1, s2, docking):
        tote = s1 and s2
        if(docking):
            if(tote):
                self.r = False
                self.g = False
                self.b = True
            else:
                if(self.count % 2 == 0):
                    self.r = True
                    self.g = False
                    self.b = False
                else:
                    self.r = False
                    self.g = False
                    self.b = False
        else:
            if(tote):
                self.r = False
                self.g = False
                self.b = True
            else:
                self.r = False
                self.g = True
                self.b = False
        try:
            gpio.output(self.config['light1'], self.r)
            gpio.output(self.config['light2'], self.g)
            gpio.output(self.config['light3'], self.b)
        
        except Exception as e:
            logger.error(f"Error changing light color: {e}")
            

class Sensor:
    def __init__(self, pin):
        self.pin = pin
        gpio.setup(self.pin, gpio.IN, pull_up_down=gpio.PUD_DOWN)
    
    def read(self):
        
        try:
            return gpio.input(self.pin)
        
        except Exception as e:
            logger.error(f"Error reading sensor: {e}")
            return None

class Station:
    def __init__(self, name, config):
        self.name = name
        self.config = config
        self.timer = 0
        self.light_stack = LightStack(self.config['gpio_pins'])
        self.sensor1 = Sensor(self.config['gpio_pins']['sensor1'])
        self.sensor2 = Sensor(self.config['gpio_pins']['sensor2'])
        self.cart_in_place = False
        self.docking = False
        self.database = Database(self.config['database'])
        self.database.connect()
        self.node = rclpy.create_node('station_' + self.name + '_node')
        self.publisher = self.node.create_publisher(Bool, 'station_' + self.name + '_publisher', 10)
        gpio.setmode(gpio.BOARD)
    
    def check_sensors(self):
        try:
            if(self.sensor1.read() and self.sensor2.read()):
                time.sleep(1)
                if(self.sensor1.read() and self.sensor2.read()):
                    return True
            return False
        except Exception as e:
            logger.error(f"Error checking sensors: {e}")
            return None
    
    def fetch_docking_status(self):
        try:
            self.database.cursor.execute("SELECT docking_in_action FROM stations WHERE name = ?", self.name)
            self.docking = self.database.cursor.fetchone()[0]
        except pyodbc.Error as e:
            logger.error(f"Error fetching docking status: {e}")
    
    def update_cart_status(self):
        try:
            self.database.cursor.execute("UPDATE stations SET cart_in_station = ? WHERE name = ?", self.cart_in_place, self.name)
            self.database.connection.commit()
        except pyodbc.Error as e:
            logger.error(f"Error updating cart status: {e}")
        
    def run(self):
        while(rclpy.ok()):
            self.fetch_docking_status()
            self.cart_in_place = self.check_sensors()
            self.update_cart_status()
            
            if(self.docking):
                self.timer = 0
                if(self.cart_in_place):
                    self.light_stack.change_color(True, True, self.docking)
                else:
                    self.light_stack.change_color(False, False, self.docking)
            else:
                if(self.cart_in_place):
                    self.timer = 0
                    self.light_stack.change_color(True, True, self.docking)
                else:
                    self.light_stack.change_color(False, False, self.docking)
                    self.timer += 1
                    if(self.timer == 10): # change to 60 in final version
                        self.light_stack.change_color(False, False, self.docking)
                        self.timer = 0

def main(args = None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: station.py <station_name>")
        sys.exit(1)
    
    station_name = sys.argv[1]
    station = Station(station_name, config)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(station.node)
    executor.spin(station.run())
    station.database.disconnect()
    gpio.cleanup()
