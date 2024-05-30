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

with open('src/amr_v4_station/src/config.json', 'r') as config_file:
    config = json.load(config_file)
    
class Database:
    def __init__(self, config):
        self.config = config
        self.connection = None
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
        self.timer = 0
        self.count = 0
        self.config = config
        gpio.setup(self.config['light1'], gpio.OUT)
        gpio.setup(self.config['light2'], gpio.OUT)
        gpio.setup(self.config['light3'], gpio.OUT)
        
        
    
    def change_color(self, cart, docking, timer):
        if(not cart and not docking):
            if(timer == 0):
                self.r = False
                self.g = True
                self.b = False
            else:
                if(self.count % 2 == 0):
                    self.r = True
                    self.g = False
                    self.b = False
                else:
                    self.r = False
                    self.g = False
                    self.b = False
        if(not cart and docking):
            if(self.count % 2 == 0):
                self.r = True
                self.g = False
                self.b = False
            else:
                self.r = False
                self.g = False
                self.b = False
        if(cart):
            self.r = False
            self.g = False
            self.b = True

        


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
        gpio.setmode(gpio.BOARD)
        self.max_time = 10
        self.timer = self.max_time
        self.light_stack = LightStack(self.config['gpio_pins'])
        self.sensor1 = Sensor(self.config['gpio_pins']['sensor1'])
        self.sensor2 = Sensor(self.config['gpio_pins']['sensor2'])
        self.cart_in_place = False
        self.docking = False
        self.database = Database(self.config['database'])
        self.database.connect()
        self.node = rclpy.create_node('station')
        self.publisher = self.node.create_publisher(Bool, 'station' + self.name + 'publisher', 10)
    
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
            self.database.cursor.execute("SELECT docking_in_action FROM Station WHERE station_name = ?", self.name)
            self.docking = self.database.cursor.fetchone()
        except pyodbc.Error as e:
            logger.error(f"Error fetching docking status: {e}")
            time.sleep(5)
    
    def update_cart_status(self):
        try:
            self.database.cursor.execute("UPDATE Station SET cart_in_place = ? WHERE station_name = ?", self.cart_in_place, self.name)
            self.database.connection.commit()
        except pyodbc.Error as e:
            logger.error(f"Error updating cart status: {e}")
            sys.exit()   

    def run(self):
        while(rclpy.ok()):
            self.fetch_docking_status()
            self.cart_in_place = self.check_sensors()
            self.update_cart_status()
            self.light_stack.change_color(self.cart_in_place, self.docking, self.timer)
            if(self.timer ==  self.max_time and not self.cart_in_place):
                self.timer == 0
            if(not self.docking and not self.cart_in_place):
                if(self.timer < self.max_time):
                    self.timer += 1
                    time.sleep(1)


            logging.info("docking status: {} cart status: {}".format(self.docking, self.cart_in_place) )

def main(args = None):
    rclpy.init(args=args)
    
    station_name = sys.argv[0]
    logging.info(station_name)
    station = Station(station_name, config)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(station.node)
    executor.spin(station.run())
    station.database.disconnect()
    gpio.cleanup()
    executor.shutdown()
