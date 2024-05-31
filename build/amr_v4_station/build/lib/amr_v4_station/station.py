#!/usr/bin/env python3

import rclpy
import sys
import time
import logging
import json
import rclpy.executors
from std_msgs.msg import Bool
import RPi.GPIO as gpio
import pyodbc

# Setup logging
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
        except Exception as e:
            logger.error(f"Database connection error: {e}")
            sys.exit(1)

    def disconnect(self):
        self.connection.close()
        logger.info("Database connection closed.")

class LightStack:
    def __init__(self, config):
        self.config = config
        self.r = False
        self.g = False
        self.b = False
        self.count = 0
        gpio.setup(self.config['light1'], gpio.OUT)
        gpio.setup(self.config['light2'], gpio.OUT)
        gpio.setup(self.config['light3'], gpio.OUT)

    def change_color(self, cart, docking, flash_red):
        if(cart and docking):
            self.blue()
        elif(not cart and docking):
            self.flash_red()
        elif(cart and not docking):
            self.blue()
        elif(not cart and not docking):
            if(flash_red):
                self.flash_red()
            else:
                self.green()

        try:
            gpio.output(self.config['light1'], self.r)
            gpio.output(self.config['light2'], self.g)
            gpio.output(self.config['light3'], self.b)
        except Exception as e:
            logger.error(f"Error changing light color: {e}")

    def blue(self):
        self.r = False
        self.g = False
        self.b = True

    def green(self):
        self.r = False
        self.g = True
        self.b = False
    
    def flash_red(self):
        if(self.count % 2 == 0):
            self.r = True # Flash red light during docking
            self.g = False
            self.b = False
        else:
            self.r = False
            self.g = False
            self.b = False
        self.count += 1

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
        self.light_stack = LightStack(self.config['gpio_pins'])
        self.sensor1 = Sensor(self.config['gpio_pins']['sensor1'])
        self.sensor2 = Sensor(self.config['gpio_pins']['sensor2'])
        self.cart_in_place = False
        self.docking = False
        self.flash_red_timer = 0
        self.timer_duration = 10
        self.flash_red = True
        self.database = Database(self.config['database'])
        self.database.connect()
        self.node = rclpy.create_node('station')
        self.publisher = None

    def check_sensors(self):
        try:
            sensor1_status = self.sensor1.read()
            sensor2_status = self.sensor2.read()
            logger.debug(f"Sensor1: {sensor1_status}, Sensor2: {sensor2_status}")
            if sensor1_status and sensor2_status:
                time.sleep(1)
                if self.sensor1.read() and self.sensor2.read():
                    return True
            return False
        except Exception as e:
            logger.error(f"Error checking sensors: {e}")
            return None

    def fetch_docking_status(self):
        try:
            self.database.cursor.execute("SELECT docking_in_action FROM Station WHERE station_name = ?", self.name)
            result = self.database.cursor.fetchone()[0]
            if result is not None:
                self.docking = result
                logger.info(f"Docking status fetched: {self.docking}")
    
            
        except Exception as e:
            logger.error(f"Error fetching docking status: {e}")
            time.sleep(5)

    def update_cart_status(self):
        try:
            self.database.cursor.execute("UPDATE Station SET cart_in_place = ? WHERE station_name = ?", self.cart_in_place, self.name)
            self.database.connection.commit()
            logger.debug(f"Cart status updated: {self.cart_in_place}")
        except Exception as e:
            logger.error(f"Error updating cart status: {e}")
            sys.exit(1)
    
    
    def reset_timer(self):
        self.flash_red_timer = self.timer_duration

    def manage_publisher(self):
        msg = Bool()
        if(self.cart_in_place):
            if(self.publisher is None):
                self.publisher = self.node.create_publisher(Bool, 'station_' + self.name + '_status', 10)
                logger.info("station publisher created")
            msg.data = True
            try:
                self.publisher.publish(msg)
            except Exception as e:
                logger.info("error in publisher: {e}")
        elif(not self.cart_in_place and not self.docking and self.publisher is not None):
            self.publisher.destroy()
            self.publisher = None
            logger.info("station publisher destroyed")
        elif(self.publisher is not None and not self.cart_in_place):
            msg.data = False
            try:
                self.publisher.publish(msg)
            except Exception as e:
                logger.info("error in publisher: {e}")


    def run(self):
        while rclpy.ok():
            self.fetch_docking_status()
            self.cart_in_place = self.check_sensors()
            self.update_cart_status()
            self.manage_publisher()
            if(not self.cart_in_place and not self.docking):
                if(self.flash_red_timer > 0):
                    self.flash_red = True
                    self.flash_red_timer -= 1
                else:
                    self.flash_red = False
            if(self.cart_in_place):
                self.reset_timer()  

            self.light_stack.change_color(self.cart_in_place, self.docking, self.flash_red)
            time.sleep(1)
            logger.info("docking status: {} cart status: {}".format(self.docking, self.cart_in_place))

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('station_params_node')
    station_name = node.declare_parameter('station_name', 'default_station').get_parameter_value().string_value
    logger.info(f"Station name: {station_name}")

    station = Station(station_name, config)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(station.node)
    executor.spin(station.run())
    station.database.disconnect()
    gpio.cleanup()
    executor.shutdown()
