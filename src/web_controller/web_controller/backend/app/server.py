#!/usr/bin/env python3
# Written by Henry Forsyth, December 20th, 2023

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int8
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
from threading import Thread
from typing import Union
from colorama import Fore

# FastAPI app initialization
app = FastAPI(
    title="Hermes Backend Controller",
    version="0.1",
    description="This is the backend controller for the Hermes project. It is responsible for managing the backend of the Hermes project.",
    contact={"name": "Henry Forsyth", "email": "robert.h.forsyth@gmail.com"},
    servers=[
        {"url": "http://localhost:5000", "description": "Local Server"},
        {"url": "http://0.0.0.0:5000", "description": "Local Server"},
    ],
)

# CORS configuration
origins = [
    "http://localhost:3000",
    "http://localhost:5000",
    "http://localhost:8000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins="*",
    allow_credentials=True,
    allow_methods=["POST", "GET", "PUT", "DELETE"],
    allow_headers=["*"],
)

# Define the command structure using Pydantic
class Move(BaseModel):
    direction: float
    speed: float
    mode: Optional[str] = "Standard"


# ROS 2 Node setup
class RobotAbstraction(Node):
    def __init__(self):
        super().__init__('robot_controller_web')
        self.wheel_speed = self.create_publisher(Float64, '/hermes/rqst_wheel_speed', 10)
        self.pivot_direction = self.create_publisher(Float64, '/hermes/rqst_pivot_direction', 10)
        self.mode_selection = self.create_publisher(Int8, '/hermes/mode', 10)
        self.get_logger().info("RobotAbstraction Node has been started.")
        
        # Subscribe to all swerve modules, A, B, C, D
        self.swerve_a = {
            "pivot_position" : 0,
            "requested_pivot_position" : 0,
        }
        
        self.swerve_b = {
            "pivot_position" : 0,
            "requested_pivot_position" : 0,
        }
        
        self.swerve_c = {
            "pivot_position" : 0,
            "requested_pivot_position" : 0,
        }
        
        self.swerve_d = {
            "pivot_position" : 0,
            "requested_pivot_position" : 0,
        }
        
        # Create subscribers for each swerve module, actual data
        self.swerve_a_sub = self.create_subscription(Float64, '/swerve_a/pivot_position', self.swerve_callback("swerve_a", "pivot_position"), 10)
        self.swerve_b_sub = self.create_subscription(Float64, '/swerve_b/pivot_position', self.swerve_callback("swerve_b", "pivot_position"), 10)
        self.swerve_c_sub = self.create_subscription(Float64, '/swerve_c/pivot_position', self.swerve_callback("swerve_c", "pivot_position"), 10)
        self.swerve_d_sub = self.create_subscription(Float64, '/swerve_d/pivot_position', self.swerve_callback("swerve_d", "pivot_position"), 10)
        
        # Also do it for the rqst_pivot_direction
        self.swerve_a_rqst_sub = self.create_subscription(Float64, '/swerve_a/rqst_pivot_direction', self.swerve_callback("swerve_a", "requested_pivot_position"), 10)
        self.swerve_b_rqst_sub = self.create_subscription(Float64, '/swerve_b/rqst_pivot_direction', self.swerve_callback("swerve_b", "requested_pivot_position"), 10)
        self.swerve_c_rqst_sub = self.create_subscription(Float64, '/swerve_c/rqst_pivot_direction', self.swerve_callback("swerve_c", "requested_pivot_position"), 10)
        self.swerve_d_rqst_sub = self.create_subscription(Float64, '/swerve_d/rqst_pivot_direction', self.swerve_callback("swerve_d", "requested_pivot_position"), 10)
        
        # Actual speed
        self.swerve_a_speed_sub = self.create_subscription(Float64, '/swerve_a/wheel_speed', self.swerve_callback("swerve_a", "speed"), 10)
        self.swerve_b_speed_sub = self.create_subscription(Float64, '/swerve_b/wheel_speed', self.swerve_callback("swerve_b", "speed"), 10)
        self.swerve_c_speed_sub = self.create_subscription(Float64, '/swerve_c/wheel_speed', self.swerve_callback("swerve_c", "speed"), 10)
        self.swerve_d_speed_sub = self.create_subscription(Float64, '/swerve_d/wheel_speed', self.swerve_callback("swerve_d", "speed"), 10)
        
        # Requested speed
        self.swerve_a_rqst_speed_sub = self.create_subscription(Float64, '/swerve_a/rqst_wheel_speed', self.swerve_callback("swerve_a", "requested_speed"), 10)
        self.swerve_b_rqst_speed_sub = self.create_subscription(Float64, '/swerve_b/rqst_wheel_speed', self.swerve_callback("swerve_b", "requested_speed"), 10)
        self.swerve_c_rqst_speed_sub = self.create_subscription(Float64, '/swerve_c/rqst_wheel_speed', self.swerve_callback("swerve_c", "requested_speed"), 10)
        self.swerve_d_rqst_speed_sub = self.create_subscription(Float64, '/swerve_d/rqst_wheel_speed', self.swerve_callback("swerve_d", "requested_speed"), 10)
        
                
    def swerve_callback(self, swerve_module, data_name):
        def callback(msg):
            getattr(self, swerve_module)[data_name] = msg.data
        return callback
    

    def publish_move(self, direction, speed, mode="Standard"):
        try:
            self.pivot_direction.publish(Float64(data=direction))
            self.wheel_speed.publish(Float64(data=speed))
            # The mode is either "Standard" or "OnADime"
            mapping = { "Standard": 0, "OnADime": 1 }
            # if not in the mapping, log an error
            if mode not in mapping:
                self.get_logger().error(f"{Fore.RED}Error: Mode {mode} not recognized. Must be either 'Standard' or 'OnADime'{Fore.RESET}")
                return
            self.mode_selection.publish(Int8(data=mapping[mode]))
        except Exception as e:
            self.get_logger().error(f"{Fore.RED}Error: {e}{Fore.RESET}")


# Create a ROS 2 node and start it in a separate thread
node = None

def start_ros_node():
    global node
    rclpy.init()
    node = RobotAbstraction()
    rclpy.spin(node)
    
# Start a separate thread to run ROS 2 node (since FastAPI runs asynchronously)
ros_thread = Thread(target=start_ros_node)
ros_thread.start()

# FastAPI endpoint to handle movement commands
@app.post("/move")
def move(moveCommand: Move):
    # Publish the move command to ROS 2
    node.publish_move(moveCommand.direction, moveCommand.speed, moveCommand.mode)

    return {"message": "Move command received", "direction": moveCommand.direction, "speed": moveCommand.speed, "mode": moveCommand.mode}


# Return all the position data
@app.get("/swerve_data")
def swerve_data():
        
    return {
        "swerve_a": node.swerve_a,
        "swerve_b": node.swerve_b,
        "swerve_c": node.swerve_c,
        "swerve_d": node.swerve_d,
    }
