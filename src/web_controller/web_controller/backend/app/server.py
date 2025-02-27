#!/usr/bin/env python3
# Written by Henry Forsyth, December 20th, 2023

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64, Int8
# geometry_msgs twist
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
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
    ang_vel: Union[float, int]
    vel_x : Union[float, int]
    vel_y : Union[float, int]


# ROS 2 Node setup
class RobotAbstraction(Node):
    def __init__(self):
        super().__init__('robot_controller_web')
        self.cmd_vel = self.create_publisher(Twist, '/hermes/cmd_vel', 10)
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
    

    def publish_move(self, moveCommand : Move):
        try:
            print(f"Publishing move command: {moveCommand}")
            self.cmd_vel.publish(Twist(
                linear=Vector3(x=float(moveCommand.vel_x), y=float(moveCommand.vel_y), z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=float(moveCommand.ang_vel))
            ))
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
    node.publish_move(moveCommand)
    return {"message": "Move command received", "command" : moveCommand}


# Return all the position data
@app.get("/swerve_data")
def swerve_data():
        
    return {
        "swerve_a": node.swerve_a,
        "swerve_b": node.swerve_b,
        "swerve_c": node.swerve_c,
        "swerve_d": node.swerve_d,
    }
