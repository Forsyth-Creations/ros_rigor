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
        self.cmd_vel_pub = self.create_publisher(Twist, '/joystick_cmd_vel', 10)
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
        
        self.current_cmd_vel = Twist()
        
        # Create subscribers for each swerve module, actual data
        self.swerve_a_sub = self.create_subscription(Float64, '/swerve_a/pivot_position', self.swerve_callback("swerve_a", "pivot_position"), 10)
        self.swerve_b_sub = self.create_subscription(Float64, '/swerve_b/pivot_position', self.swerve_callback("swerve_b", "pivot_position"), 10)
        self.swerve_c_sub = self.create_subscription(Float64, '/swerve_c/pivot_position', self.swerve_callback("swerve_c", "pivot_position"), 10)
        self.swerve_d_sub = self.create_subscription(Float64, '/swerve_d/pivot_position', self.swerve_callback("swerve_d", "pivot_position"), 10)
        
        # Create subscribers for each swerve module, actual data of command
        self.swerve_a_sub = self.create_subscription(Twist, '/swerve_a/command', self.handle_twist_callback("swerve_a", "command"), 10)
        self.swerve_b_sub = self.create_subscription(Twist, '/swerve_b/command', self.handle_twist_callback("swerve_b", "command"), 10)
        self.swerve_c_sub = self.create_subscription(Twist, '/swerve_c/command', self.handle_twist_callback("swerve_c", "command"), 10)
        self.swerve_d_sub = self.create_subscription(Twist, '/swerve_d/command', self.handle_twist_callback("swerve_d", "command"), 10)
        
        
        # Actual speed
        self.swerve_a_speed_sub = self.create_subscription(Float64, '/swerve_a/wheel_speed', self.swerve_callback("swerve_a", "speed"), 10)
        self.swerve_b_speed_sub = self.create_subscription(Float64, '/swerve_b/wheel_speed', self.swerve_callback("swerve_b", "speed"), 10)
        self.swerve_c_speed_sub = self.create_subscription(Float64, '/swerve_c/wheel_speed', self.swerve_callback("swerve_c", "speed"), 10)
        self.swerve_d_speed_sub = self.create_subscription(Float64, '/swerve_d/wheel_speed', self.swerve_callback("swerve_d", "speed"), 10)
        
        # subscribe to /hermes/cmd_vel  
        self.hermes_vel_sub = self.create_subscription(Twist, '/hermes/cmd_vel', self.velocity_callback, 10)
        
        
    def velocity_callback(self, msg):
        self.current_cmd_vel = msg
                
    def swerve_callback(self, swerve_module, data_name):
        def callback(msg):
            getattr(self, swerve_module)[data_name] = msg.data
        return callback
    
    def handle_twist_callback(self, swerve_module, data_name):
        def callback(msg):
            getattr(self, swerve_module)[data_name] = {
                "linear": {
                    "x": msg.linear.x,
                    "y": msg.linear.y,
                    "z": msg.linear.z,
                },
                "angular": {
                    "x": msg.angular.x,
                    "y": msg.angular.y,
                    "z": msg.angular.z,
                },
            }
        return callback
    

    def publish_move(self, moveCommand : Move):
        try:
            self.cmd_vel_pub.publish(Twist(
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
    
# return the actual speed data
@app.get("/velocity_data")
def velocity_data():
    return {
        "linear": {
            "x": round(node.current_cmd_vel.linear.x, 2),
            "y": round(node.current_cmd_vel.linear.y, 2),
            "z": round(node.current_cmd_vel.linear.z, 2),
        },
        "angular": {
            "x": round(node.current_cmd_vel.angular.x, 2),
            "y": round(node.current_cmd_vel.angular.y, 2),
            "z": round(node.current_cmd_vel.angular.z, 2),
        }
    }
