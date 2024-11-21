#!/usr/bin/env python3
# Written by Henry Forsyth, December 20th, 2023

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
from threading import Thread

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
    direction: str
    speed: Optional[int] = 5


# ROS 2 Node setup
class RobotPublisher(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(String, 'robot_move', 10)
        self.get_logger().info("RobotPublisher Node has been started.")

    def publish_move(self, direction, speed):
        msg = String()
        msg.data = f"Direction: {direction}, Speed: {speed}"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


# Create a ROS 2 node and start it in a separate thread
node = None

def start_ros_node():
    global node
    rclpy.init()
    node = RobotPublisher()
    rclpy.spin(node)
    
# Start a separate thread to run ROS 2 node (since FastAPI runs asynchronously)
ros_thread = Thread(target=start_ros_node)
ros_thread.start()

# FastAPI endpoint to handle movement commands
@app.post("/move")
def move(moveCommand: Move):
    # Publish the move command to ROS 2
    direction = moveCommand.direction
    speed = moveCommand.speed
    node.publish_move(direction, speed)

    return {"message": "Move command received", "direction": direction, "speed": speed}
