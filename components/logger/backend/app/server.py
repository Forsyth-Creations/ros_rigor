#!/usr/bin/env python3
# Written by Henry Forsyth, December 20th, 2023

from fastapi import FastAPI
from colorama import Fore, Style
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from fastapi import Depends

from app.routers import docker_layer

app = FastAPI(
    title="Logger API",
    version="0.1",
    description="API for the Logger",
    contact={"name": "Henry Forsyth", "email": "robert.h.forsyth@gmail.com"},
    servers=[
        {"url": "http://localhost:5000", "description": "Local Server"},
        {"url": "http://0.0.0.0:5000", "description": "Local Server"},
    ],
)



# Annotations
from typing import Annotated

# configure cors

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

app.include_router(
    docker_layer.router,
    prefix="/docker",
    tags=["docker"],
)


# -----------------------------------------------------------------------------------


@app.get("/")
def api():
    return "Congrats! You have reached the API! Have a cookie"