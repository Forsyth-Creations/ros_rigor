from fastapi import APIRouter, Depends, HTTPException
import threading
import time
import docker
import os
import json
from colorama import Fore

router = APIRouter()

logging_thread = None
# Kill event 
kill_event = threading.Event()

@router.get("/ls")
async def get_docker():
    try:
        client = docker.from_env()
        containers = client.containers.list()
        enumerated_containers = {}
        for idx, container in enumerate(containers):
            print(f"{idx}: Container ID: {container.id}: {container.name}")
            enumerated_containers[container.id] = container.name
        return enumerated_containers
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

temp_logs = {}
logger_frequency = 1  # Logging every 1 second
log_duration = 10  # Keep the last 10 seconds of logs

# Create a folder to store the logs
def create_log_folder():
    time_stamp = time.strftime("%Y%m%d-%H%M%S")
    os.makedirs(f"logs", exist_ok=True)
    folder_name = f"logs_{time_stamp}"
    os.makedirs(f"logs/{folder_name}", exist_ok=True)
    return f"logs/{folder_name}"

def log_container_stats(client):
    containers = client.containers.list()
    container_data = {}

    for container in containers:
        try:
            # Get container stats
            stats = container.stats(stream=False)
            container_id = container.id
            name = container.name

            # CPU usage
            cpu_stats = stats["cpu_stats"]
            precpu_stats = stats["precpu_stats"]
            cpu_delta = cpu_stats["cpu_usage"]["total_usage"] - precpu_stats["cpu_usage"]["total_usage"]
            system_delta = cpu_stats["system_cpu_usage"] - precpu_stats["system_cpu_usage"]
            num_cpus = len(cpu_stats["cpu_usage"]["percpu_usage"])

            cpu_percent = (cpu_delta / system_delta) * num_cpus * 100

            # Memory usage
            mem_usage = stats["memory_stats"]["usage"]
            mem_limit = stats["memory_stats"]["limit"]
            mem_percent = (mem_usage / mem_limit) * 100

            # Store the data by container ID
            container_data[container_id] = {
                "name": name,
                "cpu_percent": cpu_percent,
                "mem_usage": mem_usage,
                "mem_percent": mem_percent,
            }

        except Exception as e:
            print(f"Error logging stats for {container.name}: {e}")

    return container_data

# Write data to the folder
def write_to_logs_folder(folder_name, data):
    print(f"Writing logs to {folder_name}")
    # make a logs directory
    for container_id, stats in data.items():
        file_path = os.path.join(folder_name, f"{stats['name']}.log")
        with open(file_path, "a") as log_file:
            log_file.write(json.dumps(stats) + "\n")

# Begin a logging thread
def container_logger(kill_event):
    folder_name = create_log_folder()
    client = docker.from_env()

    global temp_logs
    temp_logs = {}  # Initialize the temp logs

    while not kill_event.is_set():
        try:
            # Get current container stats
            container_stats = log_container_stats(client)
            
            # Add current stats to the temp_logs with timestamp
            current_time = time.time()
            temp_logs[current_time] = container_stats

            # Keep only the last 10 seconds of data in temp_logs
            temp_logs = {ts: stats for ts, stats in temp_logs.items() if current_time - ts <= log_duration}

            # Commit logs to the folder by component
            write_to_logs_folder(folder_name, container_stats)

            print("Logging containers...")
        
        except Exception as e:
            print(f"Error during logging: {e}")

        time.sleep(1 / logger_frequency)

# ------------------------------------------------------------------------------------

def start_logging_thread():
    global logging_thread
    # Prevent starting a new thread if one is already running
    if logging_thread is not None and logging_thread.is_alive():
        print("Logging thread is already running")
        return

    kill_event.clear()  # Ensure the kill event is reset before starting
    logging_thread = threading.Thread(target=container_logger, args=(kill_event,))
    logging_thread.start()
    print(f"{Fore.GREEN}Logging thread started{Fore.RESET}")

def stop_logging_thread():
    if logging_thread is None or not logging_thread.is_alive():
        print("No logging thread to stop")
        return
    kill_event.set()
    logging_thread.join()
    print(f"{Fore.RED}Logging thread stopped{Fore.RESET}")
    kill_event.clear()  # Reset the kill event after stopping

@router.get("/logging/start")
async def start_logging():
    start_logging_thread()
    return {"status": "Logging started"}

@router.get("/logging/stop")
async def stop_logging():
    stop_logging_thread()
    return {"status": "Logging stopped"}

@router.get("/logging/status")
async def get_logging_status():
    if logging_thread is None or not logging_thread.is_alive():
        return False
    return True
