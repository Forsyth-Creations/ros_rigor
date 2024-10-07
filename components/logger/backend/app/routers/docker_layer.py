from fastapi import APIRouter, Depends, HTTPException
import threading
import time
import docker
import os
import json
from colorama import Fore
from fastapi.responses import StreamingResponse

router = APIRouter()

logging_thread = None

active_logging_folder_name = None

# Kill event 
kill_event = threading.Event()

# Add a lifecycle event to stop the logging thread
@router.on_event("shutdown")
def shutdown_event():
    stop_logging_thread()

@router.get("/ls")
async def get_docker():
    try:
        client = docker.from_env()
        containers = client.containers.list()
        enumerated_containers = {}
        for idx, container in enumerate(containers):
            if container.name.startswith("buildx"):
                continue
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

def log_container_stats(client, sample_number):
    containers = client.containers.list()
    container_data = {}

    for container in containers:
        # If the container starts with builx, skip it
        if container.name.startswith("buildx"):
            continue
        try:
            # Get container stats
            stats = container.stats(stream=False)
            container_id = container.id
            name = container.name

            # CPU usage
            cpu_stats = stats["cpu_stats"]
            cpu_delta = cpu_stats["cpu_usage"]["total_usage"] - stats["precpu_stats"]["cpu_usage"]["total_usage"]
            system_cpu_delta = cpu_stats["system_cpu_usage"] - stats["precpu_stats"]["system_cpu_usage"]
            num_cpus = cpu_stats["online_cpus"]
            
            if system_cpu_delta > 0.0 and cpu_delta > 0.0:
                cpu_usage = (cpu_delta / system_cpu_delta) * num_cpus * 100.0
            else:
                cpu_usage = 0.0

            # Memory usage
            memory_stats = stats["memory_stats"]
            memory_usage = memory_stats["usage"]
            memory_limit = memory_stats["limit"]
            memory_percentage = (memory_usage / memory_limit) * 100.0 if memory_limit > 0 else 0.0

            # Create a timestamp
            epoch_time = int(time.time())

            # Store the data by container ID
            container_data[container_id] = {
                "name": name,
                "cpu_percent": cpu_usage,
                "mem_usage": memory_usage,
                "mem_percent": memory_percentage,
                "timestamp": epoch_time,
                "sample_number": sample_number,
            }

        except Exception as e:
            print(f"Error logging stats for {container.name}: {e}")

    return container_data

# Write data to the folder
def write_to_logs_folder(folder_name, data):
    global active_logging_folder_name
    print(f"{Fore.YELLOW}Writing logs to {folder_name}{Fore.RESET}")
    active_logging_folder_name = folder_name
    # make a logs directory
    for container_id, stats in data.items():
        file_path = os.path.join(folder_name, f"{stats['name']}.log")
        with open(file_path, "a") as log_file:
            log_file.write(json.dumps(stats) + "\n")

# Begin a logging thread
def container_logger(kill_event):
    folder_name = create_log_folder()
    client = docker.from_env()
    sample_number = 0

    global temp_logs
    temp_logs = {}  # Initialize the temp logs

    while not kill_event.is_set():
        try:
            # Get current container stats
            container_stats = log_container_stats(client, sample_number)
            
            # Add current stats to the temp_logs with timestamp
            current_time = time.time()
            temp_logs[current_time] = container_stats

            # Keep only the last 10 seconds of data in temp_logs
            temp_logs = {ts: stats for ts, stats in temp_logs.items() if current_time - ts <= log_duration}

            # Commit logs to the folder by component
            write_to_logs_folder(folder_name, container_stats)
        
        except Exception as e:
            print(f"Error during logging: {e}")

        time.sleep(1 / logger_frequency)
        sample_number += 1

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

#  Write an endpoint to stream the logs given a name.
#  Only stream the last 10 seconds of logs
@router.get("/logging/{name}")
async def stream_logs(name: str):
    global active_logging_folder_name
    folder_name = active_logging_folder_name
    print(f"Folder name: {folder_name}. Getting logs for {name}")
    if folder_name is None:
        raise HTTPException(status_code=404, detail="No logs available")
    file_path = os.path.join(folder_name, f"{name}.log")
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="Logs not found")
    
    print("Streaming logs...")
    
    async def log_streamer():
        with open(file_path, "r") as file:
            yield '['  # Start JSON array
            first_line = True
            for line in file:
                line = line.strip()
                if line:  # Skip empty lines
                    try:
                        log_json = json.loads(line)
                    except json.JSONDecodeError:
                        raise HTTPException(status_code=500, detail="Invalid JSON format in log file")
                    
                    if not first_line:
                        yield ','
                    first_line = False
                    yield json.dumps(log_json)
            yield ']'  # End JSON array

    return StreamingResponse(log_streamer(), media_type="application/json")