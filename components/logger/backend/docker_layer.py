import docker
import time
import plotly.graph_objs as go
from plotly.subplots import make_subplots

# Initialize Docker client
client = docker.from_env()

def get_container_stats(container_id):
    # Fetch real-time stats from the container
    container = client.containers.get(container_id)
    stats = container.stats(stream=False)

    # CPU Usage Calculation
    cpu_stats = stats["cpu_stats"]
    cpu_delta = cpu_stats["cpu_usage"]["total_usage"] - stats["precpu_stats"]["cpu_usage"]["total_usage"]
    system_cpu_delta = cpu_stats["system_cpu_usage"] - stats["precpu_stats"]["system_cpu_usage"]
    num_cpus = cpu_stats["online_cpus"]

    if system_cpu_delta > 0.0 and cpu_delta > 0.0:
        cpu_usage = (cpu_delta / system_cpu_delta) * num_cpus * 100.0
    else:
        cpu_usage = 0.0

    # Memory Usage Calculation
    memory_stats = stats["memory_stats"]
    memory_usage = memory_stats["usage"]
    memory_limit = memory_stats["limit"]
    memory_percentage = (memory_usage / memory_limit) * 100.0 if memory_limit > 0 else 0.0

    return {
        "cpu_usage": cpu_usage,
        "memory_usage": memory_usage,
        "memory_limit": memory_limit,
        "memory_percentage": memory_percentage,
    }

def plot_utilization(time_intervals, cpu_usage_data, memory_usage_data, memory_percentage_data):
    # Create a figure with two y-axes: one for CPU usage and one for memory usage
    fig = make_subplots(specs=[[{"secondary_y": True}]])

    # Plot CPU usage
    fig.add_trace(
        go.Scatter(x=time_intervals, y=cpu_usage_data, name="CPU Usage (%)", line=dict(color='blue')),
        secondary_y=False
    )

    # Plot memory usage percentage
    fig.add_trace(
        go.Scatter(x=time_intervals, y=memory_percentage_data, name="Memory Usage (%)", line=dict(color='green')),
        secondary_y=True
    )

    # Add figure details
    fig.update_layout(
        title_text="Container CPU and Memory Utilization Over Time",
        xaxis_title="Time (seconds)",
    )

    # Set y-axes titles
    fig.update_yaxes(title_text="CPU Usage (%)", secondary_y=False)
    fig.update_yaxes(title_text="Memory Usage (%)", secondary_y=True)

    fig.show()

if __name__ == "__main__":
    # List all containers
    containers = client.containers.list()
    enumerated_containers = {}
    for idx, container in enumerate(containers):
        print(f"{idx}: Container ID: {container.id}: {container.name}")
        enumerated_containers[idx] = container.id
        
    # Choose a container through input
    container_idx = int(input("Enter the container index: "))
    container_id = enumerated_containers[container_idx]

    # Variables to store utilization data
    time_intervals = []
    cpu_usage_data = []
    memory_usage_data = []
    memory_percentage_data = []

    start_time = time.time()
    while time.time() - start_time < 30:  # Collect data for 30 seconds
        stats = get_container_stats(container_id)
        elapsed_time = time.time() - start_time

        # Append data for plotting
        time_intervals.append(elapsed_time)
        cpu_usage_data.append(stats['cpu_usage'])
        memory_usage_data.append(stats['memory_usage'] / (1024**2))  # Convert to MB
        memory_percentage_data.append(stats['memory_percentage'])

        # Log the stats
        print(f"Time: {elapsed_time:.2f}s | CPU Usage: {stats['cpu_usage']:.2f}% | "
              f"Memory Usage: {stats['memory_usage'] / (1024**2):.2f} MB / {stats['memory_limit'] / (1024**2):.2f} MB "
              f"({stats['memory_percentage']:.2f}%)")

        time.sleep(0.5)  # Log every 0.5 seconds

    # After 30 seconds, plot the utilization data
    plot_utilization(time_intervals, cpu_usage_data, memory_usage_data, memory_percentage_data)
