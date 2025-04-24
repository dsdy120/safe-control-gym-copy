import matplotlib.pyplot as plt

def plot_deviation_from_log(log_file):
    waypoint_x = []
    waypoint_y = []
    current_x = []
    current_y = []

    # Read data from log file
    with open(log_file, 'r') as file:
        for line in file:
            try:
                parts = line.strip().split(',')
                waypoint_x.append(float(parts[0]))
                waypoint_y.append(float(parts[1]))
                current_x.append(float(parts[-3]))
                current_y.append(float(parts[-2]))

            except (ValueError, IndexError):
                # Skip lines that don't match the expected format
                continue

    plt.figure(figsize=(10, 6))
    plt.plot(current_x, current_y, label='Current Position', color='blue', marker='o')
    plt.plot(waypoint_x, waypoint_y, label='Waypoint', color='red', marker='x')

    # Automatically adjust axis limits to fit the data
    plt.axis('equal')
    plt.autoscale()

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Deviation Over Time')
    plt.legend()
    plt.grid(True)
    plt.show()

# Example usage
plot_deviation_from_log('log.txt')