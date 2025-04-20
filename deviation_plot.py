import csv
import matplotlib.pyplot as plt
import numpy as np

with open('log.txt', 'r') as f:
    reader = csv.reader(f, delimiter=',')
    data = list(reader)
    data = [[float(x) for x in row] for row in data]
    data = np.array(data)

# Plot x-y data in 2D
plt.figure(figsize=(10, 6))
plt.plot(data[:, 0], data[:, 1], label='X-Y Data', color='blue')
plt.scatter(data[:, 0], data[:, 1], color='red', s=10)  # Scatter plot for points
plt.title('X-Y Data Plot')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.grid()
plt.legend()
plt.savefig('xy_plot.png')
plt.show()