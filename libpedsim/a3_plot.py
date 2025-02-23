import numpy as np
import matplotlib.pyplot as plt

# Data from the provided results
scenarios = ["scenario.xml", "hugeScenario.xml", "scenario_box.sml"]

# OpenMP version (parallel)
openmp_times = [
    np.mean([225, 242, 252, 297, 249]),  # scenario.xml
    np.mean([5202, 5357, 7812, 8175, 8380]),  # hugeScenario.xml
    np.mean([235, 228, 250, 244, 224])   # scenario_box.sml
]

# Sequential version
sequential_times = [
    np.mean([601, 610, 600, 595, 620]),  # scenario.xml
    np.mean([16941, 17034, 17264, 24192, 18030]),  # hugeScenario.xml
    np.mean([4, 3, 3, 5, 4])  # scenario_box.sml
]

# Plotting
fig, ax = plt.subplots(figsize=(8, 5))
x = np.arange(len(scenarios))  # label locations
width = 0.35  # width of the bars

ax.bar(x - width/2, openmp_times, width, label="OpenMP (Parallel)", color="blue")
ax.bar(x + width/2, sequential_times, width, label="Sequential", color="red")

ax.set_xlabel("Scenarios")
ax.set_ylabel("Execution Time (ms)")
ax.set_title("Performance Comparison: OpenMP vs Sequential")
ax.set_xticks(x)
ax.set_xticklabels(scenarios)
ax.legend()

plt.show()
