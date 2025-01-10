#!/usr/bin/env python3

import struct
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.colors as mcolors
import time
matplotlib.use('TkAgg')

colors = [(1, 0, 0, 0),    # Red with alpha=0
          (1, 0, 0, 1)]    # Red with alpha=1
cmap = mcolors.LinearSegmentedColormap.from_list('custom_red', colors)


def skimFrame(file):
    num_agents = struct.unpack('Q', file.read(8))[0]  # Read number of agents
    file.seek(num_agents * 4, 1) # Jump over the coordinates of the agents
    file.seek(8, 1)  # jump over the heatmap_start magic

    heatmap_height = 120 * 5
    heatmap_width = 160 * 5
    heatmap_total_elem = heatmap_height * heatmap_width

    file.seek(heatmap_total_elem, 1)


def readFrame(file):
    num_agents = struct.unpack('Q', file.read(8))[0]  # Read number of agents


    # Using numpy here isn't so beneficial...
    #raw_data = file.read(num_agents*2*2) # 2B per value and 2 values per agent
    #integers = np.frombuffer(raw_data, dtype=np.int16)
    #tuples = integers.reshape(-1, 2) # Reshape into pairs (e.g., (x, y))
    #frame_agents = [tuple(pair) for pair in tuples] # convert into python list of tuples

    frame_agents = []
    for _ in range(num_agents):
        x, y = struct.unpack('hh', file.read(4))  # Read 16-bit integers for x and y
        frame_agents.append((x, y))

    heatmap_height = 120 * 5
    heatmap_width = 160 * 5
    heatmap_total_elem = heatmap_height * heatmap_width

    frame_intensities = []

    file.read(8) # jump over the heatmap_start magic code

    # Using numpy here is about 6x faster than the struct.unpack below
    # 7s vs 42s
    raw_data = file.read(heatmap_total_elem)
    frame_heatmap = np.frombuffer(raw_data,
                                      dtype=np.uint8).reshape(600,800)

    #for _ in range(heatmap_total_elem):
    #    redValue = struct.unpack('B', file.read(1))[0]  # Read 32-bit integer intensity
    #    frame_intensities.append(redValue)

    #print(frame_agents[:5])
    #print(len(frame_agents))
    #print(len(frame_heatmap))

    return frame_agents, frame_heatmap

def ReadSingleFrame(file, offsets, idx):
    file.seek(offsets[idx])
    return readFrame(file)

def deserialize(file, max_frame, lightScan):
    # Step 1: Read the total number of frames (first 4 bytes)
    total_frames = struct.unpack('I', file.read(4))[0]
    print(f"Total frames: {total_frames}")

    total_frames = min(total_frames, max_frame)

    frames = []
    frames_heatmap = []
    frames_file_offset = []

    for frame in range(total_frames):
        frames_file_offset.append(file.tell())
        if lightScan:
            skimFrame(file)
        else:
            agents, heatmap = readFrame(file)
            frames.append(agents)
            frames_heatmap.append(heatmap)

    return frames, frames_heatmap, frames_file_offset

def splitUniqueCommonAgents(agents):
    seen = set()
    duplicates = set()

    for item in agents:
        # If item is already in seen, it's a duplicate
        if item in seen:
            duplicates.add(item)
        else:
            seen.add(item)

    # Truly unique elements = all seen elements minus duplicates
    unique_elements = seen - duplicates

    return list(unique_elements), list(duplicates)


def plot(file, offsets, num_steps):
    # Initialize the plot
    fig, ax = plt.subplots(figsize=(6, 4))  # Adjust figure size (aspect ratio ~160:120)
    plt.subplots_adjust(bottom=0.3)  # Adjust space for the widgets

    # Set up the 160x120 coordinate system
    ax.set_xlim(0, 160)
    ax.set_ylim(0, 120)
    ax.invert_yaxis()  # (0, 0) is now at the top-left corner
    ax.set_aspect('equal', adjustable='box')  # Ensure the aspect ratio stays correct

    # Add a fine grid at every 1 unit
    ax.set_xticks(range(0, 161, 5), minor=True)   # Grid at 5-unit intervals (x-axis)
    ax.set_yticks(range(0, 121, 5), minor=True)   # Grid at 5-unit intervals (y-axis)
    ax.set_xticks(range(0, 161, 20), minor=False)  # Major ticks every 20 units (x-axis)
    ax.set_yticks(range(0, 121, 20), minor=False)  # Major ticks every 20 units (y-axis)
    ax.grid(which='minor', color='gray', linestyle='--', linewidth=0.5)  # Fine grid
    ax.grid(which='major', color='gray', linestyle='--', linewidth=0.5)  # Fine grid

    # Scatter plot for points
    sc_green = ax.scatter([], [], s=10, color='green', edgecolor='black',
                          zorder=1)
    sc_red = ax.scatter([], [], s=10, color='red', edgecolor='black', zorder=2)
    step_label = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12,
                         va='top', zorder=4)

    # Heatmap
    im = ax.imshow(np.random.randint(0, 256, size=(600, 800), dtype=np.uint8), extent=[0, 160, 120, 0],
                   aspect='auto',
                   cmap=cmap,
                   #interpolation='nearest'
                   alpha=0.7,
                   zorder=3)


    # Initial plot setup
    current_step = 0
    def update_plot(step):
        global current_step
        current_step = step
        agents, heatmap = ReadSingleFrame(file, offsets, step)

        greenAgents, redAgents = splitUniqueCommonAgents(agents)
        if len(greenAgents) == 0:
            greenAgents = np.empty((0,2))
        if len(redAgents) == 0:
            redAgents = np.empty((0,2))
        sc_green.set_offsets(greenAgents)
        sc_red.set_offsets(redAgents)
        im.set_array(heatmap)

        step_label.set_text(f'Step: {step + 1}/{num_steps}')
        fig.canvas.draw_idle()

    update_plot(0)

    ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03])
    slider = Slider(ax_slider, 'Step', 1, num_steps, valinit=1, valstep=1)

    def slider_update(val):
        step = int(slider.val) - 1
        update_plot(step)

    slider.on_changed(slider_update)

    # Buttons
    ax_next = plt.axes([0.85, 0.025, 0.1, 0.04])
    ax_prev = plt.axes([0.7, 0.025, 0.1, 0.04])
    ax_play = plt.axes([0.5, 0.025, 0.1, 0.04])

    btn_next = Button(ax_next, 'Next')
    btn_prev = Button(ax_prev, 'Previous')
    btn_play = Button(ax_play, 'Play')

    def next_step(event):
        step = (current_step + 1) % num_steps
        slider.set_val(step + 1)

    def prev_step(event):
        step = (current_step - 1) % num_steps
        slider.set_val(step + 1)

    def play(event):
        for step in range(current_step, num_steps):
            slider.set_val(step + 1)
            time.sleep(0.1)  # Adjust delay for playback
            plt.pause(0.01)

    btn_next.on_clicked(next_step)
    btn_prev.on_clicked(prev_step)
    btn_play.on_clicked(play)

    plt.show()

def main():
    # Check if filename is provided as command line argument
    if len(sys.argv) != 2:
        print(f"Usage: python3 {sys.argv[0]} <filename>")
        sys.exit(1)
        #TODO add max_frame as argument
    
    # Get filename from command line argument
    filename = sys.argv[1]
    max_frame = 1000
    
    with open(filename, 'rb') as file:
        # Process the file
        start_time = time.time()
        frames, heatmaps, frames_offsets = deserialize(file, max_frame, True)
        end_time = time.time()
        print(f"Found {len(frames_offsets)} frame.")
        print(f"Total execution time: {end_time - start_time:.6f} seconds")

        plot(file, frames_offsets, len(frames_offsets))

if __name__ == '__main__':
    main()
