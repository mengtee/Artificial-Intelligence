# Artificial-Intelligence: Ronny's Rubbish Room Maze Solver

## Overview

This Python script is designed to assist Ronny in navigating through a maze of rooms, collecting and disposing of rubbish efficiently. The script utilizes the A* search algorithm to find an optimal path, considering factors such as room distances, rubbish weight, and volume.

## Features

- **Node Representation:** The code defines a `Node` class to represent rooms in the maze, storing relevant information such as coordinates, rubbish details, and A* algorithm values (F, G, H).

- **A* Search Algorithm:** The main functionality is the A* search algorithm implemented to find the optimal path from the initial state to the goal state, considering room distances and rubbish details.

- **Dynamic Search:** The script dynamically switches between collecting rubbish and disposing it based on the current state and rubbish bin capacity.

## Usage

1. **Run the Script:**
   ```bash
   python maze_solver.py

# Input State Space:

The script uses a predefined state space representing the maze, including room coordinates, rubbish details, and disposal room flags.

# Output Path and Cost:

The script outputs the calculated path and cost for Ronny to follow, including collected rubbish weight and volume.

# State Space Definition
The state_space list defines the maze layout, including coordinates, rubbish details, and disposal room flags.

# Customization
The initial state is set to the first room in the state_space list, but users can modify it for different starting points.

# Dependencies
Python 3

