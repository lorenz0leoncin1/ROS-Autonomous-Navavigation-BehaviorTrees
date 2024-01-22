# ROS Package Setup

## Overview

This repository contains essential ROS packages that constitute the core of the system. To ensure proper functionality, these packages should be placed within the following directory path: `/catkin_ws/src/...`

The main objective of this project is to enhance the capabilities of an autonomous robot through vocal interaction. The concentration of the project has been exclusively on the development of the `navigation` packages and the utilization of the ROS-Behavior-Tree library.

## Getting Started

1. **Clone the Repository:**

    ```bash
    git clone <repository_url>
    ```

2. **Move Packages to catkin Workspace:**

    - Navigate to your ROS catkin workspace:

      ```bash
      cd /path/to/catkin_ws/src/
      ```

    - Copy or move the cloned repository content into the `src` directory.

3. **Build the Workspace:**

    - Return to the catkin workspace root:

      ```bash
      cd /path/to/catkin_ws/
      ```

    - Build the workspace:

      ```bash
      catkin_make
      ```

## Directory Structure

The ROS packages follow a specific directory structure within the `src` directory:


## Usage

After successfully setting up the packages in your catkin workspace, you can proceed with launching and running the system using ROS commands.

```bash
# Example: Source the setup.bash file
source /path/to/catkin_ws/devel/setup.bash

# Run your ROS nodes or launch files
roslaunch your_package_name your_launch_file.launch
