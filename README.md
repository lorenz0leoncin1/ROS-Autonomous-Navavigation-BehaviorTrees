# ROS Package Setup

## Overview

This repository contains essential ROS packages that constitute the core of the system. To ensure proper functionality, these packages should be placed within the following directory path: `/catkin_ws/src/...`

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

