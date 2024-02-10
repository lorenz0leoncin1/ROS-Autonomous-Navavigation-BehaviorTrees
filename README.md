# Project Details

This repository is part of a larger project related to my thesis work. The thesis, written in Italian, provides additional context and in-depth details about the project. You can find the full thesis document in the link down below.

**Nota:** La tesi è scritta in italiano. Per una comprensione approfondita del progetto, ti invitiamo a leggere la tesi completa.

[Tesi di Laurea](https://drive.google.com/file/d/1E8GlbGUdTeAOGt3cpGkW1saGbpR9sHlA/view?usp=sharing)

## Project Overview

The main objective of this project is to enhance the capabilities of an autonomous robot through vocal interaction. The starting point involves basic behaviors controlled by voice commands. The goal is to take these initial behaviors and use Behavior Trees in C++ to make them more complex, increasing the level of dynamism and adaptability of the robot's behaviors.

## Key Achievements

- **Developed new features for an autonomous robot using the ROS operating system**
- **Designed and implemented new complex skills using Behavior Trees in C++**
- **Structured and implemented vocal interaction using Python and YAML, leveraging the RASA framework for vocal interaction with the robot**
- **Resolved a bug in the RASA Action Server to extend the duration of custom actions from 10 seconds to hours**

## Contact

For any inquiries or further information, feel free to contact me at [lorenzoleoncini18@gmail.com].



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


## Launch Script

Within the `navigation` package, you can find a helpful Bash script named `launch_my_dialogue.sh`. This script automates the startup process of the entire system.
