# ME 133b Robotics Course 2025

This repository contains the code and resources for the ME 133b Robotics course at Caltech for the year 2025.

## Repository Structure

- `turtlebot/`: TurtleBot3 ROS2 packages and configurations
- `turtlebot3_simulations/`: Simulation environments and configurations for TurtleBot3
- `atlas_description/`: URDF and mesh files for the Atlas robot
- `build_turtlebot.sh`: Script to build TurtleBot-related packages

## Setup Instructions

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 (Robot Operating System 2)
- Python 3.x
- Git

### Installation Steps

1. Clone this repository:
```bash
git clone https://github.com/itsyashk/ME133b-2025.git
cd ME133b-2025
```

2. Build the TurtleBot packages:
```bash
./build_turtlebot.sh
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Components

### TurtleBot3
The repository includes TurtleBot3 configurations and simulations. TurtleBot3 is a small, affordable, programmable, ROS-based mobile robot for use in education, research, hobby, and product prototyping.

### Atlas Robot
Contains the URDF description and mesh files for the Atlas humanoid robot, which can be used for visualization and simulation purposes.

## Usage

Detailed instructions for running specific components will be added as the course progresses.

## Contributing

Please follow the standard GitHub workflow:
1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to your fork
5. Submit a Pull Request

## License

This project is part of the ME 133b course at Caltech. All rights reserved.

## Contact

For any questions or issues, please open a GitHub issue in this repository.