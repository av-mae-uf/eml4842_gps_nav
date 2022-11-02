# Information
The following figure depicts all the nodes, topics, and messages that are used in the gps_nav package.
![gps_nav package](./gps_nav/data/images/gps_nav.jpg)

- Nodes are shown in ellipses.
- Topics are shown using lines with arrows.
- The messages used for each topic are shown.  Black text is used for standard messages.  Blue text is used for custom interface messages that are in the package *gps_nav_interfaces*.
- Parameters are shown in green.
- The vehicle_controller node is using an algorithm where a third order path is generated to move from the current vehicle pose to the current goal pose.  The radius of curvature at the start of this path is used to generate the motion twist to send to the vehicle.

## Installation/Build
Create a development workspace by typing the following when in your home directory. Replace the text within the <> to be whatever you want.
```bash
mkdir -p <your_dev_ws>/src
```
Move into the **src** directory.
```bash
cd <your_dev_ws>/src
```

Download the repository using
```bash
git clone https://github.com/av-mae-uf/eml4930_gps_nav.git
```
Change your directory to your root workspace directory (*~/<your_dev_ws>*) and build the workspace.
```bash
colcon build
``` 
Source your workspace with
```bash
source install/setup.bash
```
## How to run simulation
Run the command below to see the simulation at work.
```bash
ros2 launch gps_nav simulation_demo.launch.py
``` 
## How to run on vehicle
Included in the *gps_nav* package are two additional launch files:
* vehicle.launch.py
* visualization.launch.py

You should use the **vehicle.launch.py** launch file when starting the code on the vehicle. This launch file does not include the visualizations and simulator.
```bash
ros2 launch gps_nav vehicle.launch.py
```

You can run the **visualization.launch.py** launch file on your own computer in order to bring up rviz and display the path and other visualizations.
```bash
ros2 launch gps_nav visualization.launch.py
```