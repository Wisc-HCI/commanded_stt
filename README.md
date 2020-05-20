# Commanded STT

## Description
ROS2 node that listens for a hotword in the background, e.g.```hey robot```, and then transcribes speech in real time.

## Setup
Simply place the top ```commanded_stt``` folder in your workspace and run ```colcon build```. You will also need to place a valid google cloud API service key in your workspace. The API key should be in a json file.

Ensure that you have a working microphone connected to your computer.

## Running
Run the following

```
ros2 run commanded_stt listener_node /absolute/path/to/mycroft-precise
```

Currently, the only recognizable hotword is the "robot say" command.