# Unicore UM982 ROS 2 Driver

A ROS 2 driver for the Unicore UM982 dual-antenna RTK GPS receiver.

This driver provides high-frequency, high-accuracy position and heading data to the ROS 2 ecosystem by parsing the proprietary Unicore PVTSLN message format and publishing standard ROS 2 navigation messages.

## Features

- Parses Unicore PVTSLN ASCII messages
- Publishes sensor_msgs/NavSatFix for position data
- Publishes sensor_msgs/Imu for heading and velocity data
- Automatic hardware configuration
- RTK correction support via NTRIP
- ROS 2 diagnostics integration
- Configurable via ROS 2 parameters

## Requirements

- ROS 2 (Humble or later)
- Linux operating system
- Unicore UM982 dual-antenna RTK GPS receiver
- RTKLIB (str2str utility)

## Installation and Usage

[Documentation will be completed during implementation]
