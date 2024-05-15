# slamrs
[![Github Pages](https://github.com/antbern/slamrs/actions/workflows/slamrs_pages.yml/badge.svg)](https://github.com/antbern/slamrs/actions/workflows/slamrs_pages.yml)
[![Build & Test](https://github.com/antbern/slamrs/actions/workflows/slamrs_rust.yml/badge.svg?job=Check)](https://github.com/antbern/slamrs/actions/workflows/slamrs_rust.yml)

This repository contains the code for my attempt to overcome the issues faced with the old [gridmap-slam-robot](https://github.com/antbern/gridmap-slam-robot) project.
The issues were mainly related to performance of the algorithms and the absense of a possibility to publish for the web.
I have since also upgraded the system with a LIDAR unit from a Neato robot vacuum cleaner, such as those described in [ssloy/neato-xv11-lidar](https://github.com/ssloy/neato-xv11-lidar),
which also increases the amount of data being available to be processed. 

## Current Features
* Home-grown immediate mode OpenGL shape drawing library using vertex buffers and shaders.
  - Rewrite and improved version of the renderer developed in [antbern/gridmap-slam-robot](https://github.com/antbern/gridmap-slam-robot), 
  originally inspired by the [`ShapeRenderer` in `libgdx`](https://github.com/libgdx/libgdx/blob/16d873f332d6182e089c0747988b5d747844b8cd/gdx/src/com/badlogic/gdx/graphics/glutils/ShapeRenderer.java).
* A fully typed topic-based [publish-subscribe system](slamrs/pubsub/).
* A system of `Nodes` that communicate through the pub-sub system. 
    * Fully declarative configuration file for enabling and connecting nodes to topics as well as configuring their parameters.
    * A simulator for a differential drive robot with a laser range scanner (a.k.a a LIDAR).
    * Ability to connect to and control the robot with a Neato LIDAR via the serial port or over a TCP connection.
    * Implementation of point-to-plane ICP for doing scan matching (link to videos and resources).
    * Implementation of grid-based SLAM using a particle filter and Bayesian log-odds update rules.
    * Fully flexible and customizable data visualization node.

* Runs on desktop and in [the browser](https://antbern.github.io/slamrs/) through wasm! (except for nodes that load files or connects to the serial port)


* Robot with a Neato XV11 LIDAR
  * Rewrite of [gridmap-slam-robot project](https://github.com/antbern/gridmap-slam-robot) for the [iLabs Challenger RP2040 WiFi/BLE MkII](https://ilabs.se/product/challenger-rp2040-wifi-ble-with-chip-antenna/)
    board containing a `rp2040` dual-core processor and a `esp32` companion chip for network capabilities.
  * [Firmware](slamrs-robot-rtic/firmware) written using [`rtic`](https://rtic.rs/).

## Future Directions
An unfiltered (and unstructured) list of high-level ideas for future development:

* A Node that can record and replay data on topics, with support for persistence and time-stamped messages.
* Improve configuration for sensor and robot motion models, bot forward and inverse.
* Improved pointmap map that can be used for more "global" mapping with ICP.
  - Need to drop points in a smart way to keep complexity of the ICP algorightm down.
  - Can we use this with a particle filter?
* Extraction of "landmarks" from lidar data
  - Perhaps including the strength values from the LIDAR unit, would need to be added to the simulator. 
* Implement EKF or graph-based slam using the extracted landmarks.
* Add "exploration mode" where driving commands are generated to explore the world and gain more knowledge about it.
* Path generation and following by mouse click on the map.
* Add dynamic "Obstacles" to the Simulator.
* Saving and loading of maps (+global localization? could use particle filter with uniform starting locations for that)
* ~~Connection with "real robot", updated from my previous [gridmap-slam-robot project](https://github.com/antbern/gridmap-slam-robot).~~
* Arbitrary input configuration for Nodes, i.e., node inputs can be connected to topics or constants. Same with outputs.
* _Async? Can we make us of that in a good way?_ 
  - Right now only reasonable when waiting for a value to arrive through the pub-sub system, but might be relevant still.



