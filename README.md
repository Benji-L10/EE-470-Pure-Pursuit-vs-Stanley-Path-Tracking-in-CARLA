# Pure Pursuit vs. Stanley Path Tracking in CARLA

## Project Overview

This project compares two classical lateral path-tracking controllers, **Pure Pursuit** and **Stanley control**, in the CARLA autonomous driving simulator. The goal is to evaluate how each controller performs under different vehicle speeds and path geometries.

Accurate path tracking is a core requirement for autonomous vehicles because the vehicle must follow a desired trajectory while minimizing cross-track error, heading error, and unstable steering behavior. This project focuses on the lateral control portion of the autonomous driving stack.

The project was completed for **EE 470: Planning and Control for Autonomous Vehicles** at Cal Poly.

## Team Members

- Alex Nguyen
- Benjamin Ly

## Simulation Environment

- CARLA Simulator
- Python API
- Simulated autonomous vehicle following predefined reference paths

## Project Objective

The objective of this project is to implement and compare **Pure Pursuit** and **Stanley** controllers for autonomous vehicle path tracking in simulation.

The comparison focuses on:

- Cross-track error
- Heading error
- Steering smoothness
- Settling behavior after turns
- Controller performance at different speeds
- Controller behavior on different path geometries

## Controllers

### Pure Pursuit Controller

Pure Pursuit is a geometric path-tracking controller that selects a look-ahead point on the reference path and computes the steering angle needed to drive toward that point.

The steering command is computed as:

```math
\delta = \tan^{-1}\left(\frac{2L\sin(\alpha)}{L_d}\right)