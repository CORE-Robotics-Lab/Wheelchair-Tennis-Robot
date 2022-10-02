---
layout: page
title: Performance
permalink: /performance/
nav_order: 4
---



# Performance

- [Performance](#performance)
  - [Drive Train](#drive-train)
  - [Planner](#planner)
  - [Strategizer](#strategizer)
  - [Swing](#swing)
  - [Human Serve](#human-serve)
  - [Offroading](#offroading)


Here are some videos of the current state of the robot.

## Drive Train

This video shows the high speed velocity setpoint tracking of the drive train. The wheels are shown on the left and the commanded vs. actual wheel velocities are on the right. 

<video width="100%" autoplay controls loop muted poster>
    <source src="../assets/videos/motors.mp4" type="video/mp4">
</video>


## Planner

To get the wheelchair to where it needs to be in time to hit a tennis ball, the wheelchair takes the goal position and creates a local and global plan to get there. This video shows the planner generating a global plan (in green) as it executes the move from start to goal position.

<video width="100%" autoplay controls loop muted poster>
    <source src="../assets/videos/planning.mp4" type="video/mp4">
</video>

## Strategizer

This video shows the robot's perception of the expected ball trajectory overlayed on top of video from one of the cameras used for ball tracking. Here it shows how the strategizer uses the expected ball trajectory to determine where the wheelchair needs to be and moves there for the swing. 

<video width="100%" autoplay controls loop muted poster>
    <source src="../assets/videos/rollout.mp4" type="video/mp4">
</video>

## Swing

With the wheelchair in the correct spot, it is able to use the arm swing a tennis racket at the tennis ball. This video shows the wheelchair stationary swinging at a tennis ball that has the correct trajectory. 

<video width="100%" autoplay controls loop muted poster>
    <source src="../assets/videos/swing.mp4" type="video/mp4">
</video>

## Human Serve

When humans serve the ball, they inevitably add a lot of unintended spin to the ball. This makes it very hard to know where the ball is going after it hits the ground. This video shows how the wheelchair is capable of hitting the ball even served from a human. 

<video width="100%" autoplay controls loop muted poster>
    <source src="../assets/videos/human.mp4" type="video/mp4">
</video>

## Offroading

Despite being designed for driving on a tennis court, the wheelchair is also capable of traversing many other terrains. Here we demonstrate it's ability to drive on a grassy field at high speeds. 

<video width="100%" autoplay controls loop muted poster>
    <source src="../assets/videos/offroading.mp4" type="video/mp4">
</video>