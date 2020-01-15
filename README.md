# arduino-mapping

This repository contains code written to test the capabilities of environment mapping within a robotic system. The task at hand is exploring the challenge of obstacle characterisation. A baseline solution is produced with the use of an infrared (IR) proximity sensor. Subsequently, an optimised solution using two proximity sensors is produced. The goal is to understand how the optimisation influences the task of mapping. 

It also includes a report detailing the design of the experiment, implementations and evaluation of performance between the baseline and optimised solutions.

The experiment and its implementations have been carried out on a Pololu Romi 32U4 with a Sharp GP2Y0A60SZLF IR proximity sensor.

## Team

[Tim Nguyen](https://github.com/nt1m), [Andrei Nitu](https://github.com/AndreiCNitu), [Ainsley Rutterford](https://github.com/ainsleyrutterford) and [Faizaan Sakib](https://github.com/fznsakib)

Produced as part of the *Robotics Systems* unit at the University of Bristol.

## Hypothesis

> Infrared proximity sensors perform with a varying degree of accuracy, determined by the distance being measured. They are prone to a higher degree of error at the limits of their operation. External factors based on the environment and the system itself can introduce additional error. We hypothesise that with the introduction of a second sensor positioned appropriately, smaller distances can be reliably measured where previously not possible due to an increased combined range of distance covered. Furthermore, a combination of two sensor readings will help to produce better estimations.

## Results

<p align="center">
  <img src="graphs/results_visualisation.svg" alt="Results visualisation">
</p>

<p align="center">
  <img src="graphs/error_visualisation.svg" alt="Error visualisation">
</p>
