# MATLink

Checkout the branch for the chapter you want to use! or keep reading.

## What is in this repo?

This repo contains code for compiled sfuctions that are run by the matlab simulator contained in the [Matlab_simulator](https://github.com/MAGICC-UAVbook/Matlab_simulator) repository. It is resposible for the Matlab-mavlink connection for Hardware in the loop simulation. These repositories are pretty tightly coupled, meaning this repo will only work with the corrisponding branch of that repo.

## What is the concept behind Hardware in the Loop (HIL)?

So your c/c++ code compiles but it might freeze the autopilot as soon as you try to fly.  Before trying to fly the autopilot and debug while flying, first run the autopilot in the simulator where you can debug the autopilot at your desk.  The idea is that the apps that you've written on the pixhawk autopilot can't tell the difference whether they are flying or receiving inputs from the simulation.

## How do I use this?

The simulator will attempt to run the MATLink.mexa64 file. MATLink will then send and receive mavlink messages but it must be connected to a pixhawk (with an FTDI cable) to make the simulation work. Of course the autopilot must be configured correctly (running the right code and streaming the right topic, see [Firmware](https://github.com/MAGICC-UAVbook/Firmware) repo).  There is also a standalone application that can be run outside of matlab for debuging. It is the matlink_test file.

note: the HIL was only ever attempted by for the chapter 6 and chapter 8 code.  The chapter 11 branch was only used as a standalone app to send waypoints to the autopilot.

## How do I compile this stuff to run in matlab?

To my knowledge, this code has only successfully been compiled in Ubuntu 14.04 but there is no reason someone with the right skills couldn't get it to work in Windows as well.  Compiling requires the right compiler for matlab (gcc/g++ 4.7 for linux) and telling where the compiler can find the right matlab headers to make it a sfunction.  The mavlink messages are sent over serial using the boost librarys, so they need to be installed as well. See the wiki associated with this repo for more infomation.

## Why is this code so messy?

Is it possible to be proud of something and discusted with it at the same time?  This code comes a ROS node called mavros. Mavros is for connecting ROS to autopilots using the mavlink protocol.  Basically I commented and deleted ROS from mavros and figured out how to compile it into a mex function. 

## You obviously didn't figure this all out by yourself. Where are your credits.

Lots of this sfunction stuff comes from matlab documentation.  Much of which can be found at MATLABROOT/toolbox/simulink/simdemos/simfeatures/src/ on your computer, including the sfun_counter_cpp.cpp file that shows how to uses pointers and classes within a compiled sfuncstion.  I also used [this tutorial](http://www.mathworks.com/matlabcentral/fileexchange/45522-mex-cmake) by Fang Liu to fingure out how to compile it with cmake (sorry its note better credited throughout the code).  Credit should also go to Vladimir Ermakov for the mavros portion of this code (again sorry).
