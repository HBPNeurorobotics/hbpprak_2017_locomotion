# Lauron Locomotion Challenge for NRP hands-on training in 2017/18 at FZI

This repository includes the needed files for the locomotion challenge "Lauron QWOP".

Please only use this repo for the **challenge definition** and not for solutions. Fork to implement your solution.

## IJuypter Notebooks:
The more in detail explaination on how to use the Motion Primitives given are explained in the IJupyter Notebooks (http://jupyter.org/).

start the notebook with "jupyter notebook"

### Nengo
The notebook "HowToNengo" will guide you to the modifications needed to make Nengo work alongside the NRP
### Motion Primitives
The notebook "LauronQWOP" will explain how to use the Motion Primitives


## Folders
### Experiments and Models
Copy the contents of the "Experiments" and the "Models" folder into the according NRP folders.

You will then see the "Locomotion Challenge - Lauron QWOP" Experiment and you can start it.

### Nengo
motionPrimitves.py needs to be imported, see juypter notebook for more infos on how to use it
testPrimitives.py is a minimal example

## Known issues
  - you might need to start the experiment, then restart it again in order for lauron to stand and not sack to the ground
  - you might need to install some controllers:
  	- sudo apt-get install ros-kinetic-joint-state-controller
    - sudo apt-get install ros-kinetic-effort-controllers
  	- sudo apt-get install ros-kinetic-position-controllers