# ENPM667-Project-1
Nishant Awdeshkumar Pandey, Tyler Barrett, Rishikesh Jadhav

This repository contains the files needed to run our simulation for the paper "Adaptive control for UAVs equipped with a robotic arm" from Caccavale et al. 

In order to run the simulation, you will need to install the following dependencies:
- Python 3
- numpy
- sympy

This can be installed by executing `pip -r requirements.txt`. 
The latest versions of numpy and sympy will be installed to your default Python interpreter.

After these dependencies are installed, feel free to run "python main.py" in order to execute our code.

##Disclaimer
Due to the buggy nature of our code, the simulation may crash before reaching its maximum runtime of 60 seconds. 
If this is the case, feel free to run `python minimal_main.py` to run the simulation for the full 60 seconds. 
This will not generate motor inputs, but will only generate the accelerations determined by the inverse kinematics.