Code has been submitted by Huzefa Kagalwala (C48290423) and Rahil Modi (C14109603)

BASIC REQUIREMENTS:

These are the following parameters for the basic requirements:
ksi = 0.5
Sensing Horizon (dhor) = 10
Time Horizon (timehor) = 5
Maximum Force = 10
The maximum speed, goal speed, radius and other parameters are extracted from the CSV
Note: The initial 4 values mentioned here are hard-coded in the script "agent.py", because erroneous values were being called from "simulator.py". Use the simulator.py in the submission provided for extra safety 

EXTRA CREDITS:

POWER LAW:
The parameters mentioned in the "BASIC REQUIREMENTS" are also common for this scenario
tau_0 = 3
These are the seperate parameters for the epsilion = 0 and 0.02:
1) EPSILON = 0.2
k = 1.05
m = 1
2) EPSILON = 0
k = 1.272
m = 1

VELOCITY PERTURBATION:
Radius of disk from which velocity perturbations are sampled (nu) = 0.1
epsilon = 0.2
Other parameters are common
The TTC model is used here, instead of PowerLaw

ADVERSARIAL UNCERTAINTY:
Same parameters as Velocity Perturbation
TTC Model is used here instead of of PowerLaw

