# Model Predictive Path Integral Control

![mppi](https://github.com/MosesTheRedSea/MPPI/blob/master/mppi_demo.gif)

## How it works
- The following MPPI implementation is adapted from [this research paper](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7487277)
- The algorithm utilizes a monte carlo procedure by producing random noise (gaussian splatter) over a model prediction horizon
- Cost is calculated for each path in the set of paths based on a model of the robot
- Path integral: The chosen control command is a weighted sum of all the monte carlo paths
- A temperature value is provided to encourage exploration or exploitation
- The control command is applied to the robot and the process is repeated
- All-in-all, the control is the minimization of the expected cost of a trajectory weighted and combined with other trajectories:
  
- Note: The annimation may show slight overlap of robot and obstacle without detecting a collision. This is because of the drawing function giving an extra 0.5 width overlap from the actual obstacle border


