# frenet_path_planner
C++ implementation of [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame)


### Dependencies
- [Eigen/Dense](https://eigen.tuxfamily.org/dox/GettingStarted.html)
- [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)
- [nlohmann/json](https://github.com/nlohmann/json)

### Install dependencies
```
sudo apt-get install build-essential cmake libeigen3-dev python-matplotlib python-numpy

```

```
cd include
git clone https://github.com/nlohmann/json.git

```

### How to run the demo

```
mkdir build
cd build
cmake ..
make

./FRENET_TEST
```

The path generation for the whole lap will be collected and it will be plotted a naive simulation like the following one.

![Frenet Planner Demo](imgs/frenet_planner_demo.png?raw=true "Title")

### TO-DO

- [x] Make the planner in a class
- [x] Load parameters from json, removing the ones in frenet_optimal_trajectory.cpp
- [x] Correct the matplot demo logging the steps and showing the complete run in the end
- [x] Adapt the parameters to the Indy racecar
- [x] Move dynamically the obstacles
- [x] Check collision with local portion of the track borders
- [ ] Constrain the side in which is better to overtake
- [ ] Create a documentation
- [ ] Parallelize on GPU following [this](https://www.researchgate.net/publication/328036343_Path_Planning_for_Highly_Automated_Driving_on_Embedded_GPUs)

### Developers
* **Ion Grigoras** - *Main developer for the f1tenth use case*
* **Ayoub Raji** - *Corrections and refactoring* 

### References and credits

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame)
- [Path Planning for Highly Automated Driving on Embedded GPUs](https://www.researchgate.net/publication/328036343_Path_Planning_for_Highly_Automated_Driving_on_Embedded_GPUs)
- [frenet_optimal_trajectory](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/FrenetOptimalTrajectory)
- [frenet_planner_agv](https://github.com/arvindjha114/frenet_planner_agv)
