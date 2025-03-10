# qm_ho_control

Hierarchical optimization control for quadrupedal manipulator

## Dependence

* webots 2023b
* Eigen
* pinocchio
* qpOASES
* yaml-cpp

## Install

## Install Dependence
* webots 2025a

    Download webots 2025a in official website.
    Download [webots project](https://github.com/Zionshang/Webots). Open `galileo_mini_x5/worlds/galileo_mini_x5.wbt`
![robot](doc/robot.png)

* Eigen

```
sudo apt update
sudo apt install libeigen3-dev
```

* pinocchio

[Recommended installation](https://stack-of-tasks.github.io/pinocchio/download.html)

If you're stuck in this step
```
 echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
     | sudo tee /etc/apt/sources.list.d/robotpkg.list
```
Try
```
sudo echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
     | sudo tee /etc/apt/sources.list.d/robotpkg.list
```

* qpOASES

```
git clone https://github.com/coin-or/qpOASES.git
mkdir build
cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make
sudo make install
```

* yaml-cpp
```
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake ..
make
sudo make install
```
## Run

press key in webots
```
1: stance gait
2: trot gait
3: fly trot gait
4: stance trot gait
w: increase forward speed
s: increases back speed
a,d: increase lateral speed
j,l: increase angular speed
```

