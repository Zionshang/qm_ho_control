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
* webots 2023b

    Download webots 2023b in official website.Download [webots project](https://github.com/Zionshang/Webots). Open `AliengoZ1/worlds/AliengoZ1_desiredArmJoint.wbt`
![robot](doc/robot.png)

* Eigen

```
sudo apt update
sudo apt install libeigen3-dev 
sudo ln -s /usr/local/include/eigen3 /usr/include/eigen3
sudo ln -s /usr/local/include/eigen3/Eigen/ /usr/local/include/Eigen
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
sudo apt update
sudo apt install libyaml-cpp-dev 
```
