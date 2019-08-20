# Coverage Planning

## 0. Overview

Decompose the given polygon if concave

Compute a bow-shape complete coverage path for every polygon



## 1. Dependency

- Eigen

- OpenCV

- Others (You could replace with your own Struct)

  The ROS messages customized including

  -  `ros_msgs::Vector2` 
    - float64 x
    - float64 y
  - `ros_msgs::PoseWithVelocity` 
    - Vector3 position (x, y, z)
    - Quaternion orietation (x, y, z, w)
    - float64 velocity
    - int32 flag 
  - `ros_msgs::Trajectory` 
    - PoseWithVelocity[] poses



## 2. Reference

The decomposition algorithm is proposed by ZHU chuanmin, TANG jun and XU tiangui 

from College of Mechanical Engineering, Tongji University, Shanghai, China

The paper link : https://wenku.baidu.com/view/a3ccf9abf705cc1755270974.html

The reason I take this algorithm cause I came from Tongji



## 3. Run

```cmake
# build
mkdir build
cd build
cmake ..
make
# run
./planner	
```

