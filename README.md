# autoware.learn

#### Environment : 
- ubuntu 18.04.1 LTS 
- amd64
- cuda 10.0 (RTX3060)
- ros-melodic-desktop-full

#### installation : 
- autoware.ai generic DOCKER -v : 1.13.0


#### First compilation
```bash
mkdir -p autoware
cd autoware
git clone
# change the folder name from autoware.learn to src

AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
echo export LIBGL_ALWAYS_SOFTWARE=1 >> .bashrc ## for rviz
```

#### To compile a single (ros)package 
```bash
AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name> 
```

#### setup plugins
download file : libActorCollisionsPlugin.so into /usr/lib/x86_64-linux-gnu/gazebo-9/plugins/

```bash
sudo apt update
sudo apt-get install gedit
sudo apt-get install ros-melodic-urdf-tutorial
sudo apt-get install ros-melodic-rqt-robot-steering
sudo apt-get install ros-melodic-hector-gazebo-plugins
```


#### Rosbag set start time 
```
rosbag play -s n <bagname>
```

## Gazebo simulation with rviz

```
roslaunch autoware_quickstart_example mini_map.launch
roslaunch autoware_quickstart_example mini_localization.launch
roslaunch autoware_quickstart_example mini_sil_env.launch
```
![截屏2022-04-08 22 37 47](https://user-images.githubusercontent.com/57991090/162554773-d472f169-3702-40c8-974a-769fdc76ccc4.png)

## Euclidean Cluster

```
# run Gazebo simulation with rviz
roslaunch lidar_euclidean_cluster_detect lidar_euclidean_cluster_detect.launch
```

<img width="741" alt="截屏2022-04-09 05 33 50" src="https://user-images.githubusercontent.com/57991090/162554886-9c960a12-866a-49be-a081-4bb0a091efc9.png">

## kf Contour Tracker

```
# require Euclidean Cluster realtime results
roslaunch lidar_kf_contour_track lidar_kf_contour_track.launch
```

![截屏2022-04-16 02 36 38](https://user-images.githubusercontent.com/57991090/163656514-847eb0de-1b90-458a-8ddf-73cee538698c.png)


## Mapping with ndt-mapping

<a href="https://github.com/NicolasHHH/autoware.learn/blob/main/ndt-mapping.md"> Note: ndt-mapping with demo rosbag </a>


<img width="874" alt="截屏2022-03-28 19 54 45" src="https://user-images.githubusercontent.com/57991090/160462966-5d3bf778-a8e4-483d-baac-0f73cce5a3e8.png">
<img width="877" alt="截屏2022-03-28 19 58 50" src="https://user-images.githubusercontent.com/57991090/160462976-155d0ab7-e8a5-4660-87f8-989a983dbebf.png">

- ndt-mapping with simcity
 
<img width="769" alt="截屏2022-03-31 15 19 08" src="https://user-images.githubusercontent.com/57991090/161384763-1baa653b-5fe8-4d9d-a682-96a2c45d1fb7.png">

## Vector Map Builder by autoware.tools

<img width="1723" alt="截屏2022-03-31 15 43 22" src="https://user-images.githubusercontent.com/57991090/161384647-4f174281-c83a-43c9-84ac-67e77fd5ef85.png">

## Official Demo 

<img width="842" alt="截屏2022-03-28 19 47 18" src="https://user-images.githubusercontent.com/57991090/160462895-7d1c3991-8a74-4faf-9a69-a88e4fdb5b68.png">


