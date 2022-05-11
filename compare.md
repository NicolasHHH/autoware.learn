### simulation:

vehicle_gazebo_simulation_launcher/launch

	gazebo_launcher.launch : default = mcity/gazebo_city9.world
	
	world_test.launch : car_model = full_sensor actor_collisions.world +
	world_test_citysim_a.launch +
	world_test_citysim_b.launch +

```
hector_gazebo_plugins +
```

### visualization : 

vehicle_model/urdf:

	2lidars_vehicle.xacro +
	full_sensors_vehicle.xacro +

vehicle_model/config:

	calibration.yaml +
		base_link2velodyne_front:
		  x: 3.9
		  y: 0.0
		  z: 0.6
		  roll: 0.0
		  pitch: 0.0
		  yaw: 0.0

gazebo_world_description/worlds:
	
	actor_collisions.world +
	simple_city.world +

### documentation :

```
config: 
	headless_setup.yaml +

launch: 
	mini_env_demo +
	sil_env_demo +

Install hector_gazebo_plugins :opt/ros/melodic/share/
```



### core_perception

```
gnss_localizer/nodes:
	fix2tfpose.cpp
	comments 
	fix2tfpose.launch plane = 20

lidar_elidean_cluster_detect.launch
	<arg name="use_vector_map" default="false" />

points_preprocessor
	cloud_transformer.launch +
	
```



### core_planning

```
astar_search:
	comments

op_planner:
	op_trajectory_evaluator_core.cpp
	
waypoint_planner
	velocity_set.launch
	<arg name="points_topic" default="points_lanes" />

	velocity_set_option.launch
	<node pkg="waypoint_planner" type="velocity_set_lanelet2" name="velocity_set" output="screen">
	
	velocity_set_path.cpp
	if ((current_vel_ >= 0.0 && current_vel_ <= closest_vel) || (current_vel_ < 0.0 && current_vel_ > closest_vel))
    return;
```



