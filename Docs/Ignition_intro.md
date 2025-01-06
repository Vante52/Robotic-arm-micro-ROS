## GAZEBO Getting Started

* Install Gazebo Fortress
  ```sh
    sudo apt-get install ros-${ROS_DISTRO}-ros-gz
  ```
* Launch Ignition Gazebo
  ```sh
    ign gazebo shapes.sdf
  ```

* Launch Ignition Debug Mode
  ```sh
    ign gazebo shapes.sdf -v 4
  ```

* Launch Ignition Debug Mode and without GUI
  ```sh
    ign gazebo -s shapes.sdf -v 4
  ```

## GAZEBO Model Insertion from Fuel

* run 
```sh
    ign gazebo empty.sdf
```

### Tutorial

1. In the simulation go to the menu Button
2. Select save the world as ...
3. Select the location that you want to continue including at the of the file .sdf (example ras_world.sdf)
4. Choose a model in [Gazebo sim models](https://app.gazebosim.org/fuel/models)
  
5. Search for robots in the seeker option
6. Select X1 Config 6
7. Copy in the snippet clipboard.
8. Paste the content in the sdf file you save previously, before the `</world>`tag that
``` xml 
<sdf version='1.9'>
    ...
    ...
        <include>
        <uri>
        https://fuel.gazebosim.org/1.0/saurabh/models/X1 Config 5
        </uri>
        </include>
    </world>
</sdf>
```
9. Run
```sh
    ign gazebo ras_world.sdf
```

## Next
- [Ignition URDF](ignition_urdf.md)

## Previous
-  [Main](../README.md)