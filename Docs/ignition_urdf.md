## GAZEBO Spawn URDF 
* URDF files are often used in ROS to represent robot models.
* SDF can describe a world with multiple robot models.
* URDF can only describe one robot model.

### Tutorial

1. Create an empty world
```bash 
    ign gazebo empty.sdf
```
2. In a new terminal run

```bash
ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/path/to/model.urdf", name: "urdf_model"'
```
Don't forget to replace `/path/to/model.urdf`for the real path where is located your urdf.

## Next
- [Ignition Plugin](ignition_plugin.md)
## Previous
-  [Ignition Getting Started](Ignition_intro.md)