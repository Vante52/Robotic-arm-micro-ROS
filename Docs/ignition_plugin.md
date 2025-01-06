## Ignition Gazebo Plugin

1. Create a folder
```bash
mkdir ignition_plugin
```
2. Go to the folder 
```bash
cd ignition_plugin
```

3. Create the CMakeLists.txt with the following:
```cpp
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(Hello_world)

ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

ign_find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})

add_library(HelloWorld SHARED HelloWorld)
set_property(TARGET HelloWorld PROPERTY CXX_STANDARD 17)
target_link_libraries(HelloWorld
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER})
```

4. Create the HelloWorld.hh header file with:

```cpp 
/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SYSTEM_PLUGIN_HELLOWORLD_HH_
#define SYSTEM_PLUGIN_HELLOWORLD_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/sim/System.hh>

// It's good practice to use a custom namespace for your project.
namespace hello_world
{
  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
  // plugins that want to send commands.
  class HelloWorld:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate
  {
    // Plugins inheriting ISystemPostUpdate must implement the PostUpdate
    // callback. This is called at every simulation iteration after the physics
    // updates the world. The _info variable provides information such as time,
    // while the _ecm provides an interface to all entities and components in
    // simulation.
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
  };
}
#endif
```

5. Create the HelloWorld.cc file with:

```cpp 
/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// We'll use a string and the ignmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "HelloWorld.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
IGNITION_ADD_PLUGIN(
    hello_world::HelloWorld,
    gz::sim::System,
    hello_world::HelloWorld::ISystemPostUpdate)

using namespace hello_world;

// Here we implement the PostUpdate function, which is called at every
// iteration.
void HelloWorld::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &/*_ecm*/)
{
  // This is a simple example of how to get information from UpdateInfo.
  std::string msg = "Hello, world! Simulation is ";
  if (!_info.paused)
    msg += "not ";
  msg += "paused.";

  // Messages printed with ignmsg only show when running with verbosity 3 or
  // higher (i.e. ign gazebo -v 3)
  ignmsg << msg << std::endl;
}
```

6. Create the world called hello_world_plugin.sdf to include the plugin.

```html
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!--
      System plugins can be loaded from <plugin> tags attached to entities like
      the world, models, visuals, etc.
    -->
    <plugin filename="HelloWorld" name="hello_world::HelloWorld">
    </plugin>
  </world>
</sdf>
```

### Build

* Build the plugin (in the root of the folder)
```bash
    mkdir build && cd build
    cmake ../
    make
```

#### Run

* Run the simulation
```bash
    cd ..
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
    ign gazebo -v 3 hello_world_plugin.sdf
```

## Next
- [Ignition ROS](ignition_ros.md)
## Previous
-  [Ignition URDF](ignition_urdf.md)
