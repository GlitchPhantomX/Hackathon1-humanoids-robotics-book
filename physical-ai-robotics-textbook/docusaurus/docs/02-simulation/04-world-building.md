---
sidebar_position: 5
title: 'World Building: Creating Complex Simulation Environments'
description: 'Designing and implementing complex simulation environments for humanoid robotics'
---

<h1 className="main-heading">World Building: Creating Complex Simulation Environments</h1>
<div className="underline-class"></div>

Build realistic simulation environments for humanoid robot testing - from indoor spaces to outdoor terrains.

<div className="border-line"></div>

<h2 className="second-heading">Learning Objectives</h2>
<div className="underline-class"></div>

- • Design simulation environments with physics properties
- • Create indoor/outdoor environments
- • Implement dynamic and interactive objects
- • Configure lighting and visual effects
- • Optimize simulation performance

<div className="border-line"></div>

<h2 className="second-heading">Exercises</h2>
<div className="underline-class"></div>

<details>
<summary>Exercise 2.4.1: Basic Indoor Environment (⭐, ~30 min)</summary>

<h3 className="third-heading">Exercise 2.4.1: Basic Indoor Environment</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐ | **Time**: 30 min | **Requirements**: Gazebo, XML knowledge

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Room structure (walls, floor, ceiling)
- • Furniture (table, chair)
- • Lighting and physics

<h4 className="fourth-heading">Success Criteria</h4>
<div className="underline-class"></div>

- [ ] Room loads in Gazebo
- [ ] Collision/visual elements work
- [ ] Lighting is functional
- [ ] Physics stable

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
gz sdf -k your_indoor.world
gazebo your_indoor.world
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Use static flags for immovable objects
- • Validate SDF before testing

</details>

<details>
<summary>Exercise 2.4.2: Advanced Environment (⭐⭐, ~45 min)</summary>

<h3 className="third-heading">Exercise 2.4.2: Advanced Environment</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐ | **Time**: 45 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Indoor/outdoor transitions
- • Dynamic movable objects
- • Multiple lighting scenarios
- • Different surface materials

<h4 className="fourth-heading">Success Criteria</h4>
<div className="underline-class"></div>

- [ ] Static and dynamic elements work
- [ ] Surface materials configured
- [ ] Good performance

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
gazebo your_advanced.world
gz model -m moving_platform -x 1 -y 0 -z 1
```

</details>

<details>
<summary>Exercise 2.4.3: Modular World System (⭐⭐⭐, ~60 min)</summary>

<h3 className="third-heading">Exercise 2.4.3: Modular World System</h3>
<div className="underline-class"></div>

**Difficulty**: ⭐⭐⭐ | **Time**: 60 min

<h4 className="fourth-heading">Starter Code</h4>
<div className="underline-class"></div>

- • Reusable environment components
- • Performance optimization
- • Multiple testing scenarios

<h4 className="fourth-heading">Test Commands</h4>
<div className="underline-class"></div>
```bash
gz sdf -k model://room_module/model.sdf
gazebo modular_environment.world
```

<h4 className="fourth-heading">Hints</h4>
<div className="underline-class"></div>

- • Standard connection points
- • Consistent coordinate systems

</details>

<div className="border-line"></div>

<h2 className="second-heading">Troubleshooting</h2>
<div className="underline-class"></div>

<details>
<summary>Common Issues</summary>

<h3 className="third-heading">Troubleshooting</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">Slow Simulation</h4>
<div className="underline-class"></div>

**Symptoms**: Low real-time factor, high CPU/GPU usage

**Solutions**:
```xml
<visual><geometry><box><size>2 2 0.1</size></box></geometry></visual>
<model static="true"><!-- content --></model>
<physics><max_step_size>0.01</max_step_size></physics>
```

<h4 className="fourth-heading">Objects Fall Through Surfaces</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<collision><surface><contact><ode>
  <kp>1e+13</kp><kd>1</kd>
</ode></contact></surface></collision>
```

<h4 className="fourth-heading">Incorrect Lighting</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<light type="point">
  <pose>0 0 3 0 0 0</pose>
  <diffuse>0.9 0.9 0.9 1</diffuse>
</light>
```

<h4 className="fourth-heading">Floating/Sinking Objects</h4>
<div className="underline-class"></div>

**Solutions**:
```xml
<model><pose>2.0 1.0 0.5 0 0 0</pose></model>
<collision><pose>0 0 0 0 0 0</pose></collision>
```

</details>

<div className="border-line"></div>

<h2 className="second-heading">Environment Design</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Realism vs Performance</h3>
<div className="underline-class"></div>

- • **Polygon count**: Keep meshes simple
- • **Collision geometry**: Use simplified shapes
- • **Physics**: Balance realism with efficiency
- • **Textures**: Appropriate resolution

<h3 className="third-heading">Environment Types</h3>
<div className="underline-class"></div>

<h4 className="fourth-heading">Indoor</h4>
<div className="underline-class"></div>
```xml
<world name="indoor_lab">
  <include><uri>model://ground_plane</uri></include>
  <light type="point"><pose>0 0 3 0 0 0</pose></light>
  <model name="wall"><pose>-5 0 1.5 0 0 0</pose></model>
</world>
```

<h4 className="fourth-heading">Outdoor</h4>
<div className="underline-class"></div>
```xml
<world name="outdoor_park">
  <model name="terrain">
    <link><collision><geometry>
      <heightmap><uri>heightmap.png</uri></heightmap>
    </geometry></collision></link>
  </model>
</world>
```

<div className="border-line"></div>

<h2 className="second-heading">Physics Configuration</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Material Properties</h3>
<div className="underline-class"></div>
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <gravity>0 0 -9.8</gravity>
</physics>
<surface><friction><ode><mu>0.8</mu></ode></friction></surface>
```

<h3 className="third-heading">Friction Models</h3>
<div className="underline-class"></div>
```xml
<!-- High friction (rubber) -->
<surface><friction><ode><mu>1.0</mu></ode></friction></surface>
<!-- Low friction (ice) -->
<surface><friction><ode><mu>0.1</mu></ode></friction></surface>
```

<h2 className="second-heading">Dynamic Elements</h2>
<div className="underline-class"></div>

<h3 className="third-heading">Moving Objects</h3>
<div className="underline-class"></div>
```xml
<model name="moving_platform">
  <joint type="prismatic">
    <axis><xyz>0 1 0</xyz></axis>
  </joint>
</model>
```

<h3 className="third-heading">Interactive Objects</h3>
<div className="underline-class"></div>
```xml
<model name="cup">
  <link><collision><geometry>
    <cylinder><radius>0.05</radius></cylinder>
  </geometry></collision></link>
</model>
```

<h2 className="second-heading">Lighting</h2>
<div className="underline-class"></div>
```xml
<!-- Directional (sun) -->
<light type="directional"><direction>-0.5 0.1 -0.9</direction></light>
<!-- Point light -->
<light type="point"><pose>0 0 3 0 0 0</pose></light>
```

<h2 className="second-heading">Best Practices</h2>
<div className="underline-class"></div>

- • Mark static objects as static
- • Simplify collision geometry
- • Limit dynamic elements
- • Optimize mesh complexity
- • Use modular design

<h3 className="third-heading">Modular Design</h3>
<div className="underline-class"></div>
```xml
<world name="modular_environment">
  <include><uri>model://room_module_1</uri></include>
  <include><uri>model://room_module_2</uri></include>
</world>
```

<h3 className="third-heading">World Generation</h3>
<div className="underline-class"></div>
```python
import xml.etree.ElementTree as ET

def create_world_with_obstacles(world_name, obstacles):
    sdf = ET.Element('sdf', version='1.7')
    world = ET.SubElement(sdf, 'world', name=world_name)
    # Add obstacles
    tree = ET.ElementTree(sdf)
    tree.write(f'{world_name}.world')
```

<h2 className="second-heading">Summary</h2>
<div className="underline-class"></div>

Balance realism with performance when creating simulation environments. Use modular design for reusability and maintain proper physics configuration for effective humanoid robot testing.

import ReadingTime from '@site/src/components/ReadingTime';

<ReadingTime minutes={6} />