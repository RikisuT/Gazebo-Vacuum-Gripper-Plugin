# Gazebo Suction Plugin

A system plugin for Gazebo Sim (formerly Ignition Gazebo) that simulates a suction gripper.
**Tested on:** Gazebo Harmonic (gz-sim8)
**Compatibility:** Should work with any modern Gazebo version (Fortress, Garden, Harmonic, etc.) that supports the `gz-sim` and `gz-plugin` APIs.

## Features

- **Dynamic Attachment**: Uses `DetachableJoint` to create fixed joints between the gripper and target objects.
- **Configurable**: Customize suction radius, force, topic names, and target filters via SDF.
- **Vacuum Simulation**: Simulates the physics of a vacuum gripper pulling objects in.
- **Dual Modes**: Supports both contact-based and radius-based (vacuum) attachment.

## Usage

### 1. Add the Plugin to your Model

Add the following SDF snippet to your gripper model (e.g., inside the `<model>` tag):

```xml
<plugin filename="libsuction_plugin.so" name="gz::sim::systems::SuctionPlugin">
    <!-- Configuration Parameters -->
    <suction_topic>/suction/enable</suction_topic>   <!-- Topic to enable/disable suction -->
    <link_name>gripper_link</link_name>          <!-- Name of the link to attach to -->
    <filter_name>box</filter_name>                   <!-- Substring to match target model names -->
    
    <!-- Mode Selection -->
    <use_suction_radius>true</use_suction_radius>    <!-- Set to true for Vacuum Mode, false for Contact Mode -->
    
    <!-- Vacuum Mode Parameters -->
    <suction_radius>0.1</suction_radius>             <!-- Range (meters) to apply suction force -->
    <suction_force>20.0</suction_force>              <!-- Force (Newtons) to pull objects -->
</plugin>
```

### 2. Modes of Operation

#### A. Contact Sensor Mode (`use_suction_radius` = false)

This is the default mode. It requires a **Contact Sensor** on your gripper link. The plugin will only attach to objects that physically touch the gripper.

**Requirements:**

- You must add a `<sensor>` to your gripper link in the SDF:

    ```xml
    <link name="gripper_link">
        ...
        <sensor name="gripper_contact" type="contact">
            <contact>
                <collision>gripper_collision</collision> <!-- Name of your collision geometry -->
            </contact>
        </sensor>
    </link>
    ```

#### B. Radius/Vacuum Mode (`use_suction_radius` = true)

This mode simulates a vacuum. It detects objects within `suction_radius` and applies a `suction_force` to pull them towards the gripper. Once they are within a small threshold (2cm), they are attached.

**No contact sensor is required for this mode.**

### 3. Control

Publish a boolean message to the configured `suction_topic` to enable or disable the suction.

**Enable Suction:**

```bash
ign topic -t /suction/enable -m gz.msgs.Boolean -p 'data: true'
```

**Disable Suction (Detach):**

```bash
ign topic -t /suction/enable -m gz.msgs.Boolean -p 'data: false'
```

## Building

```bash
mkdir build
cd build
cmake ..
make
```

Ensure `GAZEBO_PLUGIN_PATH` includes your build directory:

```bash
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/path/to/your/build
```

## How It Works

The plugin operates as a `gz::sim::System` with `PreUpdate` and `PostUpdate` hooks.

1. **Initialization (`Configure`)**:
    - Reads parameters from the SDF (`suction_topic`, `suction_radius`, `suction_force`, etc.).
    - Sets up the topic subscriber for enabling/disabling suction.

2. **Activation**:
    - The plugin listens to the configured topic (default `/suction/enable`).
    - When `true` is received, `suctionActive` is set to true.

3. **Update Loop (`PreUpdate` / `PostUpdate`)**:
    - **Radius Mode (`PreUpdate`)**:
        - Iterates through all models in the world.
        - Filters models based on `filter_name`.
        - Calculates the distance between the gripper link and candidate models.
        - **Force Application**: If a model is within `suction_radius`, a force (`suction_force`) is applied towards the gripper using `ExternalWorldWrenchCmd`.
        - **Attachment**: If the distance is less than a threshold (0.02m), an attachment is scheduled.
    - **Contact Mode (`PostUpdate`)**:
        - Iterates through contacts reported by the **Contact Sensor**.
        - If a collision matches the `filter_name`, an attachment is scheduled.

4. **Attachment Logic**:
    - A `DetachableJoint` entity is created.
    - This joint connects the `parent_link` (gripper) and the target object's link with a `fixed` joint type.
    - This physically locks the object to the gripper, allowing it to be moved.

5. **Detachment**:
    - When `false` is received on the topic, the `DetachableJoint` entity is removed (`RequestRemoveEntity`).
    - The object is released and physics takes over (gravity, etc.).

## Attribution

This project evolved from multiple sources:

1. **Original Base**: Forked from [gazebo_ign_fortress_suction_plugin](https://github.com/pedrozambonini/gazebo_ign_fortress_suction_plugin) by Pedro Zambonini. The original plugin handled object removal (deletion).
2. **Logic Inspiration**: The suction and attachment logic was heavily inspired by the `GazeboRosVacuumGripper` from [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) (Gazebo Classic), adapted here for Gazebo Sim to use `DetachableJoint` instead of deletion.

## License

This project is licensed under the [MIT](LICENSE) license.
