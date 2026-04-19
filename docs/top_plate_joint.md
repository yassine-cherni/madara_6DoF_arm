For MAking the DC Motor Full Continuous you need this changes 

---

### Fix 1 — `madara_description/urdf/madara_core.xacro`

```xml
<!-- BEFORE -->
<joint name="top_plate_joint" type="continuous">
  <parent link="base_plate"/>
  <child link="top_plate"/>
  <origin rpy="0 0 1.5" xyz="0 0 0.048"/>
  <axis xyz="0 0 1"/>
  <limit effort="1.0" velocity="2.0"/>
</joint>

<!-- AFTER -->
<joint name="top_plate_joint" type="revolute">
  <parent link="base_plate"/>
  <child link="top_plate"/>
  <origin rpy="0 0 1.5" xyz="0 0 0.048"/>
  <axis xyz="0 0 1"/>
  <limit effort="1.0" lower="-3.14159" upper="3.14159" velocity="2.0"/>
</joint>
```

---

### Fix 2 — `madara_moveit_config/config/joint_limits.yaml`

Add position limits to match:

```yaml
  top_plate_joint:
    has_position_limits: true      # ← add this
    min_position: -3.14159         # ← add this
    max_position:  3.14159         # ← add this
    has_velocity_limits: true
    max_velocity: 1.88
    has_acceleration_limits: false
    max_acceleration: 0
```

---


```bash
ros2 launch madara_bringup gz_launch.py
```
