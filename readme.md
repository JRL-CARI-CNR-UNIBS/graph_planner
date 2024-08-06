## How to configure the human-aware motion planner for your setup

- copy [config/cari_planning.yaml](config/cari_planning.yaml) in `{your_package}`_moveit_config and change the required fields (see comments in the file)

- copy [launch/cari_planning_pipeline.launch.xml](launch/cari_planning_pipeline.launch.xml) in `{your_package}`_moveit_config and change the required fields (see comments in the file)


- in `{your_package}`_moveit_config/move_group.launch, add the lines:
```xml
<!-- CARI -->
<include file="$(dirname)/planning_pipeline.launch.xml">
  <arg name="pipeline" value="cari" />
</include>
```
