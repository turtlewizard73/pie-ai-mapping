# pie-ai-mapping

depends on project: https://github.com/turtlewizard73/pie-mapping-and-navigation.git

test with:  
```
gazebo --verbose /usr/share/gazebo-11/worlds/cafe.world
```

gazebo env variables:
```
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/turtlewizard/repos/workspace/src/pie-mapping-and-navigation/gazebo_actor_collisions_plugin/build
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/turtlewizard/repos/workspace/src/pie-ai-mapping/pie_ai_bringup/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/turtlewizard/repos/workspace/src/pie-ai-mapping/pie_ai_bringup/models
```