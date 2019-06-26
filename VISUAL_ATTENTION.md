## Visual Attention

### The projection on the environement

We use PyBullet as Physics Engine. 

Inputs: elements of the env, many human head poses
Outputs: a PointCloud2 for each pair of human-element

Process: project n raycasts from each head pose and get the hit position.


### Update the visual attention
<table><tr><td>
Algorithme (when new message is received):
  
    FOREACH new_perception:
      FOREACH human in new_perception:
        IF isNew(human):
          createNewHuman()
        FOREACH object_id, object_pointcloud in human:
          IF isNew(object_id):
            createNewObject(object_id, object_pointcloud)
          ELSE:
            updateObjectValues(object_id, object_pointcloud)
</td></tr></table>


<table><tr><td>
Algorithme updateObjectValues:
    
    current_pointcloud = list_of_objects.get(object_id)
    
    IF isEmpty(object_pointcloud):
      object.perception_time = 0
    ELSE:
      FOREACH new_point in object_pointcloud:
        near_points = KdTreeFLANNSearch(current_pointcloud, new_point, radius)
        IF isEmpty(near_points):
          addNewPoint(current_pointcloud, new_point)
        ELSE:
          FOREACH near_point in near_points:
            near_point.intensity += new_point.intensity
            updateMaxIntensity(near_point.intensity)
            object.last_time = object_pointcloud.timestamp
            object.perception_time += dtime
</td></tr></table>
