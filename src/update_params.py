import os

path = os.path.expanduser("~/sora_ws/src/ProjectSora_Ros/src/sora_slam/config/nav2_params.yaml")
with open(path, "r") as f:
    text = f.read()

# Make robot radius 0.28m
text = text.replace("robot_radius: 0.22", "robot_radius: 0.28")

# Add pointcloud to observation sources
text = text.replace("observation_sources: scan", "observation_sources: scan pointcloud")

# Add pointcloud definitions
rep = """            obstacle_min_range: 0.0
          pointcloud:
            topic: /zedm/zed_node/point_cloud/cloud_registered
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "PointCloud2"
            raytrace_max_range: 3.0
            raytrace_min_range: 0.0
            obstacle_max_range: 2.5
            obstacle_min_range: 0.0
        static_layer:"""

text = text.replace("            obstacle_min_range: 0.0\n        static_layer:", rep)

with open(path, "w") as f:
    f.write(text)
print("Updated parameters!")
