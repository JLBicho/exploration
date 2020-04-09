# exploration
Node to create a zone for exploration 

# Description
This package contains a node that creates a zone which is then decomposed in a grid of points. These points form a path which is passed thought a topic.  

## ROS Version
Kinetic

## Files
launch
  - exploration.launch
  
src
  - exploration_node.cpp

## Topics
<b>exploration_node.cpp</b>
  - Subs:
    - /clicked_point (geometry_msgs::PointStamped): points clicked in Rviz that conform the polygon 
    - /start (std_msgs::String): publsih 'start' to begin grid generator
  - Pubs:
    - /exploration_zone (geomerty_msgs::PolygonStamped): polygon of the zone
    - /points (visualization_msgs::Marker): markers of the points
    - /path (nav_msgs::Path): path to follow
  
