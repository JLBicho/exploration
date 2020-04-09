/* 	
	Author: Jose Luis Millan Valbuena

	This node is used to create a grid of a selected zone in the map.
	The grid is then used to create a path to follow thought the zone.
*/

/* Libraries */
#include <math.h>

/* ROS libraries */
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/ColorRGBA.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>

#include <nav_msgs/Path.h>


class Zone{
public:
	/* Class constructor. Initialization of publishers, subscribers and variables */ 
	Zone(){
		std::cout<<"[ZONE] Starting Zone generator"<<std::endl;

		// Subscribers
		sub_clicked_pts_ = nh_.subscribe("clicked_point", 10, &Zone::clickedPointsCb, this);
		sub_start_ = nh_.subscribe("/start", 1, &Zone::startCb, this);
		
		// Publishers
		pub_polygon_ = nh_.advertise<geometry_msgs::PolygonStamped>("exploration_zone", 10);
		pub_points_ = nh_.advertise<visualization_msgs::Marker>("points",10);
		pub_path_ = nh_.advertise<nav_msgs::Path>("path",10);
		
		// Private variables initialization
		points_.header.frame_id = "map";
		points_.ns = "points";
		points_.id = 1;
		points_.type = visualization_msgs::Marker::CUBE_LIST;
		points_.action = visualization_msgs::Marker::ADD;
		points_.pose.position.x = 0;
		points_.pose.position.y = 0;
		points_.pose.position.z = 0;
		points_.pose.orientation.x = 0;
		points_.pose.orientation.y = 0;
		points_.pose.orientation.z = 0;
		points_.pose.orientation.w = 1;
		points_.scale.x = 0.05;
		points_.scale.y = 0.05;
		points_.scale.z = 0.05;
		
		// RGBA color Green
		green_.r = 0.0;
		green_.g = 1.0;
		green_.b = 0.0;
		green_.a = 1.0;
		// RGBA color Red
		red_.r = 0.0;
		red_.g = 1.0;
		red_.b = 0.0;
		red_.a = 1.0;

		// Frame ID
		exploration_zone_.header.frame_id = "map";
		path_.header.frame_id="map";
	}

	/* Class destructor.*/
	~Zone(){
		nh_.shutdown();
		std::cout<<"\n[ZONE] Destroying Zone generator. Goodbye.\n"<<std::endl;
	}

	/* Callback for receiving clicked points in Rviz */
	void clickedPointsCb(const geometry_msgs::PointStamped::ConstPtr& msg){
		geometry_msgs::Point32 pt;
		pt.x = msg->point.x;
		pt.y = msg->point.y;
		pt.z = msg->point.z;

		exploration_zone_.polygon.points.push_back(pt);
		std::cout<<"[ZONE] Point received: "<<pt.x<<", "<<pt.y<<", "<<pt.z<<std::endl;

		// Once there are 3+ points, the polygon is published to view in Rviz
		if (exploration_zone_.polygon.points.size()>2){
			pub_polygon_.publish(exploration_zone_);
		}
	}

	/* Callback for start msg */
	void startCb(const std_msgs::String::ConstPtr& msg){
		if(msg->data == "start"){
			// Shutsdown clikedPointsCb to avoid new points after initializing
			sub_clicked_pts_.shutdown();
			std::cout<<"[ZONE] Shutting down point receiver.\nStarting grid generation."<<std::endl;
			
			// Grid generator creates the grid inside the zone
			gridGenerator();
		}else{
			std::cout<<"[ZONE] Msg received, but it is not 'start'."<<std::endl;
		}
	}

	/* This function generates a rectangular grid around the zone, and proceeds to check which points are inside the zone.*/
	void gridGenerator(){
		std::cout<<"[ZONE] Generating grid"<<std::endl;

		// Get the vectices of the surrounding rectangle
		geometry_msgs::Point down_left, up_right;
		down_left.x = 1e3;
		down_left.y = 1e3;
		up_right.x = -1e3;
		up_right.y = -1e3;
		for (std::vector<geometry_msgs::Point32>::iterator it = exploration_zone_.polygon.points.begin(); it < exploration_zone_.polygon.points.end(); it++)
		{
			if (it->x < down_left.x){
				down_left.x = it->x;
			}
			if (it->y < down_left.y){
				down_left.y = it->y;
			}
			if (it->x > up_right.x){
				up_right.x = it->x;
			}
			if (it->y > up_right.y){
				up_right.y = it->y;
			}
		}
		
		// Generates de grid
		// dX, dY represent the size of the grid, in each axis
		double dX = 0.4;
		double dY = 0.4;

		// Auxiliary variables
		geometry_msgs::Point newPoint;
		geometry_msgs::PoseStamped p;
		p.header.frame_id = "map";

		// To make a more useful path, the Y coordinate changes the wap (up to down, then down to up)
		bool down_2_up = true;
		// Loops throught all the points in the grid, checking if they are inside the zone
		for (double i = down_left.x; i <= up_right.x; i+=dX){
			if (down_2_up){
				for (double j = down_left.y; j <= up_right.y; j+=dY){
					newPoint.x = i;
					newPoint.y = j;
					points_.points.push_back(newPoint);

					// Check if the newPoint is inside. If it is, then green and it is used in the path. If not, then red and not used.
					if(inside(newPoint, exploration_zone_, down_left)){
						grid_.push_back(newPoint);
						p.pose.position = newPoint;
						path_.poses.push_back(p);
						points_.colors.push_back(green_);
					}else{
						points_.colors.push_back(red_);
					}
				}
				down_2_up = false;
			}else{
				for (double j = up_right.y; j >= down_left.y; j-=dY){
					newPoint.x = i;
					newPoint.y = j;
					points_.points.push_back(newPoint);
					if(inside(newPoint, exploration_zone_, down_left)){
						grid_.push_back(newPoint);
						p.pose.position = newPoint;
						path_.poses.push_back(p);
						points_.colors.push_back(green_);
					}else{
						points_.colors.push_back(red_);
					}
				}
				down_2_up = true;
			}
		}
		pub_points_.publish(points_);

		// Adds the orientation to the points in the path; each pose facing the next one
		tf2::Quaternion q2;
		q2.setRPY(0,0,0);
		q2.normalize();
		tf2::convert(q2,path_.poses[0].pose.orientation);

		for (std::vector<geometry_msgs::PoseStamped>::iterator it = path_.poses.begin()+1; it != path_.poses.end(); it++){
			double yaw = atan2(it->pose.position.y-(it-1)->pose.position.y,it->pose.position.x-(it-1)->pose.position.x);
			q2.setRPY(0,0,yaw);
			q2.normalize();
			tf2::convert(q2,it->pose.orientation);
		}

		pub_path_.publish(path_);
	}
	
	/* Checks if the point is inside using Ray Tracing algorithm. Traces a ray between the point and an external point */
	bool inside(geometry_msgs::Point pt, geometry_msgs::PolygonStamped poly, geometry_msgs::Point down_left){
		int count_down = 0;

		// The exterior point is created by taking some distance from the most down and left point of the rectangle around the zone
		geometry_msgs::Point exterior_pt;
		exterior_pt.x  = down_left.x-0.05;
		exterior_pt.y  = down_left.y-0.05;
		
		// Each pair of consecutive points in the polygon creates an edge of the polygon
		std::vector<geometry_msgs::Point32>::iterator start, end;
		start = exploration_zone_.polygon.points.begin();
		end = start+1; 

		// Calculate the intersection of the line from the point to the exterior point and each edge
		while(start<exploration_zone_.polygon.points.end()){

			// Auxiliary variables
			double x_intersect, y_intersect;
			double m_pt, n_pt, m_poly, n_poly;
			
			// Line from the exterior point to the point
			m_pt = (exterior_pt.y-pt.y)/(exterior_pt.x-pt.x);
			n_pt = pt.y - m_pt * pt.x;

			// Line of the edge
			m_poly = (start->y-end->y)/(start->x-end->x);
			n_poly = end->y - m_poly * end->x;

			// Intersection point
			x_intersect = (n_poly-n_pt)/(m_pt-m_poly);
			y_intersect = m_poly*x_intersect+n_poly;
			
			// Check if the intersection point is inside the rectangles made by the vertices of the edge and the external point and the point 
			if((std::min(start->x,end->x)<= x_intersect) && (x_intersect<= std::max(start->x,end->x))){
				if((std::min(start->y,end->y)<= y_intersect) && (y_intersect<= std::max(start->y,end->y))){
					if((std::min(pt.x,exterior_pt.x)<= x_intersect) && (x_intersect<= std::max(pt.x,exterior_pt.x))){
						if((std::min(pt.y,exterior_pt.y)<= y_intersect) && (y_intersect<= std::max(pt.y,exterior_pt.y))){
							count_down++;
						}
					}
				}
			}
			
			start++;
			end++;
			// The last edge is between the last point and the first
			if (end==exploration_zone_.polygon.points.end()){
				end = exploration_zone_.polygon.points.begin(); 
			}
		}
		// If the number of points is even, then the point is inside
		if (count_down%2!=0){
			return true;
		}else{
			return false;
		}
	}

private:
	// ROS variables
	ros::NodeHandle nh_;
	ros::Subscriber sub_clicked_pts_;
	ros::Subscriber sub_start_;
	ros::Publisher pub_polygon_;
	ros::Publisher pub_points_;
	ros::Publisher pub_path_;

	// ROS msgs
	geometry_msgs::PolygonStamped exploration_zone_;
	nav_msgs::Path path_;
	visualization_msgs::Marker points_;
	std_msgs::ColorRGBA green_;
	std_msgs::ColorRGBA red_;
	std::vector<geometry_msgs::Point> grid_;
};

int main(int argc, char **argv){
	// Initialize the node
	ros::init(argc, argv, "exploration_node");

	// Initialize the class that will do everything
	Zone zone;

	ros::spin();
	return 0;
}

/* TO DO:
	- Pass params such as dX, dY, frame_id,...
*/

/* Other stuff:
	- grid_ is currently not used. ¿Possible uses?
*/