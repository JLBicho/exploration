/* 	
	Author: Jose Luis Millan Valbuena

	This node is used to create a grid of a selected zone in the map.
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
	/* Class constructor. Initialization of publishers and subscribers and variables */ 
	Zone(){
		std::cout<<"Starting Zone generator"<<std::endl;

		// Subscribers
		sub_clicked_pts_ = nh_.subscribe("clicked_point", 10, &Zone::clickedPointsCb, this);
		sub_start_ = nh_.subscribe("/start", 1, &Zone::startCb, this);
		
		// Publishers
		pub_polygon_ = nh_.advertise<geometry_msgs::PolygonStamped>("exploration_zone", 10);
		pub_lines_ = nh_.advertise<visualization_msgs::Marker>("lines", 10);
		pub_points_ = nh_.advertise<visualization_msgs::Marker>("points",10);
		pub_path_ = nh_.advertise<nav_msgs::Path>("path",10);
		
		// Private variables initialization
		lines_.header.frame_id = "map";
		lines_.ns = "lines";
		lines_.id = 0;
		lines_.type = visualization_msgs::Marker::LINE_LIST;
		lines_.action = visualization_msgs::Marker::ADD;
		lines_.pose.position.x = 0;
		lines_.pose.position.y = 0;
		lines_.pose.position.z = 0;
		lines_.pose.orientation.x = 0;
		lines_.pose.orientation.y = 0;
		lines_.pose.orientation.z = 0;
		lines_.pose.orientation.w = 1;
		lines_.scale.x = 0.01;
		lines_.color.a = 1.0;
		lines_.color.r = 1.0;
		lines_.color.g = 0.0;
		lines_.color.b = 0.0;

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
		points_.color.a = 1.0;
		points_.color.r = 1.0;
		points_.color.g = 0.0;
		points_.color.b = 0.0;
		green_.r = 0.0;
		green_.g = 1.0;
		green_.b = 0.0;
		green_.a = 1.0;
	}

	~Zone(){
		nh_.shutdown();
		std::cout<<"\nDestroying Zone generator.\nGoodbye.\n"<<std::endl;
	}

	void clickedPointsCb(const geometry_msgs::PointStamped::ConstPtr& msg){
		geometry_msgs::Point32 pt;
		pt.x = msg->point.x;
		pt.y = msg->point.y;
		pt.z = msg->point.z;
		exploration_zone_.polygon.points.push_back(pt);
		std::cout<<"Point received: "<<pt.x<<", "<<pt.y<<", "<<pt.z<<std::endl;
		
		exploration_zone_.header.frame_id = "map";
		if (exploration_zone_.polygon.points.size()>2){
			pub_polygon_.publish(exploration_zone_);
		}
	}

	void startCb(const std_msgs::String::ConstPtr& msg){
		if(msg->data == "start"){
			sub_clicked_pts_.shutdown();
			std::cout<<"Shutting down point receiver.\nStarting grid generation."<<std::endl;
			gridGenerator();
		}
	}

	void gridGenerator(){
		std::cout<<"Generating grid"<<std::endl;
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
		
		// Grid
		double dY = 0.4;
		double dX = 0.4;
		geometry_msgs::Point newPoint;
		geometry_msgs::PoseStamped p;
		p.header.frame_id = "map";
		geometry_msgs::Quaternion q;
		q.x=0.0;
		q.y=0.0;
		q.z=0.0;
		q.w=1.0;
		
		// To make a more useful path, the Y coordinate changes its way (up-down) 
		bool once = true;
		for (double i = down_left.x; i <= up_right.x; i+=dX){
			if (once){
				for (double j = down_left.y; j <= up_right.y; j+=dY){
					newPoint.x = i;
					newPoint.y = j;
					points_.points.push_back(newPoint);
					if(inside(newPoint, exploration_zone_, down_left, up_right)){
						grid_.push_back(newPoint);
						p.pose.position = newPoint;
						p.pose.orientation = q;
						path_.poses.push_back(p);
						points_.colors.push_back(green_);
					}else{
						points_.colors.push_back(points_.color);
					}
				}
				once = false;
			}else{
				for (double j = up_right.y; j >= down_left.y; j-=dY){
					newPoint.x = i;
					newPoint.y = j;
					points_.points.push_back(newPoint);
					if(inside(newPoint, exploration_zone_, down_left, up_right)){
						grid_.push_back(newPoint);
						p.pose.position = newPoint;
						p.pose.orientation = q;
						path_.poses.push_back(p);
						points_.colors.push_back(green_);
					}else{
						points_.colors.push_back(points_.color);
					}
				}
				once = true;
			}
		}
		pub_points_.publish(points_);

		// Set orientation for each point, facing the next one
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

		path_.header.frame_id="map";
		pub_path_.publish(path_);
	}
	
	bool inside(geometry_msgs::Point pt, geometry_msgs::PolygonStamped poly, geometry_msgs::Point down_left, geometry_msgs::Point up_right){
		int count_down, count_up;
		count_down = 0;
		count_up = 0;

		geometry_msgs::Point new_down, new_up;
		new_down.x  = down_left.x-0.05;
		new_down.y  = down_left.y-0.05;
		new_up.x  = up_right.x+0.05;
		new_up.y  = up_right.y+0.05;
		
		// For each pair of consecutive points in the polygon
		std::vector<geometry_msgs::Point32>::iterator start, end;
		start = exploration_zone_.polygon.points.begin();
		end = start+1; 

		while(start<exploration_zone_.polygon.points.end()){

			double x_intersect, y_intersect;
			double m_pt, n_pt, m_poly, n_poly;
			
			// From down_left to pt
			m_pt = (new_down.y-pt.y)/(new_down.x-pt.x);
			n_pt = pt.y - m_pt * pt.x;

			m_poly = (start->y-end->y)/(start->x-end->x);
			n_poly = end->y - m_poly * end->x;

			x_intersect = (n_poly-n_pt)/(m_pt-m_poly);
			y_intersect = m_poly*x_intersect+n_poly;
			
			/*
			std::cout<<"[DOWN] (x,y)start = "<<start->x<<", "<<start->y<<std::endl;
			std::cout<<"[DOWN] (x,y)end = "<<end->x<<", "<<end->y<<std::endl;
			std::cout<<"[DOWN] (x,y)pt = "<<pt.x<<", "<<pt.y<<std::endl;
			std::cout<<"[DOWN] (x,y)new_down = "<<new_down.x<<", "<<new_down.y<<std::endl;
			std::cout<<"[DOWN] (x,y)_intersect = "<<x_intersect<<", "<<y_intersect<<"\n"<<std::endl;
			*/
			
			if((std::min(start->x,end->x)<= x_intersect) && (x_intersect<= std::max(start->x,end->x))){
				if((std::min(start->y,end->y)<= y_intersect) && (y_intersect<= std::max(start->y,end->y))){
					if((std::min(pt.x,new_down.x)<= x_intersect) && (x_intersect<= std::max(pt.x,new_down.x))){
						if((std::min(pt.y,new_down.y)<= y_intersect) && (y_intersect<= std::max(pt.y,new_down.y))){
							count_down++;
						}
					}
				}
			}
			/*
			// From pt to up_right
			m_pt = (new_up.y-pt.y)/(new_up.x-pt.x);
			n_pt = pt.y - m_pt * pt.x;

			x_intersect = (n_poly-n_pt)/(m_pt-m_poly);
			y_intersect = m_poly*x_intersect+n_poly;
			*/
			/*
			std::cout<<"[UP] (x,y)start = "<<start->x<<", "<<start->y<<std::endl;
			std::cout<<"[UP] (x,y)end = "<<end->x<<", "<<end->y<<std::endl;
			std::cout<<"[UP] (x,y)pt = "<<pt.x<<", "<<pt.y<<std::endl;
			std::cout<<"[UP] (x,y)new_up = "<<new_up.x<<", "<<new_up.y<<std::endl;
			std::cout<<"[UP] (x,y)_intersect = "<<x_intersect<<", "<<y_intersect<<"\n"<<std::endl;
			*/
			/*
			if((std::min(start->x,end->x)<= x_intersect) && (x_intersect<= std::max(start->x,end->x))){
				if((std::min(start->y,end->y)<= y_intersect) && (y_intersect<= std::max(start->y,end->y))){
					if((std::min(pt.x,new_up.x)<= x_intersect) && (x_intersect<= std::max(pt.x,new_up.x))){
						if((std::min(pt.y,new_up.y)<= y_intersect) && (y_intersect<= std::max(pt.y,new_up.y))){
							count_up++;
						}
					}
				}
			}
			*/
			/*
			ros::Rate loop_rate(100);
			lines_.points.clear();
			lines_.points.push_back(pt);
			lines_.points.push_back(new_down);
			lines_.points.push_back(pt);
			lines_.points.push_back(new_up);
			geometry_msgs::Point ori, dst;
			ori.x = start->x;
			ori.y = start->y;
			dst.x = end->x;
			dst.y = end->y;
			lines_.points.push_back(ori);
			lines_.points.push_back(dst);
			pub_lines_.publish(lines_);
			loop_rate.sleep();
			*/
			start++;
			end++;
			if (end==exploration_zone_.polygon.points.end()){
				end = exploration_zone_.polygon.points.begin(); 
			}


		}
		/*
		lines_.points.clear();
		pub_lines_.publish(lines_);
		*/

		if (count_down%2!=0){ //} && count_up%2!=0){
			return true;
		}else{
			return false;
		}
	}

private:
	ros::NodeHandle nh_;
	ros::Subscriber sub_clicked_pts_;
	ros::Subscriber sub_start_;
	ros::Publisher pub_polygon_;
	ros::Publisher pub_lines_;
	ros::Publisher pub_points_;
	ros::Publisher pub_path_;

	geometry_msgs::PolygonStamped exploration_zone_;
	
	std::vector<geometry_msgs::Point> grid_;
	nav_msgs::Path path_;

	visualization_msgs::Marker lines_;
	visualization_msgs::Marker points_;
	
	std_msgs::ColorRGBA green_;
};

void clickedPointsCb(const geometry_msgs::PointStamped::ConstPtr& msg){
	geometry_msgs::Point32 pt;
	pt.x = msg->point.x;
	pt.y = msg->point.y;
	pt.z = msg->point.z;
	exploration_zone.polygon.points.push_back(pt);
	std::cout<<"Point received: "<<pt.x<<", "<<pt.y<<", "<<pt.z<<std::endl;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "exploration_node");

	Zone zone;

	ros::spin();
	return 0;
}
