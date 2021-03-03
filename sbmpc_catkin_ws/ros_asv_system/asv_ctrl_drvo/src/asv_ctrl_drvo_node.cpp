#include "ros/ros.h"
#include <ros/console.h>

#include <vector>
#include <ctime>

#include "nav_msgs/OccupancyGrid.h"
#include "asv_msgs/StateArray.h"
#include "asv_msgs/Offset.h"

//#include "asv_ctrl_drvo/asv_ctrl_drvo.h"
#include "asv_ctrl_drvo.h"
#include "asv_ctrl_drvo/asv_ctrl_drvo_node.h"

#include "fstream" // for test printout!


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "asv_ctrl_drvo_node");
	ros::start();
	
	ROS_INFO("Starting drvo_node");
	
	ros::NodeHandle n;
	
	dynamicRvoNode drvo_node;
	//dynamicRvo *drvo = new dynamicRvo;
	dynamicRvoRosLink *drvo_link = new dynamicRvoRosLink; //RosLink
	
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("asv/cmd_vel",10);
	
	ros::Publisher in_pub = n.advertise<asv_msgs::Offset>("asv/offset",10); // **** change offset to input

	ros::Subscriber obstacle_sub = n.subscribe("obstacle_states", 
							1,
						 	&dynamicRvoNode::obstacleCallback,
							&drvo_node);
											   
	ros::Subscriber og_sub = n.subscribe("/map",
							1,
							&dynamicRvoNode::mapCallback,
							&drvo_node);
    
    	ros::Subscriber asv_sub	= n.subscribe("asv/state",
							1,
							&dynamicRvoNode::asvCallback,
							&drvo_node);
	
	ros::Subscriber cmd_sub = n.subscribe("asv/LOS/cmd_vel",
							1,
							&dynamicRvoNode::cmdCallback,
							&drvo_node);
	
	drvo_node.initialize(&cmd_pub, & in_pub, &obstacle_sub, &og_sub, &asv_sub, &cmd_sub, drvo_link); //RosLink
	drvo_node.start();
	
	ros::shutdown();
	return 0;	
}
	
dynamicRvoNode::dynamicRvoNode() : drvo_(NULL),
						   cmd_pub_(NULL),
						   in_pub_(NULL),
						   obstacle_sub_(NULL),
						   og_sub_(NULL),
						   asv_sub_(NULL),
						   cmd_sub_(NULL) {};

dynamicRvoNode::~dynamicRvoNode() {};

void dynamicRvoNode::initialize(ros::Publisher *cmd_pub,
					ros::Publisher *in_pub,
					ros::Subscriber *obstacle_sub,
					ros::Subscriber *og_sub,
					ros::Subscriber *asv_sub,
					ros::Subscriber *cmd_sub,
					dynamicRvoRosLink *drvo_link) //RosLink
{
	cmd_pub_ = cmd_pub;
	in_pub_ = in_pub;
	obstacle_sub_ = obstacle_sub;
	og_sub_ = og_sub;
	asv_sub_ = asv_sub;
	cmd_sub_ = cmd_sub;
	
	drvo_ = drvo_link; //RosLink
	u_d_ = 0.0; //0; // init sim
	psi_d_ = 1.57; //1.57; // 0.0; // init sim
	u_ = u_d_; //0; // init sim
	psi_ = psi_d_; // 1.57; // init sim
	
	drvo_->initialize(&obstacles_, &map_); //RosLink
}										

void dynamicRvoNode::start()
{
	double rate = 10.0;
	ros::Rate loop_rate(rate);
	static double u_last = u_d_, psi_last = psi_d_, u_os_, psi_os_;  
	std::cout << "u_d_, psi_d_ : " << u_d_ <<", " << psi_d_ <<  std::endl; // check test!
	
	clock_t tick, tock;
	double t = 0;
		
	ofstream log_asv;
	ofstream log_obst_1;
	ofstream log_obst_2;
	ofstream log_obst_3;
	
	log_asv.open("/home/giorgio/colav/mpc/sb_mpc/catkin_ws/ros_records/asv_log.csv");
	log_obst_1.open("/home/giorgio/colav/mpc/sb_mpc/catkin_ws/ros_records/obst1_log.csv");
	log_obst_2.open("/home/giorgio/colav/mpc/sb_mpc/catkin_ws/ros_records/obst2_log.csv");
	log_obst_3.open("/home/giorgio/colav/mpc/sb_mpc/catkin_ws/ros_records/obst3_log.csv");
	
	
	while (ros::ok())
	{
		// Run drvo every 5 seconds
		if (t > 5){ // 1 or 5 OK, but not less than 1s! 
			tick = clock();
			//drvo_->getBestControlInput(u_, psi_, log_asv, log_obst_1, log_obst_2, log_obst_3); //RosLink does not have this fxn!
			drvo_->getBestControlOffset(u_os_, psi_os_); //RosLink does not have this fxn!
			tock = clock() - tick;
			t = 0;
			in_.P_ca = u_os_; // u_; 
			in_.Chi_ca = psi_os_; // psi_; // NB! negative for MR interface
			in_pub_->publish(in_);
			ROS_INFO("Runtime: %0.2f", ((float)tock)/CLOCKS_PER_SEC);
			
			/*
			cmd_vel_.linear.x = u_; u_last = u_;
			cmd_vel_.angular.y = psi_; psi_last = psi_;  // *** no more offset!
			cmd_pub_->publish(cmd_vel_);
			*/
			cmd_vel_.linear.x = u_d_*u_os_; u_last = u_d_*u_os_;
			cmd_vel_.angular.y = psi_d_ + psi_os_; psi_last = psi_d_ + psi_os_;  // *** no more offset!
			cmd_pub_->publish(cmd_vel_);
			
		}else{
		
			cmd_vel_.linear.x = u_last;
			cmd_vel_.angular.y = psi_last; // *** no more offset!
			cmd_pub_->publish(cmd_vel_);		
		}
		t += 1/rate;
		/*
		cmd_vel_.linear.x = u_;
		cmd_vel_.angular.y = psi_; 
		cmd_pub_->publish(cmd_vel_);
		*/
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	log_asv.close();
	log_obst_1.close();
	log_obst_2.close();
	log_obst_3.close();
}

void dynamicRvoNode::asvCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
	drvo_->updateAsvState(msg, u_d_, psi_d_); //RosLink
}

void dynamicRvoNode::obstacleCallback(const asv_msgs::StateArray::ConstPtr & msg)
{
	obstacles_ = msg->states;
}

void dynamicRvoNode::cmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
	u_d_ = msg->linear.x;
	psi_d_ = msg->angular.y;
}

void::dynamicRvoNode::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  // Copy what we need
  map_.info.resolution = msg->info.resolution;
  map_.info.height = msg->info.height;
  map_.info.width = msg->info.width;
  map_.info.origin.position.x = msg->info.origin.position.x;
  map_.info.origin.position.y = msg->info.origin.position.y;

  ROS_INFO("r %f, h %d, w%d, px %f, py %f",
           map_.info.resolution,
           map_.info.height,
           map_.info.width,
           map_.info.origin.position.x,
           map_.info.origin.position.y);

  map_.data = msg->data;
}
