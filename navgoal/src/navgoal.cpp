/*
 *  Gaitech Educational Portal
 *
 * Copyright (c) 2016
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *   Program: Map-Based Navigation
 *
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//#include "sound_play/sound_play.h"

//std::string path_to_sounds;

/** function declarations, p = Position, o = Orientation **/
bool moveToGoal(double xpGoal, double ypGoal, double zoGoal);
char choose();

/** declare the coordinates of interest; p = Position, o = Orientation  **/
double xp_waypoint_01 = 3.0;
double yp_waypoint_01 = 5.4;
double zo_waypoint_01 = 1;
double xp_waypoint_02 = -5.5;
double yp_waypoint_02 = 6.0;
double zo_waypoint_02 = 1;
double xp_waypoint_03 = 0.3;
double yp_waypoint_03 = 0;
double zo_waypoint_03 = 1;

bool goalReached = false;

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	//sound_play::SoundClient sc;
	ros::spinOnce();
	//path_to_sounds = "/home/ros/catkin_ws/src/gaitech_edu/src/sounds/";
	//sc.playWave(path_to_sounds+"short_buzzer.wav");
	//tell the action client that we want to spin a thread by default

	char choice = 'q';
	do{
		choice =choose();
		if (choice == '1'){
			goalReached = moveToGoal(xp_waypoint_01, yp_waypoint_01, zo_waypoint_01);
		}else if (choice == '2'){
			goalReached = moveToGoal(xp_waypoint_02, yp_waypoint_02, zo_waypoint_02);
		}else if (choice == '3'){
			goalReached = moveToGoal(xp_waypoint_03, yp_waypoint_03, zo_waypoint_03);
		}
		/*if (choice!='q'){
			if (goalReached){
				ROS_INFO("Congratulations!");
				ros::spinOnce();
				sc.playWave(path_to_sounds+"ship_bell.wav");
				ros::spinOnce();

			}else{
				ROS_INFO("Hard Luck!");
				sc.playWave(path_to_sounds+"short_buzzer.wav");
			}
		}*/
	}while(choice !='q');
	return 0;
}

bool moveToGoal(double xpGoal, double ypGoal, double zoGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  xpGoal;
	goal.target_pose.pose.position.y =  ypGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = zoGoal;
	goal.target_pose.pose.orientation.w = 0.0;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}

char choose(){
	char choice='q';
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|PRESSE A KEY:"<<std::endl;
	std::cout<<"|'1': WP1 "<<std::endl;
	std::cout<<"|'2': WP2 "<<std::endl;
	std::cout<<"|'3': WP3 "<<std::endl;
	std::cout<<"|'q': Quit "<<std::endl;
	std::cout<<"|-------------------------------|"<<std::endl;
	std::cout<<"|WHERE TO GO?";
	std::cin>>choice;

	return choice;


}


