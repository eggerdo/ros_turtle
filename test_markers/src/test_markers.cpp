/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <pthread.h>
#include <deque>

pthread_mutex_t line_strip_mutex;
pthread_mutex_t transform_mutex;

visualization_msgs::Marker line_strip;

tf::TransformListener* listener;
tf::StampedTransform transform;
bool transformReady;

//void callback(const std_msgs::Empty::ConstPtr& msg)
void callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	// Very ugly/slow test code

	//std::cout << *msg << std::endl;

	bool add = false;

	tf::Vector3 vecOdom(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
	tf::Vector3 vecMap;
	pthread_mutex_lock(&transform_mutex);
	if (transformReady)
	{
		add = true;
		//vecMap = transform.invXform(vecOdom);
		vecMap = transform(vecOdom);
		//	std::cout << "transform: " << transform.getBasis().getRow(0).m_floats[0] << " " << transform.getBasis().getRow(0).m_floats[1] << " " << transform.getBasis().getRow(0).m_floats[2] << std::endl;
		//	std::cout << "transform: " << transform.getBasis().getRow(1).m_floats[0] << " " << transform.getBasis().getRow(1).m_floats[1] << " " << transform.getBasis().getRow(1).m_floats[2] << std::endl;
		//	std::cout << "transform: " << transform.getBasis().getRow(2).m_floats[0] << " " << transform.getBasis().getRow(2).m_floats[1] << " " << transform.getBasis().getRow(2).m_floats[2] << std::endl;

		//	std::cout << "rotation:" << acos(transform.getBasis().getRow(0).m_floats[0]) << " == " << atan2(transform.getOrigin().m_floats[1], transform.getOrigin().m_floats[0]) << std::endl;
		//	std::cout << "origin: " << transform.getOrigin().m_floats[0] << " " << transform.getOrigin().m_floats[1] << " " << transform.getOrigin().m_floats[2] << std::endl;
		//
		//	std::cout << "vecOdom: " << vecOdom.getX() << " " << vecOdom.getY() << " " << vecOdom.getZ() << std::endl;
		//	std::cout << "vecMap: " << vecMap.getX() << " " << vecMap.getY() << " " << vecMap.getZ() << std::endl;
	}
	pthread_mutex_unlock(&transform_mutex);




	//listener.transformPose("/map",)

	if (add)
	{
		pthread_mutex_lock(&line_strip_mutex);
		if (line_strip.points.size() < 8000)
		{
			line_strip.points.push_back(msg->pose.pose.position);
			line_strip.points.back().x = vecMap.x();
			line_strip.points.back().y = vecMap.y();
			line_strip.points.back().z = vecMap.z();
			line_strip.header.stamp = msg->header.stamp;
		}
		pthread_mutex_unlock(&line_strip_mutex);
	}
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "test_trace");
	ros::NodeHandle handle;
	
	listener = new tf::TransformListener();
	transformReady = false;

	pthread_mutex_init(&line_strip_mutex, NULL);
	pthread_mutex_init(&transform_mutex, NULL);
	
	ros::Publisher pub = handle.advertise<visualization_msgs::Marker>("visualization_marker", 100);
	ros::Subscriber sub = handle.subscribe("odom", 1, callback);
	//ros::Subscriber sub = handle.subscribe("odom", 10, &myclass::callback, &myclass_obj);

	// init marker

//	line_strip.header.frame_id = "/odom";
	line_strip.header.frame_id = "/map";
	line_strip.ns = "test_trace";
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	line_strip.id = 1;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	line_strip.scale.x = 0.3;
	// Line strip is blue
	line_strip.color.b = 1.0;
	line_strip.color.a = 1.0;


	ros::Rate rate(2); // 8000 / 2 = 4000s

	float f = 0.0;
	while (ros::ok())
	{

		pthread_mutex_lock(&transform_mutex);
		try {
			listener->lookupTransform("/map", "/odom", ros::Time(0), transform);
			transformReady = true;
		}
		catch (tf::TransformException& ex) {
			ROS_ERROR("%s",ex.what());
			transformReady = false;
		}
		pthread_mutex_unlock(&transform_mutex);


		pthread_mutex_lock(&line_strip_mutex);
		pub.publish(line_strip);
		pthread_mutex_unlock(&line_strip_mutex);

		ros::spinOnce(); // Handle events
		
		rate.sleep();
	}
}


