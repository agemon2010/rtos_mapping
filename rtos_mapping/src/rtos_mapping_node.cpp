#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <rtos_mapping/posFromMap.h>

std::string intToString(int n) {
	std::stringstream s;
	s << n;
	return s.str();
}

class TfExample
{
public:
	TfExample() :
		mapFrameId_("/map"),
		objFramePrefix_("object")
	{
		ros::NodeHandle pnh("~");
		pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
		pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

		ros::NodeHandle nh;
		subs1_ = nh.subscribe("objectsStamped", 1, &TfExample::objectsDetectedCallback, this);
		subs2_ = nh.subscribe("tf", 1, &TfExample::cameraCallback, this);
		pub_ = nh.advertise<rtos_mapping::posFromMap>("/kidscar_map",1);
	}

	void cameraCallback(const tf::tfMessage::ConstPtr& input) {	
		rtos_mapping::posFromMap output;
		std::string camera_pos ("camera_link");

		for(size_t i=0; i<input->transforms.size(); ++i) {
			if(input->transforms[i].child_frame_id.compare(camera_pos) == 0) {
				
				tf::StampedTransform pose;
				try {
					tfListener_.lookupTransform(
							mapFrameId_, 
							input->transforms[i].child_frame_id, 
							input->transforms[i].header.stamp, 
							pose);
				}
				catch(tf::TransformException & ex)
				{
					ROS_WARN("%s",ex.what());
					continue;
				}

				ROS_INFO("Camera_link [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						mapFrameId_.c_str(),
						pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
						pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
				
				output.id = 1;
				output.x = -1*pose.getOrigin().y();
				output.y = pose.getOrigin().x();
				output.z = pose.getOrigin().z();

				pub_.publish(output);
				
			}
		}
	}

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
	{
		rtos_mapping::posFromMap output;

		if(msg->objects.data.size())
		{
			for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
			{
				// get data
				int id = (int)msg->objects.data[i];
				//std::string objectFrameId = QString("%1_%2").arg(objFramePrefix_.c_str()).arg(id).toStdString(); // "object_1", "object_2"
				std::string id_str = intToString(id);
				std::string objectFrameId = objFramePrefix_.c_str();
				objectFrameId.append("_");
				objectFrameId.append(id_str);

				printf("the string is %s\n", objectFrameId.c_str());

				tf::StampedTransform pose;
				//tf::StampedTransform poseCam;
				try
				{
					// Get transformation from "object_#" frame to target frame "map"
					// The timestamp matches the one sent over TF
					tfListener_.lookupTransform(mapFrameId_, objectFrameId, msg->header.stamp, pose);
					//tfListener_.lookupTransform(msg->header.frame_id, objectFrameId, msg->header.stamp, poseCam);
				}
				catch(tf::TransformException & ex)
				{
					ROS_WARN("%s",ex.what());
					continue;
				}

				// Here "pose" is the position of the object "id" in "/map" frame.
				ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						id, mapFrameId_.c_str(),
						pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
						pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
				/*
				ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						id, msg->header.frame_id.c_str(),
						poseCam.getOrigin().x(), poseCam.getOrigin().y(), poseCam.getOrigin().z(),
						poseCam.getRotation().x(), poseCam.getRotation().y(), poseCam.getRotation().z(), poseCam.getRotation().w()); */
				output.id = id;
				output.x = -1*pose.getOrigin().y();
				output.y = pose.getOrigin().x();
				output.z = pose.getOrigin().z();


				pub_.publish(output);
			}
		}
	}

private:
	std::string mapFrameId_;
	std::string objFramePrefix_;
    ros::Subscriber subs1_;
    ros::Subscriber subs2_;
	ros::Publisher pub_;
	tf::TransformListener tfListener_;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "tf_filter");

    TfExample sync;
    ros::spin();
}
