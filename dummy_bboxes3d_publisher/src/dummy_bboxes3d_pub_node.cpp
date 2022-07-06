#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "gb_visual_detection_3d_msgs/BoundingBox3d.h"

class DummyBbox3dPublisher
{
public:
	DummyBbox3dPublisher()
	: nh_("~")
	{
		start_ = -5.0;
		end_ = 5.0;
		step_ = 0.2;
		current_pos_ = start_;

		bbox_pub_ = nh_.advertise<gb_visual_detection_3d_msgs::BoundingBox3d>(
			"/dummy_bbox3d", 1);
		marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
			"/dummy_bbox3d_marker", 1);
	}

	void
	step()
	{
		gb_visual_detection_3d_msgs::BoundingBox3d msg;
		visualization_msgs::Marker marker_msg;
		double width = 0.1;
		double distance = 3.0;

		// Compose and publish the message

		composeMsg(msg, width, distance);
		bbox_pub_.publish(msg);

		current_pos_ += step_;
		if (current_pos_ > end_)
			current_pos_ = start_;

		// Compose and publish the visual marker

		composeMarker(marker_msg, msg);
		marker_pub_.publish(marker_msg);
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher bbox_pub_;
	ros::Publisher marker_pub_;
	double start_, end_, step_, current_pos_;

	void
	composeMsg(
		gb_visual_detection_3d_msgs::BoundingBox3d & msg,
		double width, double distance)
	{
		msg.Class = "dummy";
		msg.probability = 1.0;
		msg.xmax = distance + (width / 2.0);
		msg.xmin = distance - (width / 2.0);

		msg.ymax = current_pos_ + (width / 2.0);
		msg.ymin = current_pos_ - (width / 2.0);

		msg.zmax = width / 2.0;
		msg.zmin = -width / 2.0;
	}

	void
	composeMarker(visualization_msgs::Marker & marker_msg,
		const gb_visual_detection_3d_msgs::BoundingBox3d & msg)
	{
		marker_msg.header.frame_id = "camera_link";
		marker_msg.type = visualization_msgs::Marker::CUBE;
		marker_msg.action = visualization_msgs::Marker::MODIFY;

		marker_msg.pose.position.x = (msg.xmax + msg.xmin) / 2.0;
		marker_msg.pose.position.y = (msg.ymax + msg.ymin) / 2.0;
		marker_msg.pose.position.z = (msg.zmax + msg.zmin) / 2.0;

		marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;

		marker_msg.scale.x = msg.xmax - msg.xmin;
		marker_msg.scale.y = msg.ymax - msg.ymin;
		marker_msg.scale.z = msg.zmax - msg.zmin;

		marker_msg.color.r = 1.0;
		marker_msg.color.g = 0.0;
		marker_msg.color.b = 0.0;
		marker_msg.color.a = 0.5;

		marker_msg.lifetime = ros::Duration(0.5);
	}
};

int
main(int argc, char ** argv)
{
	ros::init(argc, argv, "dummy_bboxes3d_pub_node");

	DummyBbox3dPublisher dummy_publisher;
	ros::Rate rate(2);
	while (ros::ok()) {
		dummy_publisher.step();
		ros::spinOnce();
		rate.sleep();
	}

	exit(EXIT_SUCCESS);
}