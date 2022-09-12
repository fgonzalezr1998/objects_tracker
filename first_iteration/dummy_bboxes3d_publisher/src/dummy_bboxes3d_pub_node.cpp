#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "gb_visual_detection_3d_msgs/BoundingBox3d.h"
#include "gb_visual_detection_3d_msgs/BoundingBoxes3d.h"
#include <vector>

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
		left_ = true;

		bbox_pub_ = nh_.advertise<gb_visual_detection_3d_msgs::BoundingBoxes3d>(
			"/dummy_bboxes3d", 1);
		marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
			"/dummy_bbox3d_marker", 1);
	}

	void
	step()
	{
		gb_visual_detection_3d_msgs::BoundingBoxes3d msg;
		visualization_msgs::Marker marker_msg;
		double width = 0.5;
		double distance = 3.0;

		// Compose and publish the message

		composeMsg(msg, width, distance);
		bbox_pub_.publish(msg);

		if (left_)
			current_pos_ += step_;
		else
			current_pos_ -= step_;

		if (current_pos_ > end_)
			left_ = false;
		if (current_pos_ < start_)
			left_ = true;

		// Compose and publish the visual marker

		composeMarker(marker_msg, msg);
		marker_pub_.publish(marker_msg);
	}

private:
	ros::NodeHandle nh_;
	ros::Publisher bbox_pub_;
	ros::Publisher marker_pub_;
	bool left_;
	double start_, end_, step_, current_pos_;

	void
	composeMsg(
		gb_visual_detection_3d_msgs::BoundingBoxes3d & msg,
		double width, double distance)
	{
		gb_visual_detection_3d_msgs::BoundingBox3d bbox;
		std::vector<gb_visual_detection_3d_msgs::BoundingBox3d> v;
	
		bbox.Class = "dummy";
		bbox.probability = 1.0;
		bbox.xmax = distance + (width / 2.0);
		bbox.xmin = distance - (width / 2.0);

		bbox.ymax = current_pos_ + (width / 2.0);
		bbox.ymin = current_pos_ - (width / 2.0);

		bbox.zmax = width / 2.0;
		bbox.zmin = -width / 2.0;

		msg.bounding_boxes =
			std::vector<gb_visual_detection_3d_msgs::BoundingBox3d>(1, bbox);
		msg.header.frame_id = "camera_link";
	}

	void
	composeMarker(visualization_msgs::Marker & marker_msg,
		const gb_visual_detection_3d_msgs::BoundingBoxes3d & msg)
	{
		gb_visual_detection_3d_msgs::BoundingBox3d bbox = msg.bounding_boxes.at(0);
		marker_msg.header.frame_id = "camera_link";
		marker_msg.type = visualization_msgs::Marker::CUBE;
		marker_msg.action = visualization_msgs::Marker::MODIFY;

		marker_msg.pose.position.x = (bbox.xmax + bbox.xmin) / 2.0;
		marker_msg.pose.position.y = (bbox.ymax + bbox.ymin) / 2.0;
		marker_msg.pose.position.z = (bbox.zmax + bbox.zmin) / 2.0;

		marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;

		marker_msg.scale.x = bbox.xmax - bbox.xmin;
		marker_msg.scale.y = bbox.ymax - bbox.ymin;
		marker_msg.scale.z = bbox.zmax - bbox.zmin;

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