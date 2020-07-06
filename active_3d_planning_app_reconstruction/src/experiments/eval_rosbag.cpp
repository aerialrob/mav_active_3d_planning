#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <eigen3/Eigen/Core>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "eval_rosbag");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string rosbag_path, target_directory, odometry_topic, map_topic;

    nh_private.param("rosbag", rosbag_path, std::string());
    nh_private.param("target_directory", target_directory, std::string());
    nh_private.param("odometry_topic", odometry_topic, std::string());

    if (rosbag_path.empty())
    {
        ROS_ERROR("No rosbag filename provided");
        return -1;
    }

    if (target_directory.empty())
    {
        ROS_ERROR("No target directory provided");
        return -1;
    }
    if (odometry_topic.empty())
    {
        ROS_ERROR("No odometry topic provided");
        return -1;
    }

    //------------------------------------------
    rosbag::Bag bag;

    try
    {
        bag.open(rosbag_path);
    }
    catch (rosbag::BagException &ex)
    {
        ROS_ERROR("rosbag::open thrown an exception: %s\n", ex.what());
        return -1;
    }

    std::vector<std::string> topics;
    topics.push_back(odometry_topic);
    topics.push_back(map_topic);

    FILE *traj_info_csv = fopen((target_directory + "/trajectory_info.csv").c_str(), "w");

    ROS_INFO("Opening file: %s", (target_directory + "/trajectory_info.csv").c_str());
    fprintf(traj_info_csv, "TrajLength MeanVel\n");

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    int count = 0;
    double acc_velocity, mean_velocity, traj_distance;
    Eigen::Vector3d last_position;
    bool last_position_set = false;
    for (rosbag::MessageInstance msg_instance : view)
    {
        const std::string &topic_name = msg_instance.getTopic();
        if (topic_name == odometry_topic)
        {
            nav_msgs::OdometryPtr odom = msg_instance.instantiate<nav_msgs::Odometry>();
            Eigen::Vector3d current_pose{odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z};
            Eigen::Vector3d current_vel{odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z};

            if (current_pose.norm() > 1e-4 && !last_position_set)
            {
                last_position = current_pose;
                last_position_set = true;
            }

            if (last_position_set)
            {
                double distance = (current_pose - last_position).norm();
                if (distance > 1e-4)
                {

                    traj_distance += distance;
                    last_position = current_pose;

                    double velocity = current_vel.norm();
                    acc_velocity += velocity;
                    count++;
                    mean_velocity = acc_velocity / count;
                }
            }

        }
    }
    fprintf(traj_info_csv, "%f %f \n",
            traj_distance,
            mean_velocity);

    fclose(traj_info_csv);

    return 0;
}
