#include "test_slam_node.hpp"
#include "common.hpp"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include "boost/asio.hpp"   //包含boost库函数
#include "boost/bind.hpp"
#include <iostream>
#include <math.h>


//构造函数
TestSlam::TestSlam()
{
    ros::NodeHandle private_nh_("~");

    param_.GetParam(&nh_);

    scan_pos_cal.setZero();
    odom_pos_cal.setZero();

    vertex_counter_ = 0;
    info_matrix << 500.5, 0.0, 0.0,
                   0.0, 500.5, 0.0,
                   0.0, 0.0, 500.05;

    current_time_ = ros::Time::now();

    //发布路径
    scan_path_pub_ = nh_.advertise<nav_msgs::Path>("scan_path_pub_",1,true);
    path_scan.header.stamp=current_time_;
    path_scan.header.frame_id=param_.global_frame_id;

    optimization_path_pub_ = nh_.advertise<nav_msgs::Path>("optimization_path_pub_",1,true);
    path_optimization_.header.stamp=current_time_;
    path_optimization_.header.frame_id=param_.global_frame_id;

    //进行里程计和激光雷达数据的同步
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, param_.laser_topic_name, 10);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, param_.odom_frame_id, 10);
    scan_filter_->registerCallback(boost::bind(&TestSlam::scanCallBack, this, _1));

    std::cout <<"Test_slam init,Wait for Data!!!!!!!"<<std::endl;
}

void TestSlam::pubPathMsg( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &path_pub_)
{

    current_time_ = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pose(0);
    this_pose_stamped.pose.position.y = pose(1);

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp=current_time_;
    this_pose_stamped.header.frame_id=param_.global_frame_id;
    path.poses.push_back(this_pose_stamped);
    path_pub_.publish(path);

}




bool TestSlam::getTfPose(const std::string& target_frame_id,
                         const std::string& source_frame_id,
                         Eigen::Vector3d& pose, const ros::Time& t)
{
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                               tf::Vector3(0,0,0)), t, source_frame_id);
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        tf_.transformPose(target_frame_id, ident, odom_pose);
    }


    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());



    pose << odom_pose.getOrigin().x(),
            odom_pose.getOrigin().y(),
            yaw;
    return true;
}

Eigen::Vector3d TestSlam::calcuDeltaOdom(Eigen::Vector3d odom_pose)
{
    Eigen::Vector3d d_pos;
    now_pos_ = odom_pose;

    Eigen::Matrix3d transform_matrix;
    double c = cos(last_pos_(2));
    double s = sin(last_pos_(2));
    transform_matrix<< c, s, 0,
            -s, c, 0,
            0, 0, 1;
    d_pos = transform_matrix * (now_pos_ - last_pos_);

    return d_pos;
}


void TestSlam::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg)
{
    static long int dataCnt = 0;

    Eigen::Vector3d odom_base_pose;
    Eigen::Vector3d d_odom;

    Eigen::Vector3d base_laser_pose;
    Eigen::Vector3d d_scan_laser,d_scan_base;

    Eigen::Matrix3d transform_matrix;
    double c,s;

    sensor_msgs::LaserScan scan = *laserScanMsg;

    if(!getTfPose(param_.odom_frame_id, param_.base_frame_id, odom_base_pose, scan.header.stamp)){
        d_odom = Eigen::Vector3d::Zero();
    }
    else{
        d_odom = calcuDeltaOdom(odom_base_pose);
        if(d_odom(0) < 0.02 &&
           d_odom(1) < 0.02 &&
           d_odom(2) < tfRadians(5.0))
        {
//            return ;
//            d_odom = Eigen::Vector3d::Zero();
        }
        last_pos_ = now_pos_;
        odom_increments_.push_back(d_odom);

        c = cos(odom_pos_cal(2));
        s = sin(odom_pos_cal(2));
        odom_pos_cal(0) += c*d_odom(0) - s*d_odom(1);
        odom_pos_cal(1) += s*d_odom(0) + c*d_odom(1);
        odom_pos_cal(2) += d_odom(2);
    }



//    d_odom = Eigen::Vector3d::Zero();
    d_scan_laser = scan_matcher_.ResolveScanMatch(&scan, d_odom);
    getTfPose(param_.base_frame_id, param_.laser_frame_id,
              base_laser_pose, scan.header.stamp);
    d_scan_base = PoseToTrans(base_laser_pose) * d_scan_laser;
    c = cos(scan_pos_cal(2));
    s = sin(scan_pos_cal(2));
    scan_pos_cal(0) += c*d_scan_base(0) - s*d_scan_base(1);
    scan_pos_cal(1) += s*d_scan_base(0) + c*d_scan_base(1);
    scan_pos_cal(2) += d_scan_base(2);

    pubPathMsg(scan_pos_cal, path_scan, scan_path_pub_);


    pose_graph_.AddVertex({scan_pos_cal}, vertex_counter_++);
    if(vertex_counter_ > 1){
        Edge tmp_e = {vertex_counter_-2, vertex_counter_-1, d_odom, info_matrix};
        pose_graph_.AddEdge(tmp_e);
    }


    if(vertex_counter_ == 768) {
        pose_graph_.PoseGraphOptimization(0, pose_graph_.GetVertex().size());
    }else if(!(vertex_counter_%10) && vertex_counter_ >= 30){
//        pose_graph_.PoseGraphOptimization(vertex_counter_-20, 20);
    }

    {
        path_optimization_.poses.clear();
        for(auto & vertex : pose_graph_.GetVertex())
        {
            Eigen::Vector3d pose = vertex.second.pose;

            current_time_ = ros::Time::now();
            geometry_msgs::PoseStamped v_pose_stamped;
            v_pose_stamped.pose.position.x = pose(0);
            v_pose_stamped.pose.position.y = pose(1);
            geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(pose(2));
            v_pose_stamped.pose.orientation.x = q.x;
            v_pose_stamped.pose.orientation.y = q.y;
            v_pose_stamped.pose.orientation.z = q.z;
            v_pose_stamped.pose.orientation.w = q.w;
            v_pose_stamped.header.stamp=current_time_;
            v_pose_stamped.header.frame_id=param_.global_frame_id;

            path_optimization_.poses.push_back(v_pose_stamped);
        }
        pubPathMsg(pose_graph_.GetVertex()[vertex_counter_-2].pose,path_optimization_,optimization_path_pub_);

    }


    std::cout << "Data Cnt:" << dataCnt++ << std::endl;
}





