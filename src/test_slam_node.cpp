#include "test_slam_node.hpp"
#include "common.hpp"



#include "boost/asio.hpp"   //包含boost库函数
#include "boost/bind.hpp"
#include <iostream>
#include <string>
#include <math.h>

using namespace slam;

const std::string kMapTopic_ = "map";

//构造函数
TestSlam::TestSlam(tf::TransformListener *tf)
{
    ros::NodeHandle private_nh_("~");
    tf_ = tf;

    tfB_ = new tf::TransformBroadcaster();

    param_.GetParam(&nh_);

    
    sensor_data_manager_ = std::make_shared<slam::SensorDataManager>();
    slam_processer_ = std::make_unique<slam::SlamProcessor>(sensor_data_manager_, 0.05);

    
    map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(kMapTopic_, 1, true);
    mpa_pub_thread_ = new std::thread(&TestSlam::MapPublishLoop, this, 1);
    first_map_ = true;
    map_update_index_ = -1;
    

    scan_pos_cal.setZero();
    odom_pos_cal.setZero();

    vertex_counter_ = 0;
    info_matrix << 500.5, 0.0, 0.0,
                   0.0, 500.5, 0.0,
                   0.0, 0.0, 500.05;

    laser_data_process_ = std::make_shared<LaserDataProcess>(&nh_,tf_,&param_);

    current_time_ = ros::Time::now();

    //发布路径
    scan_path_pub_ = nh_.advertise<nav_msgs::Path>("scan_path_pub_",1,true);
    path_scan.header.stamp=current_time_;
    path_scan.header.frame_id=param_.global_frame_id;

    odom_path_pub_ = nh_.advertise<nav_msgs::Path>("odom_path_pub_",1,true);
    path_odom.header.stamp=current_time_;
    path_odom.header.frame_id=param_.global_frame_id;

    optimization_path_pub_ = nh_.advertise<nav_msgs::Path>("optimization_path_pub_",1,true);
    path_optimization_.header.stamp=current_time_;
    path_optimization_.header.frame_id=param_.global_frame_id;

    //进行里程计和激光雷达数据的同步
//    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, param_.laser_topic_name, 10);
//    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, *tf_, param_.odom_frame_id, 10);
//    scan_filter_->registerCallback(boost::bind(&TestSlam::scanCallBack, this, _1));

    scan_sub_ = nh_.subscribe(param_.laser_topic_name, 5, &TestSlam::scanCallBack, this);

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
        tf_->transformPose(target_frame_id, ident, odom_pose);
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

//     laser_data_process_->ScanDataCalibrate(scan);
//
//    if(!getTfPose(param_.odom_frame_id, param_.base_frame_id, odom_base_pose, scan.header.stamp)){
//        d_odom = Eigen::Vector3d::Zero();
//    }
//    else{
//        d_odom = calcuDeltaOdom(odom_base_pose);
//        if(d_odom(0) < 0.02 &&
//           d_odom(1) < 0.02 &&
//           d_odom(2) < tfRadians(5.0))
//        {
////            return ;
////            d_odom = Eigen::Vector3d::Zero();
//        }
//        last_pos_ = now_pos_;
//        odom_increments_.push_back(d_odom);
//
//        c = cos(odom_pos_cal(2));
//        s = sin(odom_pos_cal(2));
//        odom_pos_cal(0) += c*d_odom(0) - s*d_odom(1);
//        odom_pos_cal(1) += s*d_odom(0) + c*d_odom(1);
//        odom_pos_cal(2) += d_odom(2);
//    }
//    pubPathMsg(odom_pos_cal, path_odom, odom_path_pub_);



//    d_odom = Eigen::Vector3d::Zero();
    // d_scan_laser = scan_matcher_.ResolveScanMatch(&scan, d_odom);
    // getTfPose(param_.base_frame_id, param_.laser_frame_id,
    //           base_laser_pose, scan.header.stamp);
    // d_scan_base = PoseToTrans(base_laser_pose) * d_scan_laser;
    // c = cos(scan_pos_cal(2));
    // s = sin(scan_pos_cal(2));
    // scan_pos_cal(0) += c*d_scan_base(0) - s*d_scan_base(1);
    // scan_pos_cal(1) += s*d_scan_base(0) + c*d_scan_base(1);
    // scan_pos_cal(2) += d_scan_base(2);
    // pubPathMsg(scan_pos_cal, path_scan, scan_path_pub_);

{
    // pose_graph_.AddVertex({scan_pos_cal}, vertex_counter_++);
    // if(vertex_counter_ > 1){
    //     Edge tmp_e = {vertex_counter_-2, vertex_counter_-1, d_odom, info_matrix};
    //     pose_graph_.AddEdge(tmp_e);
    // }
    // if(vertex_counter_ == 1000) {
    //     pose_graph_.PoseGraphOptimization(0, pose_graph_.GetVertex().size());
    // }else if(!(vertex_counter_%10) && vertex_counter_ >= 30){
    //     pose_graph_.PoseGraphOptimization(vertex_counter_-20, 20);
    // }
}

{
        // path_optimization_.poses.clear();
        // for(auto & vertex : pose_graph_.GetVertex())
        // {
        //     Eigen::Vector3d pose = vertex.second.pose;

        //     current_time_ = ros::Time::now();
        //     geometry_msgs::PoseStamped v_pose_stamped;
        //     v_pose_stamped.pose.position.x = pose(0);
        //     v_pose_stamped.pose.position.y = pose(1);
        //     geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(pose(2));
        //     v_pose_stamped.pose.orientation.x = q.x;
        //     v_pose_stamped.pose.orientation.y = q.y;
        //     v_pose_stamped.pose.orientation.z = q.z;
        //     v_pose_stamped.pose.orientation.w = q.w;
        //     v_pose_stamped.header.stamp=current_time_;
        //     v_pose_stamped.header.frame_id=param_.global_frame_id;

        //     path_optimization_.poses.push_back(v_pose_stamped);
        // }
        // pubPathMsg(pose_graph_.GetVertex()[vertex_counter_-2].pose,path_optimization_,optimization_path_pub_);

}


 ros::WallTime startTime = ros::WallTime::now();
    if(sensor_data_manager_->GetRangeFinder() == nullptr){
        std::unique_ptr<slam::LaserRangeFinder> range_finder = std::make_unique<slam::LaserRangeFinder>(scan.angle_min, scan.angle_max, scan.angle_increment,
                                                              scan.range_min, scan.range_max);
        sensor_data_manager_->SetRangeFinder(std::move(range_finder));
    }

    size_t size = scan.ranges.size();
    std::shared_ptr<slam::RangeDataContainer2d> range_data_container = std::make_shared<slam::RangeDataContainer2d>(size);
    range_data_container->set_sensor_origin(Eigen::Vector2d::Zero());
    double current_index_angle = scan.angle_min;
    double range_threshold = sensor_data_manager_->GetRangeFinder()->get_range_threshold();
    double resolution = slam_processer_->get_map_resolution();
    for(size_t i = 0; i < size; ++i){
        double dist = scan.ranges[i];

        if((dist > scan.range_min) && (dist < range_threshold))
        {
            dist /= resolution;
            range_data_container->AddDataPoint(Eigen::Vector2d(cos(current_index_angle) * dist, sin(current_index_angle) * dist));
        }

        current_index_angle += scan.angle_increment;
    }

    Eigen::Vector3d predict_sensor_pose = best_pose_;
//    predict_sensor_pose(0) += cos(odom_pos_cal(2))*d_odom(0) - sin(odom_pos_cal(2))*d_odom(1);
//    predict_sensor_pose(1) += sin(odom_pos_cal(2))*d_odom(0) + cos(odom_pos_cal(2))*d_odom(1);
//    predict_sensor_pose(2) += d_odom(2);
    range_data_container->set_sensor_pose(predict_sensor_pose);

    sensor_data_manager_->AddSensorData(range_data_container, slam::OdometryData(odom_base_pose));


    slam_processer_->process(predict_sensor_pose);

//ros::WallDuration duration = ros::WallTime::now() - startTime;
//ROS_INFO("TestSLAM took: %f milliseconds", duration.toSec()*1000.0f );

    best_pose_ = slam_processer_->current_sensor_pose();
//    PublishTfMapToOdom(scan.header.stamp);
    PublishTfMapToLaser(scan.header.stamp);

    // std::cout << "Data Cnt:" << dataCnt++ << std::endl;
}

const bool kDisplayFullMap = true;
void TestSlam::MapPublishLoop(float map_pub_period){

    ros::Rate r(1.0 / map_pub_period);
    std::cout << "map pub period : " << map_pub_period << std::endl;
    while(ros::ok())
    {

        ros::WallTime t1 = ros::WallTime::now();

        // std::lock_guard<std::mutex> lock(map_mutex_);
        slam_processer_->GetMapMutex().lock();

        if(first_map_){
            map_.map.info.resolution = slam_processer_->get_map_resolution();
            map_.map.info.origin.position.x = 0.0;
            map_.map.info.origin.position.y = 0.0;
            map_.map.info.origin.position.z = 0.0;
            map_.map.info.origin.orientation.x = 0.0;
            map_.map.info.origin.orientation.y = 0.0;
            map_.map.info.origin.orientation.z = 0.0;
            map_.map.info.origin.orientation.w = 1.0;

            first_map_ = false;
        }

        std::cout << "map pub running !" << std::endl;

        std::shared_ptr<slam::Map> occu_grid_map = slam_processer_->GetPubMap();
        if(occu_grid_map != nullptr && occu_grid_map->map_update_index() != map_update_index_){

            map_update_index_ = occu_grid_map->map_update_index();

            double resolution = occu_grid_map->GetCellLength();

            if(kDisplayFullMap){
                Eigen::Vector2d map_origin(occu_grid_map->GetWorldCoords(Eigen::Vector2d::Zero()));
                map_origin.array() -= occu_grid_map->GetCellLength()*0.5;
                map_.map.info.origin.position.x = map_origin.x();
                map_.map.info.origin.position.y = map_origin.y();
                map_.map.info.origin.orientation.w = 1.0;

                map_.map.info.resolution = resolution;

                map_.map.info.width = occu_grid_map->GetSizeX();
                map_.map.info.height = occu_grid_map->GetSizeY();

                int size = map_.map.info.width * map_.map.info.height;
                map_.map.data.resize(size);
                std::vector<int8_t>& data = map_.map.data;
                memset(&data[0], -1, sizeof(int8_t) * size);
                // int size = occu_grid_map->GetGridCellNum();
                for(int i = 0; i < size; ++i){
                    switch (occu_grid_map->GetGridStates(i)){
                        case GridStates_Occupied:{
                            data[i] = 100;
                            break;
                        }
                        case GridStates_Free:{
                            data[i] = 0;
                            break;
                        }
                        case GridStates_Unknown:{
                            data[i] = -1;
                            break;
                        }
                        default:
                            break;
                    }
                }
            }else{
                Eigen::Vector2d map_origin(occu_grid_map->GetWorldCoords(occu_grid_map->GetStartGrid()));
                map_origin.array() -= occu_grid_map->GetCellLength()*0.5;
                map_.map.info.origin.position.x = map_origin.x();
                map_.map.info.origin.position.y = map_origin.y();
                map_.map.info.origin.orientation.w = 1.0;
                
                map_.map.info.resolution = resolution;

                map_.map.info.width = (occu_grid_map->GetBoundSizeX());
                map_.map.info.height = (occu_grid_map->GetBoundSizeY());

                int size = map_.map.info.width * map_.map.info.height;
                map_.map.data.resize(size, static_cast<int8_t>(-1));
                std::vector<int8_t>& data = map_.map.data;

                int data_index = 0;
                int start_x = occu_grid_map->GetStartGrid()[0];
                int start_y = occu_grid_map->GetStartGrid()[1];
                int end_x = occu_grid_map->GetEndGrid()[0];
                int end_y = occu_grid_map->GetEndGrid()[1];

                for(int j = start_y; j <= end_y; ++j){
                    for(int i = start_x; i <= end_x; ++i){
                    
                        switch (occu_grid_map->GetGridStates(i,j)){
                            case GridStates_Occupied:{
                                data.at(data_index) = 100;
                                break;
                            }
                            case GridStates_Free:{
                                data.at(data_index) = 0;
                                break;
                            }
                            case GridStates_Unknown:{
                                data.at(data_index) = -1;
                                break;
                            }
                            default:
                                break;
                        }
                        data_index++;

                    }
                }
            }

        }
        
        
        map_.map.header.stamp = ros::Time::now();
        map_.map.header.frame_id = param_.global_frame_id;
       
        map_publisher_.publish(map_.map);


        ros::WallDuration t2 = ros::WallTime::now() - t1;
        std::cout << "map pub time s: " << t2.toSec() << std::endl;

        slam_processer_->GetMapMutex().unlock();

        r.sleep();
    }

    std::cout << "map pub thread end !" << std::endl;
}



void TestSlam::PublishTfMapToOdom(ros::Time time_stamp)
{
    tf::StampedTransform odom_to_laser;

    try
    {
      tf_->waitForTransform(param_.odom_frame_id, param_.laser_frame_id, time_stamp, ros::Duration(0.5));
      tf_->lookupTransform(param_.odom_frame_id, param_.laser_frame_id, time_stamp, odom_to_laser);
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
      odom_to_laser.setIdentity();
    }

    tf::Transform map_to_odom =  tf::Transform(tf::Transform(tf::createQuaternionFromRPY(0, 0, best_pose_[2]),
                                                            tf::Vector3(best_pose_[0], best_pose_[1], 0.0))
                                                   * odom_to_laser.inverse());
    tfB_->sendTransform( tf::StampedTransform (map_to_odom, time_stamp, param_.global_frame_id, param_.odom_frame_id));
}

void TestSlam::PublishTfMapToLaser(ros::Time time_stamp)
{
    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, best_pose_[2]),
                                               tf::Vector3(best_pose_[0], best_pose_[1], 0.0));


    tfB_->sendTransform( tf::StampedTransform (laser_to_map, time_stamp, param_.global_frame_id, param_.laser_frame_id));

//    std::cout << "pub laser_to_map tf !" << std::endl;
}

