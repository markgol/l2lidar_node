//--------------------------------------------------------
//
//  L2lidar_node
//  Author: Mark Stegall
// Contributor: https://github.com/pondersome
//
//  Module: l2lidar_node.cpp
//
//	Purpose:
//		The l2lidar_node app is a ROS2 package which provides an interface
//      between ROS2 and the Untiree L2 4D LiDAR module.
//      The L2 provides point cloud data and IMU data as it scans.
//      The scans are 3D (x,y,z) with intensity, rannge and ring (always 1).
//      It generates point cloud data 300 points at a time (an L2 frame).
//      The L2 uses a UPD Ethernet interface to send data.
//      This app uses the L2lidar class	software package to provide
//      the backend interface to the L2.
//		This is class is structured to be compatible with the formats
//		and interfaces needed for support in the ROS2 packages.
//
//		This ROS2 package publishes the point cloud data and IMU data
//		for ROS2 subscribers.
//
//		- Publishes (default):
//			/points	(sensor_msgs/PointCloud2)
//			/imu (sensor_msgs/Imu)
//          /tf_static
//              base_link -> l2lidar_frame (names set in config file)
//              l2lidar_frame -> l2lidar_imu (names set in config file)
//
//	Implementation
//		This is the ROS2 driver for the Unitree L2 4d LiDAR.
//      This implements a publisher node for IMU and point cloud data.
//      ROS2 nodes sources:
//          src/main.cpp
//          src/l2lidar_node.cpp
//          include/l2lidar_node.hpp
//
//      L2 driver sources are in their own folder
//          L2lidarClass
//              src/L2lidar.cpp
//              include/L2lidar.h
//              include/PCpoint.h
//              include/quaternion.h
//              include/unitree_lidar_protocolL2.h
//              include/untiree_lidar_utilitiesL2.h
//
//      Restrictions
//      The sources require Qt6.10.2 or higher.
//      This only uses Qt6 Core and Qt6 Networking.
//      This node is standalone. It does not incorporate any Gui elements.
//      DO NOT USE Qt or ROS2 Gui elements or similar resources in this node.
//
//		Target:	Ubuntu 24.04 systems with ROS2 Jazzy installed
//		Initial target hardware is RPI5 (ARM64)
//		A x86_64 imeplentation will be done after the RPI5 version
//		completed.
//
//		V0.1.0	2026-02-16	Initial package skeleton
//		V0.1.1	2026-02-18	Corrected quaternion order
//      V0.2.0 	2026-02-21 	Added aggregation of L2 frames for publishing
//                          This is needed to align point cloud publishing
//                         	aligned to the requirements for other ROS2 processing
//                          which required a scan frame to have precise time stamp
//                          for the start of the scan and relative point time
//                          for each cloud point.
//                          Changed point time from float to double
//		V0.2.1	2026-03-06	Paramterized  frame3d and imu_adjust settings
//      V0.2.2  2026-03-12  Added static robot TF publish
//      V0.2.3  2026-04-12  Added enable/disable IMU publishing
//                          changed QOS for publishers to SensorDataQoS()
//      V0.3.0  2026-04-21  Updated to L2lidarClass V1.2.0
//                          Added internal Range calibration override
//                          Added realtime override parameter setting for
//                          the following parameters:
//                              aggregateNframes
//                              imu_adjust
//                              EnableCalRangeOVR
//                              calRangeScale
//                              calRangeBias
//                          Note: realtime overrides of parameters is not persistent.
//                          If you want persistence you need to change the
//                          config yaml file.
//      V0.3.1  2026-05-11  Corrected sensor_msgs::PointCloud2Iterator<float> iter_t(cloud, "time")
//                          should have been type <double>
//	    V0.3.2  2026-05-13  Added service to stop/start rotation, contributed by https://github.com/pondersome
//                          Changed point time from float to double
//                          changed the sensor_msgs::PointCloud2Iterator<float> iter_t(cloud, "time")
//                          backed to float
//                          to a float relative time point (it should be reconstructed into
//                          an int64 in the subscriber node, using the message timestamp+relative time)
//                          Added config yaml parameters for setting the L2 timescale correction scale
//      V0.3.3  2026-05-19  Documentation change only on L2 point cloud origin.
//      V0.3.4  2026-05-31  Updated to L2lidarClass V1.3.3, corrects bugs in timestamp correction introduced
//                          in the V1.3.0 L2lidarClass
//					  Added disable/enable of IMU publishing
//      V0.3.5  2026-06-03  Added enable/disable of TF publishing of base_link.
//                          Added config parameters for accel and gyro covariances
//                          Added initialization of the accel and gyro covariances to the IMU message
//      V0.3.6  2026-06-12  Changes to CMakeList.txt file to create distribution file (subfolder dist
//                          in the build folder for platform and build (release or debug).
//      V0.3.7  2026-06-13  Added config file param for topic IDs for point cloud and imu data.
//                          Code cleanup. Changing class member variable to end with an underscore to
//                          idenfity class memebr variables versus local variables.
//                          Added startup condition to support L2 power on in standby mode.
//                          This l2lidar_node will not timeout in this case but requires a start service command
//                          to send a command to the L2 to bring it out of standby.  If the L2 automatically starts
//                          automatically then the IMU and Point cloud packets woudl still be published but l2lidar_node
//                          would never timeout.  This is implelemented using the config param standby_on_powerup_enabled
//
//      V1.0.0  2026-0x-xx  This will be the first production release.
//
//  Note: class member variables end with an _
//--------------------------------------------------------

//--------------------------------------------------------
// GPL-3.0 license
//
// This file is part of l2lidar_node.
//
// l2lidar_node is free software : you can redistribute it and /or modify it under
// the terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// l2lidar_node is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with L2diagnsotic.
// If not, see < https://www.gnu.org/licenses/>.
//--------------------------------------------------------

#include "l2lidar_node.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

//---------------------------------------------------------------------
// L2LidarNode constructor
//---------------------------------------------------------------------
L2LidarNode::L2LidarNode(int argc, char **argv)
    : Node("l2lidar_node")
{
    // ---------------------------------------------------------
    // --------Declare parameters in config file ---------------

    // UDP configuration for L2 and host
    declare_parameter<std::string>("l2_ip", "192.168.1.62");
    declare_parameter<int>("l2_port", 6101);
    declare_parameter<std::string>("host_ip", "192.168.1.2");
    declare_parameter<int>("host_port", 6201);

    // timebase correction controls
    declare_parameter<bool>("enable_l2_time_correction", true);
    declare_parameter<bool>("enable_l2_host_sync", true);
    declare_parameter<int>("l2_sync_rate_ms", 50);
    declare_parameter<int>("timeScaleNum", 2);
    declare_parameter<int>("timeScaleDenom", 1);

    // UDP latency measurement
    // normally there would be no need to enable
    // mostly used as diagnostic
    declare_parameter<bool>("enable_latency_measure", false);

    // static transforms
    declare_parameter<bool>("disable_base_link_pub", false);
    declare_parameter<std::string>("frame_id", "l2lidar_frame");
    declare_parameter<std::string>("imu_frame_id", "l2lidar_imu");
    declare_parameter<std::string>("robot_id", "base_link");
    declare_parameter<float>("robot_x", 0.0);
    declare_parameter<float>("robot_y", 0.0);
    declare_parameter<float>("robot_z", 0.0);

    // type of point cloud data expected
    declare_parameter<bool>("frame3d", true);

    // adjust point cloud data using the gravity algined IMU packet pose
    // This does not correct yaw only roll and pitch
    declare_parameter<bool>("imu_adjust", true);

    // override the L2 internal calibration for RangeScale and RangeBias
    declare_parameter<bool>("EnableCalRangeOVR", false);
    declare_parameter<double>("calRangeScale", 0.000978);
    declare_parameter<double>("calRangeBias", -365.625);

    // set the watchdog timer to timeout if no data is received from the L2
    // for this length of time.  This will cause the node to quit
    declare_parameter<int>("watchdog_timeout_ms", 35000);

    // number of L2 frames to aggregate when publishing the point cloud data
    // no aggregation is set to less then 2
    declare_parameter<int>("aggregateNframes", 38);

    // IMU publishing config params
    declare_parameter<bool>("enable_IMU_publishing", true);
    declare_parameter<double>("accel_x_covar", 0.01);
    declare_parameter<double>("accel_y_covar", 0.01);
    declare_parameter<double>("accel_z_covar", 0.01);
    declare_parameter<double>("gyro_x_covar", 0.000025);
    declare_parameter<double>("gyro_y_covar", 0.000025);
    declare_parameter<double>("gyro_z_covar", 0.0000002);

    // Topic IDs for publishing
    declare_parameter<std::string>("point_cloud_topic_id", "/points");
    declare_parameter<std::string>("imu_topic_id", "/imu/data");

    // disable node timeout if L2 is set for standby on power up
    declare_parameter<bool>("standby_on_powerup_enabled", false);

    // ---------------------------------------
    // Now get parameters from config file

    // get IDs for publish
    // frame_id_, imu_frame_id_ and robot_id are private class members
    get_parameter("disable_base_link_pub", disable_base_link_pub_);
    get_parameter("frame_id", frame_id_);
    get_parameter("imu_frame_id", imu_frame_id_);
    get_parameter("robot_id", robot_id_);
    get_parameter("robot_x", robot_x_);
    get_parameter("robot_y", robot_y_);
    get_parameter("robot_z", robot_z_);

    // get UDP parameters, these are local
    std::string l2_ip, host_ip;
    int l2_port, host_port;

    get_parameter("l2_ip", l2_ip);
    get_parameter("l2_port", l2_port);
    get_parameter("host_ip", host_ip);
    get_parameter("host_port", host_port);

    // get time correction and timebase syncing parameters
    bool latency;
    int sync_rate;

    get_parameter("enable_l2_time_correction", time_corr_);
    get_parameter("enable_l2_host_sync", host_sync_);

    int timeScaleNum;
    int timeScaleDenom;
    get_parameter("timeScaleNum", timeScaleNum);
    get_parameter("timeScaleDenom", timeScaleDenom);
    timeScaleNum_ = timeScaleNum;
    timeScaleDenom_ = timeScaleDenom;

    get_parameter("l2_sync_rate_ms", sync_rate);
    get_parameter("enable_latency_measure", latency);

	// get point cloud parameters
	
    get_parameter("frame3d", frame3d_);
    get_parameter("imu_adjust", imu_adjust_);

    // get override calibration parameters
    get_parameter("EnableCalRangeOVR", EnableCalRangeOVR_);
    get_parameter("calRangeScale", calRangeScale_);
    get_parameter("calRangeBias", calRangeBias_);

    lidar_.SetCalibrationOVR(calRangeScale_, calRangeBias_);
    lidar_.EnableCalibrationOVR(EnableCalRangeOVR_);
    lidar_.SetL2TimeScale(timeScaleNum_,timeScaleDenom_);

    // --------- Watchdog timer settings---------------
    get_parameter("watchdog_timeout_ms", watchdog_timeout_ms_);
    last_imu_time_.start();
    last_pc_time_.start();

    connect(&watchdog_timer_, &QTimer::timeout,
            this, &L2LidarNode::watchdogCheck);

    watchdog_timer_.start(500);  // check twice per second

    // ---------------- point cloud -------------------
    get_parameter("aggregateNframes", aggregateNframes_);

    // IMU publishing
    get_parameter("enable_IMU_publishing", enable_IMU_publishing_);
    get_parameter("accel_x_covar", accel_x_covar_);
    get_parameter("accel_y_covar", accel_y_covar_);
    get_parameter("accel_z_covar", accel_z_covar_);
    get_parameter("gyro_x_covar", gyro_x_covar_);
    get_parameter("gyro_y_covar", gyro_y_covar_);
    get_parameter("gyro_z_covar", gyro_z_covar_);

    // get UDP parameters, these are local
    std::string point_cloud_topic_id, imu_topic_id;
    get_parameter("point_cloud_topic_id", point_cloud_topic_id);
    get_parameter("imu_topic_id", imu_topic_id);

    // disable node timeout if L2 is set for standby on power up
    bool standby_on_powerup_enabled;
    get_parameter("standby_on_powerup_enabled", standby_on_powerup_enabled);
    // only need to stop the watchdop timer if this is true
    // watchdog_timer_ will not be started until a start service command is sent
    if(standby_on_powerup_enabled){
        watchdog_timer_.stop();
    }


    //---------------------------------------------------
    // publishing initialization

    // This node still needs to process IMU packets from the L2
    // so that rotation correction cn be applied if enabled
    // The IMU publishing is also optional
    if(enable_IMU_publishing_) {
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_id, rclcpp::SensorDataQoS());
    }

    pcl_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_id, rclcpp::SensorDataQoS());

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTransform();

    //----------------------------------------------------------------------------
    // L2 initialzation

    // initialize UDP addresses and ports for sending and receiving UDP packets
    lidar_.LidarSetCmdConfig(
        QString::fromStdString(host_ip), host_port,
        QString::fromStdString(l2_ip), l2_port);

    // enable/disable time base corrections to be applied to point cloud and IMU timestamps
    lidar_.EnableL2TimeCorrection(time_corr_);
    // enable/disable host to L2 timebase sync
    lidar_.EnableL2TSsync(host_sync_);

    // set the peroidicity of the host to L2 time sync
    lidar_.SetL2TSsyncRate(sync_rate);
    // enable/disable UDP RTT latency measurements
    lidar_.EnableLatencyMeasure(latency);

    // IMU data ready signal/slot connection
    connect(&lidar_, &L2lidar::imuReceived,
            this, &L2LidarNode::onImuReceived);

    // point cloud data ready signal/slot connection
    connect(&lidar_, &L2lidar::PCL3DReceived,
            this, &L2LidarNode::onPointCloudReceived);

    // connect to the L2
    if (!lidar_.ConnectL2()) {
        throw std::runtime_error(
            "L2lidar connected failed: " +
            lidar_.GetLastUDPError().toStdString());
    }

    // ---------------- ROS spin timer ----------------
    connect(&spin_timer_, &QTimer::timeout,
            this, &L2LidarNode::spinOnce);

    spin_timer_.start(5); // 200 Hz spin

    RCLCPP_INFO(get_logger(), "L2Lidar node started");

    //----------------------------------------------------------------
    // ---------------- live enable/disable service -------
    // ~/enable accepts std_srvs/SetBool: true -> start rotation (run mode),
    // false -> enter standby mode. Also gates the watchdog so a deliberate
    // stop doesn't fire the no-IMU-data timeout and cause a respawn.
    enable_srv_ = create_service<std_srvs::srv::SetBool>(
        "~/enable",
        [this](
            const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
            std::shared_ptr<std_srvs::srv::SetBool::Response> res)
        {
            bool ok = req->data ? lidar_.LidarStartRotation()
                                : lidar_.LidarStopRotation();
            res->success = ok;
            if (!ok) {
                res->message = "L2 command send failed; check transport state";
                return;
            }
            if (req->data) {
                // resuming: re-arm watchdog
                last_imu_time_.start();
                last_pc_time_.start();
                if (!watchdog_timer_.isActive())
                    watchdog_timer_.start(500);
                res->message = "rotation start sent; motor spin-up ~20 s";
                RCLCPP_INFO(get_logger(), "enable=true: rotation start sent");
            } else {
                // stopping: pause watchdog so it doesn't fire on the
                // expected stream gap
                watchdog_timer_.stop();
                res->message = "standby sent; watchdog paused";
                RCLCPP_INFO(get_logger(), "enable=false: standby sent, watchdog paused");
            }
        });

    // ---------------- ROS parameter callback -------
    cb_params_handle_ = this->add_on_set_parameters_callback(
        std::bind(&L2LidarNode::onParamChange, this, std::placeholders::_1)
        );
}

//---------------------------------------------------------------------
// L2LidarNode destructor
//---------------------------------------------------------------------

L2LidarNode::~L2LidarNode()
{
    spin_timer_.stop();
    watchdog_timer_.stop();
}

//---------------------------------------------------------------------
// spinOnce
//---------------------------------------------------------------------
void L2LidarNode::spinOnce()
{
    if (!rclcpp::ok())
    {
        QCoreApplication::quit();
        return;
    }

    rclcpp::spin_some(shared_from_this());
}

//---------------------------------------------------------------------
// shutdownNode
//---------------------------------------------------------------------
void L2LidarNode::shutdownNode(const std::string &reason)
{
    RCLCPP_FATAL(get_logger(), "%s", reason.c_str());

    spin_timer_.stop();
    watchdog_timer_.stop();

    lidar_.DisconnectL2();   // if available in your class

    rclcpp::shutdown();
    QCoreApplication::quit();
}

//---------------------------------------------------------------------
// shutdownNode
//---------------------------------------------------------------------
void L2LidarNode::watchdogCheck()
{
    if (!rclcpp::ok())
        return;

    qint64 imu_elapsed = last_imu_time_.elapsed();
    qint64 pc_elapsed  = last_pc_time_.elapsed();

    if (imu_elapsed > watchdog_timeout_ms_)
    {
        shutdownNode("Watchdog timeout: IMU data stalled");
        return;
    }

    if (pc_elapsed > watchdog_timeout_ms_)
    {
        shutdownNode("Watchdog timeout: PointCloud data stalled");
        return;
    }
}

//---------------------------------------------------------------------
// onImuReceived
//---------------------------------------------------------------------
void L2LidarNode::onImuReceived()
{
    last_imu_time_.restart();

    if(!enable_IMU_publishing_) {
        return;
    }
    auto imu_packet = lidar_.imu();

    sensor_msgs::msg::Imu msg;
    // time stamp comes from IMU packet not system using now()
    msg.header.stamp.sec = imu_packet.data.info.stamp.sec;
    msg.header.stamp.nanosec = imu_packet.data.info.stamp.nsec;
    msg.header.frame_id = imu_frame_id_;

    // Correct order of quaternion array
    msg.orientation.w = imu_packet.data.quaternion[0];
    msg.orientation.x = imu_packet.data.quaternion[1];
    msg.orientation.y = imu_packet.data.quaternion[2];
    msg.orientation.z = imu_packet.data.quaternion[3];
    msg.orientation_covariance[0] = -1; // flag as not valid

    // gyro
    msg.angular_velocity.x = imu_packet.data.angular_velocity[0];
    msg.angular_velocity.y = imu_packet.data.angular_velocity[1];
    msg.angular_velocity.z = imu_packet.data.angular_velocity[2];

    // gyro covariance
    msg.angular_velocity_covariance[0] = accel_x_covar_;
    msg.angular_velocity_covariance[4] = accel_y_covar_;
    msg.angular_velocity_covariance[8] = accel_z_covar_;

    // accel
    msg.linear_acceleration.x = imu_packet.data.linear_acceleration[0];
    msg.linear_acceleration.y = imu_packet.data.linear_acceleration[1];
    msg.linear_acceleration.z = imu_packet.data.linear_acceleration[2];

    // accel covariance
    msg.linear_acceleration_covariance[0] = gyro_x_covar_;
    msg.linear_acceleration_covariance[4] = gyro_y_covar_;
    msg.linear_acceleration_covariance[8] = gyro_z_covar_;

	imu_pub_->publish(msg);
}

//---------------------------------------------------------------------
// onPointCloudReceived
//---------------------------------------------------------------------
void L2LidarNode::onPointCloudReceived()
{
    static long long starttime;
    static Frame aggframe;
    static int CurrentAggFrame {0};
    bool UseAggFrame {false};

    last_pc_time_.restart(); // restart watchdog

    Frame frame;
    if (!lidar_.ConvertL2data2pointcloud(frame, frame3d_, imu_adjust_,
                                         EnableCalRangeOVR_,calRangeScale_,calRangeBias_))
        return;

    if (frame.empty())
        return;

    // aggregate frames if required
    if(aggregateNframes_ > 1 && time_corr_ && host_sync_) {
        // restart aggregation once current aggregation is completed
        if(CurrentAggFrame >= aggregateNframes_) {
            CurrentAggFrame=0;
            aggframe.clear();
        }
        // add frame to aggframe

        if(CurrentAggFrame == 0) {
            starttime = frame[0].time;
        }

        aggframe += frame;

        CurrentAggFrame++;
        if(CurrentAggFrame < aggregateNframes_) {
            // keep building up aggregated frame
            return;
        }
        // if we get here frame is accumulated so go ahead and publish
        UseAggFrame = true;
    } else {
        starttime = frame[0].time;
    }


    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;

    // Use first point timestamp as frame timestamp
    long long t0 = starttime;
    long long sec_part = (t0/1000000000);
    long long nsec_part = sec_part*1000000000;
    nsec_part = t0 - nsec_part;
    cloud.header.stamp = rclcpp::Time(sec_part, nsec_part);

    cloud.height = 1;
    if(UseAggFrame) {
        cloud.width = aggframe.size();
    } else {
        cloud.width = frame.size();
    }
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        6,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
        "range", 1, sensor_msgs::msg::PointField::FLOAT32,
        "time", 1, sensor_msgs::msg::PointField::FLOAT32
        );

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> iter_r(cloud, "range");
    sensor_msgs::PointCloud2Iterator<float> iter_t(cloud, "time");

    if(UseAggFrame) {
        modifier.resize(aggframe.size());
        for (const PCpoint &p : std::as_const(aggframe))
        {
            long long relativetime;
            float newtime;
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = p.z;
            *iter_i = p.intensity;
            *iter_r = p.range;
            relativetime = p.time-starttime;
            newtime = relativetime*1.0e-9;
            *iter_t = newtime;   // per-point timestamp in seconds

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_i;
            ++iter_r;
            ++iter_t;
        }
    } else {
        modifier.resize(frame.size());
        for (const PCpoint &p : std::as_const(frame))
        {
            long long relativetime;
            float newtime;
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = p.z;
            *iter_i = p.intensity;
            *iter_r = p.range;
            relativetime = p.time-starttime;
            newtime = relativetime*1.0e-9;
            *iter_t = newtime;   // per-point timestamp in seconds

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_i;
            ++iter_r;
            ++iter_t;
        }
    }

    pcl_pub_->publish(cloud);
}

//---------------------------------------------------------------------
// publishStaticTransform
// there are 2 static transfoms published
//      base_link -> l2lidar_frame
//      l2lidar_frame -> l2lidar_imu
//---------------------------------------------------------------------
void L2LidarNode::publishStaticTransform()
{
    // This is the transform from base_link to l2lidar_drame
    if(!disable_base_link_pub_) {
        geometry_msgs::msg::TransformStamped tf_lidar;

        tf_lidar.header.stamp = this->get_clock()->now();
        tf_lidar.header.frame_id = robot_id_;   // "base_link"
        tf_lidar.child_frame_id = frame_id_;    // "l2lidar_frame"

        tf_lidar.transform.translation.x = robot_x_;
        tf_lidar.transform.translation.y =robot_y_;
        tf_lidar.transform.translation.z = robot_z_;

        // lidar rotation matches robot rotation
        tf_lidar.transform.rotation.x = 0.0;
        tf_lidar.transform.rotation.y = 0.0;
        tf_lidar.transform.rotation.z = 0.0;
        tf_lidar.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf_lidar);

        RCLCPP_INFO(get_logger(), "Published static TF: %s -> %s",
                    robot_id_.c_str(), frame_id_.c_str());
    }

    // This is the transform from l2lidar_frame to the imu_frame

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = frame_id_;      // "l2lidar_frame"
    tf_msg.child_frame_id = imu_frame_id_;   // "l2lidar_imu"

    // adjusted to known L2 translation relative to l2lidar_frame
    tf_msg.transform.translation.x = -0.007698;
    tf_msg.transform.translation.y = -0.014655;
    tf_msg.transform.translation.z = 0.00667;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(tf_msg);

    RCLCPP_INFO(get_logger(), "Published static TF: %s -> %s",
                frame_id_.c_str(), imu_frame_id_.c_str());
}


//---------------------------------------------------------------------
// onParamChange
// This is the callback that handles parameter changes
//---------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult L2LidarNode::onParamChange(
    const std::vector<rclcpp::Parameter> &params)
{
    for (const auto &p : params) {
        if (p.get_name() == "imu_adjust") {
            bool flag = p.as_bool();
            imu_adjust_ = flag;
        }
        else if (p.get_name() == "aggregateNframes") {
            int nFrames = p.as_int();
            if (nFrames < 0 || nFrames > 4000) {
                return paramFail("aggregateNframes out of range: 0-4000");
            }
            aggregateNframes_ = nFrames;
        }
        else if (p.get_name() == "EnableCalRangeOVR") {
            bool flag = p.as_bool();
            EnableCalRangeOVR_= flag;
           //new_cfg->keyframe_translation_thresh = t;
        } else if (p.get_name() == "calRangeScale") {
            double var = p.as_double();
            if (var < 0.00025 || var > 0.002) {
                return paramFail("calRangeScale out of range: 0.00025 - 0.002");
            }
            calRangeScale_ = var;
        } else if (p.get_name() == "calRangeBias") {
            double var = p.as_double();
            if (var > 0.0 || var < -1000.0 ) return paramFail("calRangeBias out of range: 0.0 to -1000.0");
            calRangeBias_ = var;
        } else {
            return paramFail("param mismatch or can not be changed dynamically");
        }
    }

    return paramSuccess();
}

//---------------------------------------------------------------------
// paramFail
//---------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult L2LidarNode::paramFail(const std::string &msg) {
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = false;
    r.reason = msg;
    return r;
}

//---------------------------------------------------------------------
// paramSuccess
//---------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult L2LidarNode::paramSuccess() {
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    return r;
}
