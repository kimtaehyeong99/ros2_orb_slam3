/*

A bare-bones example node demonstrating the use of the Monocular mode in ORB-SLAM3

Author: Azmyin Md. Kamal
Date: 01/01/24

REQUIREMENTS
* Make sure to set path to your workspace in common.hpp file

*/

//* Includes
#include "ros2_orb_slam3/common.hpp"

//* Constructor
MonocularMode::MonocularMode() :Node("mono_node_cpp")
{
    // Declare parameters to be passsed from command line
    // https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/
    
    //* Find path to home directory
    homeDir = getenv("HOME");
    packagePath = "umi_ws/src/ros2_orb_slam3/"; // !HARDCODED, change it as necessary
    // std::cout<<"Home: "<<homeDir<<std::endl;
    
    // std::cout<<"VLSAM NODE STARTED\n\n";
    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 NODE STARTED");

    this->declare_parameter("node_name_arg", "not_given"); // Name of this agent 
    this->declare_parameter("voc_file_arg", "file_not_set"); // Needs to be overriden with appropriate name  
    this->declare_parameter("settings_file_path_arg", "file_path_not_set"); // path to settings file  
    
    //* Watchdog, populate default values
    nodeName = "not_set";
    vocFilePath = "file_not_set";
    settingsFilePath = "file_not_set";

    //* Populate parameter values
    rclcpp::Parameter param1 = this->get_parameter("node_name_arg");
    nodeName = param1.as_string();
    
    rclcpp::Parameter param2 = this->get_parameter("voc_file_arg");
    vocFilePath = param2.as_string();

    rclcpp::Parameter param3 = this->get_parameter("settings_file_path_arg");
    settingsFilePath = param3.as_string();

    // rclcpp::Parameter param4 = this->get_parameter("settings_file_name_arg");
    
  
    //* HARDCODED, set paths
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        pass;
        vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
        settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/Monocular/";
    }

    // std::cout<<"vocFilePath: "<<vocFilePath<<std::endl;
    // std::cout<<"settingsFilePath: "<<settingsFilePath<<std::endl;
    
    
    //* DEBUG print
    RCLCPP_INFO(this->get_logger(), "nodeName %s", nodeName.c_str());
    RCLCPP_INFO(this->get_logger(), "voc_file %s", vocFilePath.c_str());
    // RCLCPP_INFO(this->get_logger(), "settings_file_path %s", settingsFilePath.c_str());
    
    subexperimentconfigName = "/mono_py_driver/experiment_settings"; // topic that sends out some configuration parameters to the cpp ndoe
    pubconfigackName = "/mono_py_driver/exp_settings_ack"; // send an acknowledgement to the python node
    subImgMsgName = "/mono_py_driver/img_msg"; // topic to receive RGB image messages
    subTimestepMsgName = "/mono_py_driver/timestep_msg"; // topic to receive RGB image messages

    //* subscribe to python node to receive settings
    expConfig_subscription_ = this->create_subscription<std_msgs::msg::String>(subexperimentconfigName, 1, std::bind(&MonocularMode::experimentSetting_callback, this, _1));

    //* publisher to send out acknowledgement
    configAck_publisher_ = this->create_publisher<std_msgs::msg::String>(pubconfigackName, 10);

    //* subscrbite to the image messages coming from the Python driver node
    subImgMsg_subscription_= this->create_subscription<sensor_msgs::msg::Image>(subImgMsgName, 1, std::bind(&MonocularMode::Img_callback, this, _1));

    //* subscribe to receive the timestep
    subTimestepMsg_subscription_= this->create_subscription<std_msgs::msg::Float64>(subTimestepMsgName, 1, std::bind(&MonocularMode::Timestep_callback, this, _1));

    
    RCLCPP_INFO(this->get_logger(), "Waiting to finish handshake ......");
    
}

//* Destructor
MonocularMode::~MonocularMode()
{   
    
    // Stop all threads
    // Call method to write the trajectory file
    // Release resources and cleanly shutdown
    pAgent->Shutdown();
    pass;

}

//* Callback which accepts experiment parameters from the Python node
void MonocularMode::experimentSetting_callback(const std_msgs::msg::String& msg){
    
    // std::cout<<"experimentSetting_callback"<<std::endl;
    bSettingsFromPython = true;
    experimentConfig = msg.data.c_str();
    // receivedConfig = experimentConfig; // Redundant
    
    RCLCPP_INFO(this->get_logger(), "Configuration YAML file name: %s", this->receivedConfig.c_str());

    //* Publish acknowledgement
    auto message = std_msgs::msg::String();
    message.data = "ACK";
    
    std::cout<<"Sent response: "<<message.data.c_str()<<std::endl;
    configAck_publisher_->publish(message);

    //* Wait to complete VSLAM initialization
    initializeVSLAM(experimentConfig);

}

//* Method to bind an initialized VSLAM framework to this node
void MonocularMode::initializeVSLAM(std::string& configString){
    
    // Watchdog, if the paths to vocabular and settings files are still not set
    if (vocFilePath == "file_not_set" || settingsFilePath == "file_not_set")
    {
        RCLCPP_ERROR(get_logger(), "Please provide valid voc_file and settings_file paths");       
        rclcpp::shutdown();
    } 
    
    //* Build .yaml`s file path
    
    settingsFilePath = settingsFilePath.append(configString);
    settingsFilePath = settingsFilePath.append(".yaml"); // Example ros2_ws/src/orb_slam3_ros2/orb_slam3/config/Monocular/TUM2.yaml

    RCLCPP_INFO(this->get_logger(), "Path to settings file: %s", settingsFilePath.c_str());
    
    // NOTE if you plan on passing other configuration parameters to ORB SLAM3 Systems class, do it here
    // NOTE you may also use a .yaml file here to set these values
    sensorType = ORB_SLAM3::System::MONOCULAR; 
    enablePangolinWindow = true; // Shows Pangolin window output
    enableOpenCVWindow = true; // Shows OpenCV window output
    
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    std::cout << "MonocularMode node initialized" << std::endl; // TODO needs a better message
}

//* Callback that processes timestep sent over ROS
void MonocularMode::Timestep_callback(const std_msgs::msg::Float64& time_msg){
    // timeStep = 0; // Initialize
    timeStep = time_msg.data;
}

//* Callback to process image message and run SLAM node
void MonocularMode::Img_callback(const sensor_msgs::msg::Image& msg)
{
    // Initialize
    cv_bridge::CvImagePtr cv_ptr; //* Does not create a copy, memory efficient
    
    //* Convert ROS image to openCV image
    try
    {
        //cv::Mat im =  cv_bridge::toCvShare(msg.img, msg)->image;
        cv_ptr = cv_bridge::toCvCopy(msg); // Local scope
        
        // DEBUGGING, Show image
        // Update GUI Window
        // cv::imshow("test_window", cv_ptr->image);
        // cv::waitKey(3);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(),"Error reading image");
        return;
    }
    
    // std::cout<<std::fixed<<"Timestep: "<<timeStep<<std::endl; // Debug
    
    //* Perform all ORB-SLAM3 operations in Monocular mode
    //! Pose with respect to the camera coordinate frame not the world coordinate frame
    Sophus::SE3f Tcw = pAgent->TrackMonocular(cv_ptr->image, timeStep); 
    
    //* An example of what can be done after the pose w.r.t camera coordinate frame is computed by ORB SLAM3
    //Sophus::SE3f Twc = Tcw.inverse(); //* Pose with respect to global image coordinate, reserved for future use

}

// ============================================================================
// RGBDMode Implementation - for RGB-D cameras like RealSense D405
// ============================================================================

//* Constructor
RGBDMode::RGBDMode() : Node("rgbd_node_cpp")
{
    //* Find path to home directory
    homeDir = getenv("HOME");
    packagePath = "umi_ws/src/umi_gripper_test/ros2_orb_slam3/"; // !HARDCODED, change it as necessary

    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3-V1 RGBD NODE STARTED");

    // Declare parameters
    this->declare_parameter("settings_name", "RealSense_D405");
    this->declare_parameter("rgb_topic", "/camera/camera/color/image_rect_raw");
    this->declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter("enable_viewer", true);

    // Get parameter values
    settingsName = this->get_parameter("settings_name").as_string();
    rgbTopicName = this->get_parameter("rgb_topic").as_string();
    depthTopicName = this->get_parameter("depth_topic").as_string();
    enablePangolinWindow = this->get_parameter("enable_viewer").as_bool();

    // Set default paths
    vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/RGBD/" + settingsName + ".yaml";

    RCLCPP_INFO(this->get_logger(), "Settings name: %s", settingsName.c_str());
    RCLCPP_INFO(this->get_logger(), "Viewer enabled: %s", enablePangolinWindow ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Vocabulary file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings file: %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "RGB topic: %s", rgbTopicName.c_str());
    RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depthTopicName.c_str());

    // Initialize ORB-SLAM3 system
    initializeVSLAM();

    // Setup message filters for synchronized RGB-D subscription
    rgb_sub_.subscribe(this, rgbTopicName);
    depth_sub_.subscribe(this, depthTopicName);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&RGBDMode::rgbd_callback, this, _1, _2));

    // Initialize pose publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/camera_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orb_slam3/camera_path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    camera_path_.header.frame_id = "odom";

    RCLCPP_INFO(this->get_logger(), "RGBD Node initialized, waiting for camera data...");
    RCLCPP_INFO(this->get_logger(), "Publishing pose to /orb_slam3/camera_pose and /orb_slam3/camera_path");
}

//* Destructor
RGBDMode::~RGBDMode()
{
    if (pAgent != nullptr) {
        pAgent->Shutdown();
    }
}

//* Initialize ORB-SLAM3 system
void RGBDMode::initializeVSLAM()
{
    // Check if files exist
    std::ifstream vocFile(vocFilePath);
    std::ifstream settingsFile(settingsFilePath);

    if (!vocFile.good()) {
        RCLCPP_ERROR(this->get_logger(), "Vocabulary file not found: %s", vocFilePath.c_str());
        rclcpp::shutdown();
        return;
    }

    if (!settingsFile.good()) {
        RCLCPP_ERROR(this->get_logger(), "Settings file not found: %s", settingsFilePath.c_str());
        rclcpp::shutdown();
        return;
    }

    vocFile.close();
    settingsFile.close();

    // Initialize ORB-SLAM3 with RGBD sensor type
    sensorType = ORB_SLAM3::System::RGBD;
    // enablePangolinWindow is set from ROS2 parameter 'enable_viewer'

    RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3 in RGBD mode...");
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    bInitialized = true;
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 RGBD system initialized successfully!");
}

//* Callback to process synchronized RGB and Depth images
void RGBDMode::rgbd_callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                             const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    if (!bInitialized) {
        return;
    }

    cv_bridge::CvImageConstPtr cv_rgb_ptr;
    cv_bridge::CvImageConstPtr cv_depth_ptr;

    try {
        // Convert RGB image
        cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);

        // Convert Depth image (preserve original encoding for depth)
        cv_depth_ptr = cv_bridge::toCvShare(depth_msg);
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Get timestamp in seconds
    double timestamp = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;

    // Convert depth to CV_32F if needed (ORB-SLAM3 expects float depth)
    // NOTE: Keep depth in mm unit, ORB-SLAM3 will convert using DepthMapFactor from YAML
    cv::Mat depth_float;
    if (cv_depth_ptr->image.type() == CV_16UC1) {
        // Type conversion only, keep mm unit (ORB-SLAM3 uses DepthMapFactor for mm->m conversion)
        cv_depth_ptr->image.convertTo(depth_float, CV_32F);
    } else if (cv_depth_ptr->image.type() == CV_32FC1) {
        depth_float = cv_depth_ptr->image;
    } else {
        RCLCPP_WARN(this->get_logger(), "Unexpected depth image type: %d", cv_depth_ptr->image.type());
        cv_depth_ptr->image.convertTo(depth_float, CV_32F);
    }

    // Run ORB-SLAM3 RGBD tracking
    Sophus::SE3f Tcw = pAgent->TrackRGBD(cv_rgb_ptr->image, depth_float, timestamp);

    // Publish pose to ROS2 topics
    publishPose(Tcw, rgb_msg->header.stamp);
}

//* Publish camera pose to ROS2 topics
void RGBDMode::publishPose(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp)
{
    // Check if pose is valid (not NaN)
    if (Tcw.matrix().hasNaN()) {
        return;
    }

    // Convert world-to-camera (Tcw) to camera-to-world (Twc) for visualization
    Sophus::SE3f Twc = Tcw.inverse();

    // Extract translation and rotation
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q(Twc.rotationMatrix());

    // Create PoseStamped message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "odom";

    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // Publish pose
    pose_pub_->publish(pose_msg);

    // Add to path and publish
    camera_path_.header.stamp = stamp;
    camera_path_.poses.push_back(pose_msg);
    path_pub_->publish(camera_path_);

    // Broadcast TF transform
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "camera_link";
    tf_msg.transform.translation.x = t.x();
    tf_msg.transform.translation.y = t.y();
    tf_msg.transform.translation.z = t.z();
    tf_msg.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
}


// ============================================================================
// IMU_RGBDMode Implementation - RGBD + IMU for Visual-Inertial SLAM
// ============================================================================

//* Constructor
IMU_RGBDMode::IMU_RGBDMode() : Node("rgbd_imu_node_cpp")
{
    //* Find path to home directory
    homeDir = getenv("HOME");
    packagePath = "umi_ws/src/ros2_orb_slam3/"; // !HARDCODED, change it as necessary

    RCLCPP_INFO(this->get_logger(), "\nORB-SLAM3 IMU_RGBD NODE STARTED");

    // Declare parameters
    this->declare_parameter("settings_name", "RealSense_D405_IMU");
    this->declare_parameter("rgb_topic", "/camera/camera/color/image_rect_raw");
    this->declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    this->declare_parameter("imu_topic", "/imu/data");

    // Get parameter values
    settingsName = this->get_parameter("settings_name").as_string();
    rgbTopicName = this->get_parameter("rgb_topic").as_string();
    depthTopicName = this->get_parameter("depth_topic").as_string();
    imuTopicName = this->get_parameter("imu_topic").as_string();

    // Set paths (RGBD-Inertial config directory)
    vocFilePath = homeDir + "/" + packagePath + "orb_slam3/Vocabulary/ORBvoc.txt.bin";
    settingsFilePath = homeDir + "/" + packagePath + "orb_slam3/config/RGBD-Inertial/" + settingsName + ".yaml";

    RCLCPP_INFO(this->get_logger(), "Settings name: %s", settingsName.c_str());
    RCLCPP_INFO(this->get_logger(), "Vocabulary file: %s", vocFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Settings file: %s", settingsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "RGB topic: %s", rgbTopicName.c_str());
    RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depthTopicName.c_str());
    RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imuTopicName.c_str());

    // Initialize ORB-SLAM3 system
    initializeVSLAM();

    // Subscribe to IMU (separate subscription with buffering)
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imuTopicName, 1000,  // Large queue size for 200Hz IMU
        std::bind(&IMU_RGBDMode::imu_callback, this, std::placeholders::_1));

    // Setup message filters for synchronized RGB-D subscription
    rgb_sub_.subscribe(this, rgbTopicName);
    depth_sub_.subscribe(this, depthTopicName);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), rgb_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&IMU_RGBDMode::rgbd_callback, this, _1, _2));

    // Initialize pose publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orb_slam3/camera_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/orb_slam3/camera_path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    camera_path_.header.frame_id = "odom";

    RCLCPP_INFO(this->get_logger(), "IMU_RGBD Node initialized, waiting for camera and IMU data...");
    RCLCPP_INFO(this->get_logger(), "Publishing pose to /orb_slam3/camera_pose and /orb_slam3/camera_path");
}

//* Destructor
IMU_RGBDMode::~IMU_RGBDMode()
{
    if (pAgent != nullptr) {
        pAgent->Shutdown();
    }
}

//* Initialize ORB-SLAM3 system
void IMU_RGBDMode::initializeVSLAM()
{
    // Check if files exist
    std::ifstream vocFile(vocFilePath);
    std::ifstream settingsFile(settingsFilePath);

    if (!vocFile.good()) {
        RCLCPP_ERROR(this->get_logger(), "Vocabulary file not found: %s", vocFilePath.c_str());
        rclcpp::shutdown();
        return;
    }

    if (!settingsFile.good()) {
        RCLCPP_ERROR(this->get_logger(), "Settings file not found: %s", settingsFilePath.c_str());
        rclcpp::shutdown();
        return;
    }

    vocFile.close();
    settingsFile.close();

    // Initialize ORB-SLAM3 with IMU_RGBD sensor type
    sensorType = ORB_SLAM3::System::IMU_RGBD;
    enablePangolinWindow = true;

    RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3 in IMU_RGBD mode...");
    pAgent = new ORB_SLAM3::System(vocFilePath, settingsFilePath, sensorType, enablePangolinWindow);
    bInitialized = true;
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 IMU_RGBD system initialized successfully!");
}

//* Callback to process IMU messages
void IMU_RGBDMode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Convert ROS2 IMU message to ORB_SLAM3::IMU::Point
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    ORB_SLAM3::IMU::Point imuPoint(
        static_cast<float>(msg->linear_acceleration.x),
        static_cast<float>(msg->linear_acceleration.y),
        static_cast<float>(msg->linear_acceleration.z),
        static_cast<float>(msg->angular_velocity.x),
        static_cast<float>(msg->angular_velocity.y),
        static_cast<float>(msg->angular_velocity.z),
        t
    );

    std::lock_guard<std::mutex> lock(imuMutex_);
    imuBuffer_.push_back(imuPoint);

    // Limit buffer size (keep ~2 seconds of data at 200Hz = 400 samples)
    while (imuBuffer_.size() > 400) {
        imuBuffer_.pop_front();
    }
}

//* Get IMU measurements between two timestamps
std::vector<ORB_SLAM3::IMU::Point> IMU_RGBDMode::getImuMeasurements(double t0, double t1)
{
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    std::lock_guard<std::mutex> lock(imuMutex_);

    for (const auto& imu : imuBuffer_) {
        if (imu.t >= t0 && imu.t <= t1) {
            measurements.push_back(imu);
        }
    }

    // Remove old data from buffer
    while (!imuBuffer_.empty() && imuBuffer_.front().t < t0) {
        imuBuffer_.pop_front();
    }

    return measurements;
}

//* Callback to process synchronized RGB and Depth images
void IMU_RGBDMode::rgbd_callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                                  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    if (!bInitialized) {
        return;
    }

    cv_bridge::CvImageConstPtr cv_rgb_ptr;
    cv_bridge::CvImageConstPtr cv_depth_ptr;

    try {
        // Convert RGB image
        cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);

        // Convert Depth image (preserve original encoding for depth)
        cv_depth_ptr = cv_bridge::toCvShare(depth_msg);
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Get timestamp in seconds
    double timestamp = rgb_msg->header.stamp.sec + rgb_msg->header.stamp.nanosec * 1e-9;

    // Convert depth to CV_32F if needed
    cv::Mat depth_float;
    if (cv_depth_ptr->image.type() == CV_16UC1) {
        cv_depth_ptr->image.convertTo(depth_float, CV_32F);
    } else if (cv_depth_ptr->image.type() == CV_32FC1) {
        depth_float = cv_depth_ptr->image;
    } else {
        RCLCPP_WARN(this->get_logger(), "Unexpected depth image type: %d", cv_depth_ptr->image.type());
        cv_depth_ptr->image.convertTo(depth_float, CV_32F);
    }

    // Collect IMU measurements between previous and current frame
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    if (lastImageTime_ > 0) {
        vImuMeas = getImuMeasurements(lastImageTime_, timestamp);
        RCLCPP_DEBUG(this->get_logger(), "IMU measurements collected: %zu", vImuMeas.size());
    }
    lastImageTime_ = timestamp;

    // Run ORB-SLAM3 RGBD tracking with IMU data
    Sophus::SE3f Tcw = pAgent->TrackRGBD(cv_rgb_ptr->image, depth_float, timestamp, vImuMeas);

    // Publish pose to ROS2 topics
    publishPose(Tcw, rgb_msg->header.stamp);
}

//* Publish camera pose to ROS2 topics
void IMU_RGBDMode::publishPose(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp)
{
    // Check if pose is valid (not NaN)
    if (Tcw.matrix().hasNaN()) {
        return;
    }

    // Convert world-to-camera (Tcw) to camera-to-world (Twc) for visualization
    Sophus::SE3f Twc = Tcw.inverse();

    // Extract translation and rotation
    Eigen::Vector3f t = Twc.translation();
    Eigen::Quaternionf q(Twc.rotationMatrix());

    // Create PoseStamped message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = "odom";

    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // Publish pose
    pose_pub_->publish(pose_msg);

    // Add to path and publish
    camera_path_.header.stamp = stamp;
    camera_path_.poses.push_back(pose_msg);
    path_pub_->publish(camera_path_);

    // Broadcast TF transform
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "camera_link";
    tf_msg.transform.translation.x = t.x();
    tf_msg.transform.translation.y = t.y();
    tf_msg.transform.translation.z = t.z();
    tf_msg.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
}

