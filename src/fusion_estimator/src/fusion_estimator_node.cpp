#include <memory>
#include <iostream>
#include <filesystem>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <urdf/model.h>
#include <unitree_legged_msgs/LowState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <ros/ros.h>

#include "fusion_estimator/FusionEstimatorTest.h" 
#include "GO1FusionEstimator/Estimator/EstimatorPortN.h"
#include "GO1FusionEstimator/Sensor_Legs.h" 
#include "GO1FusionEstimator/Sensor_IMU.h" 

using namespace DataFusion;

class FusionEstimatorNode
{
public:
    FusionEstimatorNode() 
    {
        // Initialize ROS1 node handle
        ros::NodeHandle nh;
        
        // Initialize publishers and subscribers
        // lowstate_subscriber = nh.subscribe("/go1/lowstate", 10, 
        //                                    &FusionEstimatorNode::LowStateCallback, this);

        udpstate_subscriber = nh.subscribe("/smxfe/low_state_array", 10, 
                                            &FusionEstimatorNode::UdpStateCallback, this);        
        // Reset subscriber
        reset_subscriber = nh.subscribe<std_msgs::Bool>("/smxfe/reset_pose", 1, &FusionEstimatorNode::ResetCallback, this);

        fe_test_publisher = nh.advertise<fusion_estimator::FusionEstimatorTest>("SMXFE/Estimation", 10);
        smxfe_publisher = nh.advertise<nav_msgs::Odometry>("SMXFE/Odom", 10);
        smx_imu_publisher = nh.advertise<sensor_msgs::Imu>("SMXFE/Imu", 10);
        
        // Initialize TF broadcaster
        tf_broadcaster = std::make_shared<tf::TransformBroadcaster>();

        for(int i = 0; i < 2; i++)
        {
            EstimatorPortN* StateSpaceModel1_SensorsPtrs = new EstimatorPortN; // Create new structure instance
            StateSpaceModel1_Initialization(StateSpaceModel1_SensorsPtrs);     // Call initialization function
            StateSpaceModel1_Sensors.push_back(StateSpaceModel1_SensorsPtrs);  // Add pointer to container
        }

        Sensor_IMUAcc = std::make_shared<DataFusion::SensorIMUAcc>(StateSpaceModel1_Sensors[0]);
        Sensor_IMUMagGyro = std::make_shared<DataFusion::SensorIMUMagGyro>(StateSpaceModel1_Sensors[1]);
        Sensor_Legs = std::make_shared<DataFusion::SensorLegs>(StateSpaceModel1_Sensors[0]);

        // Set parameters with ROS1 parameter server
        nh.param<double>("Modify_Par_1", modify_par_1, 0.0);
        nh.param<double>("Modify_Par_2", modify_par_2, 0.0);
        nh.param<double>("Modify_Par_3", modify_par_3, 0.0);
        
        // Apply initial parameters
        UpdateSensorCalibration();
        
        ROS_INFO("Fusion Estimator Node Initialized");
    }

    // Sensor state space model creation
    std::vector<EstimatorPortN*> StateSpaceModel1_Sensors = {}; // Container declaration
    std::vector<EstimatorPortN*> StateSpaceModel2_Sensors = {}; // Container declaration

    void ObtainParameter();
    void spin() {
        ros::spin();
    }

private:
    ros::Publisher fe_test_publisher;
    ros::Publisher smxfe_publisher;
    ros::Publisher smx_imu_publisher;
    ros::Subscriber lowstate_subscriber;
    ros::Subscriber udpstate_subscriber;
    ros::Subscriber reset_subscriber;

    std::shared_ptr<DataFusion::SensorIMUAcc> Sensor_IMUAcc; 
    std::shared_ptr<DataFusion::SensorIMUMagGyro> Sensor_IMUMagGyro; 
    std::shared_ptr<DataFusion::SensorLegs> Sensor_Legs; 
    std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
    
    // ROS Parameters
    double modify_par_1;
    double modify_par_2;
    double modify_par_3;

    void UpdateSensorCalibration()
    {
        double AngleCorrect[3] = {0};
        AngleCorrect[0] = modify_par_1 * M_PI / 180.0;
        AngleCorrect[1] = modify_par_2 * M_PI / 180.0;
        AngleCorrect[2] = modify_par_3 * M_PI / 180.0;

        Eigen::AngleAxisd rollAngle(AngleCorrect[2], Eigen::Vector3d::UnitZ());  // Rotate around X axis
        Eigen::AngleAxisd pitchAngle(AngleCorrect[1], Eigen::Vector3d::UnitY()); // Rotate around Y axis
        Eigen::AngleAxisd yawAngle(AngleCorrect[0], Eigen::Vector3d::UnitX());   // Rotate around Z axis

        Sensor_IMUMagGyro->SensorQuaternion = yawAngle * pitchAngle * rollAngle;
        Sensor_IMUMagGyro->SensorQuaternionInv = Sensor_IMUMagGyro->SensorQuaternion.inverse();
        std::cout << "Sensor_IMUMagGyro->SensorQuaternion: " << Sensor_IMUMagGyro->SensorQuaternion.coeffs().transpose() << std::endl;
    }

    void LowStateCallback(const unitree_legged_msgs::LowState::ConstPtr& msg)
    {
        std_msgs::Float32MultiArray converted_msg;
        converted_msg.data.resize(37);
        for (int i = 0; i < 3; ++i) converted_msg.data[i] = msg->imu.accelerometer[i];
        for (int i = 0; i < 3; ++i) converted_msg.data[3+i] = msg->imu.rpy[i];
        for (int i = 0; i < 3; ++i) converted_msg.data[6+i] = msg->imu.gyroscope[i];
        for (int i = 0; i < 12; ++i) converted_msg.data[9+i] = msg->motorState[i].q;
        for (int i = 0; i < 12; ++i) converted_msg.data[21+i] = msg->motorState[i].dq;
        for (int i = 0; i < 4; ++i) converted_msg.data[33+i] = msg->footForce[i];
        UdpStateCallback(boost::make_shared<std_msgs::Float32MultiArray>(converted_msg));
    }
    
    void UdpStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
    {
        if (msg->data.size() != 37)
        {
            ROS_WARN("Received Float32MultiArray size mismatch. Expected 37, got %zu", msg->data.size());
            return;
        }

        const auto& data = msg->data;
        fusion_estimator::FusionEstimatorTest fusion_msg;
        fusion_msg.stamp = ros::Time::now();

        double LatestMessage[3][100] = {0};
        static double LastMessage[3][100] = {0};
        double CurrentTimestamp = fusion_msg.stamp.toSec();

        // -------------------------- 赋值 imu --------------------------
        for(int i = 0; i < 3; i++){
            fusion_msg.data_check_a[0+i] = data[i];         // accelerometer
            fusion_msg.data_check_a[3+i] = data[3+i];       // rpy
            fusion_msg.data_check_a[6+i] = data[6+i];       // gyro

            LatestMessage[0][3*i+2] = data[i];              // accelerometer for IMUAcc
            LatestMessage[1][3*i]   = data[3+i];            // rpy for IMUMagGyro
            LatestMessage[1][3*i+1] = data[6+i];            // gyro for IMUMagGyro
        }

        // Sensor Data Handle
        for(int i = 0; i < 9; i++)
        {
            if(LastMessage[0][i] != LatestMessage[0][i])
            {
                Sensor_IMUAcc->SensorDataHandle(LatestMessage[0], CurrentTimestamp);
                for(int j = 0; j < 9; j++) LastMessage[0][j] = LatestMessage[0][j];
                break;
            }
        }

        for(int i = 0; i < 9; i++)
        {
            if(LastMessage[1][i] != LatestMessage[1][i])
            {
                Sensor_IMUMagGyro->SensorDataHandle(LatestMessage[1], CurrentTimestamp);
                for(int j = 0; j < 9; j++) LastMessage[1][j] = LatestMessage[1][j];
                break;
            }
        }

        for(int i=0; i<9; i++){
            fusion_msg.estimated_xyz[i] = StateSpaceModel1_Sensors[0]->EstimatedState[i];
            fusion_msg.estimated_rpy[i] = StateSpaceModel1_Sensors[1]->EstimatedState[i];
        }

        // -------------------------- 赋值腿部 --------------------------
        for(int LegNumber = 0; LegNumber < 4; LegNumber++)
        {
            for(int i = 0; i < 3; i++)
            {
                fusion_msg.data_check_b[LegNumber*3+i] = data[9 + LegNumber*3 + i];   // q
                fusion_msg.data_check_c[LegNumber*3+i] = data[21 + LegNumber*3 + i];  // dq

                LatestMessage[2][LegNumber*3+i]        = data[9 + LegNumber*3 + i];
                LatestMessage[2][12 + LegNumber*3+i]   = data[21 + LegNumber*3 + i];
            }
            LatestMessage[2][24 + LegNumber] = data[33 + LegNumber];
            fusion_msg.others[LegNumber]     = data[33 + LegNumber];
        }

        for(int i = 0; i < 28; i++)
        {
            if(LastMessage[2][i] != LatestMessage[2][i])
            {
                Sensor_Legs->SensorDataHandle(LatestMessage[2], CurrentTimestamp);
                for(int j = 0; j < 28; j++) LastMessage[2][j] = LatestMessage[2][j];
                break;
            }
        }

        for(int i=0; i<4; i++){
            for(int j=0; j<3; j++){
                fusion_msg.data_check_d[3 * i + j]       = StateSpaceModel1_Sensors[0]->Double_Par[6 * i + j];
                fusion_msg.data_check_e[3 * i + j]       = StateSpaceModel1_Sensors[0]->Double_Par[24 + 6 * i + j];
                fusion_msg.feet_based_position[3*i + j]  = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j];
                fusion_msg.feet_based_velocity[3*i + j]  = StateSpaceModel1_Sensors[0]->Double_Par[48 + 6 * i + j + 3];
            }
        }

        // -------------------------- 发布消息 --------------------------
        fe_test_publisher.publish(fusion_msg);

        nav_msgs::Odometry SMXFE_odom;
        SMXFE_odom.header.stamp = fusion_msg.stamp;
        SMXFE_odom.header.frame_id = "base";
        SMXFE_odom.child_frame_id = "trunk";

        SMXFE_odom.pose.pose.position.x = fusion_msg.estimated_xyz[0];
        SMXFE_odom.pose.pose.position.y = fusion_msg.estimated_xyz[3];
        SMXFE_odom.pose.pose.position.z = fusion_msg.estimated_xyz[6];

        tf::Quaternion q;
        q.setRPY(fusion_msg.estimated_rpy[0], fusion_msg.estimated_rpy[3], fusion_msg.estimated_rpy[6]);

        SMXFE_odom.pose.pose.orientation.x = q.x();
        SMXFE_odom.pose.pose.orientation.y = q.y();
        SMXFE_odom.pose.pose.orientation.z = q.z();
        SMXFE_odom.pose.pose.orientation.w = q.w();
        smxfe_publisher.publish(SMXFE_odom);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(
            fusion_msg.estimated_xyz[0],
            fusion_msg.estimated_xyz[3],
            fusion_msg.estimated_xyz[6]
        ));
        transform.setRotation(q);
        tf_broadcaster->sendTransform(tf::StampedTransform(
            transform,
            fusion_msg.stamp,
            "odom",
            "base"
        ));

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = fusion_msg.stamp;
        imu_msg.header.frame_id = "trunk";
        imu_msg.linear_acceleration.x = data[0];
        imu_msg.linear_acceleration.y = data[1];
        imu_msg.linear_acceleration.z = data[2];
        imu_msg.angular_velocity.x = data[6];
        imu_msg.angular_velocity.y = data[7];
        imu_msg.angular_velocity.z = data[8];
        tf::Quaternion imu_q;
        imu_q.setRPY(data[3], data[4], data[5]);
        imu_msg.orientation.x = imu_q.x();
        imu_msg.orientation.y = imu_q.y();
        imu_msg.orientation.z = imu_q.z();
        imu_msg.orientation.w = imu_q.w();
        imu_msg.orientation_covariance[0] = -1;
        imu_msg.angular_velocity_covariance[0] = -1;
        imu_msg.linear_acceleration_covariance[0] = -1;
        smx_imu_publisher.publish(imu_msg);
    }

    
    void ResetCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data)
        {
            ROS_WARN("Received reset request. Resetting estimator states...");

            // for (auto* model : StateSpaceModel1_Sensors)
            // {
            //     for (int i = 0; i < 9; ++i)
            //         model->EstimatedState[i] = 0;

            //     for (int i = 0; i < 72; ++i)
            //         model->Double_Par[i] = 0;

            // }
            // for(int i=0; i<4; i++)
            // {
            //     Sensor_Legs->FootfallPositionRecordIsInitiated[i] = 0;
            // }
            StateSpaceModel1_Sensors[0]->EstimatedState[0] = 0;
            StateSpaceModel1_Sensors[0]->EstimatedState[3] = 0;
            StateSpaceModel1_Sensors[0]->EstimatedState[6] = 0;

            ROS_INFO("Estimator state has been reset.");
        }
    }

};


void FusionEstimatorNode::ObtainParameter()
{
    // Set default kinematic parameters for the quadruped
    Sensor_Legs->KinematicParams << 
    0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
    0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
    -0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022,
    -0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022;
    
    // Get the robot description from the parameter server
    ros::NodeHandle nh;
    std::string robot_description;
    if (!nh.getParam("robot_description", robot_description))
    {
        ROS_WARN("Failed to get robot_description from parameter server, using default values.");
        return;
    }
    
    // Parse the URDF
    urdf::Model model;
    if (!model.initString(robot_description))
    {
        ROS_WARN("Failed to parse URDF from robot_description, using default values.");
        return;
    }
    
    // Set output format: fixed decimal point, 4 decimal places
    std::cout << std::fixed << std::setprecision(4);
    
    // Define the leg name order to match KinematicParams rows
    std::vector<std::string> legs = {"FL", "FR", "RL", "RR"};
    
    // Define joint mapping structure - each joint's starting column in the 13-dimensional vector
    struct JointMapping {
        std::string suffix; // Joint suffix, e.g., "hip_joint"
        int col;            // Starting column
    };
    
    // Map hip, thigh, calf, foot_joint parameters for each leg
    std::vector<JointMapping> jointMappings = {
        { "hip_joint",   0 },
        { "thigh_joint", 3 },
        { "calf_joint",  6 },
        { "foot_joint",  9 }
    };
    
    // Update parameters for each leg
    for (size_t i = 0; i < legs.size(); i++)
    {
        const std::string& leg = legs[i];
        for (const auto& jm : jointMappings)
        {
            // Construct full joint name, e.g., "FL_hip_joint"
            std::string jointName = leg + "_" + jm.suffix;
            std::shared_ptr<const urdf::Joint> joint = model.getJoint(jointName);
            if (!joint)
            {
                std::cout << "Joint not found: " << jointName << " (" << leg << "), using default value: ";
                std::cout << Sensor_Legs->KinematicParams.row(i).segment(jm.col, 3) << std::endl;
            }
            else
            {
                urdf::Vector3 pos = joint->parent_to_joint_origin_transform.position;
                Sensor_Legs->KinematicParams(i, jm.col)     = pos.x;
                Sensor_Legs->KinematicParams(i, jm.col + 1) = pos.y;
                Sensor_Legs->KinematicParams(i, jm.col + 2) = pos.z;
                std::cout << "Obtained KinematicPar for " << jointName << ": ";
                std::cout << Sensor_Legs->KinematicParams.row(i).segment(jm.col, 3) << std::endl;
            }
        }
        
        // Update foot link collision sphere radius (stored in column 12)
        std::string footLinkName = leg + "_foot";
        std::shared_ptr<const urdf::Link> footLink = model.getLink(footLinkName);
        if (!footLink)
        {
            std::cout << "Link not found: " << footLinkName << " (" << leg << "), using default value: " 
                      << Sensor_Legs->KinematicParams(i, 12) << std::endl;
        }
        else if (footLink->collision && footLink->collision->geometry &&
                 footLink->collision->geometry->type == urdf::Geometry::SPHERE)
        {
            std::shared_ptr<urdf::Sphere> sphere = std::dynamic_pointer_cast<urdf::Sphere>(footLink->collision->geometry);
            if (sphere)
            {
                Sensor_Legs->KinematicParams(i, 12) = sphere->radius;
                std::cout << "Obtained KinematicPar for " << footLinkName << ": " 
                          << Sensor_Legs->KinematicParams(i, 12) << std::endl;
            }
        }

        // Calculate lengths
        Sensor_Legs->Par_HipLength = std::sqrt(Sensor_Legs->KinematicParams(0, 3)*Sensor_Legs->KinematicParams(0, 3) + 
                                              Sensor_Legs->KinematicParams(0, 4)*Sensor_Legs->KinematicParams(0, 4) + 
                                              Sensor_Legs->KinematicParams(0, 5)*Sensor_Legs->KinematicParams(0, 5));
        Sensor_Legs->Par_ThighLength = std::sqrt(Sensor_Legs->KinematicParams(0, 6)*Sensor_Legs->KinematicParams(0, 6) + 
                                                Sensor_Legs->KinematicParams(0, 7)*Sensor_Legs->KinematicParams(0, 7) + 
                                                Sensor_Legs->KinematicParams(0, 8)*Sensor_Legs->KinematicParams(0, 8));
        Sensor_Legs->Par_CalfLength = std::sqrt(Sensor_Legs->KinematicParams(0, 9)*Sensor_Legs->KinematicParams(0, 9) + 
                                               Sensor_Legs->KinematicParams(0, 10)*Sensor_Legs->KinematicParams(0, 10) + 
                                               Sensor_Legs->KinematicParams(0, 11)*Sensor_Legs->KinematicParams(0, 11));
        Sensor_Legs->Par_FootLength = abs(Sensor_Legs->KinematicParams(0, 12));
    }

    // Get IMU installation position
    Eigen::Vector3d IMUPosition(-0.02557, 0, 0.04232);
    std::string jointName = "imu_joint";
    std::shared_ptr<const urdf::Joint> joint = model.getJoint(jointName);
    if (!joint)
    {
        std::cout << "Joint not found: " << jointName << ", using default value: ";
        std::cout << IMUPosition.transpose() << std::endl;
    }
    else
    {
        urdf::Vector3 pos = joint->parent_to_joint_origin_transform.position;
        IMUPosition = Eigen::Vector3d(pos.x, pos.y, pos.z);
        std::cout << "Obtained Position for " << jointName << ": ";
        std::cout << IMUPosition.transpose() << std::endl;
    }
    
    // Set sensor positions
    Sensor_IMUAcc->SensorPosition[0] = IMUPosition(0);
    Sensor_IMUAcc->SensorPosition[1] = IMUPosition(1);
    Sensor_IMUAcc->SensorPosition[2] = IMUPosition(2);
    Sensor_IMUMagGyro->SensorPosition[0] = IMUPosition(0);
    Sensor_IMUMagGyro->SensorPosition[1] = IMUPosition(1);
    Sensor_IMUMagGyro->SensorPosition[2] = IMUPosition(2);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusion_estimator_node");
    FusionEstimatorNode node;
    node.ObtainParameter();
    node.spin();
    return 0;
}
