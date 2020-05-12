/**
* \file SimpleLocate.cpp
* \brief this example demonstrates basic usage of Photoneo Localization SDK
*
* \author Michal Hagara, hagara@photoneo.com
*
* \version 1.0
*
* \date 25.7.2017
*/

#include <chrono>
#include <memory>
#include <iostream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <PhoXi.h>
#include <PhoLocalization.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

//#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace pho::sdk;
using namespace std;
using namespace std::chrono_literals;

using namespace tf2;
using namespace tf2_ros;

#include <direct.h>
#define GetCurrentDir _getcwd


std::string get_current_dir() {
   char buff[FILENAME_MAX]; //create string buffer to hold path
   GetCurrentDir( buff, FILENAME_MAX );
   string current_working_dir(buff);
   return current_working_dir;
}


//TransformationMatrix4x4 get_transform_matrix(std::unique_ptr<pho::sdk::PhoLocalization> &Localization)
TransformationMatrix4x4 get_transform_matrix()
{
    std::unique_ptr<PhoLocalization> Localization;

    Localization.reset(new PhoLocalization());
    /*
    try {
        Localization.reset(new PhoLocalization());
    }
  	catch (const AuthenticationException &ex) {
        std::cout << ex.what() << std::endl;
        return -1;
    }
    */

    SceneSource Scene;
    
    Scene = SceneSource::PhoXi("2019-08-079-LC3"); // INSERT SCANNER ID!!
    //Scene = SceneSource::PhoXi("InstalledExamples-basic-example");
    /*
    try {
        ////SceneSource::PhoXi("PhoXiScanner ID goes here");
        Scene = SceneSource::PhoXi("InstalledExamples-basic-example");
        ////SceneSource::File("Path to ply goes here");
        // Scene = SceneSource::File("scene-T-fittings.ply");
    }
    catch (const PhoLocalizationException &ex) {
        std::cout << ex.what() << std::endl;
        return -1;
    }   
    */

    //cout << get_current_dir() << endl;
    //Localization->LoadLocalizationConfiguration("C:/dev/ros2_eloquent/dev_ws/install/scan_loc/include/T-fittings/T-fitting.plcf");
    
    //Localization->LoadLocalizationConfiguration("scan_loc/include/T-fittings/T-fitting.plcf");
    Localization->LoadLocalizationConfiguration("C:/dev/ros2_eloquent/dev_ws/install/scan_loc/include/T-fittings/kaross.plcf"); //INSERT PATH TO PLCF FILE, DYNAMIC LINK HAVE BEEN TROUBLESOME

    Localization->SetSceneSource(Scene);

    // Setting stop criteria manually
    //Localization->SetStopCriterion(StopCriterion::Timeout(3500));
    Localization->SetStopCriterion(StopCriterion::NumberOfResults(5));
    
    AsynchroneResultQueue AsyncQueue = Localization->StartAsync();
    TransformationMatrix4x4 Transform;

    /*
    std::ofstream myfile;
    myfile.open ("C:/dev/ros2_eloquent/testresults.txt", std::ios::app);
    std::stringstream ssss;
    ssss << "NYTT TEST\n";
    myfile << ssss.str();
    */

    while (AsyncQueue.GetNext(Transform)) {
        /*Matrix3x3 m = tf2::Matrix3x3(Transform[0][0], Transform[1][0], Transform[2][0], Transform[0][1], Transform[1][1], Transform[2][1], Transform[0][2], Transform[1][2], Transform[2][2]);
        tf2::Quaternion q = tf2::Quaternion::Quaternion();
        m.getRotation(q);
        tf2::Vector3 v = tf2::Vector3(Transform[0][3], Transform[1][3], Transform[2][3]);

        string vx, vy, vz;
        vx = to_string(v.x());
        vy = to_string(v.y());
        vz = to_string(v.z());

        string qx, qy, qz, qw;
        qx = to_string(q.x());
        qy = to_string(q.y());
        qz = to_string(q.z());
        qw = to_string(q.w());

        std::stringstream sss;
        sss << "NYTT OBJEKT\nVector x = " << vx << " y = " << vy << " z =  " << vz << " \nQuaternion x = "<< qx << " y = " << qy << "  z = " << qz << " w = "<< qw << "\n";

        myfile << sss.str();
        */
        cout << Transform << endl;
    }

    //myfile.close();

    Localization->StopAsync();

    return Transform;
}

/*
void poseCallback(get_transform_matrix, node)
{
  static tf2_ros::StaticTransformBroadcaster sbr = tf2_ros::StaticTransformBroadcaster(node);

  TransformationMatrix4x4 Transform = get_transform_matrix();
  
  Matrix3x3 m = tf2::Matrix3x3(Transform[0][0], Transform[1][0], Transform[2][0], Transform[0][1], Transform[1][1], Transform[2][1], Transform[0][2], Transform[1][2], Transform[2][2]);
  tf2::Quaternion q = tf2::Quaternion::Quaternion();
  m.getRotation(q);
  tf2::Vector3 v = tf2::Vector3(Transform[3][0], Transform[3][1], Transform[3][2]);
  tf2::Transform t = tf2::Transform(q, v);

  geometry_msgs::msg::TransformStamped transformStamped;
  const geometry_msgs::msg::Transform transform_msg = tf2::toMsg(t);
  transformStamped.transform = transform_msg;

  sbr.sendTransform(transformStamped);
}
*/

int main(int argc, char * argv[]) 
{
    //rclcpp::init(argc, argv);
    //std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("tf2_broadcaster");
    
    //std::unique_ptr<PhoLocalization> Localization;
    //get_transform_matrix(Localization);

    TransformationMatrix4x4 Transform = get_transform_matrix();

    /*cout << typeid(Transform).name() << endl;
    cout << typeid(Transform[1][1]).name() << endl;
    cout << std::to_string(Transform[0][0])  <<endl;
    */

    Matrix3x3 m = tf2::Matrix3x3(Transform[0][0], Transform[1][0], Transform[2][0], Transform[0][1], Transform[1][1], Transform[2][1], Transform[0][2], Transform[1][2], Transform[2][2]);
      //cout << typeid(m).name() << endl;
    tf2::Quaternion q = tf2::Quaternion::Quaternion();
    m.getRotation(q);

    tf2::Vector3 v = tf2::Vector3(Transform[0][3], Transform[1][3], Transform[2][3]);
    tf2::Transform t = tf2::Transform(q, v);
    
    //static tf2_ros::StaticTransformBroadcaster sbr = tf2_ros::StaticTransformBroadcaster(node);
    //tf2_ros::TransformBroadcaster br = tf2_ros::TransformBroadcaster();

    geometry_msgs::msg::TransformStamped transformStamped;

    const geometry_msgs::msg::Transform transform_msg = tf2::toMsg(t);
    transformStamped.transform = transform_msg;

    string vx, vy, vz;
    vx = to_string(v.x());
    vy = to_string(v.y());
    vz = to_string(v.z());

    string qx, qy, qz, qw;
    qx = to_string(q.x());
    qy = to_string(q.y());
    qz = to_string(q.z());
    qw = to_string(q.w());

    std::stringstream sss;
    sss << "NYTT TEST\nVector x = " << vx << " y = " << vy << " z =  " << vz << " \nQuaternion x = "<< qx << " y = " << qy << "  z = " << qz << " w = "<< qw << "\n";
    
    
    std::ofstream myfile;
    myfile.open ("C:/dev/ros2_eloquent/testresults.txt", std::ios::app); //INSERT FILENAME TO SAVE TESTRESULTS TO
    myfile << sss.str();
    myfile.close();
    
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    string roll_str = to_string(roll);
    string pitch_str = to_string(pitch);    
    string yaw_str = to_string(yaw);

    /*
    std::stringstream ss;
    ss << "cd C:\\dev\\ros2_eloquent\\ros2-windows && call local_setup.bat && ros2 run tf2_ros static_transform_publisher " << vx << " " << vy << " " << vz << " " << roll_str << " " << pitch_str << " " << yaw_str << " foo bar";
    system(ss.str().c_str());
    */
    //system("cd C:\\dev\\ros2_eloquent\\ros2-windows && call local_setup.bat && ros2 run tf2_ros static_transform_publisher 1 2 3 0.5 0.1 -1.0 foo bar");
    
    //transformStamped.header.stamp = rclcpp::Time::now(); (chrono...)
    //transformStamped.header.frame_id = "world";
    //transformStamped.child_frame_id = turtle_name;
    
    //sbr.sendTransform(transformStamped);

    //rclcpp::spin(node);
    //rclcpp::shutdown();
    
    return 0;
}