#include <ros/ros.h> 
#include <ros/console.h> 
#include <nav_msgs/Path.h> 
#include <std_msgs/String.h> 
#include <geometry_msgs/Quaternion.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <tf/transform_broadcaster.h> 
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

struct Pose {
    double timestamp;
    double tx, ty, tz;  // 位移
    double qx, qy, qz, qw;  // 四元数
};


/*   
  # imuAccNoise: 1.0919628856399554e-02
  # imuGyrNoise: 1.0451440924866223e-02
  # imuAccBiasN: 2.1851827629113259e-04
  # imuGyrBiasN: 8.1770009351257180e-05 
  # imuGravity: 9.79172227
  # imuRPYWeight: 0.01
  # extrinsicTrans: [0.047665376129920, -0.144120962294238, 0.010086423805523]
  # extrinsicRot: [0.999457092910121, -0.031581578016513, 0.009387179146325,
  #                 0.031588741556450, 0.999500765853014, -6.154009751693067e-04,
  #                 -0.009363056101964, 9.115931250498338e-04, 0.999955754029662]
  # extrinsicRPY: [0.999457092910121, -0.031581578016513, 0.009387179146325,
  #                 0.031588741556450, 0.999500765853014, -6.154009751693067e-04,
  #                 -0.009363056101964, 9.115931250498338e-04, 0.999955754029662]

  # M2DGR  IMU Settings
  imuAccNoise: 3.7686306102624571e-02
  imuGyrNoise: 2.3417543020438883e-03
  imuAccBiasN: 1.1416642385952368e-03
  imuGyrBiasN: 1.4428407712885209e-05
  imuGravity: 9.80511
  imuRPYWeight: 0.01
  extrinsicTrans: [0.27255, -0.00053,0.17954]
  extrinsicRot: [1, 0, 0,
                  0, 1, 0,
                  0, 0, 1]
  extrinsicRPY: [1,  0, 0,
                 0, 1, 0,
                  0, 0, 1] */





int main(int argc,char** argv)
{
    ros::init(argc, argv, "show_gt_node");
    ros::NodeHandle nh;
   
    ros::Publisher pub_gt_path = nh.advertise<nav_msgs::Path>("gt_path", 1, true);
    // 读取真值轨迹文件street_04
    #define DIR_GT std::getenv("HOME") + std::string("/out")
    std::string gt_file = DIR_GT + std::string("/street_04.txt");
    std::ifstream GT(gt_file);
    // 判断是否读取成功
    if(!GT.is_open())
    {
        std::cout << "Error opening file gt_file" << std::endl;
        return -1;
    }
    // // 存储解析结果的容器
    // std::vector<Pose> poses;
    // // 初始化容器大小
    // poses.reserve(10000);
    geometry_msgs::PoseStamped convert_pose;
    nav_msgs::Path gt_path;
    gt_path.header.frame_id = "world";  // 使用适当的坐标系

    nav_msgs::Path after_gt_path;
    after_gt_path.header.frame_id = "world";  // 使用适当的坐标系


    // 读取第一个真值位姿
    std::string first_line;
    std::getline(GT, first_line);
    std::istringstream first_iss(first_line);
    double first_timestamp,first_tx, first_ty, first_tz, first_qx, first_qy, first_qz, first_qw;
    if (!(first_iss >> first_timestamp >> first_tx >> first_ty >> first_tz >> first_qx >> first_qy >> first_qz >> first_qw))
    {
        ROS_WARN_STREAM("Failed to parse line: " << first_line);
        GT.close();
        return -1;
    }
    // 使用convert_pose保存第一个真值位姿到原点的变换
    convert_pose.pose.position.x = first_tx;
    convert_pose.pose.position.y = first_ty;
    convert_pose.pose.position.z = first_tz;
    convert_pose.pose.orientation.x = first_qx;
    convert_pose.pose.orientation.y = first_qy;
    convert_pose.pose.orientation.z = first_qz;
    convert_pose.pose.orientation.w = first_qw;

    // 手动旋转轨迹
    double pitch_angle = 90.0;  // 俯仰角（以度为单位）
    double yaw_angle = 45.0;    // 偏转角（以度为单位）
    double roll_angle = 45.0;   // 翻滚角（以度为单位）
    // 将角度转换为弧度
    double pitch_rad = pitch_angle * M_PI / 180.0;
    double yaw_rad = yaw_angle * M_PI / 180.0;
    double roll_rad = roll_angle * M_PI / 180.0;

    // 计算旋转的四元数
    tf::Quaternion rotation_quat;
    rotation_quat.setRPY(roll_rad, pitch_rad, yaw_rad);
    std::cout << "rotation_quat: " << rotation_quat.x() << " " << rotation_quat.y() << " " << rotation_quat.z() << " " << rotation_quat.w() << std::endl;

    std::string line;
    while (std::getline(GT, line)) 
    {
        std::istringstream iss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
        {
            ROS_WARN_STREAM("Failed to parse line: " << line);
            GT.close();
            return -1;
        }

        // 其他点通过变换进行调整
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time(timestamp);
            pose_stamped.header.frame_id = gt_path.header.frame_id;
            pose_stamped.pose.position.x = tx - convert_pose.pose.position.x;
            pose_stamped.pose.position.y = ty - convert_pose.pose.position.y;
            // pose_stamped.pose.position.z = tz - convert_pose.pose.position.z;
            pose_stamped.pose.position.z = 0; 
            pose_stamped.pose.orientation.x = 0;
            pose_stamped.pose.orientation.y = 0;
            pose_stamped.pose.orientation.z = 0;
            pose_stamped.pose.orientation.w = 0;





/*         // 组合四元数
            tf::Quaternion pose_quat(qx, qy, qz, qw);
            // 进行手动旋转
            pose_quat = rotation_quat * pose_quat;

            pose_stamped.pose.orientation.x = pose_quat.x();
            pose_stamped.pose.orientation.y = pose_quat.y();
            pose_stamped.pose.orientation.z = pose_quat.z();
            pose_stamped.pose.orientation.w = pose_quat.w();
        
         */
            gt_path.poses.push_back(pose_stamped);




}
    // std::cout<<"读取完毕"<<std::endl;
    // 关闭文件
    GT.close();

    // 读取完毕，开始发布轨迹
    ros::Rate rate(1.0);
    while (ros::ok())
    {
        pub_gt_path.publish(gt_path);
        rate.sleep();
    }
    
    // 正常退出程序
    return 0; 






}