#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class OdomMonitor : public rclcpp::Node
{
public:
    OdomMonitor() : Node("odom_monitor"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        filter_vel_x = 0;
        filter_vel_y = 0;
        filter_angular_z = 0;
        last_yaw_ = 0;
        last_x_ = 0;
        last_y_ = 0;
        dt_ms_ = 50;
        temp_pose_pub = this->create_publisher<geometry_msgs::msg::Pose>("pose_temp", 10);
        temp_twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("twist_temp", 10);
        // 定时器：每0.1秒查询一次TF
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(int(dt_ms_)),
            std::bind(&OdomMonitor::queryTransform, this));
    }

private:
    void queryTransform()
    {
        try
        {
            // 查询 odom -> base_scan 的变换
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform(
                 "odom", "base_footprint", tf2::TimePointZero);
            
            nav_msgs::msg::Odometry odom_msg_now;
            odom_msg_now.header = transformStamped.header;         
            // TODO: child_frame_id 可能不是 base_scan
            odom_msg_now.child_frame_id = transformStamped.child_frame_id;
            odom_msg_now.pose.pose.position.x = transformStamped.transform.translation.x;
            odom_msg_now.pose.pose.position.y = transformStamped.transform.translation.y;
            odom_msg_now.pose.pose.position.z = transformStamped.transform.translation.z;
            odom_msg_now.pose.pose.orientation.x = transformStamped.transform.rotation.x;
            odom_msg_now.pose.pose.orientation.y = transformStamped.transform.rotation.y;
            odom_msg_now.pose.pose.orientation.z = transformStamped.transform.rotation.z;
            odom_msg_now.pose.pose.orientation.w = transformStamped.transform.rotation.w;
            
            tf2::Quaternion quat_tf;
            quat_tf.setValue(
                transformStamped.transform.rotation.x, 
                transformStamped.transform.rotation.y, 
                transformStamped.transform.rotation.z, 
                transformStamped.transform.rotation.w
                );

            double angular_z_now = quat_tf.getAngle();
            double temp_x = (transformStamped.transform.translation.x - last_x_) / (dt_ms_ * 0.001);
            double temp_y = (transformStamped.transform.translation.y - last_y_) / (dt_ms_ * 0.001);

            // 滤波系数相加等于1，旧数据的系数越靠近1，滤波效果越强，曲线越平滑，但相位越滞后
            filter_vel_x = 0.2 * temp_x + 0.8 * filter_vel_x;
            filter_vel_y = 0.2 * temp_y + 0.8 * filter_vel_y;
            
            odom_msg_now.twist.twist.linear.z = 0;
            odom_msg_now.twist.twist.angular.x = 0;
            odom_msg_now.twist.twist.angular.y = 0;
            double temp_wz = (angular_z_now - last_yaw_) / (dt_ms_ * 0.001);

            filter_angular_z = 0.5 * temp_wz + 0.5 * filter_angular_z;

            odom_msg_now.twist.twist.angular.z = filter_angular_z;
            odom_msg_now.twist.twist.linear.x = filter_vel_x;
            odom_msg_now.twist.twist.linear.y = filter_vel_y;

            last_yaw_ = angular_z_now;
            last_x_ = transformStamped.transform.translation.x;
            last_y_ = transformStamped.transform.translation.y;
            odom_pub_->publish(odom_msg_now);

            temp_pose_pub->publish(odom_msg_now.pose.pose);
            temp_twist_pub->publish(odom_msg_now.twist.twist);

            // 打印变换信息
            RCLCPP_INFO(this->get_logger(), "Transform: odom -> base_scan");
            RCLCPP_INFO(this->get_logger(), "Translation: [%.4f, %.4f, %.4f]",
                        transformStamped.transform.translation.x,
                        transformStamped.transform.translation.y,
                        transformStamped.transform.translation.z);
            RCLCPP_INFO(this->get_logger(), "Rotation: [%.4f, %.4f, %.4f, %.4f]",
                        transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z,
                        transformStamped.transform.rotation.w);

            
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }

    tf2_ros::Buffer tf_buffer_;                          // TF Buffer
    tf2_ros::TransformListener tf_listener_;             // TF Listener
    rclcpp::TimerBase::SharedPtr timer_;                 // 定时器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr temp_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr temp_twist_pub;

    double last_yaw_;
    double last_x_;
    double last_y_;
    double dt_ms_;
    double filter_vel_x;
    double filter_vel_y;
    double filter_angular_z;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomMonitor>());
    rclcpp::shutdown();
    return 0;
}