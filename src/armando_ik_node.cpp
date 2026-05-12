#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "aruco_msgs/msg/marker_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>

class ArmandoIKNode : public rclcpp::Node {
public:
    ArmandoIKNode() : Node("armando_ik_node") {
        this->declare_parameter("robot_description", "");
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        joint_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        // FIX QOS: Rimettiamo la QoS a 10 (Reliable) per combaciare con aruco_ros
        marker_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/aruco_marker_publisher/markers", 10,
            std::bind(&ArmandoIKNode::marker_callback, this, std::placeholders::_1));

        contact_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/contact_sensor_topic", 10, 
            std::bind(&ArmandoIKNode::contact_callback, this, std::placeholders::_1));

        setup_kdl();
        RCLCPP_INFO(this->get_logger(), "=== Armando IK Expert Online ===");
    }

private:
    void setup_kdl() {
        std::string robot_desc;
        this->get_parameter("robot_description", robot_desc);
        KDL::Tree tree;
        if (!kdl_parser::treeFromString(robot_desc, tree)) return;
        tree.getChain("base_link", "crawer_base", arm_chain_);
        RCLCPP_INFO(this->get_logger(), "Catena KDL: %d giunti.", arm_chain_.getNrOfJoints());
    }

    void contact_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !target_touched_) {
            target_touched_ = true;
            RCLCPP_INFO(this->get_logger(), "!!! CONTATTO !!! Movimento interrotto.");
        }
    }

    void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
        // ORA VEDRAI QUESTO LOG!
        RCLCPP_INFO(this->get_logger(), "[VISION DEBUG] Messaggio ricevuto! Marker visti: %zu", msg->markers.size());

        if (target_touched_ || msg->markers.empty()) return;

        for (const auto & marker : msg->markers) {
            if (marker.id == 0) {
                RCLCPP_INFO(this->get_logger(), "[VISION DEBUG] Target 0 identificato. Avvio calcolo IK...");
                process_target(marker);
                break;
            }
        }
    }

    void process_target(const aruco_msgs::msg::Marker & marker) {
        geometry_msgs::msg::PoseStamped in_pose, out_pose;
        in_pose.header.frame_id = marker.header.frame_id;
        in_pose.header.stamp = rclcpp::Time(0); // Usa sempre l'ultima TF disponibile
        in_pose.pose = marker.pose.pose;

        try {
            out_pose = tf_buffer_->transform(in_pose, "base_link", tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException & ex) {
            // VIA IL THROTTLE: Ora il terminale griderà l'errore esatto!
            RCLCPP_ERROR(this->get_logger(), "[IK DEBUG] Errore TF: %s", ex.what());
            return;
        }

        KDL::JntArray q_out(arm_chain_.getNrOfJoints());
        if (solve_ik_relaxed(out_pose.pose, q_out)) {
            // VIA IL THROTTLE
            RCLCPP_INFO(this->get_logger(), "[IK DEBUG] IK OK -> Invio Comando ai Giunti");
            send_command(q_out);
        } else {
            // VIA IL THROTTLE
            RCLCPP_ERROR(this->get_logger(), "[IK DEBUG] IK Fallito: Cubo irraggiungibile (X: %.2f, Y: %.2f, Z: %.2f)", 
                out_pose.pose.position.x, out_pose.pose.position.y, out_pose.pose.position.z);
        }
    }

    bool solve_ik_relaxed(const geometry_msgs::msg::Pose & target, KDL::JntArray & q_out) {
        double yaw_base = std::atan2(target.position.y, target.position.x);
        
        KDL::Frame frame_target(
            KDL::Rotation::RPY(0, M_PI/2, yaw_base), 
            KDL::Vector(target.position.x, target.position.y, target.position.z - 0.025)
        );

        KDL::JntArray q_init(arm_chain_.getNrOfJoints());
        for(unsigned int i=0; i<arm_chain_.getNrOfJoints(); i++) q_init(i) = 0.0;

        KDL::ChainFkSolverPos_recursive fk_solver(arm_chain_);
        KDL::ChainIkSolverVel_pinv ik_vel_solver(arm_chain_);
        KDL::ChainIkSolverPos_NR ik_pos_solver(arm_chain_, fk_solver, ik_vel_solver, 1000, 1e-2);

        return (ik_pos_solver.CartToJnt(q_init, frame_target, q_out) >= 0);
    }

    void send_command(const KDL::JntArray & joints) {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {"j0", "j1", "j2", "j3"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (unsigned int i = 0; i < joints.rows(); ++i) {
            point.positions.push_back(joints(i));
        }
        point.time_from_start = rclcpp::Duration::from_seconds(0.5);
        msg.points.push_back(point);
        joint_pub_->publish(msg);
    }

    KDL::Chain arm_chain_;
    bool target_touched_ = false;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmandoIKNode>());
    rclcpp::shutdown();
    return 0;
}