#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "aruco_msgs/msg/marker_array.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class ArmandoIKNode : public rclcpp::Node {
public:
    ArmandoIKNode() : Node("armando_ik_node") {
        // Parametri iniziali
        this->declare_parameter("target_id", 0);
        this->declare_parameter<std::string>("controller_action_name", "/joint_trajectory_controller/follow_joint_trajectory");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, this->get_parameter("controller_action_name").as_string());

        rclcpp::QoS qos_profile(10);
        qos_profile.reliable();
        qos_profile.durability_volatile();

        marker_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/aruco_marker_publisher/markers", qos_profile,
            std::bind(&ArmandoIKNode::marker_callback, this, std::placeholders::_1));

        contact_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/contact_sensor_topic", 10, 
            std::bind(&ArmandoIKNode::contact_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "=== Armando IK EXPERT (Full Version) Online ===");
        RCLCPP_INFO(this->get_logger(), "Usa 'ros2 param set /armando_ik_node target_id -1' per tornare in HOME.");
    }

private:
    void contact_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !target_touched_) {
            target_touched_ = true;
            RCLCPP_INFO(this->get_logger(), "!!! CONTATTO RILEVATO !!!");
        }
    }

    void marker_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
        int current_target_id = this->get_parameter("target_id").as_int();
        
        // Se l'utente chiede il target -1, mandiamo il robot in HOME e usciamo
        if (current_target_id == -1) {
            if (last_target_id_ != -1) {
                RCLCPP_INFO(this->get_logger(), "Comando HOME ricevuto. Rientro...");
                send_action_goal({0.0, 0.0, 0.0, 0.0});
                last_target_id_ = -1;
                has_valid_target_ = false;
            }
            return;
        }

        if (target_touched_ || is_moving_) return;

        // Rilevamento cambio target dinamico
        if (current_target_id != last_target_id_) {
            RCLCPP_INFO(this->get_logger(), "Passaggio a Target ID: %d", current_target_id);
            last_target_id_ = current_target_id;
            has_valid_target_ = false;
            last_target_x_ = -999.0;
        }

        if (msg->markers.empty()) return;

        for (const auto & marker : msg->markers) {
            if (marker.id == current_target_id) { 
                process_target(marker);
                return; 
            }
        }
    }

    void process_target(const aruco_msgs::msg::Marker & marker) {
        geometry_msgs::msg::PoseStamped in_pose, out_pose;
        in_pose.header.frame_id = marker.header.frame_id;
        in_pose.header.stamp = rclcpp::Time(0);
        in_pose.pose = marker.pose.pose;

        try {
            out_pose = tf_buffer_->transform(in_pose, "base_link", tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException & ex) {
            return;
        }
        
        double tx = out_pose.pose.position.x;
        double ty = out_pose.pose.position.y;
        double tz = out_pose.pose.position.z;

        double dist = std::sqrt(std::pow(tx - last_target_x_, 2) + std::pow(ty - last_target_y_, 2));
        
        if (!has_valid_target_ || dist >= 0.01) {
            std::vector<double> q_out;
            if (solve_ik_analytical(tx, ty, tz, q_out)) {
                RCLCPP_INFO(this->get_logger(), "Target ID %d Reachable. Invio traiettoria...", last_target_id_);
                send_action_goal(q_out);
                last_target_x_ = tx;
                last_target_y_ = ty;
                has_valid_target_ = true;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "ID %d FUORI PORTATA FISICA (X:%.2f Y:%.2f Z:%.2f)", last_target_id_, tx, ty, tz);
            }
        }
    }

    bool solve_ik_analytical(double x, double y, double z, std::vector<double>& q_out) {
        q_out.resize(4, 0.0);
        
        // Parametri Reali + Margine di Elasticità (0.004m) per i limiti del tavolo
        double z_off = 0.1015; 
        double l1 = 0.1150; 
        double l2 = 0.1150; 
        double l3 = 0.0750;

        // 1. Calcolo Base (j0) con Correzione Offset URDF
        double q0 = std::atan2(y, x); 
        
        // Correzione basata sul tuo rpy="0 0 -1.57" dell'URDF
        // Se il braccio va dal lato opposto, cambia + in -
        double q0_cmd = q0 - (M_PI / 2.0); 
        
        while (q0_cmd > M_PI) q0_cmd -= 2.0 * M_PI;
        while (q0_cmd < -M_PI) q0_cmd += 2.0 * M_PI;

        // 2. Cinematica 2D sul piano radiale
        double r = std::sqrt(x*x + y*y);
        double phi = -M_PI / 2.0; 

        double rw = r - l3 * std::cos(phi);
        double zw = (z - z_off) - l3 * std::sin(phi);
        double d = std::sqrt(rw*rw + zw*zw);

        if (d > (l1 + l2) || d < std::abs(l1 - l2)) return false;

        // 3. Calcolo angoli interni
        double cos_q2 = (d*d - l1*l1 - l2*l2) / (2 * l1 * l2);
        if (cos_q2 > 1.0) cos_q2 = 1.0; 
        if (cos_q2 < -1.0) cos_q2 = -1.0;
        
        double q2 = -std::acos(cos_q2); 

        double alpha = std::atan2(zw, rw);
        double beta = std::acos((l1*l1 + d*d - l2*l2) / (2 * l1 * d));
        double q1 = alpha + beta;

        // 4. Mappatura sui giunti Armando
        q_out[0] = q0_cmd; 
        q_out[1] = q1 - (M_PI / 2.0); 
        q_out[2] = q2;
        q_out[3] = phi - (q1 + q2);

        return true;
    }

    void send_action_goal(const std::vector<double>& positions) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {"j0", "j1", "j2", "j3"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.velocities = {0.0, 0.0, 0.0, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        
        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        
        send_goal_options.goal_response_callback = [this](GoalHandleFollowJointTrajectory::SharedPtr goal_handle) {
            if (!goal_handle) {
                this->is_moving_ = false;
            } else {
                this->is_moving_ = true;
            }
        };

        send_goal_options.result_callback = [this](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
            this->is_moving_ = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Movimento completato.");
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // Memoria di stato
    int last_target_id_ = -1;
    bool target_touched_ = false;
    bool is_moving_ = false;
    bool has_valid_target_ = false;
    double last_target_x_ = -999.0, last_target_y_ = -999.0;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contact_sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmandoIKNode>());
    rclcpp::shutdown();
    return 0;
}