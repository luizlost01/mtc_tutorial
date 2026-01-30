#include <memory>
#include <string>
#include <vector>
#include <algorithm> // for transform and tolower

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// MoveIt Messages
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

// TF2 Headers
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include "tf2/exceptions.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class BottleCollision : public rclcpp::Node {
public:
    BottleCollision() : Node("bottle_collision") {
        // Initialize TF Buffer and Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers and Subscribers
        collision_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);
        
        subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/fbot_vision/fr/object_markers", 
            10, 
            std::bind(&BottleCollision::listener_callback, this, _1)
        );
    }

private:
    moveit_msgs::msg::CollisionObject create_collision_object(const geometry_msgs::msg::Point& position, const std::string& frame_id) {
        moveit_msgs::msg::CollisionObject collision_obj;
        collision_obj.header.frame_id = frame_id;
        collision_obj.header.stamp = this->now();
        
        collision_obj.id = "bottle_obstacle";

        // --- 1. PRIMITIVA DA GARRAFA ---
        shape_msgs::msg::SolidPrimitive bottle_prim;
        bottle_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        bottle_prim.dimensions = {0.12, 0.12, 0.25}; // x, y, z

        geometry_msgs::msg::Pose bottle_pose;
        bottle_pose.position = position;
        bottle_pose.position.z -= 0.12; 
        bottle_pose.orientation.w = 1.0;

        // --- 2. PRIMITIVA DA BASE/MESA ---
        shape_msgs::msg::SolidPrimitive base_prim;
        base_prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        base_prim.dimensions = {0.30, 0.30, 0.02}; // 30cm x 30cm x 2cm

        geometry_msgs::msg::Pose base_pose;
        base_pose.position = position;
        // Ajuste para ficar logo abaixo da garrafa
        base_pose.position.z -= 0.255; 
        base_pose.orientation.w = 1.0;

        // Adiciona primitivas ao objeto
        collision_obj.primitives.push_back(bottle_prim);
        collision_obj.primitive_poses.push_back(bottle_pose);

        collision_obj.primitives.push_back(base_prim);
        collision_obj.primitive_poses.push_back(base_pose);
        
        collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        return collision_obj;
    } // Fim da função create_collision_object

    void listener_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        for (const auto& marker : msg->markers) {
            std::string marker_text = marker.text;
            std::transform(marker_text.begin(), marker_text.end(), marker_text.begin(), ::tolower);

            if (marker_text.find("bottle") != std::string::npos) {
                
                geometry_msgs::msg::PoseStamped pose_camera;
                pose_camera.header.frame_id = "camera_color_optical_frame";
                pose_camera.header.stamp = marker.header.stamp;
                pose_camera.pose = marker.pose;

                try {
                    geometry_msgs::msg::PoseStamped pose_base;
                    
                    pose_base = tf_buffer_->transform(
                        pose_camera, 
                        "link_base", 
                        tf2::durationFromSec(1.0)
                    );

                    RCLCPP_INFO(this->get_logger(), "Colisão atualizada em X: %.3f", pose_base.pose.position.x);

                    auto real_collision = create_collision_object(pose_base.pose.position, "link_base");
                    
                    moveit_msgs::msg::PlanningScene planning_scene;
                    planning_scene.world.collision_objects.push_back(real_collision);
                    planning_scene.is_diff = true;

                    collision_pub_->publish(planning_scene);

                } catch (const tf2::TransformException & ex) {
                    RCLCPP_WARN(this->get_logger(), "TF Falhou: %s", ex.what());
                    return;
                }
            }
        }
    } // Fim da função listener_callback

    // Variáveis membro
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr collision_pub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;

}; // Fim da classe BottleCollision

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BottleCollision>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}