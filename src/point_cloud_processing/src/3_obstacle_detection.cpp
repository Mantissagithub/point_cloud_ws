#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std;

class TerrainNavigator : public rclcpp::Node {
public:
    enum Direction {
        LEFT,
        RIGHT,
        FRONT,
        BACK
    };

    enum TerrainType {
        SAFE,
        OBSTACLE,
        PIT,
        CRITICAL
    };

    struct TerrainAnalysis {
        TerrainType type;
        float distance;
        Direction obstacleDirection;
        Eigen::Vector3f directionVector;
    };

    TerrainNavigator() : Node("terrain_navigator") {
        point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10,
            std::bind(&TerrainNavigator::processPointCloud, this, std::placeholders::_1)
        );

        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        terrain_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/terrain_map", 10);
    }

private:
    static constexpr float OBSTACLE_HEIGHT_THRESHOLD = 0.65;
    static constexpr float PIT_DEPTH_THRESHOLD = -0.15;
    static constexpr float SAFE_DISTANCE = 2.0;
    static constexpr float CRITICAL_DISTANCE = 0.5;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_map_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

    void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud);
    }

    TerrainAnalysis analyzePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
        TerrainAnalysis result{TerrainType::SAFE, std::numeric_limits<float>::infinity(), Direction::FRONT, Eigen::Vector3f::Zero()};

        std::vector<std::pair<float, float>> directionSectors = {
            {-M_PI/4, M_PI/4},     // FRONT
            {M_PI/4, 3*M_PI/4},    // LEFT
            {-3*M_PI/4, -M_PI/4},  // RIGHT
        };

        for (const auto& point : cloud->points) {
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            float angle = std::atan2(point.y, point.x);

            Direction currentDirection = Direction::FRONT;
            for (size_t i = 0; i < directionSectors.size(); ++i) {
                if (angle >= directionSectors[i].first && angle < directionSectors[i].second) {
                    currentDirection = static_cast<Direction>(i);
                    break;
                }
            }

            if (point.z > OBSTACLE_HEIGHT_THRESHOLD && distance < SAFE_DISTANCE) {
                if (distance < result.distance) {
                    result.type = TerrainType::OBSTACLE;
                    result.distance = distance;
                    result.obstacleDirection = currentDirection;
                    result.directionVector = Eigen::Vector3f(point.x, point.y, point.z).normalized();

                    RCLCPP_WARN(this->get_logger(), 
                        "Obstacle in %s: Distance=%.2f, Height=%.2f", 
                        getDirectionName(currentDirection).c_str(), distance, point.z
                    );
                }
            }

            if (point.z < PIT_DEPTH_THRESHOLD && distance < SAFE_DISTANCE) {
                if (distance < result.distance) {
                    result.type = TerrainType::PIT;
                    result.distance = distance;
                    result.obstacleDirection = currentDirection;
                    result.directionVector = Eigen::Vector3f(point.x, point.y, point.z).normalized();
                }
            }
        }

        if (result.distance < CRITICAL_DISTANCE && result.type != TerrainType::SAFE) {
            result.type = TerrainType::CRITICAL;
            RCLCPP_ERROR(this->get_logger(), "Critical Obstacle in %s!", getDirectionName(result.obstacleDirection).c_str());
        }

        return result;
    }

    geometry_msgs::msg::Twist generateNavigationCommand(const TerrainAnalysis& terrain) {
        geometry_msgs::msg::Twist cmd_vel;

        switch (terrain.type) {
            case TerrainType::SAFE:
                cmd_vel.linear.x = 0.5;
                cmd_vel.angular.z = 0.0;
                break;
            
            case TerrainType::OBSTACLE:
                switch (terrain.obstacleDirection) {
                    case Direction::FRONT:
                        cmd_vel.linear.x = 0.1;
                        cmd_vel.angular.z = terrain.directionVector.y() > 0 ? -0.7 : 0.7;
                        break;
                    case Direction::LEFT:
                        cmd_vel.linear.x = 0.2;
                        cmd_vel.angular.z = 0.5;
                        break;
                    case Direction::RIGHT:
                        cmd_vel.linear.x = 0.2;
                        cmd_vel.angular.z = -0.5;
                        break;
                    case Direction::BACK:
                        cmd_vel.linear.x = -0.2;
                        cmd_vel.angular.z = 0.0;
                        break;
                }
                break;

            case TerrainType::PIT:
                cmd_vel.linear.x = 0.1;
                switch (terrain.obstacleDirection) {
                    case Direction::LEFT:
                        cmd_vel.angular.z = 0.5;
                        break;
                    case Direction::RIGHT:
                        cmd_vel.angular.z = -0.5;
                        break;
                    case Direction::FRONT:
                        cmd_vel.angular.z = terrain.directionVector.y() > 0 ? 0.5 : -0.5;
                        break;
                    case Direction::BACK:
                        cmd_vel.linear.x = -0.1;
                        cmd_vel.angular.z = 0.0;
                        break;
                }
                break;

            case TerrainType::CRITICAL:
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                break;
        }

        return cmd_vel;
    }

    std::string getDirectionName(Direction dir) {
        switch (dir) {
            case Direction::FRONT: return "FRONT";
            case Direction::LEFT:  return "LEFT";
            case Direction::RIGHT: return "RIGHT";
            case Direction::BACK:  return "BACK";
            default: return "UNKNOWN";
        }
    }

    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        RCLCPP_INFO(this->get_logger(), 
            "Point Cloud Stats: Total Points=%zu", 
            cloud->points.size()
        );

        preprocessPointCloud(cloud);

        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);
        voxel_grid.filter(*cloud);

        TerrainAnalysis terrain = analyzePointCloud(cloud);

        auto cmd_vel = generateNavigationCommand(terrain);
        cmd_vel_pub->publish(cmd_vel);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TerrainNavigator>());
    rclcpp::shutdown();
    return 0;
}
