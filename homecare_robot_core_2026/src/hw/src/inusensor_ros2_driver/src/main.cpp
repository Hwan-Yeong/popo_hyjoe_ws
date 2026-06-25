#include <rclcpp/rclcpp.hpp>
#include "inusensor_ros2_driver/inusensor_node.hpp"
#include <iostream>
#include <exception>

int main(int argc, char* argv[])
{
    std::cout << "Starting InuSensor ROS2 Driver..." << std::endl;
    
    try {
        // Initialize ROS2
        std::cout << "Initializing ROS2..." << std::endl;
        rclcpp::init(argc, argv);
        
        std::cout << "Creating InuSensor node..." << std::endl;
        
        // Create and run the InuSensor node
        auto node = std::make_shared<inusensor_ros2_driver::InuSensorNode>();
        
        std::cout << "<<<<<< Inuitive M4.51S ROS2 Driver >>>>>>" << std::endl;
        std::cout << "Node created successfully, starting spin..." << std::endl;
        
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        // Spin the node
        executor.spin();
                
        std::cout << "Node spinning ended normally" << std::endl;
        
    } catch (const rclcpp::exceptions::RCLError& e) {
        std::cerr << "RCL Error in InuSensor node: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    } catch (const std::runtime_error& e) {
        std::cerr << "Runtime error in InuSensor node: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception in InuSensor node: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception in InuSensor node" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    
    // Cleanup
    std::cout << "Shutting down ROS2..." << std::endl;
    rclcpp::shutdown();
    return 0;
}