#include "rclcpp/rclcpp.hpp"
#include <hardware_interface/resource_manager.hpp>
#include <controller_manager/controller_manager.hpp>
#include "hoverboard.h"

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);

    // auto node = rclcpp::Node::make_shared("hoverboard_driver");

    std::string controller_manager_node_name = "hoverboard_manager";

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto cm = std::make_shared<controller_manager::ControllerManager>(executor, controller_manager_node_name);

    // NOTE: Threading model could be improved.
    std::thread cm_thread([cm]() {

        auto const period = std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());
        auto const cm_now = std::chrono::nanoseconds(cm->now().nanoseconds());
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time{cm_now};

        rclcpp::Time previous_time = cm->now();

        while(rclcpp::ok()){

            auto const current_time = cm->now();
            auto const measured_period = current_time - previous_time;
            previous_time = current_time;

            cm->read(cm->now(), measured_period);
            cm->update(cm->now(), measured_period);
            cm->write(cm->now(), measured_period);

            next_iteration_time += period;

            // std::this_thread::sleep_for(std::chrono::milliseconds(20));
            std::this_thread::sleep_until(next_iteration_time);
        }
    });

    executor->add_node(cm);
    executor->spin();

    rclcpp::shutdown();

    return 0;
}
