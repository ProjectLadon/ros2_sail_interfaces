/**
 * 
 * (c) Ladon Robotics 2024
 * 
 * All rights reserved
 * 
*/

#pragma once

#include <string>
#include <memory>
#include <functional>
#include <chrono>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "sail_interfaces/srv/get_pid_tuning.hpp"
#include "sail_interfaces/msg/pid_tuning.hpp"

namespace sail_interfaces_mocks
{
    class GetPidTuningMockServer : public rclcpp::Node
    {
    public:
        using GetPidTune = sail_interfaces::srv::GetPidTuning;

        explicit GetPidTuningMockServer (
            const std::string &node_name, 
            const std::string &srv_name) 
            : Node (node_name, rclcpp::NodeOptions())
        {
            service = create_service<GetPidTune>(
                srv_name, std::bind(
                    &GetPidTuningMockServer::handle_srv, this, 
                    std::placeholders::_1, std::placeholders::_2
                )
            );
        }

        void set_tuning(const sail_interfaces::msg::PidTuning &tune) { tuning = tune; }

    private:
        rclcpp::Service<GetPidTune>::SharedPtr service;
        sail_interfaces::msg::PidTuning tuning;

        void handle_srv(const std::shared_ptr<GetPidTune::Request> req,
            std::shared_ptr<GetPidTune::Response> res
        )
        {
            res->current_tune = tuning;
        }
    };

    // class GetPidTuningMockClient : public rclcpp::Node
    // {
    // public:
    //     explicit GetPidTuningMockClient (
    //         const std::string &node_name, 
    //         const std::string &srv_name) 
    //         : Node (node_name, rclcpp::NodeOptions())
    //     {}

    // private:
    // };

} // namespace sail_interfaces_mocks