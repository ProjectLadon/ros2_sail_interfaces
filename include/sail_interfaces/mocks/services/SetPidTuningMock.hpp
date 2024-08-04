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
#include "sail_interfaces/srv/set_pid_tuning.hpp"
#include "sail_interfaces/msg/pid_tuning.hpp"

namespace sail_interfaces_mocks
{
    class SetPidTuningMockServer : public rclcpp::Node
    {
    public:
        using SetPidTune = sail_interfaces::srv::SetPidTuning;

        explicit SetPidTuningMockServer (
            const std::string &node_name, 
            const std::string &srv_name) 
            : Node (node_name, rclcpp::NodeOptions())
        {
            service = create_service<SetPidTune>(
                srv_name, std::bind(
                    &SetPidTuningMockServer::handle_srv, this, 
                    std::placeholders::_1, std::placeholders::_2
                )
            );
        }

        const sail_interfaces::msg::PidTuning &get_tuning() { return tuning; }
        void set_success(bool s) { success = s; }

    private:
        rclcpp::Service<SetPidTune>::SharedPtr service;
        sail_interfaces::msg::PidTuning tuning;
        bool success;

        void handle_srv(const std::shared_ptr<SetPidTune::Request> req,
            std::shared_ptr<SetPidTune::Response> res
        )
        {
            tuning = req->new_tune;
            res->success = success;
        }
    };

    // class SetPidTuningMockClient : public rclcpp::Node
    // {
    // public:
    //     explicit SetPidTuningMockClient (
    //         const std::string &node_name, 
    //         const std::string &srv_name) 
    //         : Node (node_name, rclcpp::NodeOptions())
    //     {}

    // private:
    // };

} // namespace sail_interfaces_mocks