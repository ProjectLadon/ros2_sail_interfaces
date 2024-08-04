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
#include "sail_interfaces/srv/switch_pid_debug.hpp"

namespace sail_interfaces_mocks
{
    class SwitchPidDebugMockServer : public rclcpp::Node
    {
    public:
        using Switchsrv = sail_interfaces::srv::SwitchPidDebug;

        explicit SwitchPidDebugMockServer (
            const std::string &node_name, 
            const std::string &srv_name) 
            : Node (node_name, rclcpp::NodeOptions())
        {
            service = create_service<Switchsrv>(
                srv_name, std::bind(
                    &SwitchPidDebugMockServer::handle_srv, this, 
                    std::placeholders::_1, std::placeholders::_2
                )
            );
        }

        const std::string &get_label() { return label; }
        const uint8_t &get_id() { return id; }
        const bool get_debug() { return debug; }
        void set_success(bool s) { success = s; }

    private:
        rclcpp::Service<Switchsrv>::SharedPtr service;
        std::string label;
        uint8_t id;
        bool debug;
        bool success;

        void handle_srv(const std::shared_ptr<Switchsrv::Request> req,
            std::shared_ptr<Switchsrv::Response> res
        )
        {
            label = req->label;
            id = req->id;
            debug = req->debug;
            res->success = success;
        }
    };

    // class SwitchPidDebugMockClient : public rclcpp::Node
    // {
    // public:
    //     explicit SwitchPidDebugMockClient (
    //         const std::string &node_name, 
    //         const std::string &srv_name) 
    //         : Node (node_name, rclcpp::NodeOptions())
    //     {}

    // private:
    // };

} // namespace sail_interfaces_mocks