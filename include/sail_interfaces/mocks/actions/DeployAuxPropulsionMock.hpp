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
#include "rclcpp_action/rclcpp_action.hpp"
#include "sail_interfaces/action/deploy_aux_propulsion.hpp"
#include "std_msgs/msg/header.hpp"

namespace sail_interfaces_mocks
{
    class DeployAuxPropulsionMockServer : public rclcpp::Node
    {
    public:
        using Deploy = sail_interfaces::action::DeployAuxPropulsion;
        using GoalHandleDeploy = rclcpp_action::ServerGoalHandle<Deploy>;

        explicit  DeployAuxPropulsionMockServer (
            const std::string &node_name, 
            const std::string &action_name) 
            : Node (node_name, rclcpp::NodeOptions())
        {
            actionServer = rclcpp_action::create_server<Deploy>(
                this, action_name, 
                std::bind(&DeployAuxPropulsionMockServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2), 
                std::bind(&DeployAuxPropulsionMockServer::handle_cancel, this, std::placeholders::_1), 
                std::bind(&DeployAuxPropulsionMockServer::handle_accepted, this, std::placeholders::_1)
            );
            isAccept = true;
            isActive = false;
        }

        bool is_active() { return isActive; }
        bool is_accepting() { return isAccept; }
        bool is_defering() { return isDefer; }
        bool was_canceled() { return wasCanceled; }
        void set_accept(bool accept) { isAccept = accept; }
        void set_defer(bool defer) { isDefer = defer; }

        std::shared_ptr<GoalHandleDeploy> get_handle() { return goalHandle; }

        bool generate_feedback(
            const std::shared_ptr<GoalHandleDeploy> handle,
            const float &sec_to_finish,
            const float &sec_from_start,
            const float &fraction_complete,
            const std_msgs::msg::Header header
        ) 
        {
            if (!isActive or !handle) { return false; }
            auto feedback = std::make_shared<Deploy::Feedback>();
            feedback->sec_to_finish = sec_to_finish;
            feedback->sec_from_start = sec_from_start;
            feedback->fraction_complete = fraction_complete;
            feedback->header = header;
            handle->publish_feedback(feedback);
            return true;
        }

        bool end_goal(
            const std::shared_ptr<GoalHandleDeploy> handle,
            const rclcpp_action::ResultCode result_code,
            const bool &retracted,
            const bool &extended,
            const std_msgs::msg::Header header
        ) 
        {
            if (!isActive or !handle) { return false; }
            auto result = std::make_shared<Deploy::Result>();
            result->retracted = retracted;
            result->extended = extended;
            result->header = header;
            switch (result_code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    handle->succeed(result);
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    handle->abort(result);
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    handle->canceled(result);
                    break;
                case rclcpp_action::ResultCode::UNKNOWN:
                    return false; 
                default:
                    return false;
            }
            isActive = false;
            return true;
        }

    private:
        rclcpp_action::Server<Deploy>::SharedPtr actionServer;
        std::shared_ptr<GoalHandleDeploy> goalHandle;
        bool isAccept;
        bool isActive;
        bool isDefer;
        bool wasCanceled;

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const Deploy::Goal> goal
        )
        {
            if (isAccept) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
            else if (isDefer) { return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER; }
            else { return rclcpp_action::GoalResponse::REJECT; }
        }

        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleDeploy> handle
        )
        {
            isActive = false;
            wasCanceled = true;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleDeploy> handle)
        {
            wasCanceled = false;
            goalHandle = handle;
            isActive = true;
            isAccept = false;
        }

    };
} // namespace sail_interfaces_mocks

