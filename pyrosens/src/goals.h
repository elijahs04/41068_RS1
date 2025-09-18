#ifndef GOALS_H
#define GOALS_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <thread>

class Goals: public rclcpp::Node {
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        Goals();
        ~Goals();

        bool sendGoalAndWait(double x, double y, double theta);
        void runGoals();

    private:
        std::thread runThread;
        
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};

#endif // GOALS_H