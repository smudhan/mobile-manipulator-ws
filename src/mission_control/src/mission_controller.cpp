#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/bool.hpp"

#include "mission_control/mission_state.hpp"
#include "mission_control/action/pick.hpp"
#include "mission_control/action/place.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class MissionController : public rclcpp::Node
{
public:

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using Pick = mission_control::action::Pick;
  using Place = mission_control::action::Place;

  using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

  using GoalHandlePick =
    rclcpp_action::ClientGoalHandle<Pick>;

  using GoalHandlePlace =
    rclcpp_action::ClientGoalHandle<Place>;

  MissionController()
  : Node("mission_controller"),
    current_state_(mission_control::MissionState::IDLE),
    place_goal_received_(false),
    navigation_active_(false),
    pick_active_(false),
    place_active_(false)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Mission Controller started");

    RCLCPP_INFO(
      this->get_logger(),
      "Initial state: IDLE");

    // ============================================================
    // RViz 2D Goal Pose subscriber
    // ============================================================

    goal_pose_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(
          &MissionController::goal_pose_callback,
          this,
          std::placeholders::_1));

    // ============================================================
    // Emergency Stop subscriber
    // ============================================================

    emergency_subscriber_ =
      this->create_subscription<std_msgs::msg::Bool>(
        "/emergency",
        10,
        std::bind(
          &MissionController::emergency_callback,
          this,
          std::placeholders::_1));

    // ============================================================
    // Emergency Stop velocity publisher
    // ============================================================

    cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);

    // ============================================================
    // Nav2 action client
    // ============================================================

    nav_client_ =
      rclcpp_action::create_client<NavigateToPose>(
        this,
        "navigate_to_pose");

    // ============================================================
    // Pick action client
    // ============================================================

    pick_client_ =
      rclcpp_action::create_client<Pick>(
        this,
        "pick");

    // ============================================================
    // Place action client
    // ============================================================

    place_client_ =
      rclcpp_action::create_client<Place>(
        this,
        "place");

    // ============================================================
    // FSM timer
    // ============================================================

    timer_ =
      this->create_wall_timer(
        500ms,
        std::bind(
          &MissionController::state_machine_callback,
          this));
  }

private:

  // ============================================================
  // RViz GOAL CALLBACK
  // ============================================================

  void goal_pose_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Do not accept new goals during emergency stop
    if (
      current_state_ ==
      mission_control::MissionState::EMERGENCY_STOP)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Goal ignored: EMERGENCY STOP is active");

      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received 2D Goal Pose: x=%.2f, y=%.2f",
      msg->pose.position.x,
      msg->pose.position.y);

    // ------------------------------------------------------------
    // First goal = pickup
    // ------------------------------------------------------------

    if (
      current_state_ ==
      mission_control::MissionState::IDLE)
    {
      pickup_pose_ = *msg;

      RCLCPP_INFO(
        this->get_logger(),
        "Using goal as PICKUP location");

      current_state_ =
        mission_control::MissionState::NAVIGATE_TO_PICKUP;

      return;
    }

    // ------------------------------------------------------------
    // Second goal = place
    //
    // This can arrive while the robot is:
    // NAVIGATE_TO_PICKUP
    // PICK
    // WAIT_FOR_PLACE_GOAL
    // ------------------------------------------------------------

    if (
      current_state_ ==
        mission_control::MissionState::NAVIGATE_TO_PICKUP ||
      current_state_ ==
        mission_control::MissionState::PICK ||
      current_state_ ==
        mission_control::MissionState::WAIT_FOR_PLACE_GOAL)
    {
      place_pose_ = *msg;
      place_goal_received_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Stored goal as PLACE location");

      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "Goal received but current state does not accept it");
  }

  // ============================================================
  // EMERGENCY STOP CALLBACK
  // ============================================================

  void emergency_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
  {
    // ------------------------------------------------------------
    // Emergency stop activated
    // ------------------------------------------------------------

    if (msg->data)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "EMERGENCY STOP ACTIVATED");

      // ----------------------------------------------------------
      // Stop mobile base
      // ----------------------------------------------------------

      geometry_msgs::msg::Twist stop_cmd;

      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.linear.z = 0.0;

      stop_cmd.angular.x = 0.0;
      stop_cmd.angular.y = 0.0;
      stop_cmd.angular.z = 0.0;

      cmd_vel_publisher_->publish(stop_cmd);

      // ----------------------------------------------------------
      // Cancel active Nav2 goal
      // ----------------------------------------------------------

      if (navigation_active_)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Canceling active Nav2 goal");

        nav_client_->async_cancel_all_goals();

        navigation_active_ = false;
      }

      // ----------------------------------------------------------
      // Enter emergency stop state
      // ----------------------------------------------------------

      current_state_ =
        mission_control::MissionState::EMERGENCY_STOP;

      RCLCPP_ERROR(
        this->get_logger(),
        "Robot is now in EMERGENCY_STOP state");
    }

    // ------------------------------------------------------------
    // Emergency stop reset
    // ------------------------------------------------------------

    else
    {
      if (
        current_state_ ==
        mission_control::MissionState::EMERGENCY_STOP)
      {
        RCLCPP_WARN(
          this->get_logger(),
          "Emergency stop reset");

        // Make sure the robot remains stopped
        geometry_msgs::msg::Twist stop_cmd;

        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.y = 0.0;
        stop_cmd.linear.z = 0.0;

        stop_cmd.angular.x = 0.0;
        stop_cmd.angular.y = 0.0;
        stop_cmd.angular.z = 0.0;

        cmd_vel_publisher_->publish(stop_cmd);

        // Reset mission state
        current_state_ =
          mission_control::MissionState::IDLE;

        place_goal_received_ = false;
        navigation_active_ = false;
        pick_active_ = false;
        place_active_ = false;

        pickup_pose_ =
          geometry_msgs::msg::PoseStamped();

        place_pose_ =
          geometry_msgs::msg::PoseStamped();

        RCLCPP_INFO(
          this->get_logger(),
          "Mission Controller reset to IDLE");
      }
    }
  }

  // ============================================================
  // NAV2
  // ============================================================

  void send_navigation_goal(
    const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!nav_client_->wait_for_action_server(2s))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Nav2 action server not available");

      navigation_active_ = false;

      current_state_ =
        mission_control::MissionState::RECOVERY;

      return;
    }

    NavigateToPose::Goal goal;
    goal.pose = pose;

    RCLCPP_INFO(
      this->get_logger(),
      "Sending navigation goal to Nav2");

    RCLCPP_INFO(
      this->get_logger(),
      "Goal position: x=%.2f, y=%.2f",
      pose.pose.position.x,
      pose.pose.position.y);

    navigation_active_ = true;

    auto options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    options.goal_response_callback =
      std::bind(
        &MissionController::navigation_goal_response_callback,
        this,
        std::placeholders::_1);

    options.result_callback =
      std::bind(
        &MissionController::navigation_result_callback,
        this,
        std::placeholders::_1);

    nav_client_->async_send_goal(
      goal,
      options);
  }

  void navigation_goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Nav2 rejected navigation goal");

      navigation_active_ = false;

      current_state_ =
        mission_control::MissionState::RECOVERY;

      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Nav2 accepted the navigation goal");
  }

  void navigation_result_callback(
    const GoalHandleNavigateToPose::WrappedResult & result)
  {
    navigation_active_ = false;

    // If emergency stop is active, don't change state
    if (
      current_state_ ==
      mission_control::MissionState::EMERGENCY_STOP)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Navigation result ignored: EMERGENCY STOP is active");

      return;
    }

    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:

        RCLCPP_INFO(
          this->get_logger(),
          "Navigation completed successfully");

        if (
          current_state_ ==
          mission_control::MissionState::NAVIGATE_TO_PICKUP)
        {
          current_state_ =
            mission_control::MissionState::PICK;

          RCLCPP_INFO(
            this->get_logger(),
            "Transition: NAVIGATE_TO_PICKUP -> PICK");
        }
        else if (
          current_state_ ==
          mission_control::MissionState::NAVIGATE_TO_PLACE)
        {
          current_state_ =
            mission_control::MissionState::PLACE;

          RCLCPP_INFO(
            this->get_logger(),
            "Transition: NAVIGATE_TO_PLACE -> PLACE");
        }

        break;

      case rclcpp_action::ResultCode::ABORTED:

        RCLCPP_ERROR(
          this->get_logger(),
          "Navigation aborted");

        current_state_ =
          mission_control::MissionState::RECOVERY;

        break;

      case rclcpp_action::ResultCode::CANCELED:

        RCLCPP_WARN(
          this->get_logger(),
          "Navigation canceled");

        current_state_ =
          mission_control::MissionState::RECOVERY;

        break;

      default:

        RCLCPP_ERROR(
          this->get_logger(),
          "Unknown navigation result");

        current_state_ =
          mission_control::MissionState::RECOVERY;

        break;
    }
  }

  // ============================================================
  // PICK ACTION CLIENT
  // ============================================================

  void send_pick_goal()
  {
    if (!pick_client_->wait_for_action_server(2s))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Pick action server not available");

      pick_active_ = false;

      current_state_ =
        mission_control::MissionState::RECOVERY;

      return;
    }

    Pick::Goal goal;

    goal.object_id = "mission_object";

    RCLCPP_INFO(
      this->get_logger(),
      "Sending PICK action to arm controller");

    pick_active_ = true;

    auto options =
      rclcpp_action::Client<Pick>::SendGoalOptions();

    options.goal_response_callback =
      std::bind(
        &MissionController::pick_goal_response_callback,
        this,
        std::placeholders::_1);

    options.feedback_callback =
      std::bind(
        &MissionController::pick_feedback_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2);

    options.result_callback =
      std::bind(
        &MissionController::pick_result_callback,
        this,
        std::placeholders::_1);

    pick_client_->async_send_goal(
      goal,
      options);
  }

  void pick_goal_response_callback(
    const GoalHandlePick::SharedPtr & goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Pick goal rejected");

      pick_active_ = false;

      current_state_ =
        mission_control::MissionState::RECOVERY;

      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Pick goal accepted by arm controller");
  }

  void pick_feedback_callback(
    GoalHandlePick::SharedPtr,
    const std::shared_ptr<const Pick::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "PICK feedback: %s (%.0f%%)",
      feedback->state.c_str(),
      feedback->progress * 100.0f);
  }

  void pick_result_callback(
    const GoalHandlePick::WrappedResult & result)
  {
    pick_active_ = false;

    // Ignore result if emergency stop is active
    if (
      current_state_ ==
      mission_control::MissionState::EMERGENCY_STOP)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Pick result ignored: EMERGENCY STOP is active");

      return;
    }

    if (
      result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result->success)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Pick completed successfully");

      RCLCPP_INFO(
        this->get_logger(),
        "Transition: PICK -> WAIT_FOR_PLACE_GOAL");

      current_state_ =
        mission_control::MissionState::WAIT_FOR_PLACE_GOAL;
    }
    else
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Pick operation failed: %s",
        result.result->message.c_str());

      current_state_ =
        mission_control::MissionState::RECOVERY;
    }
  }

  // ============================================================
  // PLACE ACTION CLIENT
  // ============================================================

  void send_place_goal()
  {
    if (!place_client_->wait_for_action_server(2s))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Place action server not available");

      place_active_ = false;

      current_state_ =
        mission_control::MissionState::RECOVERY;

      return;
    }

    Place::Goal goal;

    goal.object_id = "mission_object";

    RCLCPP_INFO(
      this->get_logger(),
      "Sending PLACE action to arm controller");

    place_active_ = true;

    auto options =
      rclcpp_action::Client<Place>::SendGoalOptions();

    options.goal_response_callback =
      std::bind(
        &MissionController::place_goal_response_callback,
        this,
        std::placeholders::_1);

    options.feedback_callback =
      std::bind(
        &MissionController::place_feedback_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2);

    options.result_callback =
      std::bind(
        &MissionController::place_result_callback,
        this,
        std::placeholders::_1);

    place_client_->async_send_goal(
      goal,
      options);
  }

  void place_goal_response_callback(
    const GoalHandlePlace::SharedPtr & goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Place goal rejected");

      place_active_ = false;

      current_state_ =
        mission_control::MissionState::RECOVERY;

      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Place goal accepted by arm controller");
  }

  void place_feedback_callback(
    GoalHandlePlace::SharedPtr,
    const std::shared_ptr<const Place::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "PLACE feedback: %s (%.0f%%)",
      feedback->state.c_str(),
      feedback->progress * 100.0f);
  }

  void place_result_callback(
    const GoalHandlePlace::WrappedResult & result)
  {
    place_active_ = false;

    // Ignore result if emergency stop is active
    if (
      current_state_ ==
      mission_control::MissionState::EMERGENCY_STOP)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Place result ignored: EMERGENCY STOP is active");

      return;
    }

    if (
      result.code == rclcpp_action::ResultCode::SUCCEEDED &&
      result.result->success)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Place completed successfully");

      current_state_ =
        mission_control::MissionState::MISSION_COMPLETE;
    }
    else
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Place operation failed: %s",
        result.result->message.c_str());

      current_state_ =
        mission_control::MissionState::RECOVERY;
    }
  }

  // ============================================================
  // STATE MACHINE
  // ============================================================

  void state_machine_callback()
  {
    switch (current_state_)
    {
      // ----------------------------------------------------------
      // IDLE
      // ----------------------------------------------------------

      case mission_control::MissionState::IDLE:

        break;

      // ----------------------------------------------------------
      // NAVIGATE TO PICKUP
      // ----------------------------------------------------------

      case mission_control::MissionState::NAVIGATE_TO_PICKUP:

        if (!navigation_active_)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "State: NAVIGATE_TO_PICKUP");

          send_navigation_goal(pickup_pose_);
        }

        break;

      // ----------------------------------------------------------
      // PICK
      // ----------------------------------------------------------

      case mission_control::MissionState::PICK:

        if (!pick_active_)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "State: PICK");

          send_pick_goal();
        }

        break;

      // ----------------------------------------------------------
      // WAIT FOR PLACE GOAL
      // ----------------------------------------------------------

      case mission_control::MissionState::WAIT_FOR_PLACE_GOAL:

        if (place_goal_received_)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "Place goal received");

          place_goal_received_ = false;

          current_state_ =
            mission_control::MissionState::NAVIGATE_TO_PLACE;
        }

        break;

      // ----------------------------------------------------------
      // NAVIGATE TO PLACE
      // ----------------------------------------------------------

      case mission_control::MissionState::NAVIGATE_TO_PLACE:

        if (!navigation_active_)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "State: NAVIGATE_TO_PLACE");

          send_navigation_goal(place_pose_);
        }

        break;

      // ----------------------------------------------------------
      // PLACE
      // ----------------------------------------------------------

      case mission_control::MissionState::PLACE:

        if (!place_active_)
        {
          RCLCPP_INFO(
            this->get_logger(),
            "State: PLACE");

          send_place_goal();
        }

        break;

      // ----------------------------------------------------------
      // MISSION COMPLETE
      // ----------------------------------------------------------

      case mission_control::MissionState::MISSION_COMPLETE:

        RCLCPP_INFO(
          this->get_logger(),
          "Mission complete -> IDLE");

        current_state_ =
          mission_control::MissionState::IDLE;

        pickup_pose_ =
          geometry_msgs::msg::PoseStamped();

        place_pose_ =
          geometry_msgs::msg::PoseStamped();

        place_goal_received_ = false;

        break;

      // ----------------------------------------------------------
      // RECOVERY
      // ----------------------------------------------------------

      case mission_control::MissionState::RECOVERY:

        RCLCPP_WARN(
          this->get_logger(),
          "State: RECOVERY");

        break;

      // ----------------------------------------------------------
      // EMERGENCY STOP
      // ----------------------------------------------------------

      case mission_control::MissionState::EMERGENCY_STOP:

        RCLCPP_ERROR(
          this->get_logger(),
          "State: EMERGENCY_STOP");

        // Keep publishing zero velocity while emergency stop
        // remains active.
        {
          geometry_msgs::msg::Twist stop_cmd;

          stop_cmd.linear.x = 0.0;
          stop_cmd.linear.y = 0.0;
          stop_cmd.linear.z = 0.0;

          stop_cmd.angular.x = 0.0;
          stop_cmd.angular.y = 0.0;
          stop_cmd.angular.z = 0.0;

          cmd_vel_publisher_->publish(stop_cmd);
        }

        break;
    }
  }

  // ============================================================
  // MEMBERS
  // ============================================================

  mission_control::MissionState current_state_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ------------------------------------------------------------
  // Subscribers / Publishers
  // ------------------------------------------------------------

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_pose_subscriber_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    emergency_subscriber_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_publisher_;

  // ------------------------------------------------------------
  // Nav2
  // ------------------------------------------------------------

  rclcpp_action::Client<NavigateToPose>::SharedPtr
    nav_client_;

  // ------------------------------------------------------------
  // Manipulation
  // ------------------------------------------------------------

  rclcpp_action::Client<Pick>::SharedPtr
    pick_client_;

  rclcpp_action::Client<Place>::SharedPtr
    place_client_;

  // ------------------------------------------------------------
  // Goals
  // ------------------------------------------------------------

  geometry_msgs::msg::PoseStamped pickup_pose_;
  geometry_msgs::msg::PoseStamped place_pose_;

  // ------------------------------------------------------------
  // State flags
  // ------------------------------------------------------------

  bool place_goal_received_;
  bool navigation_active_;
  bool pick_active_;
  bool place_active_;
};


// ================================================================
// MAIN
// ================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<MissionController>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}