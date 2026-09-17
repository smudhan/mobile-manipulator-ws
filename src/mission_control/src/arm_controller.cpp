#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "mission_control/action/pick.hpp"
#include "mission_control/action/place.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node
{
public:

  using FollowJointTrajectory =
    control_msgs::action::FollowJointTrajectory;

  using Pick =
    mission_control::action::Pick;

  using Place =
    mission_control::action::Place;

  using GoalHandleTrajectory =
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  using GoalHandlePick =
    rclcpp_action::ServerGoalHandle<Pick>;

  using GoalHandlePlace =
    rclcpp_action::ServerGoalHandle<Place>;

  ArmController()
  : Node("arm_controller"),
    operation_active_(false)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Arm Controller started");

    // ----------------------------------------------------------
    // Joint trajectory controller
    // ----------------------------------------------------------

    trajectory_client_ =
      rclcpp_action::create_client<FollowJointTrajectory>(
        this,
        "/joint_trajectory_controller/follow_joint_trajectory");

    // ----------------------------------------------------------
    // PICK action server
    // ----------------------------------------------------------

    pick_server_ =
      rclcpp_action::create_server<Pick>(
        this,
        "/pick",

        std::bind(
          &ArmController::pick_goal_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2),

        std::bind(
          &ArmController::pick_cancel_callback,
          this,
          std::placeholders::_1),

        std::bind(
          &ArmController::pick_accepted_callback,
          this,
          std::placeholders::_1));

    // ----------------------------------------------------------
    // PLACE action server
    // ----------------------------------------------------------

    place_server_ =
      rclcpp_action::create_server<Place>(
        this,
        "/place",

        std::bind(
          &ArmController::place_goal_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2),

        std::bind(
          &ArmController::place_cancel_callback,
          this,
          std::placeholders::_1),

        std::bind(
          &ArmController::place_accepted_callback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Action servers ready: /pick and /place");
  }

private:

  // ============================================================
  // VERIFIED JOINT NAMES
  // ============================================================

  const std::vector<std::string> joint_names_ = {
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6"
  };


  // ============================================================
  // VERIFIED ARM POSES
  // ============================================================

  const std::vector<double> home_pose_ = {
    0.00,
    0.00,
    1.40,
    -1.40,
    0.00,
    0.00
  };

  const std::vector<double> pick_pose_ = {
    0.00,
    1.45,
    1.00,
    -1.40,
    0.00,
    0.00
  };

  const std::vector<double> place_pose_ = {
    0.00,
    1.45,
    1.00,
    -1.40,
    0.00,
    0.00
  };


  // ============================================================
  // PICK GOAL CALLBACK
  // ============================================================

  rclcpp_action::GoalResponse pick_goal_callback(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Pick::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received PICK request for object: %s",
      goal->object_id.c_str());

    if (operation_active_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Arm is busy");

      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  rclcpp_action::CancelResponse pick_cancel_callback(
    const std::shared_ptr<GoalHandlePick>)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "PICK cancellation requested");

    return rclcpp_action::CancelResponse::ACCEPT;
  }


  void pick_accepted_callback(
    const std::shared_ptr<GoalHandlePick> goal_handle)
  {
    std::thread(
      std::bind(
        &ArmController::execute_pick,
        this,
        std::placeholders::_1),
      goal_handle).detach();
  }


  // ============================================================
  // PLACE GOAL CALLBACK
  // ============================================================

  rclcpp_action::GoalResponse place_goal_callback(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Place::Goal> goal)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Received PLACE request for object: %s",
      goal->object_id.c_str());

    if (operation_active_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Arm is busy");

      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  rclcpp_action::CancelResponse place_cancel_callback(
    const std::shared_ptr<GoalHandlePlace>)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "PLACE cancellation requested");

    return rclcpp_action::CancelResponse::ACCEPT;
  }


  void place_accepted_callback(
    const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    std::thread(
      std::bind(
        &ArmController::execute_place,
        this,
        std::placeholders::_1),
      goal_handle).detach();
  }


  // ============================================================
  // MOVE ARM
  // ============================================================

  bool move_arm(
    const std::vector<double> & positions,
    const std::string & pose_name)
  {
    if (!trajectory_client_->action_server_is_ready())
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Joint trajectory controller unavailable");

      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Moving arm to %s pose",
      pose_name.c_str());

    FollowJointTrajectory::Goal goal;

    goal.trajectory.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint point;

    point.positions = positions;

    point.time_from_start =
      rclcpp::Duration::from_seconds(3.0);

    goal.trajectory.points.push_back(point);

    auto send_goal_future =
      trajectory_client_->async_send_goal(goal);

    // We cannot spin another executor here.
    // This function is executed from the action thread.
    //
    // The action client future is therefore polled while
    // allowing ROS callbacks to continue through the main
    // executor.

    while (rclcpp::ok())
    {
      auto status =
        send_goal_future.wait_for(100ms);

      if (status == std::future_status::ready)
      {
        break;
      }
    }

    if (!rclcpp::ok())
    {
      return false;
    }

    auto goal_handle =
      send_goal_future.get();

    if (!goal_handle)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Trajectory rejected for %s",
        pose_name.c_str());

      return false;
    }

    auto result_future =
      trajectory_client_->async_get_result(goal_handle);

    while (rclcpp::ok())
    {
      auto status =
        result_future.wait_for(100ms);

      if (status == std::future_status::ready)
      {
        break;
      }
    }

    if (!rclcpp::ok())
    {
      return false;
    }

    auto result =
      result_future.get();

    if (
      result.code ==
      rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Arm reached %s pose",
        pose_name.c_str());

      return true;
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "Arm failed to reach %s pose",
      pose_name.c_str());

    return false;
  }


  // ============================================================
  // EXECUTE PICK
  // ============================================================

  void execute_pick(
    const std::shared_ptr<GoalHandlePick> goal_handle)
  {
    operation_active_ = true;

    auto feedback =
      std::make_shared<Pick::Feedback>();

    auto result =
      std::make_shared<Pick::Result>();

    RCLCPP_INFO(
      this->get_logger(),
      "Starting PICK operation");

    // ----------------------------------------------------------
    // PICK POSE
    // ----------------------------------------------------------

    feedback->state = "MOVING_TO_PICK";
    feedback->progress = 0.25f;

    goal_handle->publish_feedback(feedback);

    if (!move_arm(pick_pose_, "PICK"))
    {
      result->success = false;
      result->message = "Failed to reach PICK pose";

      goal_handle->abort(result);

      operation_active_ = false;
      return;
    }

    // ----------------------------------------------------------
    // MOCK GRASP
    // ----------------------------------------------------------

    feedback->state = "GRASPING";
    feedback->progress = 0.50f;

    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(
      this->get_logger(),
      "Mock grasp executed");

    std::this_thread::sleep_for(1s);

    // ----------------------------------------------------------
    // HOME
    // ----------------------------------------------------------

    feedback->state = "RETURNING_HOME";
    feedback->progress = 0.75f;

    goal_handle->publish_feedback(feedback);

    if (!move_arm(home_pose_, "HOME"))
    {
      result->success = false;
      result->message = "Failed to return HOME after PICK";

      goal_handle->abort(result);

      operation_active_ = false;
      return;
    }

    // ----------------------------------------------------------
    // SUCCESS
    // ----------------------------------------------------------

    feedback->state = "COMPLETE";
    feedback->progress = 1.0f;

    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Pick completed successfully";

    goal_handle->succeed(result);

    operation_active_ = false;

    RCLCPP_INFO(
      this->get_logger(),
      "PICK operation completed");
  }


  // ============================================================
  // EXECUTE PLACE
  // ============================================================

  void execute_place(
    const std::shared_ptr<GoalHandlePlace> goal_handle)
  {
    operation_active_ = true;

    auto feedback =
      std::make_shared<Place::Feedback>();

    auto result =
      std::make_shared<Place::Result>();

    RCLCPP_INFO(
      this->get_logger(),
      "Starting PLACE operation");

    // ----------------------------------------------------------
    // PLACE POSE
    // ----------------------------------------------------------

    feedback->state = "MOVING_TO_PLACE";
    feedback->progress = 0.25f;

    goal_handle->publish_feedback(feedback);

    if (!move_arm(place_pose_, "PLACE"))
    {
      result->success = false;
      result->message = "Failed to reach PLACE pose";

      goal_handle->abort(result);

      operation_active_ = false;
      return;
    }

    // ----------------------------------------------------------
    // MOCK RELEASE
    // ----------------------------------------------------------

    feedback->state = "RELEASING";
    feedback->progress = 0.50f;

    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(
      this->get_logger(),
      "Mock release executed");

    std::this_thread::sleep_for(1s);

    // ----------------------------------------------------------
    // HOME
    // ----------------------------------------------------------

    feedback->state = "RETURNING_HOME";
    feedback->progress = 0.75f;

    goal_handle->publish_feedback(feedback);

    if (!move_arm(home_pose_, "HOME"))
    {
      result->success = false;
      result->message = "Failed to return HOME after PLACE";

      goal_handle->abort(result);

      operation_active_ = false;
      return;
    }

    // ----------------------------------------------------------
    // SUCCESS
    // ----------------------------------------------------------

    feedback->state = "COMPLETE";
    feedback->progress = 1.0f;

    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Place completed successfully";

    goal_handle->succeed(result);

    operation_active_ = false;

    RCLCPP_INFO(
      this->get_logger(),
      "PLACE operation completed");
  }


  // ============================================================
  // MEMBERS
  // ============================================================

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
    trajectory_client_;

  rclcpp_action::Server<Pick>::SharedPtr
    pick_server_;

  rclcpp_action::Server<Place>::SharedPtr
    place_server_;

  bool operation_active_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ArmController>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}