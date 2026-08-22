#include "demo2_apriltag_docking/tag_policy.hpp"
#include "demo2_apriltag_docking/task_policy.hpp"

#include <chrono>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <string>
#include <stdexcept>
#include <unordered_map>

#include <action_msgs/msg/goal_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav2_msgs/action/dock_robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace demo2_apriltag_docking
{

class DockingTaskBridge : public rclcpp::Node
{
  using DockRobot = nav2_msgs::action::DockRobot;
  using GoalHandle = rclcpp_action::ClientGoalHandle<DockRobot>;

public:
  DockingTaskBridge()
  : Node("docking_task_bridge")
  {
    declare_parameter<std::string>("dock_mapping_file", "");
    declare_parameter<int>("target_tag_id", 0);
    declare_parameter<std::string>("dock_action_name", "/dock_robot");
    declare_parameter<std::string>("start_service", "/demo2/start_docking");
    declare_parameter<std::string>("cancel_service", "/demo2/cancel_docking");
    declare_parameter<std::string>("guard_topic", "/guard/docking_allowed");
    declare_parameter<bool>("guard_required", false);
    declare_parameter<double>("guard_timeout", 2.0);
    declare_parameter<std::string>("state_topic", "/demo2/docking_state");
    declare_parameter<double>("max_staging_time", 60.0);
    declare_parameter<bool>("navigate_to_staging_pose", true);

    const auto mapping_file = get_parameter("dock_mapping_file").as_string();
    if (mapping_file.empty()) {
      throw std::invalid_argument("dock_mapping_file must be set");
    }
    specs_ = load_dock_specs(mapping_file);
    const int target_tag_id = get_parameter("target_tag_id").as_int();
    const auto dock_it = specs_.find(target_tag_id);
    if (dock_it == specs_.end()) {
      throw std::invalid_argument("target_tag_id is not mapped");
    }
    target_dock_ = dock_it->second;
    policy_ = std::make_unique<TaskPolicy>(
      get_parameter("guard_required").as_bool(), get_parameter("guard_timeout").as_double());

    rclcpp::QoS state_qos(rclcpp::KeepLast(1));
    state_qos.reliable().transient_local();
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      get_parameter("state_topic").as_string(), state_qos);
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    rclcpp::QoS guard_qos(rclcpp::KeepLast(1));
    guard_qos.reliable().durability_volatile();
    guard_subscription_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("guard_topic").as_string(), guard_qos,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        policy_->update_guard(message->data, now_seconds());
        check_guard();
      });
    action_client_ = rclcpp_action::create_client<DockRobot>(
      this, get_parameter("dock_action_name").as_string());
    start_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("start_service").as_string(),
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        on_start(response);
      });
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("cancel_service").as_string(),
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        on_cancel(response);
      });
    guard_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {check_guard();});
    publish_state("IDLE", diagnostic_msgs::msg::DiagnosticStatus::OK);
  }

private:
  static std::string feedback_state_name(const std::uint8_t state)
  {
    switch (state) {
      case 1: return "NAV_TO_STAGING";
      case 2: return "INITIAL_PERCEPTION";
      case 3: return "CONTROLLING";
      case 4: return "WAIT_FOR_CHARGE";
      case 5: return "RETRY";
      default: return "UNKNOWN";
    }
  }

  double now_seconds() const {return get_clock()->now().seconds();}

  void on_start(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    const auto decision = policy_->can_start(now_seconds());
    if (!decision.first) {
      response->success = false;
      response->message = decision.second;
      publish_state(decision.second, diagnostic_msgs::msg::DiagnosticStatus::WARN);
      return;
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      response->success = false;
      response->message = "DOCK_ACTION_UNAVAILABLE";
      publish_state(response->message, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      return;
    }

    DockRobot::Goal goal;
    goal.use_dock_id = true;
    goal.dock_id = target_dock_.dock_id;
    goal.max_staging_time = static_cast<float>(get_parameter("max_staging_time").as_double());
    goal.navigate_to_staging_pose = get_parameter("navigate_to_staging_pose").as_bool();

    rclcpp_action::Client<DockRobot>::SendGoalOptions options;
    options.goal_response_callback =
      [this](GoalHandle::SharedPtr goal_handle) {on_goal_response(goal_handle);};
    options.feedback_callback =
      [this](GoalHandle::SharedPtr, const std::shared_ptr<const DockRobot::Feedback> feedback) {
        publish_state(feedback_state_name(feedback->state), diagnostic_msgs::msg::DiagnosticStatus::OK);
      };
    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {on_result(result);};
    policy_->mark_active();
    guard_cancel_reason_.reset();
    action_client_->async_send_goal(goal, options);
    response->success = true;
    response->message = "DOCKING_REQUESTED";
    publish_state("WAITING_FOR_ACTION", diagnostic_msgs::msg::DiagnosticStatus::OK);
  }

  void on_cancel(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (!policy_->action_active() || !goal_handle_) {
      response->success = false;
      response->message = "NO_ACTIVE_DOCKING";
      return;
    }
    action_client_->async_cancel_goal(goal_handle_);
    response->success = true;
    response->message = "CANCEL_REQUESTED";
  }

  void check_guard()
  {
    const auto reason = policy_->cancel_reason(now_seconds());
    if (reason.has_value() && goal_handle_ && !guard_cancel_reason_.has_value()) {
      guard_cancel_reason_ = *reason;
      action_client_->async_cancel_goal(goal_handle_);
      publish_state(*reason, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    }
  }

  void on_goal_response(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      policy_->mark_idle();
      goal_handle_.reset();
      publish_state("FAILED", diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      return;
    }
    goal_handle_ = goal_handle;
    check_guard();
  }

  void on_result(const GoalHandle::WrappedResult & wrapped)
  {
    std::string state;
    std::uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      state = guard_cancel_reason_.value_or("CANCELED");
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result->success) {
      state = "SUCCEEDED";
      level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else {
      state = "FAILED";
    }
    policy_->mark_idle();
    goal_handle_.reset();
    publish_state(state, level);
  }

  void publish_state(const std::string & state, const std::uint8_t level)
  {
    if (state != last_state_) {
      std_msgs::msg::String message;
      message.data = state;
      state_publisher_->publish(message);
      last_state_ = state;
    }
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now().to_msg();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_name();
    status.hardware_id = "demo2_apriltag_docking";
    status.level = level;
    status.message = state;
    diagnostics.status.push_back(status);
    diagnostic_publisher_->publish(diagnostics);
  }

  std::unordered_map<int, DockSpec> specs_;
  DockSpec target_dock_{};
  std::unique_ptr<TaskPolicy> policy_;
  rclcpp_action::Client<DockRobot>::SharedPtr action_client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr guard_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr guard_timer_;
  std::optional<std::string> guard_cancel_reason_;
  std::string last_state_;
};

}  // namespace demo2_apriltag_docking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<demo2_apriltag_docking::DockingTaskBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    fprintf(stderr, "docking_task_bridge: %s\n", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
