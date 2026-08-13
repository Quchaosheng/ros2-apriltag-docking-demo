#include "demo2_apriltag_docking_cpp/adapters/dock_spec_loader.hpp"
#include "demo2_apriltag_docking_cpp/core/task_policy.hpp"

#include <action_msgs/msg/goal_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav2_msgs/action/dock_robot.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace demo2_apriltag_docking_cpp::nodes {
namespace {

using DockRobot = nav2_msgs::action::DockRobot;
using GoalHandle = rclcpp_action::ClientGoalHandle<DockRobot>;
using DiagnosticValues = std::vector<std::pair<std::string, std::string>>;

std::string feedback_state_name(std::uint16_t value)
{
  switch (value) {
    case DockRobot::Feedback::NAV_TO_STAGING_POSE:
      return "NAV_TO_STAGING";
    case DockRobot::Feedback::INITIAL_PERCEPTION:
      return "INITIAL_PERCEPTION";
    case DockRobot::Feedback::CONTROLLING:
      return "CONTROLLING";
    case DockRobot::Feedback::WAIT_FOR_CHARGE:
      return "WAIT_FOR_CHARGE";
    case DockRobot::Feedback::RETRY:
      return "RETRY";
    default:
      return "UNKNOWN";
  }
}

diagnostic_msgs::msg::DiagnosticStatus make_status(
  const std::string & node_name,
  std::uint8_t level,
  const std::string & state,
  const DiagnosticValues & values)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = node_name;
  status.hardware_id = "demo2_apriltag_docking";
  status.level = level;
  status.message = state;
  status.values.reserve(values.size());
  for (const auto & [key, value] : values) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(std::move(item));
  }
  return status;
}

}  // namespace

class DockingTaskBridge : public rclcpp::Node {
public:
  DockingTaskBridge()
  : Node("docking_task_bridge")
  {
    declare_parameters();

    const std::string mapping_file = get_parameter("dock_mapping_file").as_string();
    if (mapping_file.empty()) {
      throw std::invalid_argument("dock_mapping_file must be set");
    }
    const auto specs = adapters::load_dock_specs(mapping_file);
    const int target_tag_id = static_cast<int>(get_parameter("target_tag_id").as_int());
    const auto dock = specs.find(target_tag_id);
    if (dock == specs.end()) {
      throw std::invalid_argument(
              "target_tag_id " + std::to_string(target_tag_id) + " is not mapped");
    }
    target_dock_ = dock->second;

    policy_ = std::make_unique<core::TaskPolicy>(
      get_parameter("guard_required").as_bool(),
      get_parameter("guard_timeout").as_double());
    max_staging_time_ = get_parameter("max_staging_time").as_double();
    navigate_to_staging_pose_ = get_parameter("navigate_to_staging_pose").as_bool();

    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      get_parameter("state_topic").as_string(), state_qos);
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
    guard_subscription_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("guard_topic").as_string(),
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile(),
      std::bind(&DockingTaskBridge::on_guard, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<DockRobot>(
      this, get_parameter("dock_action_name").as_string());
    start_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("start_service").as_string(),
      std::bind(
        &DockingTaskBridge::on_start,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("cancel_service").as_string(),
      std::bind(
        &DockingTaskBridge::on_cancel,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    guard_timer_ = rclcpp::create_timer(
      this,
      get_clock(),
      rclcpp::Duration::from_seconds(0.1),
      std::bind(&DockingTaskBridge::check_guard, this));
    publish_state("IDLE", diagnostic_msgs::msg::DiagnosticStatus::OK);
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("dock_mapping_file", "");
    declare_parameter<std::int64_t>("target_tag_id", 0);
    declare_parameter<std::string>("dock_action_name", "/dock_robot");
    declare_parameter<std::string>("start_service", "/demo2/start_docking");
    declare_parameter<std::string>("cancel_service", "/demo2/cancel_docking");
    declare_parameter<std::string>("guard_topic", "/guard/docking_allowed");
    declare_parameter<bool>("guard_required", false);
    declare_parameter<double>("guard_timeout", 2.0);
    declare_parameter<std::string>("state_topic", "/demo2/docking_state");
    declare_parameter<double>("max_staging_time", 60.0);
    declare_parameter<bool>("navigate_to_staging_pose", true);
  }

  void on_start(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    const auto [allowed, reason] = policy_->can_start(now_seconds());
    if (!allowed) {
      response->success = false;
      response->message = reason;
      publish_state(reason, diagnostic_msgs::msg::DiagnosticStatus::WARN);
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      response->success = false;
      response->message = "DOCK_ACTION_UNAVAILABLE";
      publish_state(response->message, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
      return;
    }

    DockRobot::Goal goal;
    goal.use_dock_id = true;
    goal.dock_id = target_dock_.dock_id;
    goal.max_staging_time = static_cast<float>(max_staging_time_);
    goal.navigate_to_staging_pose = navigate_to_staging_pose_;

    policy_->mark_active();
    const std::uint64_t generation = ++generation_;
    active_generation_ = generation;
    goal_handle_.reset();
    guard_cancel_reason_.reset();
    cancel_sent_ = false;

    typename rclcpp_action::Client<DockRobot>::SendGoalOptions options;
    options.goal_response_callback =
      [this, generation](const GoalHandle::SharedPtr & handle) {
        on_goal_response(generation, handle);
      };
    options.feedback_callback =
      [this, generation](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const DockRobot::Feedback> feedback) {
        on_feedback(generation, feedback);
      };
    options.result_callback =
      [this, generation](const GoalHandle::WrappedResult & result) {
        on_result(generation, result);
      };

    try {
      action_client_->async_send_goal(goal, options);
    } catch (const std::exception & error) {
      reset_action_state();
      response->success = false;
      response->message = "GOAL_RESPONSE_ERROR";
      publish_state(
        "FAILED",
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        {{"reason", "GOAL_RESPONSE_ERROR"}, {"error_msg", error.what()}});
      return;
    }

    response->success = true;
    response->message = "DOCKING_REQUESTED";
    publish_state(
      "WAITING_FOR_ACTION",
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      {{"dock_id", target_dock_.dock_id}});
  }

  void on_cancel(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!policy_->action_active() || !goal_handle_) {
      response->success = false;
      response->message = "NO_ACTIVE_DOCKING";
      return;
    }
    send_cancel_once(active_generation_);
    response->success = true;
    response->message = "CANCEL_REQUESTED";
  }

  void on_guard(const std_msgs::msg::Bool::ConstSharedPtr message)
  {
    policy_->update_guard(message->data, now_seconds());
    check_guard();
  }

  void check_guard()
  {
    const auto reason = policy_->cancel_reason(now_seconds());
    if (reason.has_value() && !guard_cancel_reason_.has_value()) {
      guard_cancel_reason_ = *reason;
      publish_state(*reason, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    }
    if (guard_cancel_reason_.has_value() && goal_handle_) {
      send_cancel_once(active_generation_);
    }
  }

  void send_cancel_once(std::uint64_t generation)
  {
    if (!is_current(generation) || !goal_handle_ || cancel_sent_) {
      return;
    }
    cancel_sent_ = true;
    try {
      action_client_->async_cancel_goal(goal_handle_);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      // The result callback owns the terminal state if the goal already finished.
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "failed to request docking cancellation: %s", error.what());
    }
  }

  void on_goal_response(std::uint64_t generation, const GoalHandle::SharedPtr & handle)
  {
    if (!is_current(generation)) {
      if (handle) {
        action_client_->stop_callbacks(handle);
      }
      return;
    }
    if (!handle) {
      finalize_failure("REJECTED");
      return;
    }
    goal_handle_ = handle;
    // Let rclcpp_action make the goal result-aware before the Guard timer cancels it.
  }

  void on_feedback(
    std::uint64_t generation,
    const std::shared_ptr<const DockRobot::Feedback> & feedback)
  {
    if (!is_current(generation)) {
      return;
    }
    publish_state(
      feedback_state_name(feedback->state),
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      {{"num_retries", std::to_string(feedback->num_retries)}});
  }

  void on_result(std::uint64_t generation, const GoalHandle::WrappedResult & wrapped)
  {
    if (!is_current(generation)) {
      return;
    }

    std::string state;
    std::uint8_t level;
    if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      state = guard_cancel_reason_.value_or("CANCELED");
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else if (
      wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result && wrapped.result->success)
    {
      state = "SUCCEEDED";
      level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else {
      state = "FAILED";
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    const auto result = wrapped.result;
    reset_action_state();
    publish_state(
      state,
      level,
      {
        {"error_code", result ? std::to_string(result->error_code) : "0"},
        {"error_msg", result ? result->error_msg : ""},
        {"num_retries", result ? std::to_string(result->num_retries) : "0"},
      });
  }

  void finalize_failure(const std::string & reason)
  {
    reset_action_state();
    publish_state(
      "FAILED",
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      {{"reason", reason}});
  }

  void reset_action_state()
  {
    policy_->mark_idle();
    goal_handle_.reset();
    guard_cancel_reason_.reset();
    cancel_sent_ = false;
    active_generation_ = 0;
  }

  bool is_current(std::uint64_t generation) const
  {
    return policy_->action_active() && generation == active_generation_;
  }

  void publish_state(
    const std::string & state,
    std::uint8_t level,
    const DiagnosticValues & values = {})
  {
    if (!last_state_.has_value() || *last_state_ != state) {
      std_msgs::msg::String message;
      message.data = state;
      state_publisher_->publish(message);
      last_state_ = state;
    }

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = get_clock()->now();
    diagnostics.status.push_back(make_status(get_name(), level, state, values));
    diagnostic_publisher_->publish(diagnostics);
  }

  double now_seconds() const
  {
    return get_clock()->now().seconds();
  }

  core::DockSpec target_dock_;
  std::unique_ptr<core::TaskPolicy> policy_;
  double max_staging_time_{60.0};
  bool navigate_to_staging_pose_{true};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr guard_subscription_;
  rclcpp_action::Client<DockRobot>::SharedPtr action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::TimerBase::SharedPtr guard_timer_;
  GoalHandle::SharedPtr goal_handle_;
  std::optional<std::string> guard_cancel_reason_;
  bool cancel_sent_{false};
  std::uint64_t generation_{0};
  std::uint64_t active_generation_{0};
  std::optional<std::string> last_state_;
};

}  // namespace demo2_apriltag_docking_cpp::nodes

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<demo2_apriltag_docking_cpp::nodes::DockingTaskBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("docking_task_bridge"), "%s", error.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
