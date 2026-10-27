// Physics-backed ideal vacuum constraint; this does not model seal or adhesive forces.
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <rclcpp/rclcpp.hpp>
#include <sort_place_interfaces/msg/part_array.hpp>
#include <sort_place_interfaces/srv/suction.hpp>

namespace sort_place
{
using namespace std::chrono_literals;
using Suction = sort_place_interfaces::srv::Suction;

class CellVacuum final : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate,
                         public gz::sim::ISystemPostUpdate
{
  struct Request
  {
    std::string object;
    bool enable{}, done{}, success{};
    std::string message;
    std::chrono::steady_clock::time_point deadline;
  };

public:
  ~CellVacuum() override
  {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      stopping_ = true;
      cv_.notify_all();
    }
    if (executor_) executor_->cancel();
    if (thread_.joinable()) thread_.join();
    service_.reset();
    publisher_.reset();
    node_.reset();
    if (context_ && context_->is_valid()) context_->shutdown("Gazebo plugin unloaded");
  }

  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("cell_vacuum", rclcpp::NodeOptions().context(context_));
    publisher_ = node_->create_publisher<sort_place_interfaces::msg::PartArray>("/sort_place/parts", 10);
    service_ = node_->create_service<Suction>("/sort_place/suction",
      [this](const Suction::Request::SharedPtr input, Suction::Response::SharedPtr output)
      {
        std::unique_lock<std::mutex> guard(mutex_);
        auto request = std::make_shared<Request>();
        request->object = input->object_id;
        request->enable = input->enable;
        request->deadline = std::chrono::steady_clock::now() + 1s;
        pending_ = request;
        cv_.wait_until(guard, request->deadline,
                       [&] { return request->done || stopping_; });
        if (pending_ == request) pending_.reset();  // expired requests cannot execute later
        output->success = request->done && request->success;
        output->message = request->done ? request->message : "No simulation update before deadline";
      });
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
    executor_->add_node(node_);
    thread_ = std::thread([this] { executor_->spin(); });
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused) return;
    std::lock_guard<std::mutex> guard(mutex_);
    if (!pending_ || pending_->done) return;
    auto request = pending_;
    auto finish = [&](bool success, const std::string &message)
    {
      request->done = true;
      request->success = success;
      request->message = message;
      cv_.notify_all();
    };
    if (std::chrono::steady_clock::now() >= request->deadline)
    {
      finish(false, "Expired command");
      return;
    }
    auto robot = ecm.EntityByComponents(gz::sim::components::Model(),
                                       gz::sim::components::Name("scara"));
    if (robot == gz::sim::kNullEntity)
    {
      finish(false, "SCARA has not spawned");
      return;
    }
    auto tool = gz::sim::Model(robot).LinkByName(ecm, "nozzle_link");
    if (tool == gz::sim::kNullEntity)
    {
      finish(false, "nozzle_link not found");
      return;
    }
    if (!request->enable)
    {
      if (!held_.empty() && held_ != request->object)
      {
        finish(false, "Requested object is not the held object");
        return;
      }
      if (joint_ != gz::sim::kNullEntity) ecm.RequestRemoveEntity(joint_);
      joint_ = gz::sim::kNullEntity;
      held_.clear();
      finish(true, "Vacuum released");
      return;
    }
    if (!held_.empty())
    {
      finish(held_ == request->object, "Vacuum already occupied");
      return;
    }
    if (request->object.rfind("part_", 0) != 0)
    {
      finish(false, "Object name must start with part_");
      return;
    }
    auto object = ecm.EntityByComponents(gz::sim::components::Model(),
                                        gz::sim::components::Name(request->object));
    auto child = object == gz::sim::kNullEntity ? gz::sim::kNullEntity :
                 gz::sim::Model(object).LinkByName(ecm, "link");
    if (child == gz::sim::kNullEntity)
    {
      finish(false, "Part/link not found");
      return;
    }
    const auto tcp = gz::sim::worldPose(tool, ecm);
    const auto part = gz::sim::worldPose(child, ecm);
    const auto delta = tcp.Pos() - part.Pos();
    if (std::hypot(delta.X(), delta.Y()) > 0.004 || delta.Z() < 0.003 || delta.Z() > 0.010 ||
        std::abs(part.Rot().Roll()) > 0.10 || std::abs(part.Rot().Pitch()) > 0.10)
    {
      finish(false, "Part is outside the vacuum capture envelope");
      return;
    }
    joint_ = ecm.CreateEntity();
    ecm.CreateComponent(joint_, gz::sim::components::DetachableJoint({tool, child, "fixed"}));
    held_ = request->object;
    finish(true, "Vacuum constraint created");
  }

  void PostUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || info.simTime - last_publish_ < 50ms) return;
    last_publish_ = info.simTime;
    sort_place_interfaces::msg::PartArray message;
    message.header.frame_id = "world";
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    message.header.stamp.sec = static_cast<int32_t>(ns / 1000000000);
    message.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000);
    ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [&](const gz::sim::Entity &entity, const auto *, const auto *name)
      {
        const auto &id = name->Data();
        if (id.rfind("part_", 0) != 0) return true;
        auto link = gz::sim::Model(entity).LinkByName(ecm, "link");
        if (link == gz::sim::kNullEntity) return true;
        const auto pose = gz::sim::worldPose(link, ecm);
        sort_place_interfaces::msg::Part part;
        part.object_id = id;
        const auto end = id.find('_', 5);
        part.class_id = id.substr(5, end == std::string::npos ? end : end - 5);
        part.pose.position.x = pose.Pos().X();
        part.pose.position.y = pose.Pos().Y();
        part.pose.position.z = pose.Pos().Z();
        part.pose.orientation.x = pose.Rot().X();
        part.pose.orientation.y = pose.Rot().Y();
        part.pose.orientation.z = pose.Rot().Z();
        part.pose.orientation.w = pose.Rot().W();
        part.confidence = 1.0F;  // simulator ground truth, NOT an RGB detector
        part.attached = id == held_;
        message.parts.push_back(part);
        return true;
      });
    publisher_->publish(message);
  }

private:
  std::shared_ptr<rclcpp::Context> context_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Service<Suction>::SharedPtr service_;
  rclcpp::Publisher<sort_place_interfaces::msg::PartArray>::SharedPtr publisher_;
  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::shared_ptr<Request> pending_;
  bool stopping_{};
  std::string held_;
  gz::sim::Entity joint_{gz::sim::kNullEntity};
  std::chrono::steady_clock::duration last_publish_{};
};
}  // namespace sort_place

GZ_ADD_PLUGIN(sort_place::CellVacuum, gz::sim::System,
              sort_place::CellVacuum::ISystemConfigure,
              sort_place::CellVacuum::ISystemPreUpdate,
              sort_place::CellVacuum::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(sort_place::CellVacuum, "sort_place::CellVacuum")
