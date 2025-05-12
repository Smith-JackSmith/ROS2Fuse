#ifndef TOPICFS_NODE_HPP
#define TOPICFS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <map>
#include <vector>
#include <set>
#include <mutex>

class topicfsNode : public rclcpp::Node
{
public:
  topicfsNode();

  std::unordered_map<std::string, std::string> latest_messages;
  std::unordered_map<std::string, uint64_t> message_versions_;
  std::mutex messages_mutex;
  std::map<std::string, std::string> topic_types_;
  std::map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;
  std::vector<std::string> writable_topics_;

  void subscribe_to_topic(const std::string& topic_name, const std::string& topic_type);
  std::vector<std::string> get_topics();

private:
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  void discover_topics();
};

#endif // TOPICFS_NODE_HPP