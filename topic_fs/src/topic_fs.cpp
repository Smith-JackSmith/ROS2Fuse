// Copyright 2025 Jack Sidman Smith
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <errno.h>
#define FUSE_USE_VERSION 29  // TODO(Jack): Update to latest FUSE version
#include <fuse.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

// For base64 encoding
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>

// Simple base64 encoding function (replace with a library if preferred)
std::string base64_encode(const uint8_t* data, size_t length)
{
  static const std::string base64_chars = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve((length + 2) / 3 * 4);

  for (size_t i = 0; i < length; i += 3)
  {
    uint32_t b = (data[i] << 16) | ((i + 1 < length ? data[i + 1] : 0) << 8) | (i + 2 < length ? data[i + 2] : 0);
    encoded.push_back(base64_chars[(b >> 18) & 0x3F]);
    encoded.push_back(base64_chars[(b >> 12) & 0x3F]);
    encoded.push_back(i + 1 < length ? base64_chars[(b >> 6) & 0x3F] : '=');
    encoded.push_back(i + 2 < length ? base64_chars[b & 0x3F] : '=');
  }

  return encoded;
}

class topicfsNode : public rclcpp::Node
{
public:
  topicfsNode()
      : Node("ros2_fuse_node")
  {
    RCLCPP_INFO(this->get_logger(), "TopicFS Utility - Created by Jack Sidman Smith");
    discover_topics();
  }

  std::unordered_map<std::string, std::string> latest_messages;
  std::mutex messages_mutex;
  std::map<std::string, std::string> topic_types_;

  void subscribe_to_topic(const std::string& topic_name, const std::string& topic_type)
{
  if (subscriptions_.count(topic_name))
  {
    RCLCPP_WARN(this->get_logger(), "Already subscribed to topic: %s", topic_name.c_str());
    return;
  }

  try
  {
    // Use reliable QoS with keep-last-10 to match common ROS 2 publishers
    rclcpp::QoS qos(10);
    // qos.reliable();
    qos.durability_volatile();
    auto sub = create_generic_subscription(
        topic_name, topic_type, qos,
        [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
          RCLCPP_DEBUG(this->get_logger(), "Received message on topic: %s", topic_name.c_str());
          const auto& buffer = serialized_msg->get_rcl_serialized_message();
          if (buffer.buffer_length == 0)
          {
            RCLCPP_WARN(this->get_logger(), "Received empty message on topic: %s", topic_name.c_str());
            return;
          }
          std::string encoded = base64_encode(buffer.buffer, buffer.buffer_length);
          nlohmann::json j;
          j["data"] = encoded;
          std::lock_guard<std::mutex> lock(messages_mutex);
          latest_messages[topic_name] = j.dump();
          RCLCPP_DEBUG(this->get_logger(), "Stored message for %s: %s", topic_name.c_str(), j.dump().c_str());
        });
    if(!sub || sub->get_subscription_handle() == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create subscription for topic: %s (%s)", topic_name.c_str(), topic_type.c_str());
      return;
    }
    subscriptions_[topic_name] = sub;
    topic_types_[topic_name] = topic_type;
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s (%s) with QoS reliable, keep-last-10", topic_name.c_str(), topic_type.c_str());

    // Log subscription status
    auto sub_info = sub->get_topic_name();
    RCLCPP_INFO(this->get_logger(), "Subscription active for %s", sub_info);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to topic %s (%s): %s", topic_name.c_str(), topic_type.c_str(), e.what());
  }
}

  std::vector<std::string> get_topics()
  {
    std::lock_guard<std::mutex> lock(messages_mutex);
    std::vector<std::string> topics;
    for (const auto& pair : topic_types_)
    {
      std::string topic = pair.first;
      if (!topic.empty() && topic[0] == '/')
      {
        topic = topic.substr(1); // Remove leading slash for FUSE paths
      }
      topics.push_back(topic);
    }
    return topics;
  }

private:
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;

  void discover_topics()
  {
    auto topic_names_and_types = get_topic_names_and_types();
    std::set<std::string> seen_topics; // Track unique topics
    for (const auto& [topic, types] : topic_names_and_types)
    {
      if (!types.empty() && seen_topics.insert(topic).second)
      {
        RCLCPP_INFO(this->get_logger(), "Found topic: %s, type: %s", topic.c_str(), types[0].c_str());
        subscribe_to_topic(topic, types[0]);
      }
    }
  }
};

// Global ROS2 node
std::shared_ptr<topicfsNode> ros2_node;

// FUSE operations
static int topicfs_getattr(const char* path, struct stat* stbuf)
{
  memset(stbuf, 0, sizeof(struct stat));
  std::string spath(path);

  if (spath == "/")
  {
    stbuf->st_mode = S_IFDIR | 0755;
    stbuf->st_nlink = 2;
    return 0;
  }

  auto topics = ros2_node->get_topics();
  std::string path_no_slash = spath.substr(1); // Remove leading /

  // Check if path is a topic or a parent directory
  bool is_topic = false;
  for (const auto& topic : topics)
  {
    if (path_no_slash == topic)
    {
      is_topic = true;
      break;
    }
    // Check if path is a prefix of a topic (parent directory)
    if (topic.compare(0, path_no_slash.length(), path_no_slash) == 0 &&
        (topic.length() == path_no_slash.length() || topic[path_no_slash.length()] == '/'))
    {
      stbuf->st_mode = S_IFDIR | 0755;
      stbuf->st_nlink = 2;
      return 0;
    }
  }

  // Handle topic directory
  if (is_topic)
  {
    stbuf->st_mode = S_IFDIR | 0755;
    stbuf->st_nlink = 2;
    return 0;
  }

  // Handle files under topic (latest, info)
  size_t pos = spath.rfind('/');
  if (pos != std::string::npos && pos > 0)
  {
    std::string topic = spath.substr(1, pos - 1);
    std::string file = spath.substr(pos + 1);
    std::string original_topic = "/" + topic;
    if (std::find(topics.begin(), topics.end(), topic) != topics.end())
    {
      if (file == "latest" || file == "info")
      {
        stbuf->st_mode = S_IFREG | 0444;
        stbuf->st_nlink = 1;
        std::lock_guard<std::mutex> lock(ros2_node->messages_mutex);
        if (file == "latest" && ros2_node->latest_messages.count(original_topic))
        {
          stbuf->st_size = ros2_node->latest_messages[original_topic].size();
        }
        else if (file == "info")
        {
          std::string type = ros2_node->topic_types_.count(original_topic) ? ros2_node->topic_types_[original_topic] : "unknown";
          std::string info = "Topic: " + original_topic + "\nType: " + type + "\n";
          stbuf->st_size = info.size();
        }
        return 0;
      }
    }
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "getattr: path %s not found", spath.c_str());
  return -ENOENT;
}

static int topicfs_readdir(const char* path, void* buf, fuse_fill_dir_t filler, off_t offset, struct fuse_file_info* fi)
{
  (void)fi;
  (void)offset;

  if (!ros2_node)
  {
    std::cerr << "Error: ros2_node is null in readdir" << std::endl;
    return -EIO;
  }

  std::string spath(path);
  RCLCPP_DEBUG(ros2_node->get_logger(), "readdir: path=%s", spath.c_str());

  if (spath == "/")
  {
    if (filler(buf, ".", nullptr, 0) || filler(buf, "..", nullptr, 0))
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "filler failed for root directory");
      return -ENOMEM;
    }
    // List top-level directories (first component of each topic)
    std::set<std::string> top_level_dirs;
    auto topics = ros2_node->get_topics();
    for (const auto& topic : topics)
    {
      size_t pos = topic.find('/');
      std::string top_level = (pos == std::string::npos) ? topic : topic.substr(0, pos);
      top_level_dirs.insert(top_level);
    }
    for (const auto& dir : top_level_dirs)
    {
      if (filler(buf, dir.c_str(), nullptr, 0))
      {
        RCLCPP_ERROR(ros2_node->get_logger(), "filler failed for top-level dir %s", dir.c_str());
        return -ENOMEM;
      }
    }
    return 0;
  }

  std::string path_no_slash = spath.substr(1);
  auto topics = ros2_node->get_topics();

  // Check if path is a topic
  if (std::find(topics.begin(), topics.end(), path_no_slash) != topics.end())
  {
    if (filler(buf, ".", nullptr, 0) || filler(buf, "..", nullptr, 0) ||
        filler(buf, "latest", nullptr, 0) || filler(buf, "info", nullptr, 0))
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "filler failed for topic directory %s", path_no_slash.c_str());
      return -ENOMEM;
    }
    return 0;
  }

  // Check if path is a parent directory
  std::vector<std::string> subdirs;
  std::vector<std::string> matching_topics;
  for (const auto& topic : topics)
  {
    if (topic.compare(0, path_no_slash.length(), path_no_slash) == 0 &&
        topic.length() > path_no_slash.length() && topic[path_no_slash.length()] == '/')
    {
      std::string remaining = topic.substr(path_no_slash.length() + 1);
      size_t pos = remaining.find('/');
      std::string next_dir = (pos == std::string::npos) ? remaining : remaining.substr(0, pos);
      subdirs.push_back(next_dir);
    }
    else if (topic == path_no_slash)
    {
      matching_topics.push_back(topic);
    }
  }

  if (!subdirs.empty() || !matching_topics.empty())
  {
    if (filler(buf, ".", nullptr, 0) || filler(buf, "..", nullptr, 0))
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "filler failed for directory %s", path_no_slash.c_str());
      return -ENOMEM;
    }
    for (const auto& dir : subdirs)
    {
      if (filler(buf, dir.c_str(), nullptr, 0))
      {
        RCLCPP_ERROR(ros2_node->get_logger(), "filler failed for subdirectory %s", dir.c_str());
        return -ENOMEM;
      }
    }
    return 0;
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "readdir: path %s not found", spath.c_str());
  return -ENOENT;
}

static int topicfs_open(const char* path, struct fuse_file_info* fi)
{
  (void)fi;
  std::string spath(path);
  size_t pos = spath.rfind('/');
  if (pos == std::string::npos || pos == 0)
  {
    return -ENOENT;
  }

  std::string topic = spath.substr(1, pos - 1);
  std::string file = spath.substr(pos + 1);
  auto topics = ros2_node->get_topics();
  if (std::find(topics.begin(), topics.end(), topic) != topics.end() &&
      (file == "latest" || file == "info"))
  {
    return 0;
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "open: path %s not found", spath.c_str());
  return -ENOENT;
}

static int topicfs_read(const char* path, char* buf, size_t size, off_t offset, struct fuse_file_info* fi)
{
  (void)fi;
  if (offset < 0)
  {
    return -EINVAL;
  }
  std::string spath(path);
  size_t pos = spath.rfind('/');
  if (pos == std::string::npos || pos == 0)
  {
    return -ENOENT;
  }

  std::string topic = spath.substr(1, pos - 1);
  std::string file = spath.substr(pos + 1);
  std::string original_topic = "/" + topic;
  std::lock_guard<std::mutex> lock(ros2_node->messages_mutex);

  if (file == "latest" && ros2_node->latest_messages.count(original_topic))
  {
    const std::string& data = ros2_node->latest_messages[original_topic];
    if (static_cast<std::string::size_type>(offset) >= data.size())
    {
      return 0;
    }
    size_t len = std::min(size, data.size() - static_cast<std::string::size_type>(offset));
    memcpy(buf, data.c_str() + offset, len);
    return len;
  }
  else if (file == "info")
  {
    std::string type = ros2_node->topic_types_.count(original_topic) ? ros2_node->topic_types_[original_topic] : "unknown";
    std::string info = "Topic: " + original_topic + "\nType: " + type + "\n";
    RCLCPP_DEBUG(ros2_node->get_logger(), "Reading info for %s: %s", original_topic.c_str(), info.c_str());
    if (static_cast<std::string::size_type>(offset) >= info.size())
    {
      return 0;
    }
    size_t len = std::min(size, info.size() - static_cast<std::string::size_type>(offset));
    memcpy(buf, info.c_str() + offset, len);
    return len;
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "read: path %s not found", spath.c_str());
  return -ENOENT;
}

static int collect_entries(void* buf, const char* name, const struct stat* stbuf, off_t off)
{
  (void)stbuf;
  (void)off;
  std::vector<std::string>* entries = static_cast<std::vector<std::string>*>(buf);
  entries->push_back(name);
  return 0;
}

void list_fuse_filesystem(rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Listing FUSE filesystem structure:");
  std::vector<std::string> root_entries;
  if (topicfs_readdir("/", &root_entries, collect_entries, 0, nullptr) == 0)
  {
    RCLCPP_INFO(logger, "/");
    for (const auto& entry : root_entries)
    {
      if (entry != "." && entry != "..")
      {
        RCLCPP_INFO(logger, "  /%s", entry.c_str());
        std::string t_path = "/" + entry;
        std::vector<std::string> t_entries;
        if (topicfs_readdir(t_path.c_str(), &t_entries, collect_entries, 0, nullptr) == 0)
        {
          for (const auto& topic_entry : t_entries)
          {
            if (topic_entry != "." && topic_entry != "..")
            {
              RCLCPP_INFO(logger, "    /%s/%s", entry.c_str(), topic_entry.c_str());
            }
          }
        }
        else
        {
          RCLCPP_ERROR(logger, "Failed to read topic directory: %s", t_path.c_str());
        }
      }
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to read root directory");
  }
}

static struct fuse_operations topicfs_oper;
void init_fuse_operations()
{
  memset(&topicfs_oper, 0, sizeof(struct fuse_operations));
  topicfs_oper.getattr = topicfs_getattr;
  topicfs_oper.readdir = topicfs_readdir;
  topicfs_oper.open = topicfs_open;
  topicfs_oper.read = topicfs_read;
}

int main(int argc, char* argv[])
{
  try
  {
    rclcpp::init(argc, argv);
    ros2_node = std::make_shared<topicfsNode>();

    // Get mount_point parameter
    std::string mount_point;
    ros2_node->declare_parameter<std::string>("mount_point", "~/fuse_mount");
    ros2_node->get_parameter("mount_point", mount_point);

    // Check for command-line argument (e.g., mount_point=/mnt/rosTopics)
    for (int i = 1; i < argc; ++i)
    {
      std::string arg(argv[i]);
      if (arg.find("mount_point:=") == 0)
      {
        mount_point = arg.substr(std::string("mount_point:=").length());
        RCLCPP_INFO(ros2_node->get_logger(), "Overriding mount_point from command line: %s", mount_point.c_str());
      }
    }

    // Expand ~ to home directory
    if (!mount_point.empty() && mount_point[0] == '~')
    {
      const char* home = getenv("HOME");
      if (home)
      {
        mount_point = std::string(home) + mount_point.substr(1);
      }
      else
      {
        RCLCPP_ERROR(ros2_node->get_logger(), "Failed to resolve ~: HOME environment variable not set");
        rclcpp::shutdown();
        return 1;
      }
    }

    // Check if mount point is already in use
    FILE* mount_file = fopen("/proc/mounts", "r");
    if (mount_file)
    {
      char line[1024];
      while (fgets(line, sizeof(line), mount_file))
      {
        if (strstr(line, mount_point.c_str()) && strstr(line, "fuse"))
        {
          RCLCPP_WARN(ros2_node->get_logger(), "Mount point %s is already in use", mount_point.c_str());
          std::string cmd = "fusermount -u " + mount_point;
          int ret = system(cmd.c_str());
          if (ret != 0)
          {
            RCLCPP_ERROR(ros2_node->get_logger(), "Failed to unmount %s. Please unmount manually and try again.", mount_point.c_str());
            fclose(mount_file);
            rclcpp::shutdown();
            return 1;
          }
          RCLCPP_INFO(ros2_node->get_logger(), "Successfully unmounted stale mount point %s", mount_point.c_str());
          break;
        }
      }
      fclose(mount_file);
    }

    // Ensure mount point directory exists
    if (mkdir(mount_point.c_str(), 0755) != 0 && errno != EEXIST)
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "Failed to create mount point %s: %s", mount_point.c_str(), strerror(errno));
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(ros2_node->get_logger(), "Starting FUSE filesystem at %s", mount_point.c_str());

    // List FUSE filesystem structure
    list_fuse_filesystem(ros2_node->get_logger());

    // Initialize FUSE operations
    init_fuse_operations();

    // Spin ROS2 node in a separate thread
    std::thread ros_thread([]() {
        RCLCPP_INFO(ros2_node->get_logger(), "Spinning ROS2 node");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(ros2_node);
        while (rclcpp::ok())
        {
          executor.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        RCLCPP_INFO(ros2_node->get_logger(), "ROS2 node spinning finished");
        // Ensure FUSE is unmounted on exit
//      rclcpp::spin(ros2_node);
      rclcpp::shutdown();
    });

    // Prepare arguments for fuse_main
    std::vector<char*> fuse_args = {(char*)"topic_fs", (char*)"-f", (char*)mount_point.c_str(), nullptr};
    int fuse_argc = 3;
    char** fuse_argv = fuse_args.data();

    // Run FUSE
    RCLCPP_INFO(ros2_node->get_logger(), "calling fuse_main");
    int ret = fuse_main(fuse_argc, fuse_argv, &topicfs_oper, nullptr);
    RCLCPP_INFO(ros2_node->get_logger(), "fuse_main exited");
    if (ret != 0)
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "fuse_main failed with return code: %d", ret);
    } 
    else 
    {
        RCLCPP_INFO(ros2_node->get_logger(), "fuse_main exited with return code: %d", ret);
    }

    // Ensure FUSE is unmounted on exit
    std::string cmd = "fusermount -u " + mount_point;
    if (system(cmd.c_str()) != 0)
    {
      RCLCPP_WARN(ros2_node->get_logger(), "Failed to unmount %s during shutdown", mount_point.c_str());
    }

    // Cleanup
    RCLCPP_INFO(ros2_node->get_logger(), "FUSE filesystem stopped with return code: %d", ret);
    rclcpp::shutdown();
    ros_thread.join();
    return ret;
  }
  catch (const std::exception& e)
  {
    // Ensure FUSE is unmounted on exception
    std::string mount_point;
    if (ros2_node)
    {
      ros2_node->get_parameter("mount_point", mount_point);
      if (!mount_point.empty() && mount_point[0] == '~')
      {
        const char* home = getenv("HOME");
        if (home)
        {
          mount_point = std::string(home) + mount_point.substr(1);
        }
      }
      std::string cmd = "fusermount -u " + mount_point;
      if (system(cmd.c_str()) != 0)
      {
        std::cerr << "Failed to unmount " << mount_point << " during exception handling" << std::endl;
      }
    }

    std::cerr << "Exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}