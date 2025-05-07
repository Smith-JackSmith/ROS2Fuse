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
#define FUSE_USE_VERSION 29  // TODO(Jack): Update this to the latest FUSE version
#include <fuse.h>
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <cstring>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

// ROS2 message type (example: std_msgs/String)
#include <std_msgs/msg/string.hpp>

// Node to manage ROS2 subscriptions
class topicfsNode : public rclcpp::Node
{
public:
topicfsNode()
    : Node("ros2_fuse_node")
{
    // Discover available topics
    discover_topics();
}

// Store latest message for each topic
std::unordered_map<std::string, std::string> latest_messages;
std::mutex messages_mutex;

// Subscribe to a topic dynamically
void subscribe_to_topic(const std::string& topic_name)
{
    if (subscriptions_.count(topic_name))
    {
        RCLCPP_WARN(this->get_logger(), "Already subscribed to topic: %s", topic_name.c_str());
        return;
    }
    // Example: Subscribe to std_msgs/String topics
    auto sub = create_subscription<std_msgs::msg::String>(
        topic_name, 10,
        [this, topic_name](const std_msgs::msg::String::SharedPtr msg) {
            nlohmann::json j;
            j["data"] = msg->data;
            std::lock_guard<std::mutex> lock(messages_mutex);
            latest_messages[topic_name] = j.dump();
        });
    subscriptions_[topic_name] = sub;
    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic_name.c_str());
}

// Get list of topics
std::vector<std::string> get_topics()
{
    std::lock_guard<std::mutex> lock(messages_mutex);
    std::vector<std::string> topics;
    for (const auto& pair : topic_types_)
    {
        std::string topic = pair.first;
        // Remove leading '/' for FUSE directory names
        if (!topic.empty() && topic[0] == '/')
        {
            topic = topic.substr(1);
        }
        topics.push_back(topic);
    }
    return topics;
}

private:
std::map<std::string, std::string> topic_types_;
std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

void discover_topics()
{
    auto topic_names_and_types = get_topic_names_and_types();
    for (const auto& [topic, types] : topic_names_and_types)
    {
        RCLCPP_INFO(this->get_logger(), "Found topic: %s, type: %s", topic.c_str(),
                    types.empty() ? "none" : types[0].c_str());
        if (!types.empty() && types[0] == "std_msgs/msg/String")
        {
            topic_types_[topic] = types[0];         // Store type
            subscribe_to_topic(topic);         // Subscribe only to std_msgs/String topics
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

    // Check if path is a topic (directory)
    auto topics = ros2_node->get_topics();
    std::string topic_name = spath.substr(1);  // Remove leading '/'

    if (std::find(topics.begin(), topics.end(), topic_name) != topics.end())
    {
        stbuf->st_mode = S_IFDIR | 0755;
        stbuf->st_nlink = 2;
        return 0;
    }

    // Check if path is a file under a topic (e.g., /topic/latest or /topic/info)
    size_t pos = spath.find('/', 1);
    if (pos != std::string::npos)
    {
        std::string topic = spath.substr(1, pos - 1);
        std::string file = spath.substr(pos + 1);
        if (std::find(topics.begin(), topics.end(), topic) != topics.end())
        {
            if (file == "latest" || file == "info")
            {
                stbuf->st_mode = S_IFREG | 0444;
                stbuf->st_nlink = 1;
                std::lock_guard<std::mutex> lock(ros2_node->messages_mutex);
                if (file == "latest" && ros2_node->latest_messages.count("/" + topic))
                {
                    stbuf->st_size = ros2_node->latest_messages["/" + topic].size();
                } else if (file == "info")
                {
                    stbuf->st_size = topic.size() + 10;  // Approximate size
                }
                return 0;
            }
        }
    }

    return -ENOENT;
}

static int topicfs_readdir(
    const char* path, void* buf, fuse_fill_dir_t filler,
    off_t offset, struct fuse_file_info* fi)
{
    (void) fi;          // Suppress unused parameter warning
    (void) offset;      // Suppress unused parameter warning

    if (!ros2_node)
    {
        std::cerr << "Error: ros2_node is null in readdir" << std::endl;
        return -EIO;    // Input/output error
    }

    std::string spath(path);
    std::cerr << "readdir: path=" << spath << std::endl;    // Debug log

    if (spath == "/")
    {
        if (filler(buf, ".", nullptr, 0) || filler(buf, "..", nullptr, 0))
        {
            std::cerr << "Error: filler failed for root directory" << std::endl;
            return -ENOMEM;
        }
        auto topics = ros2_node->get_topics();
        std::cerr << "readdir: found " << topics.size() << " topics" << std::endl;
        for (const auto& topic : topics)
        {
            if (filler(buf, topic.c_str(), nullptr, 0))
            {
                std::cerr << "Error: filler failed for topic " << topic << std::endl;
                return -ENOMEM;
            }
        }
        return 0;
    }

    auto topics = ros2_node->get_topics();
    std::string topic_name = spath.substr(1);
    if (std::find(topics.begin(), topics.end(), topic_name) != topics.end())
    {
        if (filler(buf, ".", nullptr, 0) || filler(buf, "..", nullptr, 0) ||
            filler(buf, "latest", nullptr, 0) || filler(buf, "info", nullptr, 0))
        {
            std::cerr << "Error: filler failed for topic directory " << topic_name << std::endl;
            return -ENOMEM;
        }
        return 0;
    }

    std::cerr << "readdir: path " << spath << " not found" << std::endl;
    return -ENOENT;
}

static int topicfs_open(const char* path, struct fuse_file_info* fi)
{
    (void) fi;      // Suppress unused parameter warning

    std::string spath(path);
    size_t pos = spath.find('/', 1);
    if (pos == std::string::npos)
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

    return -ENOENT;
}

static int topicfs_read(
    const char* path, char* buf, size_t size, off_t offset,
    struct fuse_file_info* fi)
{
    (void) fi;          // Suppress unused parameter warning

    if (offset < 0)
    {
        return -EINVAL;     // Invalid argument if offset is negative
    }
    std::string spath(path);
    size_t pos = spath.find('/', 1);
    if (pos == std::string::npos)
    {
        return -ENOENT;
    }

    std::string topic = spath.substr(1, pos - 1);
    std::string file = spath.substr(pos + 1);
    std::lock_guard<std::mutex> lock(ros2_node->messages_mutex);

    if (file == "latest" && ros2_node->latest_messages.count("/" + topic))
    {
        const std::string& data = ros2_node->latest_messages["/" + topic];
        if (static_cast<std::string::size_type>(offset) >= data.size())
        {
            return 0;
        }
        size_t len = std::min(size, data.size() - static_cast<std::string::size_type>(offset));
        memcpy(buf, data.c_str() + offset, len);
        return len;
    } else if (file == "info")
    {
        std::string info = "Topic: " + topic + "\nType: std_msgs/String\n";
        if (static_cast<std::string::size_type>(offset) >= info.size())
        {
            return 0;
        }
        size_t len = std::min(size, info.size() - static_cast<std::string::size_type>(offset));
        memcpy(buf, info.c_str() + offset, len);
        return len;
    }

    return -ENOENT;
}

// Function to collect directory entries for listing
static int collect_entries(void* buf, const char* name, const struct stat* stbuf, off_t off)
{
    (void) stbuf;       // Unused
    (void) off;         // Unused
    std::vector<std::string>* entries = static_cast<std::vector<std::string>*>(buf);
    entries->push_back(name);
    return 0;
}

// Function to list all files and directories in the FUSE filesystem
void list_fuse_filesystem(rclcpp::Logger logger)
{
    RCLCPP_INFO(logger, "Listing FUSE filesystem structure:");

    // List root directory (/)
    std::vector<std::string> root_entries;
    if (topicfs_readdir("/", &root_entries, collect_entries, 0, nullptr) == 0)
    {
        RCLCPP_INFO(logger, "/");
        for (const auto& entry : root_entries)
        {
            if (entry != "." && entry != "..")
            {
                RCLCPP_INFO(logger, "  /%s", entry.c_str());
                // List contents of each topic directory
                std::string t_path = "/" + entry;  // apologies for the variable name, blame LINT
                std::vector<std::string> t_entries;  // blame LINT
                if( 0 == topicfs_readdir(t_path.c_str(), &t_entries, collect_entries, 0, nullptr)  )
                {
                    for (const auto& topic_entry : t_entries)
                    {
                        if (topic_entry != "." && topic_entry != "..")
                        {
                            RCLCPP_INFO(logger, "    /%s/%s", entry.c_str(), topic_entry.c_str());
                        }
                    }
                } else
                {
                    RCLCPP_ERROR(logger, "Failed to read topic directory: %s", t_path.c_str());
                }
            }
        }
    } else
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
    try {
        // Initialize ROS2
        rclcpp::init(argc, argv);
        ros2_node = std::make_shared<topicfsNode>();
        RCLCPP_INFO(ros2_node->get_logger(), "Topic FS Utility - Created by Jack Sidman Smith");
        RCLCPP_INFO(ros2_node->get_logger(), "Starting FUSE filesystem");

        // List FUSE filesystem structure
        list_fuse_filesystem(ros2_node->get_logger());

        // Initialize FUSE operations
        init_fuse_operations();

        // Spin ROS2 node in a separate thread
        std::thread ros_thread([]() {
                               rclcpp::spin(ros2_node);
                               rclcpp::shutdown();
            });

        // Run FUSE
        int ret = fuse_main(argc, argv, &topicfs_oper, nullptr);

        // Cleanup
        RCLCPP_INFO(ros2_node->get_logger(), "FUSE filesystem stopped with return code: %d", ret);
        rclcpp::shutdown();
        ros_thread.join();
        return ret;
    }
    catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }
}
