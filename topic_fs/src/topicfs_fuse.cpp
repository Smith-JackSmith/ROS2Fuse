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

#include "topic_fs/topicfs_fuse.hpp"
#include <algorithm>
#include <set>
#include <cstring>
#include <chrono>
#include <thread>

extern std::shared_ptr<topicfsNode> ros2_node;
struct fuse_operations topicfs_oper;

std::vector<uint8_t> base64_decode(const std::string& encoded)
{
  static const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::vector<uint8_t> decoded;
  decoded.reserve((encoded.size() * 3) / 4);

  for (size_t i = 0; i < encoded.size(); i += 4)
  {
    uint32_t b = 0;
    for (int j = 0; j < 4; ++j)
    {
      if (i + j < encoded.size() && encoded[i + j] != '=')
      {
        auto pos = base64_chars.find(encoded[i + j]);
        if (pos == std::string::npos)
        {
          throw std::runtime_error("Invalid base64 character");
        }
        b |= pos << (18 - j * 6);
      }
    }
    decoded.push_back((b >> 16) & 0xFF);
    if (decoded.size() < encoded.size() * 3 / 4) decoded.push_back((b >> 8) & 0xFF);
    if (decoded.size() < encoded.size() * 3 / 4) decoded.push_back(b & 0xFF);
  }

  return decoded;
}

int topicfs_getattr(const char* path, struct stat* stbuf)
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
  std::string path_no_slash = spath.substr(1);

  bool is_topic = false;
  for (const auto& topic : topics)
  {
    if (path_no_slash == topic)
    {
      is_topic = true;
      break;
    }
    if (topic.compare(0, path_no_slash.length(), path_no_slash) == 0 &&
        (topic.length() == path_no_slash.length() || topic[path_no_slash.length()] == '/'))
    {
      stbuf->st_mode = S_IFDIR | 0755;
      stbuf->st_nlink = 2;
      return 0;
    }
  }

  if (is_topic)
  {
    stbuf->st_mode = S_IFDIR | 0755;
    stbuf->st_nlink = 2;
    return 0;
  }

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
      else if (file == "command" && ros2_node->publishers_.count(original_topic))
      {
        stbuf->st_mode = S_IFREG | 0222;
        stbuf->st_nlink = 1;
        stbuf->st_size = 0;
        return 0;
      }
    }
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "getattr: path %s not found", spath.c_str());
  return -ENOENT;
}

int topicfs_readdir(const char* path, void* buf, fuse_fill_dir_t filler, off_t offset, struct fuse_file_info* fi)
{
  (void)fi;
  (void)offset;

  if (!ros2_node)
  {
    RCLCPP_ERROR(ros2_node->get_logger(), "ros2_node is null in readdir");
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

  if (std::find(topics.begin(), topics.end(), path_no_slash) != topics.end())
  {
    if (filler(buf, ".", nullptr, 0) || filler(buf, "..", nullptr, 0) ||
        filler(buf, "latest", nullptr, 0) || filler(buf, "info", nullptr, 0))
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "filler failed for topic directory %s", path_no_slash.c_str());
      return -ENOMEM;
    }
    if (ros2_node->publishers_.count("/" + path_no_slash))
    {
      if (filler(buf, "command", nullptr, 0))
      {
        RCLCPP_ERROR(ros2_node->get_logger(), "filler failed for command file in %s", path_no_slash.c_str());
        return -ENOMEM;
      }
    }
    return 0;
  }

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

int topicfs_open(const char* path, struct fuse_file_info* fi)
{
  std::string spath(path);
  size_t pos = spath.rfind('/');
  if (pos == std::string::npos || pos == 0)
  {
    return -ENOENT;
  }

  std::string topic = spath.substr(1, pos - 1);
  std::string file = spath.substr(pos + 1);
  std::string original_topic = "/" + topic;
  auto topics = ros2_node->get_topics();

  if (std::find(topics.begin(), topics.end(), topic) != topics.end())
  {
    if (file == "latest" || file == "info")
    {
      if ((fi->flags & O_ACCMODE) != O_RDONLY)
      {
        return -EACCES;
      }
      return 0;
    }
    else if (file == "command" && ros2_node->publishers_.count(original_topic))
    {
      if ((fi->flags & O_ACCMODE) != O_WRONLY && (fi->flags & O_ACCMODE) != O_RDWR)
      {
        return -EACCES;
      }
      return 0;
    }
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "open: path %s not found", spath.c_str());
  return -ENOENT;
}

int topicfs_read(const char* path, char* buf, size_t size, off_t offset, struct fuse_file_info* fi)
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

  if (file == "latest")
  {
    std::unique_lock<std::mutex> lock(ros2_node->messages_mutex);
    if (!ros2_node->latest_messages.count(original_topic))
    {
      RCLCPP_DEBUG(ros2_node->get_logger(), "No data for %s/latest", original_topic.c_str());
      return -ENOENT;
    }

    std::string data = ros2_node->latest_messages[original_topic];
    uint64_t last_version = ros2_node->message_versions_[original_topic];

    if (static_cast<std::string::size_type>(offset) >= data.size())
    {
      const int max_attempts = 100;
      for (int i = 0; i < max_attempts; ++i)
      {
        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        lock.lock();

        if (ros2_node->message_versions_[original_topic] != last_version)
        {
          data = ros2_node->latest_messages[original_topic];
          last_version = ros2_node->message_versions_[original_topic];
          RCLCPP_DEBUG(ros2_node->get_logger(), "New message for %s/latest (version %lu)", original_topic.c_str(), last_version);
          break;
        }
      }
    }

    if (static_cast<std::string::size_type>(offset) >= data.size())
    {
      return 0;
    }

    size_t len = std::min(size, data.size() - static_cast<std::string::size_type>(offset));
    memcpy(buf, data.c_str() + offset, len);
    RCLCPP_DEBUG(ros2_node->get_logger(), "Read %zu bytes from %s/latest at offset %ld", len, original_topic.c_str(), offset);
    return len;
  }
  else if (file == "info")
  {
    std::lock_guard<std::mutex> lock(ros2_node->messages_mutex);
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

int topicfs_write(const char* path, const char* buf, size_t size, off_t offset, struct fuse_file_info* fi)
{
  (void)fi;
  (void)offset;
  std::string spath(path);
  size_t pos = spath.rfind('/');
  if (pos == std::string::npos || pos == 0)
  {
    return -ENOENT;
  }

  std::string topic = spath.substr(1, pos - 1);
  std::string file = spath.substr(pos + 1);
  std::string original_topic = "/" + topic;

  if (file == "command" && ros2_node->publishers_.count(original_topic))
  {
    std::lock_guard<std::mutex> lock(ros2_node->messages_mutex);
    std::string data(buf, size);
    RCLCPP_DEBUG(ros2_node->get_logger(), "Writing to %s/command: %s", original_topic.c_str(), data.c_str());
    try
    {
      nlohmann::json j = nlohmann::json::parse(data);
      if (!j.contains("data") || !j["data"].is_string())
      {
        RCLCPP_ERROR(ros2_node->get_logger(), "Invalid JSON for %s/command: missing or invalid 'data' field (expected base64)", original_topic.c_str());
        return -EINVAL;
      }
      std::string encoded = j["data"].get<std::string>();

      std::vector<uint8_t> decoded = base64_decode(encoded);
      rclcpp::SerializedMessage serialized_msg(decoded.size());
      memcpy(serialized_msg.get_rcl_serialized_message().buffer, decoded.data(), decoded.size());
      serialized_msg.get_rcl_serialized_message().buffer_length = decoded.size();

      ros2_node->publishers_[original_topic]->publish(serialized_msg);
      RCLCPP_DEBUG(ros2_node->get_logger(), "Published message to %s", original_topic.c_str());
      return size;
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "Failed to parse or publish to %s/command: %s", original_topic.c_str(), e.what());
      return -EINVAL;
    }
  }

  RCLCPP_ERROR(ros2_node->get_logger(), "write: path %s not found or not writable", spath.c_str());
  return -ENOENT;
}

int collect_entries(void* buf, const char* name, const struct stat* stbuf, off_t off)
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

void init_fuse_operations()
{
  memset(&topicfs_oper, 0, sizeof(struct fuse_operations));
  topicfs_oper.getattr = topicfs_getattr;
  topicfs_oper.readdir = topicfs_readdir;
  topicfs_oper.open = topicfs_open;
  topicfs_oper.read = topicfs_read;
  topicfs_oper.write = topicfs_write;
}