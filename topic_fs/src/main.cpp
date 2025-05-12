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

#include "topic_fs/topicfs_node.hpp"
#include "topic_fs/topicfs_fuse.hpp"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

std::shared_ptr<topicfsNode> ros2_node;

int main(int argc, char* argv[])
{
  try
  {
    rclcpp::init(argc, argv);
    ros2_node = std::make_shared<topicfsNode>();
    RCLCPP_INFO(ros2_node->get_logger(), "Topic FS Utility - Created by Jack Sidman Smith");

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
            RCLCPP_ERROR(ros2_node->get_logger(), "Failed to unmount %s: %s. Please unmount manually and try again.", mount_point.c_str(), strerror(errno));
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

    // Ensure mount point directory exists and is writable
    struct stat st;
    if (stat(mount_point.c_str(), &st) != 0)
    {
      if (mkdir(mount_point.c_str(), 0755) != 0)
      {
        RCLCPP_ERROR(ros2_node->get_logger(), "Failed to create mount point %s: %s", mount_point.c_str(), strerror(errno));
        rclcpp::shutdown();
        return 1;
      }
    }
    else if (!S_ISDIR(st.st_mode))
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "Mount point %s is not a directory", mount_point.c_str());
      rclcpp::shutdown();
      return 1;
    }
    else if (access(mount_point.c_str(), W_OK) != 0)
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "Mount point %s is not writable: %s", mount_point.c_str(), strerror(errno));
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
      rclcpp::spin(ros2_node);
      rclcpp::shutdown();
    });

    // Prepare arguments for fuse_main
    std::vector<char*> fuse_args = {(char*)"topic_fs", (char*)"-f", (char*)mount_point.c_str(), nullptr};
    int fuse_argc = 3;
    char** fuse_argv = fuse_args.data();

    // Run FUSE in foreground and log errors
    RCLCPP_INFO(ros2_node->get_logger(), "Calling fuse_main with args: topic_fs -f %s", mount_point.c_str());
    int ret = fuse_main(fuse_argc, fuse_argv, &topicfs_oper, nullptr);
    if (ret != 0)
    {
      RCLCPP_ERROR(ros2_node->get_logger(), "fuse_main failed with return code: %d. Error: %s", ret, strerror(errno));
    }

    // Ensure FUSE is unmounted on exit
    std::string cmd = "fusermount -u " + mount_point;
    if (system(cmd.c_str()) != 0)
    {
      RCLCPP_WARN(ros2_node->get_logger(), "Failed to unmount %s during shutdown: %s", mount_point.c_str(), strerror(errno));
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
        std::cerr << "Failed to unmount " << mount_point << " during exception handling: " << strerror(errno) << std::endl;
      }
    }

    std::cerr << "Exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}