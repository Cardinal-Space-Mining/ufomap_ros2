/**
 * UFOMap Mapping
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap_mapping
 * License: BSD 3
 *
 */

/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, D. Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_MAP_MAPPING_SERVER_H
#define UFO_MAP_MAPPING_SERVER_H

// UFO
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>

#include <ufomap_msgs/msg/ufo_map_stamped.hpp>
#include <ufomap_msgs/srv/clear_volume.hpp>
#include <ufomap_msgs/srv/get_map.hpp>
#include <ufomap_msgs/srv/reset.hpp>
#include <ufomap_msgs/srv/save_map.hpp>

// ROS
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// STD
#include <future>
#include <variant>
#include <vector>

namespace ufomap_mapping
{
class UFOMapNode : public rclcpp::Node
{
 public:
	UFOMapNode();

 private:
	void cloudCallback(const sensor_msgs::msg::PointCloud2 &msg);

	void publishInfo();

	bool getMapCallback(const std::shared_ptr<ufomap_msgs::srv::GetMap::Request> request,
	                    std::shared_ptr<ufomap_msgs::srv::GetMap::Response> response);

	bool clearVolumeCallback(
	    const std::shared_ptr<ufomap_msgs::srv::ClearVolume::Request> request,
	    std::shared_ptr<ufomap_msgs::srv::ClearVolume::Response> response);

	bool resetCallback(const std::shared_ptr<ufomap_msgs::srv::Reset::Request> request,
	                   std::shared_ptr<ufomap_msgs::srv::Reset::Response> response);

	bool saveMapCallback(const std::shared_ptr<ufomap_msgs::srv::SaveMap::Request> request,
	                     std::shared_ptr<ufomap_msgs::srv::SaveMap::Response> response);

	OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
	void initParams();
	rcl_interfaces::msg::SetParametersResult parameterCallback(
	    const std::vector<rclcpp::Parameter> &params);
	void updateFromParams();

	void timerCallback();

 private:
	bool node_init;

	// Subscribers
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
	unsigned int cloud_in_queue_size_;

	// Publishers
	std::vector<rclcpp::Publisher<ufomap_msgs::msg::UFOMapStamped>::SharedPtr> map_pub_;
	unsigned int map_queue_size_;
	rclcpp::TimerBase::SharedPtr pub_timer_;
	double pub_rate_;
	rclcpp::Duration update_rate_;
	rclcpp::Time last_update_time_;
	rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr info_pub_;

	// Services
	rclcpp::Service<ufomap_msgs::srv::GetMap>::SharedPtr get_map_server_;
	rclcpp::Service<ufomap_msgs::srv::ClearVolume>::SharedPtr clear_volume_server_;
	rclcpp::Service<ufomap_msgs::srv::Reset>::SharedPtr reset_server_;
	rclcpp::Service<ufomap_msgs::srv::SaveMap>::SharedPtr save_map_server_;

	// TF2
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	rclcpp::Duration transform_timeout_;

	//
	// UFO Parameters
	//

	// Map
	std::variant<std::monostate, ufo::map::OccupancyMap, ufo::map::OccupancyMapColor> map_;
	std::string frame_id_;

	// Integration
	double max_range_;
	ufo::map::DepthType insert_depth_;
	bool simple_ray_casting_;
	unsigned int early_stopping_;
	bool async_;

	// Clear robot
	bool clear_robot_;
	std::string robot_frame_id_;
	double robot_height_;
	double robot_radius_;
	int clearing_depth_;

	// Publishing
	bool compress_;
	bool update_part_of_map_;
	ufo::map::DepthType publish_depth_;
	std::future<void> update_async_handler_;

	//
	// Information
	//

	// Integration
	double min_integration_time_;
	double max_integration_time_ = 0.0;
	double accumulated_integration_time_ = 0.0;
	int num_integrations_ = 0;

	// Clear robot
	double min_clear_time_;
	double max_clear_time_ = 0.0;
	double accumulated_clear_time_ = 0.0;
	int num_clears_ = 0;

	// Publish update
	double min_update_time_;
	double max_update_time_ = 0.0;
	double accumulated_update_time_ = 0.0;
	int num_updates_ = 0;

	// Publish whole
	double min_whole_time_;
	double max_whole_time_ = 0.0;
	double accumulated_whole_time_ = 0.0;
	int num_wholes_ = 0;

	// Verbose
	bool verbose_;
};
}  // namespace ufomap_mapping

#endif  // UFO_MAP_MAPPING_SERVER_H
