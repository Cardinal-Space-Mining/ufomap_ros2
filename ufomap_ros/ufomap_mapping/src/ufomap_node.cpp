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

// UFO
#include <ufomap_mapping/conversions.h>
#include <ufomap_mapping/ufomap_node.h>

#include <ufomap_msgs/msg/ufo_map_stamped.hpp>

// STD
#include <chrono>
#include <future>
#include <numeric>

namespace ufomap_mapping
{
UFOMapNode::UFOMapNode() : Node("ufomap_node")
{
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// Dynamically reconfigurable parameters (params.yaml)
	this->declare_parameter<std::string>("frame_id", "map");
	this->declare_parameter<double>("max_range", -1.0);
	this->declare_parameter<int>("insert_depth", 0);
	this->declare_parameter<int>("insert_depth", 0);
	this->declare_parameter<bool>("simple_ray_casting", false);
	this->declare_parameter<int>("early_stopping", 0);
	this->declare_parameter<bool>("async", true);
	this->declare_parameter<bool>("clear_robot", false);
	this->declare_parameter<std::string>("robot_frame_id", "base_link");
	this->declare_parameter<double>("robot_height", 0.2);
	this->declare_parameter<double>("robot_radius", 0.5);
	this->declare_parameter<int>("clearing_depth", 0);
	this->declare_parameter<bool>("compress", false);
	this->declare_parameter<bool>("update_part_of_map", true);
	this->declare_parameter<double>("update_rate", 0.0);
	this->declare_parameter<int>("publish_depth", 4);
	this->declare_parameter<double>("prob_hit", 0.7);
	this->declare_parameter<double>("prob_miss", 0.4);
	this->declare_parameter<double>("clamping_thres_min", 0.1192);
	this->declare_parameter<double>("clamping_thres_max", 0.971);
	this->declare_parameter<double>("pub_rate", 0.0);
	this->declare_parameter<double>("transform_timeout", 0.1);
	this->declare_parameter<int>("map_queue_size", 10);
	this->declare_parameter<int>("cloud_in_queue_size", 10);
	// this->declare_parameter<bool>("map_latch", false);
	this->declare_parameter<bool>("verbose", false);

	this->initParams();

	// Launch file parameters (launch.py)
	this->declare_parameter<double>("resolution", 0.05);
	this->declare_parameter<int>("depth_levels", 16);
	this->declare_parameter<int>("num_workers", 1);
	this->declare_parameter<bool>("color_map", true);

	// Set up map
	double resolution = this->get_parameter("resolution").as_double();
	ufo::map::DepthType depth_levels = this->get_parameter("depth_levels").as_int();

	// Automatic pruning is disabled so we can work in multiple threads for subscribers,
	// services and publishers
	if (this->get_parameter("color_map").as_bool()) {
		map_.emplace<ufo::map::OccupancyMapColor>(resolution, depth_levels, false);
	} else {
		map_.emplace<ufo::map::OccupancyMap>(resolution, depth_levels, false);
	}

	// Enable min/max change detection
	std::visit(
	    [](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    map.enableMinMaxChangeDetection(true);
		    }
	    },
	    map_);

	// Set up dynamic reconfigure server
	this->add_on_set_parameters_callback(
	    std::bind(&UFOMapNode::parameterCallback, this, std::placeholders::_1));

	// Set up publisher
	info_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("info", 10);

	// Enable services
	get_map_server_ = this->create_service<ufomap_msgs::srv::GetMap>(
	    "get_map", std::bind(&UFOMapNode::getMapCallback, this, std::placeholders::_1,
	                         std::placeholders::_2));
	clear_volume_server_ = this->create_service<ufomap_msgs::srv::ClearVolume>(
	    "clear_volume", std::bind(&UFOMapNode::clearVolumeCallback, this,
	                              std::placeholders::_1, std::placeholders::_2));
	reset_server_ = this->create_service<ufomap_msgs::srv::Reset>(
	    "reset", std::bind(&UFOMapNode::resetCallback, this, std::placeholders::_1,
	                       std::placeholders::_2));
	save_map_server_ = this->create_service<ufomap_msgs::srv::SaveMap>(
	    "save_map", std::bind(&UFOMapNode::saveMapCallback, this, std::placeholders::_1,
	                          std::placeholders::_2));
}

void UFOMapNode::cloudCallback(const sensor_msgs::msg::PointCloud2 &msg)
{
	ufo::math::Pose6 transform;
	try {
		transform = ufomap_ros_conversion::rosToUfo(
		    tf_buffer_
		        ->lookupTransform(frame_id_, msg.header.frame_id, msg.header.stamp,
		                          transform_timeout_)
		        .transform);
	} catch (tf2::TransformException &ex) {
		auto &clk = *this->get_clock();
		RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "%s", ex.what());
		return;
	}

	std::visit(
	    [this, &msg, &transform](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    auto start = std::chrono::steady_clock::now();

			    // Update map
			    ufo::map::PointCloudColor cloud;
			    ufomap_ros_conversion::rosToUfo(msg, cloud);
			    cloud.transform(transform, true);

			    map.insertPointCloudDiscrete(transform.translation(), cloud, max_range_,
			                                 insert_depth_, simple_ray_casting_,
			                                 early_stopping_, async_);

			    double integration_time =
			        std::chrono::duration<float, std::chrono::seconds::period>(
			            std::chrono::steady_clock::now() - start)
			            .count();

			    if (0 == num_integrations_ || integration_time < min_integration_time_) {
				    min_integration_time_ = integration_time;
			    }
			    if (integration_time > max_integration_time_) {
				    max_integration_time_ = integration_time;
			    }
			    accumulated_integration_time_ += integration_time;
			    ++num_integrations_;

			    // Clear robot
			    if (clear_robot_) {
				    start = std::chrono::steady_clock::now();

				    try {
					    transform = ufomap_ros_conversion::rosToUfo(
					        tf_buffer_
					            ->lookupTransform(frame_id_, robot_frame_id_, msg.header.stamp,
					                              transform_timeout_)
					            .transform);
				    } catch (tf2::TransformException &ex) {
					    auto &clk = *this->get_clock();
					    RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "%s", ex.what());
					    return;
				    }

				    ufo::map::Point3 r(robot_radius_, robot_radius_, robot_height_ / 2.0);
				    ufo::geometry::AABB aabb(transform.translation() - r,
				                             transform.translation() + r);
				    map.setValueVolume(aabb, map.getClampingThresMin(), clearing_depth_);

				    double clear_time =
				        std::chrono::duration<float, std::chrono::seconds::period>(
				            std::chrono::steady_clock::now() - start)
				            .count();
				    if (0 == num_clears_ || clear_time < min_clear_time_) {
					    min_clear_time_ = clear_time;
				    }
				    if (clear_time > max_clear_time_) {
					    max_clear_time_ = clear_time;
				    }
				    accumulated_clear_time_ += clear_time;
				    ++num_clears_;
			    }

			    rclcpp::Time header_stamp_(msg.header.stamp.sec, msg.header.stamp.nanosec);
			    // Publish update
			    if (!map_pub_.empty() && update_part_of_map_ && map.validMinMaxChange() &&
			        (last_update_time_.seconds() == 0 &&
			             last_update_time_.nanoseconds() ==
			                 0 ||  // Check if last update time is invalid
			         (header_stamp_ - last_update_time_) >= update_rate_)) {
				    bool can_update = true;
				    if (update_async_handler_.valid()) {
					    can_update = std::future_status::ready ==
					                 update_async_handler_.wait_for(std::chrono::seconds(0));
				    }

				    if (can_update) {
					    last_update_time_ = header_stamp_;
					    start = std::chrono::steady_clock::now();

					    ufo::geometry::AABB aabb(map.minChange(), map.maxChange());
					    // TODO: should this be here?
					    map.resetMinMaxChangeDetection();

					    update_async_handler_ = std::async(
					        std::launch::async, [this, aabb, stamp = msg.header.stamp]() {
						        std::visit(
						            [this, &aabb, stamp](auto &map) {
							            if constexpr (!std::is_same_v<std::decay_t<decltype(map)>,
							                                          std::monostate>) {
								            for (int i = 0; i < map_pub_.size(); ++i) {
									            if (map_pub_[i] &&
									                (0 < map_pub_[i]->get_subscription_count())) {
										            auto msg =
										                std::make_shared<ufomap_msgs::msg::UFOMapStamped>();
										            if (ufomap_ros_conversion::ufoToMsg(map, msg->map, aabb,
										                                                compress_, i)) {
											            msg->header.stamp = stamp;
											            msg->header.frame_id = frame_id_;
											            map_pub_[i]->publish(*msg);
										            }
									            }
								            }
							            }
						            },
						            map_);
					        });

					    double update_time =
					        std::chrono::duration<float, std::chrono::seconds::period>(
					            std::chrono::steady_clock::now() - start)
					            .count();
					    if (0 == num_updates_ || update_time < min_update_time_) {
						    min_update_time_ = update_time;
					    }
					    if (update_time > max_update_time_) {
						    max_update_time_ = update_time;
					    }
					    accumulated_update_time_ += update_time;
					    ++num_updates_;
				    }
			    }

			    publishInfo();
		    }
	    },
	    map_);
}

void UFOMapNode::publishInfo()
{
	if (verbose_) {
		printf("\nTimings:\n");
		if (0 != num_integrations_) {
			printf("\tIntegration time (s): %5d %09.6f\t(%09.6f +- %09.6f)\n",
			       num_integrations_, accumulated_integration_time_,
			       accumulated_integration_time_ / num_integrations_, max_integration_time_);
		}
		if (0 != num_clears_) {
			printf("\tClear time (s):       %5d %09.6f\t(%09.6f +- %09.6f)\n", num_clears_,
			       accumulated_clear_time_, accumulated_clear_time_ / num_clears_,
			       max_clear_time_);
		}
		if (0 != num_updates_) {
			printf("\tUpdate time (s):      %5d %09.6f\t(%09.6f +- %09.6f)\n", num_updates_,
			       accumulated_update_time_, accumulated_update_time_ / num_updates_,
			       max_update_time_);
		}
		if (0 != num_wholes_) {
			printf("\tWhole time (s):       %5d %09.6f\t(%09.6f +- %09.6f)\n", num_wholes_,
			       accumulated_whole_time_, accumulated_whole_time_ / num_wholes_,
			       max_whole_time_);
		}
	}

	if (info_pub_ && 0 < info_pub_->get_subscription_count()) {
		diagnostic_msgs::msg::DiagnosticStatus msg;
		msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
		msg.name = "UFOMap mapping timings";
		msg.values.resize(12);
		msg.values[0].key = "Min integration time (ms)";
		msg.values[0].value = std::to_string(min_integration_time_);
		msg.values[1].key = "Max integration time (ms)";
		msg.values[1].value = std::to_string(max_integration_time_);
		msg.values[2].key = "Average integration time (ms)";
		msg.values[2].value =
		    std::to_string(accumulated_integration_time_ / num_integrations_);
		msg.values[3].key = "Min clear time (ms)";
		msg.values[3].value = std::to_string(min_clear_time_);
		msg.values[4].key = "Max clear time (ms)";
		msg.values[4].value = std::to_string(max_clear_time_);
		msg.values[5].key = "Average clear time (ms)";
		msg.values[5].value = std::to_string(accumulated_clear_time_ / num_clears_);
		msg.values[6].key = "Min update time (ms)";
		msg.values[6].value = std::to_string(min_update_time_);
		msg.values[7].key = "Max update time (ms)";
		msg.values[7].value = std::to_string(max_update_time_);
		msg.values[8].key = "Average update time (ms)";
		msg.values[8].value = std::to_string(accumulated_update_time_ / num_updates_);
		msg.values[9].key = "Min whole time (ms)";
		msg.values[10].key = "Max whole time (ms)";
		msg.values[10].value = std::to_string(max_whole_time_);
		msg.values[11].key = "Average whole time (ms)";
		msg.values[11].value = std::to_string(accumulated_whole_time_ / num_wholes_);
		info_pub_->publish(msg);
	}
}

bool UFOMapNode::getMapCallback(
    const std::shared_ptr<ufomap_msgs::srv::GetMap::Request> request,
    std::shared_ptr<ufomap_msgs::srv::GetMap::Response> response)
{
	std::visit(
	    [this, &request, &response](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    ufo::geometry::BoundingVolume bv =
			        ufomap_ros_conversion::msgToUfo(request->bounding_volume);
			    response->success = ufomap_ros_conversion::ufoToMsg(
			        map, response->map, bv, request->compress, request->depth);
		    } else {
			    response->success = false;
		    }
	    },
	    map_);
	return true;
}

bool UFOMapNode::clearVolumeCallback(
    const std::shared_ptr<ufomap_msgs::srv::ClearVolume::Request> request,
    std::shared_ptr<ufomap_msgs::srv::ClearVolume::Response> response)
{
	std::visit(
	    [this, &request, &response](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    ufo::geometry::BoundingVolume bv =
			        ufomap_ros_conversion::msgToUfo(request->bounding_volume);
			    for (auto &b : bv) {
				    map.setValueVolume(b, map.getClampingThresMin(), request->depth);
			    }
			    response->success = true;
		    } else {
			    response->success = false;
		    }
	    },
	    map_);
	return true;
}

bool UFOMapNode::resetCallback(
    const std::shared_ptr<ufomap_msgs::srv::Reset::Request> request,
    std::shared_ptr<ufomap_msgs::srv::Reset::Response> response)
{
	std::visit(
	    [this, &request, &response](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    map.clear(request->new_resolution, request->new_depth_levels);
			    response->success = true;
		    } else {
			    response->success = false;
		    }
	    },
	    map_);
	return true;
}

bool UFOMapNode::saveMapCallback(
    const std::shared_ptr<ufomap_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<ufomap_msgs::srv::SaveMap::Response> response)
{
	std::visit(
	    [this, &request, &response](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    ufo::geometry::BoundingVolume bv =
			        ufomap_ros_conversion::msgToUfo(request->bounding_volume);
			    response->success = map.write(request->filename, bv, request->compress,
			                                  request->depth, 1, request->compression_level);
		    } else {
			    response->success = false;
		    }
	    },
	    map_);
	return true;
}

void UFOMapNode::timerCallback()
{
	std_msgs::msg::Header header;
	header.stamp = this->get_clock()->now();
	header.frame_id = frame_id_;

	if (!map_pub_.empty()) {
		for (int i = 0; i < map_pub_.size(); ++i) {
			if (map_pub_[i] && 0 < map_pub_[i]->get_subscription_count()) {
				std::visit(
				    [this, &header, i](auto &map) {
					    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>,
					                                  std::monostate>) {
						    auto start = std::chrono::steady_clock::now();

						    auto msg = std::make_shared<ufomap_msgs::msg::UFOMapStamped>();
						    if (ufomap_ros_conversion::ufoToMsg(
						            map, msg->map, ufo::geometry::BoundingVolume(), compress_, i)) {
							    msg->header = header;
							    map_pub_[i]->publish(*msg);
						    }

						    double whole_time =
						        std::chrono::duration<float, std::chrono::seconds::period>(
						            std::chrono::steady_clock::now() - start)
						            .count();
						    if (0 == num_wholes_ || whole_time < min_whole_time_) {
							    min_whole_time_ = whole_time;
						    }
						    if (whole_time > max_whole_time_) {
							    max_whole_time_ = whole_time;
						    }
						    accumulated_whole_time_ += whole_time;
						    ++num_wholes_;
					    }
				    },
				    map_);
			}
		}
	}
	publishInfo();
}

void UFOMapNode::initParams()
{
	// Read parameters
	frame_id_ = this->get_parameter("frame_id").as_string();

	verbose_ = this->get_parameter("verbose").as_bool();

	max_range_ = this->get_parameter("max_range").as_double();
	insert_depth_ = this->get_parameter("insert_depth").as_int();
	simple_ray_casting_ = this->get_parameter("simple_ray_casting").as_bool();
	early_stopping_ = this->get_parameter("early_stopping").as_int();
	async_ = this->get_parameter("async").as_bool();

	clear_robot_ = this->get_parameter("clear_robot").as_bool();
	robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();
	robot_height_ = this->get_parameter("robot_height").as_double();
	robot_radius_ = this->get_parameter("robot_radius").as_double();
	clearing_depth_ = this->get_parameter("clearing_depth").as_int();

	compress_ = this->get_parameter("compress").as_bool();
	update_part_of_map_ = this->get_parameter("update_part_of_map").as_bool();
	publish_depth_ = this->get_parameter("publish_depth").as_int();

	this->updateFromParams();
}

rcl_interfaces::msg::SetParametersResult UFOMapNode::parameterCallback(
    const std::vector<rclcpp::Parameter> &params)
{
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;

	for (const auto &param : params) {
		if (param.get_name() == "frame_id")
			frame_id_ = param.as_string();
		else if (param.get_name() == "verbose")
			verbose_ = param.as_bool();
		else if (param.get_name() == "max_range")
			max_range_ = param.as_double();
		else if (param.get_name() == "insert_depth")
			insert_depth_ = param.as_int();
		else if (param.get_name() == "simple_ray_casting")
			simple_ray_casting_ = param.as_bool();
		else if (param.get_name() == "early_stopping")
			early_stopping_ = param.as_int();
		else if (param.get_name() == "async")
			async_ = param.as_bool();
		else if (param.get_name() == "clear_robot")
			clear_robot_ = param.as_bool();
		else if (param.get_name() == "robot_frame_id")
			robot_frame_id_ = param.as_string();
		else if (param.get_name() == "robot_height")
			robot_height_ = param.as_double();
		else if (param.get_name() == "robot_radius")
			robot_radius_ = param.as_double();
		else if (param.get_name() == "clearing_depth")
			clearing_depth_ = param.as_int();
		else if (param.get_name() == "compress")
			compress_ = param.as_bool();
		else if (param.get_name() == "update_part_of_map")
			update_part_of_map_ = param.as_bool();
		else if (param.get_name() == "publish_depth")
			publish_depth_ = param.as_int();
		else if (param.get_name() == "pub_rate")
			pub_rate_ = param.as_double();
		else if (param.get_name() == "map_queue_size")
			map_queue_size_ = param.as_int();
	}

	this->updateFromParams();

	return result;
}

void UFOMapNode::updateFromParams()
{
	std::visit(
	    [this](auto &map) {
		    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
			    map.setProbHit(this->get_parameter("prob_hit").as_double());
			    map.setProbMiss(this->get_parameter("prob_miss").as_double());
			    map.setClampingThresMin(this->get_parameter("clamping_thres_min").as_double());
			    map.setClampingThresMax(this->get_parameter("clamping_thres_max").as_double());
		    }
	    },
	    map_);

	transform_timeout_ = rclcpp::Duration::from_seconds(
	    this->get_parameter("transform_timeout").as_double());

	// Set up publisher
	const int new_map_queue_size_ = this->get_parameter("map_queue_size").as_int();
	if (map_pub_.empty() || map_queue_size_ != new_map_queue_size_) {
		map_pub_.resize(publish_depth_ + 1);
		for (int i = 0; i < map_pub_.size(); ++i) {
			map_queue_size_ = new_map_queue_size_;
			std::string final_topic = i == 0 ? "map" : "map_depth_" + std::to_string(i);
			map_pub_[i] = this->create_publisher<ufomap_msgs::msg::UFOMapStamped>(
			    final_topic, map_queue_size_);
			// map_pub_[i] = nh_priv_.advertise<ufomap_msgs::msg::UFOMapStamped>(
			//     final_topic, map_queue_size_,
			//     boost::bind(&UFOMapNode::mapConnectCallback, this, _1, i),
			//     ros::SubscriberStatusCallback(), ros::VoidConstPtr(), new_map_latch_);
		}
	}

	// Set up subscriber
	const int new_cloud_in_queue_size_ =
	    this->get_parameter("cloud_in_queue_size").as_int();
	if (!cloud_sub_ || cloud_in_queue_size_ != new_cloud_in_queue_size_) {
		cloud_in_queue_size_ = new_cloud_in_queue_size_;
		cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		    "cloud_in", rclcpp::QoS(cloud_in_queue_size_),
		    std::bind(&UFOMapNode::cloudCallback, this, std::placeholders::_1));
	}

	// Set up timer
	const double new_pub_rate_ = this->get_parameter("pub_rate").as_double();
	if (!pub_timer_ || pub_rate_ != new_pub_rate_) {
		pub_rate_ = new_pub_rate_;
		if (0 < pub_rate_) {
			pub_timer_ = this->create_wall_timer(
			    std::chrono::milliseconds(static_cast<int>(1000 / pub_rate_)),
			    std::bind(&UFOMapNode::timerCallback, this));
		} else {
			pub_timer_ = nullptr;
		}
	}

	update_rate_ = rclcpp::Duration::from_seconds(
	    1.0 / this->get_parameter("update_rate").as_double());
}

}  // namespace ufomap_mapping
