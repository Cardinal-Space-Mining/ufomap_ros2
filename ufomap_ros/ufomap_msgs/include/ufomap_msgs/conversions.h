/**
 * UFOMap ROS message conversions
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufo_ros
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

#ifndef UFOMAP_ROS_MSGS_CONVERSIONS_H
#define UFOMAP_ROS_MSGS_CONVERSIONS_H

// UFO
#include <ufo/geometry/aabb.h>
#include <ufo/geometry/bounding_volume.h>
#include <ufo/geometry/frustum.h>
#include <ufo/geometry/line_segment.h>
#include <ufo/geometry/obb.h>
#include <ufo/geometry/plane.h>
#include <ufo/geometry/point.h>
#include <ufo/geometry/ray.h>
#include <ufo/geometry/sphere.h>

// ROS msg
#include <geometry_msgs/msg/point.hpp>

// UFO msg
#include <ufomap_msgs/msg/aabb.hpp>
#include <ufomap_msgs/msg/bounding_volume.hpp>
#include <ufomap_msgs/msg/frustum.hpp>
#include <ufomap_msgs/msg/line_segment.hpp>
#include <ufomap_msgs/msg/obb.hpp>
#include <ufomap_msgs/msg/plane.hpp>
#include <ufomap_msgs/msg/ray.hpp>
#include <ufomap_msgs/msg/sphere.hpp>
#include <ufomap_msgs/msg/ufo_map.hpp>

// STD
#include <type_traits>

namespace ufomap_msgs
{
//
// ROS message type to UFO type
//

ufo::geometry::Point msgToUfo(geometry_msgs::msg::Point const& point);

ufo::geometry::AABB msgToUfo(ufomap_msgs::msg::AABB const& aabb);

ufo::geometry::Plane msgToUfo(ufomap_msgs::msg::Plane const& plane);

ufo::geometry::Frustum msgToUfo(ufomap_msgs::msg::Frustum const& frustum);

ufo::geometry::LineSegment msgToUfo(ufomap_msgs::msg::LineSegment const& line_segment);

ufo::geometry::OBB msgToUfo(ufomap_msgs::msg::OBB const& obb);

ufo::geometry::Ray msgToUfo(ufomap_msgs::msg::Ray const& ray);

ufo::geometry::Sphere msgToUfo(ufomap_msgs::msg::Sphere const& sphere);

ufo::geometry::BoundingVolume msgToUfo(ufomap_msgs::msg::BoundingVolume const& msg);

//
// UFO type to ROS message type
//

geometry_msgs::msg::Point ufoToMsg(ufo::geometry::Point const& point);

ufomap_msgs::msg::AABB ufoToMsg(ufo::geometry::AABB const& aabb);

ufomap_msgs::msg::Plane ufoToMsg(ufo::geometry::Plane const& plane);

ufomap_msgs::msg::Frustum ufoToMsg(ufo::geometry::Frustum const& frustum);

ufomap_msgs::msg::LineSegment ufoToMsg(ufo::geometry::LineSegment const& line_segment);

ufomap_msgs::msg::OBB ufoToMsg(ufo::geometry::OBB const& obb);

ufomap_msgs::msg::Ray ufoToMsg(ufo::geometry::Ray const& ray);

ufomap_msgs::msg::Sphere ufoToMsg(ufo::geometry::Sphere const& sphere);

ufomap_msgs::msg::BoundingVolume ufoToMsg(
    ufo::geometry::BoundingVolume const& bounding_volume);

//
// ROS message type to UFO type
//

template <typename TreeType>
bool msgToUfo(ufomap_msgs::msg::UFOMap const& msg, TreeType& tree)
{
	std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
	                              std::ios_base::binary);
	if (!msg.data.empty()) {
		data_stream.write((char const*)&msg.data[0], msg.data.size());
		return tree.readData(data_stream, msgToUfo(msg.info.bounding_volume),
		                     msg.info.resolution, msg.info.depth_levels,
		                     msg.info.uncompressed_data_size, msg.info.compressed);
	}
	return false;
}

//
// UFO type to ROS message type
//

template <typename TreeType>
bool ufoToMsg(TreeType const& tree, ufomap_msgs::msg::UFOMap& msg, bool compress = false,
              unsigned int depth = 0, int compression_acceleration_level = 1,
              int compression_level = 0)
{
	return ufoToMsg(tree, msg, ufo::geometry::BoundingVolume(), compress, depth,
	                compression_acceleration_level, compression_level);
}

template <typename TreeType, typename BoundingType>
bool ufoToMsg(TreeType const& tree, ufomap_msgs::msg::UFOMap& msg,
              BoundingType const& bounding_volume, bool compress = false,
              unsigned int depth = 0, int compression_acceleration_level = 1,
              int compression_level = 0)
{
	ufo::geometry::BoundingVolume bv;
	bv.add(bounding_volume);
	return ufoToMsg(tree, msg, bv, compress, depth, compression_acceleration_level,
	                compression_level);
}

template <typename TreeType>
bool ufoToMsg(TreeType const& tree, ufomap_msgs::msg::UFOMap& msg,
              ufo::geometry::BoundingVolume const& bounding_volume, bool compress = false,
              unsigned int depth = 0, int compression_acceleration_level = 1,
              int compression_level = 0)
{
	msg.info.version = tree.getFileVersion();
	msg.info.id = tree.getTreeType();
	msg.info.resolution = tree.getResolution();
	msg.info.depth_levels = tree.getTreeDepthLevels();
	msg.info.compressed = compress;
	msg.info.bounding_volume = ufoToMsg(bounding_volume);

	std::stringstream data_stream(std::ios_base::in | std::ios_base::out |
	                              std::ios_base::binary);
	msg.info.uncompressed_data_size =
	    tree.writeData(data_stream, bounding_volume, compress, depth,
	                   compression_acceleration_level, compression_level);
	if (0 > msg.info.uncompressed_data_size) {
		return false;
	}

	std::string const& data_string = data_stream.str();
	msg.data = std::vector<int8_t>(data_string.begin(), data_string.end());
	return true;
}

}  // namespace ufomap_msgs

#endif  // UFOMAP_ROS_MSGS_CONVERSIONS_H