/**
 * UFO ROS integration
 *
 * @author D. Duberg, KTH Royal Institute of Technology, Copyright (c) 2020.
 * @see https://github.com/UnknownFreeOccupied/ufomap_ros
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

// UFO ROS
#include <ufomap_ros/conversions.h>

// ROS
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ufomap_ros_conversions
{
void getFields(sensor_msgs::msg::PointCloud2 const& cloud, bool& has_x, bool& has_y,
               bool& has_z, bool& has_rgb)
{
	has_x = false;
	has_y = false;
	has_z = false;
	has_rgb = false;

	for (auto const& field : cloud.fields) {
		if ("x" == field.name) {
			has_x = true;
		} else if ("y" == field.name) {
			has_y = true;
		} else if ("z" == field.name) {
			has_z = true;
		} else if ("rgb" == field.name) {
			has_rgb = true;
		} else if ("r" == field.name) {
			has_rgb = true;
		} else if ("g" == field.name) {
			has_rgb = true;
		} else if ("b" == field.name) {
			has_rgb = true;
		}
	}
}

void rosToUfo(sensor_msgs::msg::PointCloud2 const& cloud_in,
              ufo::map::PointCloud& cloud_out)
{
	cloud_out.reserve(cloud_in.data.size() / cloud_in.point_step);

	bool has_x, has_y, has_z, has_rgb;
	getFields(cloud_in, has_x, has_y, has_z, has_rgb);

	if (!has_x || !has_y || !has_z) {
		throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	}

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");
	for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
		if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
			cloud_out.push_back(ufo::map::Point3(*iter_x, *iter_y, *iter_z));
		}
	}
}

void rosToUfo(sensor_msgs::msg::PointCloud2 const& cloud_in,
              ufo::map::PointCloudColor& cloud_out)
{
	cloud_out.reserve(cloud_in.data.size() / cloud_in.point_step);

	bool has_x, has_y, has_z, has_rgb;
	getFields(cloud_in, has_x, has_y, has_z, has_rgb);

	if (!has_x || !has_y || !has_z) {
		throw std::runtime_error("cloud_in missing one or more of the xyz fields");
	}

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");

	if (has_rgb) {
		sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(cloud_in, "r");
		sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(cloud_in, "g");
		sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(cloud_in, "b");

		for (; iter_x != iter_x.end();
		     ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
			if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z) &&
			    !std::isnan(*iter_r) && !std::isnan(*iter_g) && !std::isnan(*iter_b)) {
				cloud_out.push_back(
				    ufo::map::Point3Color(*iter_x, *iter_y, *iter_z, *iter_r, *iter_g, *iter_b));
			}
		}

	} else {
		// TODO: Should this throw?
		// throw std::runtime_error("cloud_in missing one or more of the rgb fields");

		for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
			if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
				cloud_out.push_back(ufo::map::Point3Color(*iter_x, *iter_y, *iter_z));
			}
		}
	}
}

void ufoToRos(ufo::map::PointCloud const& cloud_in,
              sensor_msgs::msg::PointCloud2& cloud_out)
{
	bool has_x, has_y, has_z, has_rgb;
	getFields(cloud_out, has_x, has_y, has_z, has_rgb);

	sensor_msgs::PointCloud2Modifier cloud_out_modifier(cloud_out);
	cloud_out_modifier.setPointCloud2FieldsByString(1, "xyz");
	cloud_out_modifier.resize(cloud_in.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_out, "z");

	for (size_t i = 0; i < cloud_in.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
		*iter_x = cloud_in[i][0];
		*iter_y = cloud_in[i][1];
		*iter_z = cloud_in[i][2];
	}
}

void ufoToRos(ufo::map::PointCloudColor const& cloud_in,
              sensor_msgs::msg::PointCloud2& cloud_out)
{
	bool has_x, has_y, has_z, has_rgb;
	getFields(cloud_out, has_x, has_y, has_z, has_rgb);

	sensor_msgs::PointCloud2Modifier cloud_out_modifier(cloud_out);
	cloud_out_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
	cloud_out_modifier.resize(cloud_in.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_out, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_out, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_out, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_out, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_out, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_out, "b");

	for (size_t i = 0; i < cloud_in.size();
	     ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
		*iter_x = cloud_in[i][0];
		*iter_y = cloud_in[i][1];
		*iter_z = cloud_in[i][2];
		*iter_r = cloud_in[i].getColor().r;
		*iter_g = cloud_in[i].getColor().g;
		*iter_b = cloud_in[i].getColor().b;
	}
}

// Vector3/Point

void rosToUfo(geometry_msgs::msg::Point const& point_in, ufo::math::Vector3& point_out)
{
	point_out.x() = point_in.x;
	point_out.y() = point_in.y;
	point_out.z() = point_in.z;
}

void rosToUfo(geometry_msgs::msg::Vector3 const& point_in, ufo::math::Vector3& point_out)
{
	point_out.x() = point_in.x;
	point_out.y() = point_in.y;
	point_out.z() = point_in.z;
}

ufo::math::Vector3 rosToUfo(geometry_msgs::msg::Point const& point)
{
	return ufo::math::Vector3(point.x, point.y, point.z);
}

void ufoToRos(ufo::math::Vector3 const& point_in, geometry_msgs::msg::Point& point_out)
{
	point_out.x = point_in.x();
	point_out.y = point_in.y();
	point_out.z = point_in.z();
}

void ufoToRos(ufo::math::Vector3 const& point_in, geometry_msgs::msg::Vector3& point_out)
{
	point_out.x = point_in.x();
	point_out.y = point_in.y();
	point_out.z = point_in.z();
}

geometry_msgs::msg::Point ufoToRos(ufo::math::Vector3 const& point)
{
	geometry_msgs::msg::Point point_out;
	ufoToRos(point, point_out);
	return point_out;
}

// Quaternion
void rosToUfo(geometry_msgs::msg::Quaternion const& quaternion_in,
              ufo::math::Quaternion& quaternion_out)
{
	quaternion_out.x() = quaternion_in.x;
	quaternion_out.y() = quaternion_in.y;
	quaternion_out.z() = quaternion_in.z;
	quaternion_out.w() = quaternion_in.w;
}

ufo::math::Quaternion rosToUfo(geometry_msgs::msg::Quaternion const& quaternion)
{
	return ufo::math::Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
}

void ufoToRos(ufo::math::Quaternion const& quaternion_in,
              geometry_msgs::msg::Quaternion& quaternion_out)
{
	quaternion_out.x = quaternion_in.x();
	quaternion_out.y = quaternion_in.y();
	quaternion_out.z = quaternion_in.z();
	quaternion_out.w = quaternion_in.w();
}

geometry_msgs::msg::Quaternion ufoToRos(ufo::math::Quaternion const& quaternion)
{
	geometry_msgs::msg::Quaternion quaternion_out;
	ufoToRos(quaternion, quaternion_out);
	return quaternion_out;
}

// Transforms
void rosToUfo(geometry_msgs::msg::Transform const& transform_in,
              ufo::math::Pose6& transform_out)
{
	rosToUfo(transform_in.translation, transform_out.translation());
	rosToUfo(transform_in.rotation, transform_out.rotation());
}

ufo::math::Pose6 rosToUfo(geometry_msgs::msg::Transform const& transform)
{
	return ufo::math::Pose6(transform.translation.x, transform.translation.y,
	                        transform.translation.z, transform.rotation.w,
	                        transform.rotation.x, transform.rotation.y,
	                        transform.rotation.z);
}

void ufoToRos(ufo::math::Pose6 const& transform_in,
              geometry_msgs::msg::Transform& transform_out)
{
	ufoToRos(transform_in.translation(), transform_out.translation);
	ufoToRos(transform_in.rotation(), transform_out.rotation);
}

geometry_msgs::msg::Transform ufoToRos(ufo::math::Pose6 const& transform)
{
	geometry_msgs::msg::Transform transform_out;
	ufoToRos(transform, transform_out);
	return transform_out;
}

ufo::geometry::Point msgToUfo(geometry_msgs::msg::Point const& point)
{
	return ufo::geometry::Point(point.x, point.y, point.z);
}

ufo::geometry::AABB msgToUfo(ufomap_msgs::msg::AABB const& aabb)
{
	ufo::geometry::AABB a;
	a.center = msgToUfo(aabb.center);
	a.half_size = msgToUfo(aabb.half_size);
	return a;
}

ufo::geometry::Plane msgToUfo(ufomap_msgs::msg::Plane const& plane)
{
	return ufo::geometry::Plane(msgToUfo(plane.normal), plane.distance);
}

ufo::geometry::Frustum msgToUfo(ufomap_msgs::msg::Frustum const& frustum)
{
	ufo::geometry::Frustum f;
	for (size_t i = 0; i < frustum.planes.size(); ++i) {
		f.planes[i] = msgToUfo(frustum.planes[i]);
	}
	return f;
}

ufo::geometry::LineSegment msgToUfo(ufomap_msgs::msg::LineSegment const& line_segment)
{
	return ufo::geometry::LineSegment(msgToUfo(line_segment.start),
	                                  msgToUfo(line_segment.end));
}

ufo::geometry::OBB msgToUfo(ufomap_msgs::msg::OBB const& obb)
{
	return ufo::geometry::OBB(msgToUfo(obb.center), msgToUfo(obb.half_size),
	                          msgToUfo(obb.rotation));
}

ufo::geometry::Ray msgToUfo(ufomap_msgs::msg::Ray const& ray)
{
	return ufo::geometry::Ray(msgToUfo(ray.origin), msgToUfo(ray.direction));
}

ufo::geometry::Sphere msgToUfo(ufomap_msgs::msg::Sphere const& sphere)
{
	return ufo::geometry::Sphere(msgToUfo(sphere.center), sphere.radius);
}

ufo::geometry::BoundingVolume msgToUfo(ufomap_msgs::msg::BoundingVolume const& msg)
{
	ufo::geometry::BoundingVolume bv;
	for (ufomap_msgs::msg::AABB const& aabb : msg.aabbs) {
		bv.add(msgToUfo(aabb));
	}
	for (ufomap_msgs::msg::Frustum const& frustum : msg.frustums) {
		bv.add(msgToUfo(frustum));
	}
	for (ufomap_msgs::msg::LineSegment const& line_segment : msg.line_segments) {
		bv.add(msgToUfo(line_segment));
	}
	for (ufomap_msgs::msg::OBB const& obb : msg.obbs) {
		bv.add(msgToUfo(obb));
	}
	for (ufomap_msgs::msg::Plane const& plane : msg.planes) {
		bv.add(msgToUfo(plane));
	}
	for (geometry_msgs::msg::Point const& point : msg.points) {
		bv.add(msgToUfo(point));
	}
	for (ufomap_msgs::msg::Ray const& ray : msg.rays) {
		bv.add(msgToUfo(ray));
	}
	for (ufomap_msgs::msg::Sphere const& sphere : msg.spheres) {
		bv.add(msgToUfo(sphere));
	}
	return bv;
}

geometry_msgs::msg::Point ufoToMsg(ufo::geometry::Point const& point)
{
	geometry_msgs::msg::Point msg;
	msg.x = point.x();
	msg.y = point.y();
	msg.z = point.z();
	return msg;
}

ufomap_msgs::msg::AABB ufoToMsg(ufo::geometry::AABB const& aabb)
{
	ufomap_msgs::msg::AABB msg;
	msg.center = ufoToMsg(aabb.center);
	msg.half_size = ufoToMsg(aabb.half_size);
	return msg;
}

ufomap_msgs::msg::Plane ufoToMsg(ufo::geometry::Plane const& plane)
{
	ufomap_msgs::msg::Plane msg;
	msg.normal = ufoToMsg(plane.normal);
	msg.distance = plane.distance;
	return msg;
}

ufomap_msgs::msg::Frustum ufoToMsg(ufo::geometry::Frustum const& frustum)
{
	ufomap_msgs::msg::Frustum msg;
	for (size_t i = 0; i < msg.planes.size(); ++i) {
		msg.planes[i] = ufoToMsg(frustum.planes[i]);
	}
	return msg;
}

ufomap_msgs::msg::LineSegment ufoToMsg(ufo::geometry::LineSegment const& line_segment)
{
	ufomap_msgs::msg::LineSegment msg;
	msg.start = ufoToMsg(line_segment.start);
	msg.end = ufoToMsg(line_segment.end);
	return msg;
}

ufomap_msgs::msg::OBB ufoToMsg(ufo::geometry::OBB const& obb)
{
	ufomap_msgs::msg::OBB msg;
	msg.center = ufoToMsg(obb.center);
	msg.half_size = ufoToMsg(obb.half_size);
	// TODO: Fix
	// msg.rotation = ufoToMsg(obb.rotation);
	return msg;
}

ufomap_msgs::msg::Ray ufoToMsg(ufo::geometry::Ray const& ray)
{
	ufomap_msgs::msg::Ray msg;
	msg.origin = ufoToMsg(ray.origin);
	msg.direction = ufoToMsg(ray.direction);
	return msg;
}

ufomap_msgs::msg::Sphere ufoToMsg(ufo::geometry::Sphere const& sphere)
{
	ufomap_msgs::msg::Sphere msg;
	msg.center = ufoToMsg(sphere.center);
	msg.radius = sphere.radius;
	return msg;
}

ufomap_msgs::msg::BoundingVolume ufoToMsg(
    ufo::geometry::BoundingVolume const& bounding_volume)
{
	ufomap_msgs::msg::BoundingVolume msg;
	for (ufo::geometry::BoundingVar const& bv : bounding_volume) {
		std::visit(
		    [&msg](auto&& arg) -> void {
			    using T = std::decay_t<decltype(arg)>;
			    if constexpr (std::is_same_v<T, ufo::geometry::AABB>) {
				    msg.aabbs.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Frustum>) {
				    msg.frustums.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::LineSegment>) {
				    msg.line_segments.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::OBB>) {
				    msg.obbs.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Plane>) {
				    msg.planes.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::math::Vector3>) {
				    msg.points.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Ray>) {
				    msg.rays.push_back(ufoToMsg(arg));
			    } else if constexpr (std::is_same_v<T, ufo::geometry::Sphere>) {
				    msg.spheres.push_back(ufoToMsg(arg));
			    }
		    },
		    bv);
	}
	return msg;
}

}  // namespace ufomap_ros