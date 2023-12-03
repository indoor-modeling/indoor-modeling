/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: ipa_room_segmentation
 *
 * \author
 * Author: Florian Jordan
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 08.2015
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include "room_segmentation_server.h"

#include "../RoomSegentation/common/include/meanshift2d.h"


#include <boost/algorithm/string.hpp>

static bool DEBUG_DISPLAYS=false;

RoomSegmentationServer::RoomSegmentationServer(int room_seg_algorithm, std::string path, float map_resolution, float x, float y)
{
	// parameters to check if the algorithms need to be trained (not part of dynamic reconfigure)
	train_semantic_ = false;
	std::cout << "room_segmentation/train_semantic_ = " << train_semantic_ << std::endl;
	load_semantic_features_ = false;
	std::cout << "room_segmentation/load_semantic_features = " << load_semantic_features_ << std::endl;
	train_vrf_ = false;
	std::cout << "room_segmentation/train_vrf_ = " << train_vrf_ << std::endl;


	// parameters
	std::cout << "\n------------------------------------\nRoom Segmentation Parameters:\n------------------------------------\n";
	room_segmentation_algorithm_ = room_seg_algorithm;
	std::cout << "room_segmentation/room_segmentation_algorithm = " << room_segmentation_algorithm_ << std::endl << std::endl;
	if (room_segmentation_algorithm_ == 1)
		std::cout << ("You have chosen the morphological segmentation method.") << std::endl;
	else if (room_segmentation_algorithm_ == 2)
		ROS_INFO("You have chosen the distance segmentation method.");
	else if (room_segmentation_algorithm_ == 3)
		ROS_INFO("You have chosen the voronoi segmentation method.");
	else if (room_segmentation_algorithm_ == 4)
		ROS_INFO("You have chosen the semantic segmentation method.");
	else if (room_segmentation_algorithm_ == 5)
		ROS_INFO("You have chosen the voronoi random field segmentation method.");
	std::cout << std::endl;

	if (room_segmentation_algorithm_ == 1) //set morphological parameters
	{
		room_upper_limit_morphological_ = 200;//47
		std::cout << "room_segmentation/room_area_factor_upper_limit = " << room_upper_limit_morphological_ << std::endl;
		room_lower_limit_morphological_ = 1.9;//1.6//20
		std::cout << "room_segmentation/room_area_factor_lower_limit = " << room_lower_limit_morphological_ << std::endl;
	}
	if (room_segmentation_algorithm_ == 2) //set distance parameters
	{
		room_upper_limit_distance_ = 163.0;
		std::cout << "room_segmentation/room_area_factor_upper_limit = " << room_upper_limit_distance_ << std::endl;
		room_lower_limit_distance_ = 0.35;
		std::cout << "room_segmentation/room_area_factor_lower_limit = " << room_lower_limit_distance_ << std::endl;
	}
	if (room_segmentation_algorithm_ == 99) //set passthrough parameters
	{
		room_upper_limit_passthrough_ = 10000000.0;
		std::cout << "room_segmentation/room_area_factor_upper_limit = " << room_upper_limit_passthrough_ << std::endl;
		room_lower_limit_passthrough_ = 0.0;
		std::cout << "room_segmentation/room_area_factor_lower_limit = " << room_lower_limit_passthrough_ << std::endl;
	}

	map_path = path;
	_map_resolution = map_resolution;
	map_origin = cv::Point2d(x, y);
	display_segmented_map_ = true;
	publish_segmented_map_ = false;
	return_format_in_pixel = false;
	return_format_in_meter = false;
	std::cout << "room_segmentation/display_segmented_map_ = " << display_segmented_map_ << std::endl;
	/*node_handle_.param("publish_segmented_map", publish_segmented_map_, false);
	std::cout << "room_segmentation/publish_segmented_map_ = " << publish_segmented_map_ << std::endl;*/

}


void RoomSegmentationServer::execute_segmentation_server()
{
	// override pre-set segmentation algorithm on request
	const int stored_room_segmentation_algorithm = room_segmentation_algorithm_;

	ROS_INFO("*****Segmentation action server*****");
	printf("map resolution is : %f\n", _map_resolution);
	printf("segmentation algorithm: %d\n", room_segmentation_algorithm_);

	//converting the map msg in cv format
	cv::Mat original_img = cv::imread(map_path, 0);
	cv::imshow("sda", original_img);
	cv::waitKey();
	//set the resolution and the limits for the actual goal and the Map origin
	const float map_resolution = _map_resolution;
	map_origin = cv::Point2d(map_origin_position_x, map_origin_position_y);

	//segment the given map
	cv::Mat segmented_map;
	if (room_segmentation_algorithm_ == 1)
	{
		MorphologicalSegmentation morphological_segmentation; //morphological segmentation method
		morphological_segmentation.segmentMap(original_img, segmented_map, map_resolution, room_lower_limit_morphological_, room_upper_limit_morphological_);
	}
	else if (room_segmentation_algorithm_ == 2)
	{
		DistanceSegmentation distance_segmentation; //distance segmentation method
		distance_segmentation.segmentMap(original_img, segmented_map, map_resolution, room_lower_limit_distance_, room_upper_limit_distance_);
	}
	else if (room_segmentation_algorithm_ == 99)
	{
		// pass through segmentation: takes a map which is already separated into unconnected areas and returns these as the resulting segmentation in the format of this program
		// todo: closing operation explicitly for bad maps --> needs parameterization
		//original_img.convertTo(segmented_map, CV_32SC1, 256, 0);		// occupied space = 0, free space = 65280
		cv::Mat original_img_eroded, temp;
		cv::erode(original_img, temp, cv::Mat(), cv::Point(-1, -1), 3);
		cv::dilate(temp, original_img_eroded, cv::Mat(), cv::Point(-1, -1), 3);
		original_img_eroded.convertTo(segmented_map, CV_32SC1, 256, 0);		// occupied space = 0, free space = 65280
		int label_index = 1;

//		cv::imshow("original_img", original_img_eroded);
//		cv::waitKey();

		for (int y = 0; y < segmented_map.rows; y++)
		{
			for (int x = 0; x < segmented_map.cols; x++)
			{
				// if original map is occupied space here or if the segmented map has already received a label for that cell --> skip
				if (original_img_eroded.at<uchar>(y,x) != 255 || segmented_map.at<int>(y,x)!=65280)
					continue;

				// fill each room area with a unique id
				cv::Rect rect;
				cv::floodFill(segmented_map, cv::Point(x,y), label_index, &rect, 0, 0, 4);

				// determine filled area
				double area = 0;
				for (int v = rect.y; v < segmented_map.rows; v++)
					for (int u = rect.x; u < segmented_map.cols; u++)
						if (segmented_map.at<int>(v,u)==label_index)
							area += 1.;
				area = map_resolution * map_resolution * area;	// convert from cells to m^2

				// exclude too small and too big rooms
				if (area < room_lower_limit_passthrough_ || area > room_upper_limit_passthrough_)
				{
					for (int v = rect.y; v < segmented_map.rows; v++)
						for (int u = rect.x; u < segmented_map.cols; u++)
							if (segmented_map.at<int>(v,u)==label_index)
								segmented_map.at<int>(v,u) = 0;
				}
				else
					label_index++;
			}
		}
		std::cout << "Labeled " << label_index-1 << " segments." << std::endl;
	}
	else
	{
		ROS_INFO("Undefined algorithm selected.");
		room_segmentation_algorithm_ = stored_room_segmentation_algorithm;
		return;
	}

	ROS_INFO("********Segmented the map************");
	//	looping_rate.sleep();

	// get the min/max-values and the room-centers
	// compute room label codebook
	std::map<int, size_t> label_vector_index_codebook; // maps each room label to a position in the rooms vector
	size_t vector_index = 0;
	for (int v = 0; v < segmented_map.rows; ++v)
	{
		for (int u = 0; u < segmented_map.cols; ++u)
		{
			const int label = segmented_map.at<int>(v, u);
			if (label > 0 && label < 65280) // do not count walls/obstacles or free space as label
			{
				if (label_vector_index_codebook.find(label) == label_vector_index_codebook.end())
				{
					label_vector_index_codebook[label] = vector_index;
					vector_index++;
				}
			}
		}
	}
	//min/max y/x-values vector for each room. Initialized with extreme values
	std::vector<int> min_x_value_of_the_room(label_vector_index_codebook.size(), 100000000);
	std::vector<int> max_x_value_of_the_room(label_vector_index_codebook.size(), 0);
	std::vector<int> min_y_value_of_the_room(label_vector_index_codebook.size(), 100000000);
	std::vector<int> max_y_value_of_the_room(label_vector_index_codebook.size(), 0);
	//vector of the central Point for each room, initially filled with Points out of the map
	std::vector<int> room_centers_x_values(label_vector_index_codebook.size(), -1);
	std::vector<int> room_centers_y_values(label_vector_index_codebook.size(), -1);
	//***********************Find min/max x and y coordinate and center of each found room********************
	//check y/x-value for every Pixel and make the larger/smaller value to the current value of the room
	for (int y = 0; y < segmented_map.rows; ++y)
	{
		for (int x = 0; x < segmented_map.cols; ++x)
		{
			const int label = segmented_map.at<int>(y, x);
			if (label > 0 && label < 65280) //if Pixel is white or black it is no room --> doesn't need to be checked
			{
				const int index = label_vector_index_codebook[label];
				min_x_value_of_the_room[index] = std::min(x, min_x_value_of_the_room[index]);
				max_x_value_of_the_room[index] = std::max(x, max_x_value_of_the_room[index]);
				max_y_value_of_the_room[index] = std::max(y, max_y_value_of_the_room[index]);
				min_y_value_of_the_room[index] = std::min(y, min_y_value_of_the_room[index]);
			}
		}
	}
	//get centers for each room
//	for (size_t idx = 0; idx < room_centers_x_values.size(); ++idx)
//	{
//		if (max_x_value_of_the_room[idx] != 0 && max_y_value_of_the_room[idx] != 0 && min_x_value_of_the_room[idx] != 100000000 && min_y_value_of_the_room[idx] != 100000000)
//		{
//			room_centers_x_values[idx] = (min_x_value_of_the_room[idx] + max_x_value_of_the_room[idx]) / 2;
//			room_centers_y_values[idx] = (min_y_value_of_the_room[idx] + max_y_value_of_the_room[idx]) / 2;
//			cv::circle(segmented_map, cv::Point(room_centers_x_values[idx], room_centers_y_values[idx]), 2, cv::Scalar(200*256), CV_FILLED);
//		}
//	}
	// use distance transform and mean shift to find good room centers that are reachable by the robot
	// first check whether a robot radius shall be applied to obstacles in order to exclude room center points that are not reachable by the robot
	cv::Mat segmented_map_copy = segmented_map;
	cv::Mat connection_to_other_rooms = cv::Mat::zeros(segmented_map.rows, segmented_map.cols, CV_8UC1);	// stores for each pixel whether a path to another rooms exists for a robot of size robot_radius
	if (robot_radius > 0.0)
	{
		// consider robot radius for exclusion of non-reachable points
		segmented_map_copy = segmented_map.clone();
		cv::Mat map_8u, eroded_map;
		segmented_map_copy.convertTo(map_8u, CV_8UC1, 1., 0.);
		int number_of_erosions = (robot_radius / map_resolution);
		cv::erode(map_8u, eroded_map, cv::Mat(), cv::Point(-1, -1), number_of_erosions);
		for (int v=0; v<segmented_map_copy.rows; ++v)
			for (int u=0; u<segmented_map_copy.cols; ++u)
				if (eroded_map.at<uchar>(v,u) == 0)
					segmented_map_copy.at<int>(v,u) = 0;

		// compute connectivity of remaining accessible room cells to other rooms
		bool stop = false;
		while (stop == false)
		{
			stop = true;
			for (int v=1; v<segmented_map_copy.rows-1; ++v)
			{
				for (int u=1; u<segmented_map_copy.cols-1; ++u)
				{
					// skip already identified cells
					if (connection_to_other_rooms.at<uchar>(v,u) != 0)
						continue;

					// only consider cells labeled as a room
					const int label = segmented_map_copy.at<int>(v,u);
					if (label <= 0 || label >= 65280)
						continue;

					for (int dv=-1; dv<=1; ++dv)
					{
						for (int du=-1; du<=1; ++du)
						{
							if (dv==0 && du==0)
								continue;
							const int neighbor_label = segmented_map_copy.at<int>(v+dv,u+du);
							if (neighbor_label>0 && neighbor_label<65280 && (neighbor_label!=label || (neighbor_label==label && connection_to_other_rooms.at<uchar>(v+dv,u+du)==255)))
							{
								// either the room cell has a direct border to a different room or the room cell has a neighbor from the same room label with a connecting path to another room
								connection_to_other_rooms.at<uchar>(v,u) = 255;
								stop = false;
							}
						}
					}
				}
			}
		}
	}
	// compute the room centers
	MeanShift2D ms;
	for (std::map<int, size_t>::iterator it = label_vector_index_codebook.begin(); it != label_vector_index_codebook.end(); ++it)
	{
		int trial = 1; 	// use robot_radius to avoid room centers that are not accessible by a robot with a given radius
		if (robot_radius <= 0.)
			trial = 2;

		for (; trial <= 2; ++trial)
		{
			// compute distance transform for each room on the room cells that have some connection to another room (trial 1) or just on all cells of that room (trial 2)
			const int label = it->first;
			int number_room_pixels = 0;
			cv::Mat room = cv::Mat::zeros(segmented_map_copy.rows, segmented_map_copy.cols, CV_8UC1);
			for (int v = 0; v < segmented_map_copy.rows; ++v)
				for (int u = 0; u < segmented_map_copy.cols; ++u)
					if (segmented_map_copy.at<int>(v, u) == label && (trial==2 || connection_to_other_rooms.at<uchar>(v,u)==255))
					{
						room.at<uchar>(v, u) = 255;
						++number_room_pixels;
					}
			if (number_room_pixels == 0)
				continue;
			cv::Mat distance_map; //variable for the distance-transformed map, type: CV_32FC1
			cv::distanceTransform(room, distance_map, CV_DIST_L2, 5);
			// find point set with largest distance to obstacles
			double min_val = 0., max_val = 0.;
			cv::minMaxLoc(distance_map, &min_val, &max_val);
			std::vector<cv::Vec2d> room_cells;
			for (int v = 0; v < distance_map.rows; ++v)
				for (int u = 0; u < distance_map.cols; ++u)
					if (distance_map.at<float>(v, u) > max_val * 0.95f)
						room_cells.push_back(cv::Vec2d(u, v));
			if (room_cells.size()==0)
				continue;
			// use meanshift to find the modes in that set
			cv::Vec2d room_center = ms.findRoomCenter(room, room_cells, map_resolution);
			const int index = it->second;
			room_centers_x_values[index] = room_center[0];
			room_centers_y_values[index] = room_center[1];

			if (room_cells.size() > 0)
				break;
		}
	}

	// convert the segmented map into an indexed map which labels the segments with consecutive numbers (instead of arbitrary unordered labels in segmented map)
	cv::Mat indexed_map = segmented_map.clone();
	for (int y = 0; y < segmented_map.rows; ++y)
	{
		for (int x = 0; x < segmented_map.cols; ++x)
		{
			const int label = segmented_map.at<int>(y,x);
			if (label > 0 && label < 65280)
				indexed_map.at<int>(y,x) = label_vector_index_codebook[label]+1;//start value from 1 --> 0 is reserved for obstacles
		}
	}

	if (display_segmented_map_ == true)
	{
		// colorize the segmented map with the indices of the room_center vector
		cv::Mat color_segmented_map = indexed_map.clone();
		color_segmented_map.convertTo(color_segmented_map, CV_8U);
		cv::cvtColor(color_segmented_map, color_segmented_map, CV_GRAY2BGR);
		for(size_t i = 1; i <= room_centers_x_values.size(); ++i)
		{
			//choose random color for each room
			const cv::Vec3b color((rand() % 250) + 1, (rand() % 250) + 1, (rand() % 250) + 1);
			for(size_t v = 0; v < indexed_map.rows; ++v)
				for(size_t u = 0; u < indexed_map.cols; ++u)
					if(indexed_map.at<int>(v,u) == i)
						color_segmented_map.at<cv::Vec3b>(v,u) = color;
		}
//		cv::Mat disp = segmented_map.clone();
		for (size_t index = 0; index < room_centers_x_values.size(); ++index)
#if CV_MAJOR_VERSION<=3
			cv::circle(color_segmented_map, cv::Point(room_centers_x_values[index], room_centers_y_values[index]), 2, cv::Scalar(256), CV_FILLED);
#else
			//cv::circle(color_segmented_map, cv::Point(room_centers_x_values[index], room_centers_y_values[index]), 2, cv::Scalar(256), cv::FILLED);
#endif

		//cv::imwrite("area3_full_gray_seg_map.png", indexed_map);
		cv::imshow("segmentation", color_segmented_map);
		cv::waitKey();
		//cv::imwrite("area3_full_ori_seg_map.png", color_segmented_map);
	}

	//if (publish_segmented_map_ == true)
	//{
	//	// "colorize" the segmented map with gray scale values
	//	nav_msgs::OccupancyGrid segmented_grid;
	//	segmented_grid.header.stamp = ros::Time::now();
	//	segmented_grid.header.frame_id = "map";
	//	segmented_grid.info.resolution = map_resolution;
	//	segmented_grid.info.width = indexed_map.cols;
	//	segmented_grid.info.height = indexed_map.rows;
	//	segmented_grid.info.origin.position.x = map_origin.x;
	//	segmented_grid.info.origin.position.y = map_origin.y;
	//	segmented_grid.data.resize(segmented_grid.info.width*segmented_grid.info.height);
	//	std::map<int, int> colors;
	//	//choose random color for each room
	//	colors[0] = 0;
	//	for(int i = 1; i <= room_centers_x_values.size(); ++i)
	//		colors[i] = 20 + rand() % 81;
	//	int i=0;
	//	for(int v = 0; v < indexed_map.rows; ++v)
	//		for(int u = 0; u < indexed_map.cols; ++u, ++i)
	//			segmented_grid.data[i] = colors[indexed_map.at<int>(v,u)];
	//	map_pub_.publish(segmented_grid);
	//}

	////****************publish the results**********************
	//ipa_building_msgs::MapSegmentationResult action_result;
	////converting the cv format in map msg format
	//cv_bridge::CvImage cv_image;
	//cv_image.header.stamp = ros::Time::now();
	//cv_image.encoding = "32SC1";
	//cv_image.image = indexed_map;
	//cv_image.toImageMsg(action_result.segmented_map);

	////setting value to the action msgs to publish
	//action_result.map_resolution = goal->map_resolution;
	//action_result.map_origin = goal->map_origin;

	////setting massages in pixel value
	//action_result.room_information_in_pixel.clear();
	_room_information_in_pixel.clear();
	if (return_format_in_pixel == true)
	{
		std::vector<RoomInformation> room_information(room_centers_x_values.size());
		for (size_t i=0; i<room_centers_x_values.size(); ++i)
		{
			room_information[i].room_center.x = room_centers_x_values[i];
			room_information[i].room_center.y = room_centers_y_values[i];
			room_information[i].room_min_max.points.resize(2);
			room_information[i].room_min_max.points[0].x = min_x_value_of_the_room[i];
			room_information[i].room_min_max.points[0].y = min_y_value_of_the_room[i];
			room_information[i].room_min_max.points[1].x = max_x_value_of_the_room[i];
			room_information[i].room_min_max.points[1].y = max_y_value_of_the_room[i];
		}
		_room_information_in_pixel = room_information;

		// returning doorway points if the vector is not empty
		/*if(doorway_points_.empty() == false)
		{
			std::vector<cv::Point2d> found_doorway_points(doorway_points_.size());
			for(size_t i = 0; i < doorway_points_.size(); ++i)
			{
				found_doorway_points[i].x = doorway_points_[i].x;
				found_doorway_points[i].y = doorway_points_[i].y;
			}
			doorway_points_.clear();

			action_result.doorway_points = found_doorway_points;
		}*/
	}
	//setting messages in meter
	_room_information_in_meter.clear();
	if (return_format_in_meter == true)
	{
		std::vector<RoomInformation> room_information(room_centers_x_values.size());
		for (size_t i=0; i<room_centers_x_values.size(); ++i)
		{
			room_information[i].room_center.x = convert_pixel_to_meter_for_x_coordinate(room_centers_x_values[i], map_resolution, map_origin);
			room_information[i].room_center.y = convert_pixel_to_meter_for_y_coordinate(room_centers_y_values[i], map_resolution, map_origin);
			room_information[i].room_min_max.points.resize(2);
			room_information[i].room_min_max.points[0].x = convert_pixel_to_meter_for_x_coordinate(min_x_value_of_the_room[i], map_resolution, map_origin);
			room_information[i].room_min_max.points[0].y = convert_pixel_to_meter_for_y_coordinate(min_y_value_of_the_room[i], map_resolution, map_origin);
			room_information[i].room_min_max.points[1].x = convert_pixel_to_meter_for_x_coordinate(max_x_value_of_the_room[i], map_resolution, map_origin);
			room_information[i].room_min_max.points[1].y = convert_pixel_to_meter_for_y_coordinate(max_y_value_of_the_room[i], map_resolution, map_origin);
		}
		_room_information_in_meter = room_information;

		// returning doorway points if the vector is not empty
		/*if(doorway_points_.empty() == false)
		{
			std::vector<geometry_msgs::Point32> found_doorway_points(doorway_points_.size());
			for(size_t i = 0; i < doorway_points_.size(); ++i)
			{
				found_doorway_points[i].x = convert_pixel_to_meter_for_x_coordinate(doorway_points_[i].x, map_resolution, map_origin);;
				found_doorway_points[i].y = convert_pixel_to_meter_for_y_coordinate(doorway_points_[i].y, map_resolution, map_origin);
			}
			doorway_points_.clear();

			action_result.doorway_points = found_doorway_points;
		}*/
	}

	ROS_INFO("********Map segmentation finished************");
}

bool RoomSegmentationServer::extractAreaMapFromLabeledMap(int _segment_of_interest)
{
	// convert the Image msg in cv format
	cv::Mat segmented_map = cv::imread(map_path);

	// create a new map that only contains the segment with the label of interest
	cv::Mat segmented_area = cv::Mat::zeros(segmented_map.rows, segmented_map.cols, CV_8UC1);
	const int segment_of_interest = _segment_of_interest;
	for (int v=0; v<segmented_map.rows; ++v)
	{
		for (int u=0; u<segmented_map.cols; ++u)
		{
			if (segmented_map.at<int>(v,u)==segment_of_interest)
			{
				segmented_area.at<uchar>(v,u) == 255;
			}
		}
	}

	// convert the cv format in Image msg format
	cv::imshow("interest map", segmented_area);

	return true;
}

int main(int argc, char** argv)
{
	//std::string path = "./common/files/my_maps/grid_with_door2.png";
	/*std::string path = "D:/dev/室内点云分割/CODE/data/room_map_chen2.png";
	std::string wall_path = "D:/dev/室内点云分割/CODE/data/wall_map_multi2.png";*/
	
	///*std::string path = "D:/dev/室内点云分割/CODE/area5/room_map.png";
	//std::string wall_path = "D:/dev/室内点云分割/CODE/area5/wall_map.png";

	//std::string path = "D:/dev/室内点云分割/CODE/unbalance_small_scene/room_map.png";
	//std::string wall_path = "D:/dev/室内点云分割/CODE/unbalance_small_scene/room_map_slice.png";

	//std::string path = "D:/dev/室内点云分割/CODE/area5_full/room_map.png";
	//std::string wall_path = "D:/dev/室内点云分割/CODE/area5_full/wall_map.png";

	//std::string path = "D:/dev/室内点云分割/CODE/area4/room_map.png";
	//std::string wall_path = "D:/dev/室内点云分割/CODE/area4/wall_map.png";

	//std::string path = "D:/dev/室内点云分割/CODE/area3/room_map.png";
	//std::string wall_path = "D:/dev/室内点云分割/CODE/area3/wall_map.png";

	//cv::Mat room = cv::imread(path, 0);
	//cv::Mat wall = cv::imread(wall_path, 0);
	//cv::Mat in = room - wall;
	//cv::imshow("sads", in);
	//cv::waitKey();
	//cv::imwrite("area_3_minus.png", in);

	std::string path = "area_3_minus.png";
	RoomSegmentationServer segmentationAlgorithmObj(1, path, 0.2, 0, 0);
	ROS_INFO("Action Server for room segmentation has been initialized......");
	segmentationAlgorithmObj.execute_segmentation_server();

	return 0;
}
