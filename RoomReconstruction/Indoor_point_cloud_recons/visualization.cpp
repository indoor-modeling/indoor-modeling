#include "visualization.h"


cv::Mat show_face_on_map(Face_handle f, cv::Mat map, pcInfo info)
{
	cv::Mat face_on_map(map.size(), CV_8UC3);
	if (map.type() == CV_8UC1)
	{
		// 构造等大小的彩色图
		for (int y = 0; y < map.rows; y++)
			for (int x = 0; x < map.cols; x++)
			{
				auto gray = map.at<uchar>(y, x);
				cv::Vec3b& pixel = face_on_map.at<cv::Vec3b>(y, x);
				pixel[0] = cv::abs(255 - gray);
				pixel[1] = cv::abs(127 - gray);
				pixel[2] = cv::abs(0 - gray);
			}
	}
	else
		face_on_map = map.clone();
	std::vector<Point> polygon;
	Arrangement::Ccb_halfedge_const_circulator first = f->outer_ccb();
	Arrangement::Ccb_halfedge_const_circulator curr = f->outer_ccb();
	do {
		typename Arrangement::Halfedge_const_handle he = curr;
		polygon.push_back(he->source()->point());
	} while (++curr != first);
	for (int i = 0; i < polygon.size(); i++)
	{
		if (i != polygon.size() - 1)
		{
			auto source = cv::Point2d(CGAL::to_double((polygon[i].x() - info.map_origin.x) / info.map_resolution),
				CGAL::to_double((polygon[i].y() - info.map_origin.y) / info.map_resolution));
			auto target = cv::Point2d(CGAL::to_double((polygon[i + 1].x() - info.map_origin.x) / info.map_resolution),
				CGAL::to_double((polygon[i + 1].y() - info.map_origin.y) / info.map_resolution));
			cv::line(face_on_map, source, target, cv::Scalar(255, 0, 0));
		}
		else
		{
			auto source = cv::Point2d(CGAL::to_double((polygon[i].x() - info.map_origin.x) / info.map_resolution),
				CGAL::to_double((polygon[i].y() - info.map_origin.y) / info.map_resolution));
			auto target = cv::Point2d(CGAL::to_double((polygon[0].x() - info.map_origin.x) / info.map_resolution),
				CGAL::to_double((polygon[0].y() - info.map_origin.y) / info.map_resolution));
			cv::line(face_on_map, source, target, cv::Scalar(255, 0, 0));
		}
	}
	return face_on_map;
}

cv::Mat show_adjacent_on_map(Halfedge_handle edge, cv::Mat map, pcInfo info)
{
	cv::Mat edge_on_map(map.size(), CV_8UC3);
	if (map.type() == CV_8UC1)
	{
		// 构造等大小的彩色图
		for (int y = 0; y < map.rows; y++)
			for (int x = 0; x < map.cols; x++)
			{
				auto gray = map.at<uchar>(y, x);
				cv::Vec3b& pixel = edge_on_map.at<cv::Vec3b>(y, x);
				pixel[0] = cv::abs(255 - gray);
				pixel[1] = cv::abs(127 - gray);
				pixel[2] = cv::abs(0 - gray);
			}
	}
	else
		edge_on_map = map.clone();

	auto source = cv::Point2d(CGAL::to_double((edge->source()->point().x() - info.map_origin.x) / info.map_resolution),
		CGAL::to_double((edge->source()->point().y() - info.map_origin.y) / info.map_resolution));
	auto target = cv::Point2d(CGAL::to_double((edge->target()->point().x() - info.map_origin.x) / info.map_resolution),
		CGAL::to_double((edge->target()->point().y() - info.map_origin.y) / info.map_resolution));
	cv::line(edge_on_map, source, target, cv::Scalar(0, 0, 255));
	return edge_on_map;
}

cv::Mat show_random_points_on_map(std::vector<Point> random_points, cv::Mat map, pcInfo info)
{
	cv::Mat map_clone = map.clone();
	for (int i = 0; i < random_points.size(); i++)
	{
		auto p = cv::Point2d(CGAL::to_double(random_points[i].x()),
			CGAL::to_double(random_points[i].y()));
		cv::circle(map_clone, p, 1, cv::Scalar(255, 0, 0));
	}
	return map_clone;
}

void show_arr_on_map(Arrangement arr, cv::Mat map, pcInfo info)
{
	cv::Mat arr_map(map.size(), CV_8UC3);

	// 构造等大小的彩色图
	for (int y = 0; y < map.rows; y++)
		for (int x = 0; x < map.cols; x++)
		{
			auto gray = map.at<uchar>(y, x);
			cv::Vec3b& pixel = arr_map.at<cv::Vec3b>(y, x);
			pixel[0] = cv::abs(255 - gray);
			pixel[1] = cv::abs(127 - gray);
			pixel[2] = cv::abs(0 - gray);
		}

	for (auto it = arr.faces_begin(); it != arr.faces_end(); ++it)
	{
		std::vector<Point> polygon;
		if (!it->is_unbounded() && face_is_simple(it))
		{
			Arrangement::Ccb_halfedge_const_circulator first = it->outer_ccb();
			Arrangement::Ccb_halfedge_const_circulator curr = it->outer_ccb();
			do {
				typename Arrangement::Halfedge_const_handle he = curr;
				polygon.push_back(he->source()->point());
			} while (++curr != first);
		}
		for (int i = 0; i < polygon.size(); i++)
		{
			if (i != polygon.size() - 1)
			{
				auto source = cv::Point2d(CGAL::to_double((polygon[i].x() - info.map_origin.x) / info.map_resolution),
					CGAL::to_double((polygon[i].y() - info.map_origin.y) / info.map_resolution));
				auto target = cv::Point2d(CGAL::to_double((polygon[i + 1].x() - info.map_origin.x) / info.map_resolution),
					CGAL::to_double((polygon[i + 1].y() - info.map_origin.y) / info.map_resolution));
				cv::line(arr_map, source, target, cv::Scalar(255, 0, 0));
			}
			else
			{
				auto source = cv::Point2d(CGAL::to_double((polygon[i].x() - info.map_origin.x) / info.map_resolution),
					CGAL::to_double((polygon[i].y() - info.map_origin.y) / info.map_resolution));
				auto target = cv::Point2d(CGAL::to_double((polygon[0].x() - info.map_origin.x) / info.map_resolution),
					CGAL::to_double((polygon[0].y() - info.map_origin.y) / info.map_resolution));
				cv::line(arr_map, source, target, cv::Scalar(255, 0, 0));
			}
		}
	}

	cv::imshow("hh", arr_map);
	cv::waitKey();
	cv::imwrite("./res/arr_on_gridmap.png", arr_map);
}

void show_Segments_on_map(std::vector<Segment>& segments, cv::Mat map, pcInfo info)
{
	cv::Mat segments_on_map(map.size(), CV_8UC3);
	if (map.type() == CV_8UC1)
	{
		// 构造等大小的彩色图
		for (int y = 0; y < map.rows; y++)
			for (int x = 0; x < map.cols; x++)
			{
				auto gray = map.at<uchar>(y, x);
				cv::Vec3b& pixel = segments_on_map.at<cv::Vec3b>(y, x);
				pixel[0] = cv::abs(255 - gray);
				pixel[1] = cv::abs(127 - gray);
				pixel[2] = cv::abs(0 - gray);
			}
	}
	else
		segments_on_map = map.clone();
	for (auto s : segments) {
		auto source = cv::Point2d(CGAL::to_double((s.source().x() - info.map_origin.x) / info.map_resolution),
			CGAL::to_double((s.source().y() - info.map_origin.y) / info.map_resolution));
		auto target = cv::Point2d(CGAL::to_double((s.target().x() - info.map_origin.x) / info.map_resolution),
			CGAL::to_double((s.target().y() - info.map_origin.y) / info.map_resolution));
		cv::line(segments_on_map, source, target, cv::Scalar(0, 0, 255));
	}
	
	cv::imshow("segments_on_map", segments_on_map);
	cv::waitKey();

}


