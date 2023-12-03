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
	//std::cout << "这根线段显示的坐标:" << source << target << std::endl;
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

void show_arr_on_map(Arrangement arr, cv::Mat map, pcInfo info, float resize)
{
	cv::Mat arr_map(map.size(), CV_8UC3);
	if (map.type() == CV_8UC1)
	{
		// 构造等大小的彩色图
		for (int y = 0; y < map.rows; y++)
			for (int x = 0; x < map.cols; x++)
			{
				auto gray = map.at<uchar>(y, x);
				cv::Vec3b& pixel = arr_map.at<cv::Vec3b>(y, x);
				//if (gray == 0)
				//{
				//	// 室外灰色
				//	pixel[0] = cv::abs(182);
				//	pixel[1] = cv::abs(181);
				//	pixel[2] = cv::abs(181);

				//}
				//else
				//{
				//	// 室内米色
				//	pixel[0] = cv::abs(206);
				//	pixel[1] = cv::abs(255);
				//	pixel[2] = cv::abs(255);

				//}
				pixel[0] = cv::abs(206);
				pixel[1] = cv::abs(255);
				pixel[2] = cv::abs(255);
			}
	}
	else
		arr_map = map.clone();
	

	

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
				cv::line(arr_map, source, target, cv::Scalar(182, 182, 182));
			}
			else
			{
				auto source = cv::Point2d(CGAL::to_double((polygon[i].x() - info.map_origin.x) / info.map_resolution),
					CGAL::to_double((polygon[i].y() - info.map_origin.y) / info.map_resolution));
				auto target = cv::Point2d(CGAL::to_double((polygon[0].x() - info.map_origin.x) / info.map_resolution),
					CGAL::to_double((polygon[0].y() - info.map_origin.y) / info.map_resolution));
				cv::line(arr_map, source, target, cv::Scalar(182, 182, 182));
			}
		}
	}
	cv::Mat up_sample;
	cv::resize(arr_map, up_sample, cv::Size(), resize, resize, cv::INTER_LINEAR);
	cv::imshow("hh", up_sample);
	cv::waitKey();
	//cv::imwrite("./res/arr_on_gridmap.png", arr_map);
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
	int i = 1;
	for (auto s : segments) {
		auto source = cv::Point2d(CGAL::to_double((s.source().x() - info.map_origin.x) / info.map_resolution),
			CGAL::to_double((s.source().y() - info.map_origin.y) / info.map_resolution));
		auto target = cv::Point2d(CGAL::to_double((s.target().x() - info.map_origin.x) / info.map_resolution),
			CGAL::to_double((s.target().y() - info.map_origin.y) / info.map_resolution));
		std::cout << i << std::endl;
		cv::line(segments_on_map, source, target, cv::Scalar(0, 0, 255));
		cv::imshow("segments_on_map", segments_on_map);
		cv::waitKey();
		i++;
	}
	
	

}



cv::Mat show_graph_label_on_map(ArrGraph graph, Arrangement arr, cv::Mat map, float map_resolution, cv::Point2d map_origin, int resize)
{
	cv::Mat arr_map(map.size(), CV_8UC3);

	// 构造等大小的彩色图
	for (int y = 0; y < map.rows; y++)
		for (int x = 0; x < map.cols; x++)
		{
			auto gray = map.at<uchar>(y, x);
			cv::Vec3b& pixel = arr_map.at<cv::Vec3b>(y, x);
			if (gray == 0)
			{
				// 室外灰色
				pixel[0] = cv::abs(182);
				pixel[1] = cv::abs(181);
				pixel[2] = cv::abs(181);

			}
			else
			{
				// 室内米色
				pixel[0] = cv::abs(206);
				pixel[1] = cv::abs(255);
				pixel[2] = cv::abs(255);

			}

		}

	auto overlay = arr_map.clone();
	Face_index_map index_map(arr);
	// 顶点标签 0:室外 1:室内
	for (int i = 0; i < graph.g.m_vertices.size(); i++)
	{
		auto f = index_map.face(graph.g[i].index);
		std::vector<cv::Point> polylines;
		Arrangement::Ccb_halfedge_const_circulator outer_ccb = f->outer_ccb();
		Arrangement::Ccb_halfedge_const_circulator curr = outer_ccb;
		do {
			Arrangement::Halfedge_const_handle e = curr;
			polylines.push_back(cv::Point(CGAL::to_double(e->source()->point().x() - map_origin.x) / map_resolution, CGAL::to_double(e->source()->point().y() - map_origin.y) / map_resolution));
		} while (++curr != outer_ccb);
		std::vector<std::vector<cv::Point>> ppolylines = { polylines };
		


		if (graph.g.m_vertices[i].m_property.label == 0)
			cv::fillPoly(overlay, ppolylines, cv::Scalar(181, 181, 182));
		else
			cv::fillPoly(overlay, ppolylines, cv::Scalar(206, 255, 255));
		cv::polylines(overlay, ppolylines, true, CV_RGB(102, 102, 102));

	}
	/*auto alpha = 0.5;
	cv::Mat result;
	cv::addWeighted(overlay, alpha, arr_map, 1 - alpha, 0, result);*/

	/*cv::Mat up_sample;
	cv::resize(overlay, up_sample, cv::Size(), resize, resize, cv::INTER_LINEAR);*/

	return overlay;
}

cv::Mat show_all_adjacent_lines_on_map(ArrGraph& graph, Arrangement& arr,cv::Mat map, pcInfo info)
{
	Face_index_map index_map(arr);

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

	for (auto i = graph.g.m_edges.begin(); i != graph.g.m_edges.end(); i++)
	{
		auto source_face = index_map.face(graph.g[i->m_source].index);
		auto target_face = index_map.face(graph.g[i->m_target].index);
		auto edge = find_two_arr_face_adjacent_edge(source_face, target_face);
		
		auto source = cv::Point2d(CGAL::to_double((edge->source()->point().x() - info.map_origin.x) / info.map_resolution),
			CGAL::to_double((edge->source()->point().y() - info.map_origin.y) / info.map_resolution));
		auto target = cv::Point2d(CGAL::to_double((edge->target()->point().x() - info.map_origin.x) / info.map_resolution),
			CGAL::to_double((edge->target()->point().y() - info.map_origin.y) / info.map_resolution));
		//std::cout << "这根线段显示的坐标:" << source << target << std::endl;
		cv::line(edge_on_map, source, target, cv::Scalar(0, 0, 255));
		
		
	}
	return edge_on_map;
}

cv::Mat gen_multi_rooms_on_map(Face_index_map index_map , cv::Mat map, pcInfo info, std::map<int, std::vector<int>> label_with_rooms)
{
	
	cv::Mat arr_map(map.size(), CV_8UC3);


	// 遍历标签
	for (auto iter = label_with_rooms.begin(); iter != label_with_rooms.end(); iter++) {
		// 为相同标签的cell选择颜色
		cv::Vec3b color;
		if ((*iter).first != 0)
			color = cv::Vec3b((rand() % 250) + 1, (rand() % 250) + 1, (rand() % 250) + 1);
		else
			color = cv::Vec3b(102, 102, 102);
		auto face_list = (*iter).second;
		// 遍历单个cell
		for (auto face_idx : face_list) {
			auto face = index_map.face(face_idx);

			std::vector<Point> polygon;
			std::vector<cv::Point> cPolygon;
			if (!face->is_unbounded() && face_is_simple(face))
			{
				Arrangement::Ccb_halfedge_const_circulator first = face->outer_ccb();
				Arrangement::Ccb_halfedge_const_circulator curr = face->outer_ccb();
				do {
					typename Arrangement::Halfedge_const_handle he = curr;
					polygon.push_back(he->source()->point());
				} while (++curr != first);
			}
			for (int i = 0; i < polygon.size(); i++)
			{
				auto source = cv::Point2d(CGAL::to_double((polygon[i].x() - info.map_origin.x) / info.map_resolution),
					CGAL::to_double((polygon[i].y() - info.map_origin.y) / info.map_resolution));
				cPolygon.push_back(source);
			}
			cv::fillPoly(arr_map, cPolygon, color, 8, 0);
			cv::polylines(arr_map, cPolygon, true, cv::Scalar(0, 255, 255), 1, 8, 0);
		}
	}
	
	//cv::imwrite("./res/arr_on_gridmap.png", arr_map);
	return arr_map;
}


cv::Mat return_error_label_on_map(ArrGraph graph, Arrangement arr, cv::Mat map, float map_resolution, cv::Point2d map_origin) {
	cv::Mat arr_map(map.size(), CV_8UC3);

	// 构造等大小的彩色图
	for (int y = 0; y < map.rows; y++)
		for (int x = 0; x < map.cols; x++)
		{
			auto gray = map.at<uchar>(y, x);
			cv::Vec3b& pixel = arr_map.at<cv::Vec3b>(y, x);
			if (gray == 0)
			{
				// 室外灰色
				pixel[0] = cv::abs(182);
				pixel[1] = cv::abs(181);
				pixel[2] = cv::abs(181);

			}
			else
			{
				// 室内米色
				pixel[0] = cv::abs(206);
				pixel[1] = cv::abs(255);
				pixel[2] = cv::abs(255);

			}

		}

	auto overlay = arr_map.clone();
	Face_index_map index_map(arr);

	for (int i = 0; i < graph.g.m_vertices.size(); i++)
	{
		auto f = index_map.face(graph.g[i].index);
		std::vector<cv::Point> polylines;
		Arrangement::Ccb_halfedge_const_circulator outer_ccb = f->outer_ccb();
		Arrangement::Ccb_halfedge_const_circulator curr = outer_ccb;
		do {
			Arrangement::Halfedge_const_handle e = curr;
			polylines.push_back(cv::Point(CGAL::to_double(e->source()->point().x() - map_origin.x) / map_resolution, CGAL::to_double(e->source()->point().y() - map_origin.y) / map_resolution));
		} while (++curr != outer_ccb);
		std::vector<std::vector<cv::Point>> ppolylines = { polylines };



		if (graph.g.m_vertices[i].m_property.label == 0) //室外
			cv::fillPoly(overlay, ppolylines, cv::Scalar(181, 181, 182));
		else if(graph.g.m_vertices[i].m_property.label == 1) //TP
			cv::fillPoly(overlay, ppolylines, cv::Scalar(206, 255, 255));
		else if (graph.g.m_vertices[i].m_property.label == 2)//FP
		{
			std::cout << "B " << "FP: " << i << std::endl;
			cv::fillPoly(overlay, ppolylines, cv::Scalar(255, 0, 0));
			/*cv::imshow("FP_B", overlay);
			cv::waitKey();*/
		}
		else if (graph.g.m_vertices[i].m_property.label == 3)//FN
		{
			std::cout << "G " << "FN: " << i << std::endl;
			cv::fillPoly(overlay, ppolylines, cv::Scalar(0, 255, 0));
			cv::imshow("FN_G", overlay);
			cv::waitKey();
		}
		cv::polylines(overlay, ppolylines, true, CV_RGB(102, 102, 102));

	}

	return overlay;
}


void show_arr_and_vertex_on_map(Arrangement arr, cv::Mat map, pcInfo info, float resize)
{
	cv::Mat arr_map(map.rows, map.cols, CV_8UC3, cv::Scalar(255,255,255));

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
				
				cv::line(arr_map, source, target, cv::Scalar(155, 155, 155));
				cv::circle(arr_map, source,6,cv::Scalar(155,155,155),-1);
				cv::circle(arr_map, target, 6, cv::Scalar(155, 155, 155), -1);
			}
			else
			{
				auto source = cv::Point2d(CGAL::to_double((polygon[i].x() - info.map_origin.x) / info.map_resolution),
					CGAL::to_double((polygon[i].y() - info.map_origin.y) / info.map_resolution));
				auto target = cv::Point2d(CGAL::to_double((polygon[0].x() - info.map_origin.x) / info.map_resolution),
					CGAL::to_double((polygon[0].y() - info.map_origin.y) / info.map_resolution));
				cv::line(arr_map, source, target, cv::Scalar(155, 155, 155));
				cv::circle(arr_map, source, 6, cv::Scalar(155, 155, 155), -1);
				cv::circle(arr_map, target, 6, cv::Scalar(155, 155, 155), -1);
			}
		}
	}
	cv::Mat up_sample;
	cv::resize(arr_map, up_sample, cv::Size(), resize, resize, cv::INTER_LINEAR);
	cv::imshow("hh", up_sample);
	cv::waitKey();
	//cv::imwrite("./res/arr_on_gridmap.png", arr_map);
}

cv::Mat return_rooms_boundary(cv::Mat map, pcInfo info, std::map<int, std::vector<Segment>> rooms_bound)
{
	cv::Mat arr_map(map.size(), CV_8UC3);

	// 构造等大小的彩色图
	for (int y = 0; y < map.rows; y++)
		for (int x = 0; x < map.cols; x++)
		{
			auto gray = map.at<uchar>(y, x);
			cv::Vec3b& pixel = arr_map.at<cv::Vec3b>(y, x);
			if (gray == 0)
			{
				// 室外灰色
				pixel[0] = cv::abs(182);
				pixel[1] = cv::abs(181);
				pixel[2] = cv::abs(181);

			}
			else
			{
				// 室内米色
				pixel[0] = cv::abs(206);
				pixel[1] = cv::abs(255);
				pixel[2] = cv::abs(255);

			}

		}

	cv::RNG rng(123456);
	for (auto room : rooms_bound)
	{
		cv::Scalar random_color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		for (auto s : room.second)
		{
			auto source = cv::Point2d(CGAL::to_double((s.source().x() - info.map_origin.x) / info.map_resolution),
				CGAL::to_double((s.source().y() - info.map_origin.y) / info.map_resolution));
			auto target = cv::Point2d(CGAL::to_double((s.target().x() - info.map_origin.x) / info.map_resolution),
				CGAL::to_double((s.target().y() - info.map_origin.y) / info.map_resolution));
			cv::line(arr_map, source, target, random_color, 2);
		}
	}
	return arr_map;
}
cv::Mat return_rooms_boundary(cv::Mat map, pcInfo info, std::map<int, std::vector<cv::Point2d>> rooms_bound)
{
	cv::Mat arr_map(map.size(), CV_8UC3);

	// 构造等大小的彩色图
	for (int y = 0; y < map.rows; y++)
		for (int x = 0; x < map.cols; x++)
		{
			auto gray = map.at<uchar>(y, x);
			cv::Vec3b& pixel = arr_map.at<cv::Vec3b>(y, x);
			if (gray == 0)
			{
				// 室外灰色
				pixel[0] = cv::abs(182);
				pixel[1] = cv::abs(181);
				pixel[2] = cv::abs(181);

			}
			else
			{
				// 室内米色
				pixel[0] = cv::abs(206);
				pixel[1] = cv::abs(255);
				pixel[2] = cv::abs(255);

			}

		}

	cv::RNG rng(123456);
	for (auto room : rooms_bound)
	{
		std::vector<cv::Point> polylines;
		cv::Scalar random_color(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		for (auto s : room.second)
		{
			auto p = cv::Point((s.x - info.map_origin.x) / info.map_resolution,
				(s.y - info.map_origin.y) / info.map_resolution);
			polylines.push_back(p);
		}
		std::vector<std::vector<cv::Point> > pts = {polylines};
		cv::polylines(arr_map, pts, false, random_color, 2);
	}
	return arr_map;
}