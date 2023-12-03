#include "CGAL_utils.h"



bool face_is_simple(Face_handle f)
{
	std::vector<Point> polygon_list;
	Arrangement::Ccb_halfedge_const_circulator outer_ccb = f->outer_ccb();
	Arrangement::Ccb_halfedge_const_circulator curr = outer_ccb;
	do {
		Arrangement::Halfedge_const_handle e = curr;
		polygon_list.push_back(e->source()->point());
	} while (++curr != outer_ccb);
	if (!CGAL::is_simple_2(polygon_list.begin(), polygon_list.end()))
		return false;
	else
		return true;
}

std::vector<Face_handle> find_adjacent_face(Arrangement arr, Face_handle f)
{
	std::vector<Face_handle> adjacent_face;
	Arrangement::Ccb_halfedge_const_circulator outer_ccb = f->outer_ccb();
	Arrangement::Ccb_halfedge_const_circulator curr = outer_ccb;
	do {
		Arrangement::Halfedge_const_handle e = curr;
		// 找到 每个 面 的 相邻面
		Face_const_handle neighbour_face = e->twin()->face();
		if (!neighbour_face->is_unbounded())
		{
			// const-->non const

			Face_handle f = arr.non_const_handle(neighbour_face);
			adjacent_face.push_back(f);
		}
	} while (++curr != outer_ccb);
	return adjacent_face;
}

Halfedge_handle find_two_arr_face_adjacent_edge(Face_handle v1, Face_handle v2)
{
	auto edge_cir = v1->outer_ccb();
	auto edge_begin = edge_cir;
	do
	{
		if (edge_begin->twin()->face() == v2)
		{
			Halfedge_handle adjacent_dege = Halfedge_handle(edge_begin);
			return adjacent_dege;
		}
	} while (++edge_begin != edge_cir);
	return Halfedge_handle();
}

std::vector<Point> point2map(std::vector<Point> random_points, pcInfo info)
{
	std::vector<Point> map_point;
	for (auto p : random_points)
	{
		map_point.push_back(Point((p.x() - info.map_origin.x) / info.map_resolution, (p.y() - info.map_origin.y) / info.map_resolution));
	}
	return map_point;
}

std::vector<int> point_label(std::vector<Point> map_point, cv::Mat map)
{
	std::vector<int> label;
	for (auto p : map_point)
	{
		auto i0 = int(CGAL::to_double(p.y()));
		auto i1 = int(CGAL::to_double(p.x()));

		auto res = int(map.at<uchar>(i0, i1));
		label.push_back(res);
	}
	return label;
}

std::vector<Point> line2map(Halfedge_handle he, float sample_num, pcInfo info, int thickness)
{
	std::vector<Point> sample_line_point;
	std::vector<Point> adjacent_line_point;

	Point source = he->source()->point();
	Point target = he->target()->point();
	auto source_project = Point((source.x() - info.map_origin.x) / info.map_resolution, (source.y() - info.map_origin.y) / info.map_resolution);
	auto target_project = Point((target.x() - info.map_origin.x) / info.map_resolution, (target.y() - info.map_origin.y) / info.map_resolution);

	// 判断 线段是否 水平
	Segment s(source_project, target_project);
	auto sum_sample_num = int(CGAL::to_double(s.squared_length()))* sample_num;
	if (sum_sample_num <= 1)
		sum_sample_num = 2;
	//std::cout << "sample_num:" << sample_num << std::endl;
	if (s.is_horizontal())
	{
		if (thickness != 1)
		{
			int offset = (thickness - 1) / 2;
			for (int i = 1; i < offset + 1; i++)
			{
				if (target_project.y() - i < 0 || source_project.y() - i < 0)
					continue;

				adjacent_line_point.push_back(Point(source_project.x(), source_project.y() + i));
				adjacent_line_point.push_back(Point(target_project.x(), target_project.y() + i));

				adjacent_line_point.push_back(Point(source_project.x(), source_project.y() - i));
				adjacent_line_point.push_back(Point(target_project.x(), target_project.y() - i));
			}
			CGAL::Points_on_segment_2<Point> g(source_project, target_project, sum_sample_num);
			std::copy_n(g, sum_sample_num, std::back_inserter(sample_line_point));
			/*std::cout << source_project << target_project<< std::endl;
			std::cout << sample_line_point[0] << std::endl;*/
			for (int i = 0; i < adjacent_line_point.size() - 1; i += 2)
			{
				CGAL::Points_on_segment_2<Point> adjacent_g(adjacent_line_point[i], adjacent_line_point[i + 1], sum_sample_num);
				std::copy_n(adjacent_g, sum_sample_num, std::back_inserter(sample_line_point));
			}
			return sample_line_point;
		}
		else
		{
			CGAL::Points_on_segment_2<Point> g(source_project, target_project, sum_sample_num);
			std::copy_n(g, sum_sample_num, std::back_inserter(sample_line_point));
			return sample_line_point;
		}
	}
	else
	{
		if (thickness != 1)
		{
			int offset = (thickness - 1) / 2;
			for (int i = 1; i < offset + 1; i++)
			{
				if (target_project.x() - i < 0 || source_project.x() - i < 0)
					continue;

				adjacent_line_point.push_back(Point(source_project.x() + i, source_project.y()));
				adjacent_line_point.push_back(Point(target_project.x() + i, target_project.y()));

				adjacent_line_point.push_back(Point(source_project.x() - i, source_project.y()));
				adjacent_line_point.push_back(Point(target_project.x() - i, target_project.y()));
			}
			CGAL::Points_on_segment_2<Point> g(source_project, target_project, sum_sample_num);

			std::copy_n(g, sum_sample_num, std::back_inserter(sample_line_point));
			/*std::cout << source_project << target_project << std::endl;
			std::cout << sample_line_point[0] << std::endl;*/
			for (int i = 0; i < adjacent_line_point.size() - 1; i += 2)
			{
				CGAL::Points_on_segment_2<Point> adjacent_g(adjacent_line_point[i], adjacent_line_point[i + 1], sum_sample_num);
				std::copy_n(adjacent_g, sum_sample_num, std::back_inserter(sample_line_point));
			}
			return sample_line_point;
		}
		else
		{
			CGAL::Points_on_segment_2<Point> g(source_project, target_project, sum_sample_num);
			std::copy_n(g, sum_sample_num, std::back_inserter(sample_line_point));
			return sample_line_point;
		}
	}
}

std::vector<int> line_label(std::vector<Point> line_sample_point, cv::Mat map)
{
	std::vector<int> label;
	for (auto p : line_sample_point)
	{
		//std::cout << p << std::endl;
		if (p.y() > map.rows || p.y() < 0) continue;
		if (p.x() > map.cols || p.x() < 0)continue;
		label.push_back(map.at<uchar>(CGAL::to_double(p.y()), CGAL::to_double(p.x())));
	}
	return label;
}
int cal_label_num(std::vector<int> point_label, int label)
{
	int label_num = std::count(point_label.begin(), point_label.end(), label);
	return label_num;
}


void cellDecom::setSegments(std::vector<Segment> lines)
{
	_lines = lines;
}

void cellDecom::decompsoition()
{
	//CGAL::insert(arr, _lines);
	for (auto line : _lines)
	{
		CGAL::insert_curve(arr, line);
	}
}

void cellDecom::saveSegments(std::string path)
{
	std::ofstream out;
	out.open(path);
	for (auto line : _lines)
		out << line << std::endl;
	out.close();
}

void cellDecom::readSegments(std::string path)
{
	std::ifstream in;
	in.open(path);
	Segment s;
	std::string oneLine;
	while (in >> s)
		_lines.push_back(s);
	in.close();
}

void cellDecom::saveCell(std::string path)
{
	std::ofstream out_file(path);
	out_file << arr;
	out_file.close();
}

void cellDecom::readCell(std::string path)
{
	std::ifstream in(path);
	in >> arr;
	in.close();
}

