#include "Graphcut.h"

int GraphCut::calVertexLabel(Face_handle f)
{
	/*cv::Mat img = show_face_on_map(f, room_map, _info);
	cv::imshow("face on map", img);
	cv::waitKey();*/
	auto tri = Trianglution(f);

	//tri.showRandomPoints();

	auto point_in_map = point2map(tri.get_random_points(), _info);
	std::cout << "points num:" << point_in_map.size() << std::endl;

	auto face_point_label = point_label(point_in_map, room_map);

	auto label_255_num = cal_label_num(face_point_label, 255);
	std::cout << "255 per:" << float(label_255_num / (face_point_label.size() * 1.0)) << std::endl;

	/////////// 可视化 面
	//auto map1 = show_face_on_map(f, room_map, _info);
	///*cv::imshow("1", map1);
	//cv::waitKey();*/
	//auto map2 = show_random_points_on_map(point_in_map, map1, _info);
	//cv::imshow("2", map2);
	//cv::waitKey();
	/////////

	if (float(label_255_num / (face_point_label.size() * 1.0)) > thresold)
		return 1;
	else
		return 0;
}

float GraphCut::calVertexWeight(Face_handle f, int flag)
{


	auto t = Trianglution(f);
	//t.showRandomPoints();
	auto point_in_map = point2map(t.get_random_points(), _info);
	auto face_point_label = point_label(point_in_map, room_map);
	int label_num;
	float data_term;
	if (flag == 0)
	{
		label_num = cal_label_num(face_point_label, 0);
		data_term = 1.0 - (label_num / (face_point_label.size() * 1.0 + 1.0));
		//data_term = 0.0001;
		//std::cout << "outdoor cost = " << data_term << std::endl;
	}
	else
	{
		label_num = cal_label_num(face_point_label, 255);
		data_term = 1.0 - (label_num / (face_point_label.size() * 1.0 + 1.0));
		//data_term = 0.0001;
		//std::cout << "interior cost = " << data_term << std::endl;
	}


	return data_term;
}

float GraphCut::calEdgeWight(Face_handle source, Face_handle target, float sample_num_per_meter, int thickness)
{
	auto adjacent_edge = find_two_arr_face_adjacent_edge(source, target);
	auto line_in_map = line2map(adjacent_edge, sample_num_per_meter, _wall_info, thickness);
	/*auto line_point_map = line_label(line_in_map, wall_map);
	auto label_255_num = cal_label_num(line_point_map, 255);*/

	float max_wall_probability = -1;
	for (int i = 0; i < thickness; i++) {
		std::vector<Point> sample_line;
		std::copy_n(line_in_map.begin() + (int)i * (line_in_map.size() / thickness), line_in_map.size() / thickness, std::back_inserter(sample_line));
		auto line_2_map = line_label(sample_line, wall_map);
		auto label_255_num = cal_label_num(line_2_map, 255);
		//std::cout << (label_255_num / (line_2_map.size() * 1.0 + 1.0)) << std::endl;
		if (max_wall_probability < (label_255_num / (line_2_map.size() * 1.0 + 1.0)))
			max_wall_probability = (label_255_num / (line_2_map.size() * 1.0 + 1.0));
	}
	std::cout << "wall per = " << max_wall_probability << std::endl;
	 //可视化 线
	/*auto map1 = show_face_on_map(source, wall_map, _wall_info);
	auto map2 = show_face_on_map(target, map1, _wall_info);
	auto map3 = show_adjacent_on_map(adjacent_edge, map2, _wall_info);
	cv::Mat up_sample;
	cv::resize(map3, up_sample, cv::Size(), 3, 3, cv::INTER_LINEAR);
	cv::imshow("1", up_sample);
	cv::waitKey();*/
	
	return max_wall_probability;
}

void GraphCut::setVertexLabel(ArrGraph& graph)
{
	for (int i = 0; i < graph.g.m_vertices.size(); i++)
	{
		std::cout << "Face " << i << " ";
		graph.g.m_vertices[i].m_property.area = Trianglution(index_map.face(graph.g[i].index)).area();
		auto v_l = calVertexLabel(index_map.face(graph.g[i].index));
		graph.g.m_vertices[i].m_property.label = v_l;
		std::cout << "face:" << i <<
			" label:" << graph.g.m_vertices[i].m_property.label <<
			" cost0:" << graph.g.m_vertices[i].m_property.cost[0] <<
			" cost1:" << graph.g.m_vertices[i].m_property.cost[1] << std::endl;
	}

}

void GraphCut::setVertexWeight(ArrGraph& graph)
{
	for (int i = 0; i < graph.g.m_vertices.size(); i++)
	{
		std::cout << "Face " << i << " ";

		if (graph.g.m_vertices[i].m_property.label == 0)
		{
			auto v_t = calVertexWeight(index_map.face(graph.g[i].index), 0);
			graph.g.m_vertices[i].m_property.cost[graph.g.m_vertices[i].m_property.label] = v_t * face_alpha;
			std::cout << "outdoor cost = " << graph.g.m_vertices[i].m_property.cost[graph.g.m_vertices[i].m_property.label] << std::endl;
		}
		else
		{
			auto v_t = calVertexWeight(index_map.face(graph.g[i].index), 1);
			graph.g.m_vertices[i].m_property.cost[graph.g.m_vertices[i].m_property.label] = v_t * face_alpha;
			std::cout << "indoor cost = " << graph.g.m_vertices[i].m_property.cost[graph.g.m_vertices[i].m_property.label] << std::endl;
		}
		
	}
}

void GraphCut::setEdgeWight(ArrGraph& graph, float sample_num_per_meter, int thickness)
{
	int edge_flag = 0;
	for (auto i = graph.g.m_edges.begin(); i != graph.g.m_edges.end(); i++)
	{
		std::cout << "Edge " << edge_flag++ << " ";
		auto source_face = index_map.face(graph.g[i->m_source].index);
		auto target_face = index_map.face(graph.g[i->m_target].index);
		auto e_w = calEdgeWight(source_face, target_face, sample_num_per_meter, thickness);
		if (graph.g[i->m_source].label == graph.g[i->m_target].label)
			e_w = 0;
		else
			e_w = 1 - e_w;
		i->m_property.weight = e_w * edge_alpha;
		
		
		/*if (e_w >= 0.3) {
			std::cout << "wall cost:" << i->m_property.weight << std::endl;
			cv::Mat m1 = show_face_on_map(source_face, wall_map, _info);
			cv::Mat m2 = show_face_on_map(target_face, m1, _info);
			cv::imshow("121", m2);
			cv::waitKey();
		}*/
			
	}
}

void GraphCut::setAlpha(float data_alpha, float smooth_alpha)
{
	face_alpha = data_alpha;
	edge_alpha = smooth_alpha;
}

GraphCut::GraphCut(const std::string& _room_path, const std::string& _wall_path, pcInfo info, pcInfo wall_info, Arrangement& arr)
{
	room_map = cv::imread(_room_path, 0);
	wall_map = cv::imread(_wall_path, 0);
	map_resolution = info.map_resolution;
	map_origin = info.map_origin;
	_info = info;
	_wall_info = wall_info;
	index_map = Face_index_map(arr);
}

GraphCut::GraphCut(cv::Mat in_room_map, cv::Mat in_wall_map, pcInfo info, pcInfo wall_info, Arrangement& arr)
{
	room_map = in_room_map.clone();
	wall_map = in_wall_map.clone();
	map_resolution = info.map_resolution;
	map_origin = info.map_origin;
	_info = info;
	_wall_info = wall_info;
	index_map = Face_index_map(arr);
}

void GraphCut::graph_cut(ArrGraph& graph)
{
	std::cerr << std::endl << "Alpha expansion..." << std::endl << std::endl;
	CGAL::alpha_expansion_graphcut(graph.g,
		boost::get(&Edge_property::weight, graph.g),
		boost::get(&Vertex_property::cost, graph.g),
		boost::get(&Vertex_property::label, graph.g),
		CGAL::parameters::vertex_index_map(boost::get(boost::vertex_index, graph.g)));
}