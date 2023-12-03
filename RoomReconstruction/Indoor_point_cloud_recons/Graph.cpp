#include "Graph.h"

Arrangement ArrGraph::read_Arr(std::string path)
{
	Arrangement arr;
	std::ifstream in_file(path);
	in_file >> arr;
	in_file.close();
	return arr;
}

void ArrGraph::Arr2Graph(Arrangement& arr)
{
	// Create a mapping of the arrangement faces to indices.
	Face_index_map index_map(arr);

	// 创建 闭合面 数目 的 节点

	std::cout << "Arr_num_Vertex: " << arr.number_of_faces() << std::endl;
	std::cout << "Arr_num_unbounded_vertex: " << arr.number_of_unbounded_faces() << std::endl;
	// 添加 顶点及边
	for (auto it = arr.faces_begin(); it != arr.faces_end(); it++)
	{
		if (!it->is_unbounded())
		{
			// 检查是否为简单图形（即无交叉，是面等）
			if (face_is_simple(it))
			{
				// 添加节点属性
				int it_id = index_map[it];
				// TODO------label 依据 栅格地图 节点 确定
				int it_label = 1;
				// 当前面在图Graph中的vertex
				vertex_descriptor curr_center_graph_vertex;

				// 判断面Face是否在图Graph
				// 不在 进入 {}
				auto find_result = std::find(index_arr.begin(), index_arr.end(), it_id);
				if (find_result == index_arr.end())
				{
					// 将面Face添加到图Graph中
					vertex_descriptor it_graph_vertex = boost::add_vertex(g);

					g[it_graph_vertex].index = it_id;
					g[it_graph_vertex].label = it_label;
					g[it_graph_vertex].cost.resize(num_room, 1);
					// g[it_graph_vertex].cost[it_label] = 0;

					// 标记当前节点存储
					index_arr.push_back(it_id);
					index_graph.push_back(it_graph_vertex);

					curr_center_graph_vertex = it_graph_vertex;
				}
				else
					curr_center_graph_vertex = index_graph[find_result - index_arr.begin()];

				Arrangement::Ccb_halfedge_const_circulator outer_ccb = it->outer_ccb();
				Arrangement::Ccb_halfedge_const_circulator curr = outer_ccb;
				do {
					Arrangement::Halfedge_const_handle e = curr;
					// 找到 每个 面 的 相邻面
					Face_const_handle neighbour_face = e->twin()->face();
					if (!neighbour_face->is_unbounded())
					{
						// const-->non const
						Face_handle f = arr.non_const_handle(neighbour_face);
						if (face_is_simple(f))
						{// 添加节点属性
							int neighbour_id = index_map[f];
							// TODO------label 依据 栅格地图 节点 确定
							int neighbour_label = 1;

							// 判断面Face是否在图Graph
						   // 不在 进入 {}
							if (std::find(index_arr.begin(), index_arr.end(), neighbour_id) == index_arr.end())
							{// 将相邻面NeighbourFace添加到图Graph中
								vertex_descriptor neighbour_graph_vertex = boost::add_vertex(g);

								g[neighbour_graph_vertex].index = neighbour_id;
								g[neighbour_graph_vertex].label = neighbour_label;
								g[neighbour_graph_vertex].cost.resize(num_room, 1);
								g[neighbour_graph_vertex].cost[neighbour_label] = 0;

								// 标记当前节点存储
								index_graph.push_back(neighbour_graph_vertex);
								index_arr.push_back(neighbour_id);

								// 添加 边
								edge_descriptor edge = boost::add_edge(curr_center_graph_vertex, neighbour_graph_vertex, g).first;
								// TODO------权重weight依据栅格地图确定
								g[edge].weight = EdgeWeight;
							}
						}
					}
				} while (++curr != outer_ccb);
			}
		}
	}
	std::cout << "Graph num vertex: " << boost::num_vertices(g) << std::endl;
	// boost::write_graphviz(std::cout, g);
}

int GraphWeight::calVertexLabel(Face_handle f)
{
	auto tri = Trianglution(f);

	auto point_in_map = point2map(tri.get_random_points(), _info);
	std::cout << "points num:" << point_in_map.size() << std::endl;

	auto face_point_label = point_label(point_in_map, room_map);

	auto label_255_num = cal_label_num(face_point_label, 255);
	std::cout << "255 per:" << float(label_255_num / (face_point_label.size() * 1.0)) << std::endl;

	/////////// 可视化 面
	//auto map1 = show_face_on_map(f, room_map, map_resolution, map_origin);
	///*cv::imshow("1", map1);
	//cv::waitKey();*/
	//auto map2 = show_random_points_on_map(point_in_map, map1, map_resolution, map_origin);
	//cv::imshow("2", map2);
	//cv::waitKey();
	/////////

	if (float(label_255_num / (face_point_label.size() * 1.0)) > thresold)
		return 1;
	else
		return 0;
}

float GraphWeight::calVertexWeight(Face_handle f, int flag)
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
	}
	else
	{
		label_num = cal_label_num(face_point_label, 255);
		data_term = 1.0 - (label_num / (face_point_label.size() * 1.0 + 1.0));
	}
	std::cout << "interior per = " << data_term << std::endl;

	return data_term;
}

float GraphWeight::calEdgeWight(Face_handle source, Face_handle target)
{
	auto adjacent_edge = find_two_arr_face_adjacent_edge(source, target);
	auto line_in_map = line2map(adjacent_edge, 50, _info, 3);
	auto line_point_map = line_label(line_in_map, wall_map);
	auto label_255_num = cal_label_num(line_point_map, 255);
	//std::cout << "wall per = " << label_255_num / (line_point_map.size() * 1.0 + 1.0) << std::endl;
	// 可视化 线
	/*auto map1 = show_face_on_map(source, wall_map, map_resolution, map_origin);
	auto map2 = show_face_on_map(target, map1, map_resolution, map_origin);
	auto map3 = show_adjacent_on_map(adjacent_edge, map2, map_resolution, map_origin);
	cv::imshow("1", map3);
	cv::waitKey();*/
	//
	return label_255_num / (line_point_map.size() * 1.0 + 1.0);
}

void GraphWeight::setVertexLabel(ArrGraph graph)
{
	for (int i = 0; i < graph.g.m_vertices.size(); i++)
	{
		std::cout << "Face " << i << " ";
		auto v_l = calVertexLabel(index_map.face(graph.g[i].index));
		graph.g.m_vertices[i].m_property.label = v_l;
	}
}

void GraphWeight::setVertexWeight(ArrGraph graph)
{
	for (int i = 0; i < graph.g.m_vertices.size(); i++)
	{
		std::cout << "Face " << i << " ";

		if (graph.g.m_vertices[i].m_property.label == 0)
		{
			auto v_t = calVertexWeight(index_map.face(graph.g[i].index), 0);
			graph.g.m_vertices[i].m_property.cost[graph.g.m_vertices[i].m_property.label] = v_t * face_alpha;
		}
		else
		{
			auto v_t = calVertexWeight(index_map.face(graph.g[i].index), 1);
			graph.g.m_vertices[i].m_property.cost[graph.g.m_vertices[i].m_property.label] = v_t * face_alpha;
		}

	}
}

void GraphWeight::setEdgeWight(ArrGraph graph)
{
	for (auto i = graph.g.m_edges.begin(); i != graph.g.m_edges.end(); i++)
	{
		//std::cout << "Edge " << edge_flag++ << " ";
		auto source_face = index_map.face(graph.g[i->m_source].index);
		auto target_face = index_map.face(graph.g[i->m_target].index);
		auto e_w = calEdgeWight(source_face, target_face);
		if (graph.g[i->m_source].label == graph.g[i->m_target].label)
			e_w = 0;
		else
			e_w = 1 - e_w;
		i->m_property.weight = e_w * edge_alpha;
	}
}

GraphWeight::GraphWeight(const std::string& _room_path, const std::string& _wall_path, pcInfo info, Arrangement arr)
{
	room_map = cv::imread(_room_path, 0);
	wall_map = cv::imread(_wall_path, 0);
	map_resolution = info.map_resolution;
	map_origin = info.map_origin;
	_info = info;
	index_map = Face_index_map(arr);
}

GraphWeight::GraphWeight(cv::Mat in_room_map, cv::Mat in_wall_map, pcInfo info, Arrangement arr)
{
	room_map = in_room_map.clone();
	wall_map = in_wall_map.clone();
	map_resolution = info.map_resolution;
	map_origin = info.map_origin;
	_info = info;
	index_map = Face_index_map(arr);
}

void graph_cut(ArrGraph graph)
{
	std::cerr << std::endl << "Alpha expansion..." << std::endl << std::endl;
	CGAL::alpha_expansion_graphcut(graph.g,
		boost::get(&Edge_property::weight, graph.g),
		boost::get(&Vertex_property::cost, graph.g),
		boost::get(&Vertex_property::label, graph.g),
		CGAL::parameters::vertex_index_map(boost::get(boost::vertex_index, graph.g)));
}
