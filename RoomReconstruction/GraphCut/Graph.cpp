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
								//g[neighbour_graph_vertex].cost[neighbour_label] = 0;

								// 标记当前节点存储
								index_graph.push_back(neighbour_graph_vertex);
								index_arr.push_back(neighbour_id);

								
								// 添加 边
								edge_descriptor edge = boost::add_edge(curr_center_graph_vertex, neighbour_graph_vertex, g).first;
								// TODO------权重weight依据栅格地图确定
								g[edge].weight = EdgeWeight;
							}
							else
							{
								vertex_descriptor neighbour_graph_vertex = index_graph[std::find(index_arr.begin(), index_arr.end(), neighbour_id) - index_arr.begin()];
								// 检查两个顶点之间是否已经存储相邻边
								auto is_exist_edge_2_vertex = boost::edge(curr_center_graph_vertex, neighbour_graph_vertex, g);
								if (!is_exist_edge_2_vertex.second) {
									// 添加 边
									edge_descriptor edge = boost::add_edge(curr_center_graph_vertex, neighbour_graph_vertex, g).first;
									// TODO------权重weight依据栅格地图确定
									g[edge].weight = EdgeWeight;
								}
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

