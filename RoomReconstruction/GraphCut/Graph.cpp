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

	// ���� �պ��� ��Ŀ �� �ڵ�

	std::cout << "Arr_num_Vertex: " << arr.number_of_faces() << std::endl;
	std::cout << "Arr_num_unbounded_vertex: " << arr.number_of_unbounded_faces() << std::endl;
	// ��� ���㼰��
	for (auto it = arr.faces_begin(); it != arr.faces_end(); it++)
	{
		if (!it->is_unbounded())
		{
			// ����Ƿ�Ϊ��ͼ�Σ����޽��棬����ȣ�
			if (face_is_simple(it))
			{
				// ��ӽڵ�����
				int it_id = index_map[it];
				// TODO------label ���� դ���ͼ �ڵ� ȷ��
				int it_label = 1;
				// ��ǰ����ͼGraph�е�vertex
				vertex_descriptor curr_center_graph_vertex;

				// �ж���Face�Ƿ���ͼGraph
				// ���� ���� {}
				auto find_result = std::find(index_arr.begin(), index_arr.end(), it_id);
				if (find_result == index_arr.end())
				{
					// ����Face��ӵ�ͼGraph��
					vertex_descriptor it_graph_vertex = boost::add_vertex(g);

					g[it_graph_vertex].index = it_id;
					g[it_graph_vertex].label = it_label;
					g[it_graph_vertex].cost.resize(num_room, 1);
					// g[it_graph_vertex].cost[it_label] = 0;

					// ��ǵ�ǰ�ڵ�洢
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
					// �ҵ� ÿ�� �� �� ������
					Face_const_handle neighbour_face = e->twin()->face();
					if (!neighbour_face->is_unbounded())
					{
						// const-->non const
						Face_handle f = arr.non_const_handle(neighbour_face);
						if (face_is_simple(f))
						{// ��ӽڵ�����
							int neighbour_id = index_map[f];
							// TODO------label ���� դ���ͼ �ڵ� ȷ��
							int neighbour_label = 1;

							// �ж���Face�Ƿ���ͼGraph
						   // ���� ���� {}
							if (std::find(index_arr.begin(), index_arr.end(), neighbour_id) == index_arr.end())
							{// ��������NeighbourFace��ӵ�ͼGraph��
								vertex_descriptor neighbour_graph_vertex = boost::add_vertex(g);

								g[neighbour_graph_vertex].index = neighbour_id;
								g[neighbour_graph_vertex].label = neighbour_label;
								g[neighbour_graph_vertex].cost.resize(num_room, 1);
								//g[neighbour_graph_vertex].cost[neighbour_label] = 0;

								// ��ǵ�ǰ�ڵ�洢
								index_graph.push_back(neighbour_graph_vertex);
								index_arr.push_back(neighbour_id);

								
								// ��� ��
								edge_descriptor edge = boost::add_edge(curr_center_graph_vertex, neighbour_graph_vertex, g).first;
								// TODO------Ȩ��weight����դ���ͼȷ��
								g[edge].weight = EdgeWeight;
							}
							else
							{
								vertex_descriptor neighbour_graph_vertex = index_graph[std::find(index_arr.begin(), index_arr.end(), neighbour_id) - index_arr.begin()];
								// �����������֮���Ƿ��Ѿ��洢���ڱ�
								auto is_exist_edge_2_vertex = boost::edge(curr_center_graph_vertex, neighbour_graph_vertex, g);
								if (!is_exist_edge_2_vertex.second) {
									// ��� ��
									edge_descriptor edge = boost::add_edge(curr_center_graph_vertex, neighbour_graph_vertex, g).first;
									// TODO------Ȩ��weight����դ���ͼȷ��
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

