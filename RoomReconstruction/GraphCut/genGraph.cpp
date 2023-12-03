//#include "Graph.h"
//#include "Graphcut.h"
//#include "visualization.h"
//#include "roomSeg.h"

//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../data/pcInfo.txt", info);
//	readPCInfo("../data/wall_info.txt", wall_info);
//
//	cv::Mat map = cv::imread("../data/wall_map_0.15m_2.png", 0); 
//	//cv::Mat room_map = cv::imread("../data/grid_with_door2.png", 0);
//	cv::Mat room_map = cv::imread("../data/room_map_chen2.png", 0);
//	// 读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "..//cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	cv::Mat adjacent_wall= show_all_adjacent_lines_on_map(graph, arr, map, wall_info);
//	cv::Mat up_sample;
//	cv::resize(adjacent_wall, up_sample, cv::Size(), 3, 3, cv::INTER_LINEAR);
//	cv::imshow("adjacent_wall", up_sample);
//	cv::waitKey();
//	
//	//show_arr_on_map(arr, map, wall_info, 3);
//	// show_arr_on_map(arr, map,info);
//	float a, b;
//	std::cin >> a >> b;
//	std::cout << a << " " << b << std::endl;
//	GraphCut gw(room_map, map, info, wall_info, arr);
//	gw.setAlpha(a, b);
//	gw.setVertexLabel(graph);
//	show_graph_label_on_map(graph, arr, map, wall_info.map_resolution, wall_info.map_origin, 3);
//	gw.setVertexWeight(graph);
//	gw.setEdgeWight(graph, 5, 5);
//
//	gw.graph_cut(graph);
//
//	show_graph_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//
//	
//}


//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../data1/pcInfo.txt", info);
//	readPCInfo("../data1/wall_info.txt", wall_info);
//
//	cv::Mat map = cv::imread("../data1/wall_map.png", 0);
//	cv::Mat room_map = cv::imread("../data1/room_map.png", 0);
//	// 读取 Cell，将其转换为graph
//	std::string arr_path = "../res1/Cell/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	cv::Mat adjacent_wall = show_all_adjacent_lines_on_map(graph, arr, map, wall_info);
//	cv::Mat up_sample;
//	cv::resize(adjacent_wall, up_sample, cv::Size(), 3, 3, cv::INTER_LINEAR);
//	cv::imshow("adjacent_wall", up_sample);
//	cv::waitKey();
//
//	//show_arr_on_map(arr, map, wall_info, 3);
//	// show_arr_on_map(arr, map,info);
//	float a, b;
//	std::cin >> a >> b;
//	std::cout << a << " " << b << std::endl;
//	GraphCut gw(room_map, map, info, wall_info, arr);
//	gw.setAlpha(a, b);
//	gw.setVertexLabel(graph);
//	show_graph_label_on_map(graph, arr, map, wall_info.map_resolution, wall_info.map_origin, 3);
//	gw.setVertexWeight(graph);
//	gw.setEdgeWight(graph, 5, 5);
//
//	gw.graph_cut(graph);
//
//	show_graph_label_on_map(graph, arr, map, wall_info.map_resolution, wall_info.map_origin, 3);
//
//	for (int i = 0; i < graph.g.m_vertices.size(); i++) {
//		std::cout << "face:"<<i<<
//			" label:" << graph.g.m_vertices[i].m_property.label << 
//			" cost0:" << graph.g.m_vertices[i].m_property.cost[0] <<
//			" cost0:" << graph.g.m_vertices[i].m_property.cost[1]<<std::endl;
//	}
//}


//// //按平面提取
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../data/pcInfo.txt", info);
//	readPCInfo("../data/pcInfo.txt", wall_info);
//
//	//cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);
//	cv::Mat map = cv::imread("../data/wall_map_multi.png", 0);
//	//cv::Mat room_map = cv::imread("../data/grid_with_door2.png", 0);
//	cv::Mat room_map = cv::imread("../data/room_map_chen2.png", 0);
//	// 读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../Plane_line2_simply/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	/*cv::Mat adjacent_wall = show_all_adjacent_lines_on_map(graph, arr, map, wall_info);
//	cv::Mat up_sample;
//	cv::resize(adjacent_wall, up_sample, cv::Size(), 1, 1, cv::INTER_LINEAR);
//	cv::imshow("adjacent_wall", up_sample);
//	cv::waitKey();*/
//
//	//show_arr_on_map(arr, map, wall_info, 3);
//	// show_arr_on_map(arr, map,info);
//
//	float a, b;
//	std::cin >> a >> b;
//	std::cout << a << " " << b << std::endl;
//	GraphCut gw(room_map, map, info, wall_info, arr);
//	gw.setAlpha(a, b);
//	gw.setVertexLabel(graph);
//	saveCellArea("../res/confuseMatrix/area1.txt",graph);
//	saveCellLabel("../res/confuseMatrix/gt1.txt",graph);
//	cv::Mat gt = show_graph_label_on_map(graph, arr, map, wall_info.map_resolution, wall_info.map_origin, 1);
//	cv::imshow("truth", gt);
//	cv::waitKey();
//	gw.setVertexWeight(graph);
//	gw.setEdgeWight(graph, 3, 5);
//
//	gw.graph_cut(graph);
//
//	saveCellLabel("../res/confuseMatrix/pred1.txt", graph);
//	cv::Mat indoor_map = show_graph_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("indoor", indoor_map);
//	cv::waitKey();
//
//	/*cv::Mat seg_map = cv::imread("gray_seg_map.png", 0);
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");*/
//}

// 小场景提取
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../unbalance_small_scene/room_info.txt", info);
//	readPCInfo("../unbalance_small_scene/wall_info.txt", wall_info);
//
//	cv::Mat map = cv::imread("../unbalance_small_scene/wall_map_slice.png", 0);
//	//cv::Mat room_map = cv::imread("../data/grid_with_door2.png", 0);
//	cv::Mat room_map = cv::imread("../unbalance_small_scene/room_map.png", 0);
//	// 读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../unbalance_small_scene/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	/*cv::Mat adjacent_wall = show_all_adjacent_lines_on_map(graph, arr, map, wall_info);
//	cv::Mat up_sample;
//	cv::resize(adjacent_wall, up_sample, cv::Size(), 1, 1, cv::INTER_LINEAR);
//	cv::imshow("adjacent_wall", up_sample);
//	cv::waitKey();*/
//
//	//show_arr_on_map(arr, map, wall_info, 3);
//	// show_arr_on_map(arr, map,info);
//	float a, b;
//	std::cin >> a >> b;
//	std::cout << a << " " << b << std::endl;
//	GraphCut gw(room_map, map, info, wall_info, arr);
//	gw.setAlpha(a, b);
//	gw.setVertexLabel(graph);
//	//show_graph_label_on_map(graph, arr, map, wall_info.map_resolution, wall_info.map_origin, 1);
//	gw.setVertexWeight(graph);
//	gw.setEdgeWight(graph, 1, 3);
//
//	gw.graph_cut(graph);
//
//	cv::Mat graph_map = show_graph_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	/*cv::imshow("graph_cut", graph_map);
//	auto save = cv::waitKey();
//	std::cout << save;*/
//	cv::imwrite("../unbalance_small_scene/graphcut_map_alpha_"+std::to_string(a)+".png", graph_map);
//}

// 区域5
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../area5/pcInfo.txt", info);
//	readPCInfo("../area5/pcInfo.txt", wall_info);
//
//	//cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);
//	cv::Mat map = cv::imread("../area5/wall_map.png", 0);
//	//cv::Mat room_map = cv::imread("../data/grid_with_door2.png", 0);
//	cv::Mat room_map = cv::imread("../area5/room_map.png", 0);
//	// 读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../area5/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	/*cv::Mat adjacent_wall = show_all_adjacent_lines_on_map(graph, arr, map, wall_info);
//	cv::Mat up_sample;
//	cv::resize(adjacent_wall, up_sample, cv::Size(), 1, 1, cv::INTER_LINEAR);
//	cv::imshow("adjacent_wall", up_sample);
//	cv::waitKey();*/
//
//	//show_arr_on_map(arr, map, wall_info, 3);
//	// show_arr_on_map(arr, map,info);
//
//	float a, b;
//	std::cin >> a >> b;
//	std::cout << a << " " << b << std::endl;
//	GraphCut gw(room_map, map, info, wall_info, arr);
//	gw.setAlpha(a, b);
//	gw.setVertexLabel(graph);
//	saveCellArea("../area5/area.txt",graph);
//	saveCellLabel("../area5/gt.txt",graph);
//	cv::Mat gt = show_graph_label_on_map(graph, arr, map, wall_info.map_resolution, wall_info.map_origin, 1);
//	cv::imshow("truth", gt);
//	cv::waitKey();
//	gw.setVertexWeight(graph);
//	gw.setEdgeWight(graph, 3, 13);
//
//	gw.graph_cut(graph);
//
//	saveCellLabel("../area5/pred.txt", graph);
//	cv::Mat indoor_map = show_graph_label_on_map(graph, arr, map, info.map_resolution, info.map_origin);
//	cv::imshow("indoor", indoor_map);
//	cv::waitKey();
//
//	/*cv::Mat seg_map = cv::imread("gray_seg_map.png", 0);
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");*/
//}

//// 区域5
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../area5_full/info/pcInfo.txt", info);
//	readPCInfo("../area5_full/info/pcInfo.txt", wall_info);
//
//	cv::Mat map = cv::imread("../area5_full/wall_map.png", 0);
//	cv::Mat room_map = cv::imread("../area5_full/room_map.png", 0);
//	// 读取 Cell，将其转换为graph
//	std::string arr_path = "../area5_full/cell.dat";
//
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	/*cv::Mat adjacent_wall = show_all_adjacent_lines_on_map(graph, arr, map, wall_info);
//	cv::Mat up_sample;
//	cv::resize(adjacent_wall, up_sample, cv::Size(), 1, 1, cv::INTER_LINEAR);
//	cv::imshow("adjacent_wall", up_sample);
//	cv::waitKey();*/
//
//	show_arr_on_map(arr, map, wall_info, 1);
//	show_arr_on_map(arr, map,info);
//
//	float a, b;
//	std::cin >> a >> b;
//	std::cout << a << " " << b << std::endl;
//	GraphCut gw(room_map, map, info, wall_info, arr);
//	gw.setAlpha(a, b);
//	gw.setVertexLabel(graph);
//	saveCellArea("../area5_full/plane_graph_cut/area.txt",graph);
//	saveCellLabel("../area5_full/plane_graph_cut/gt.txt",graph);
//	cv::Mat gt = show_graph_label_on_map(graph, arr, map, wall_info.map_resolution, wall_info.map_origin, 1);
//	cv::imshow("truth", gt);
//	cv::waitKey();
//	gw.setVertexWeight(graph);
//	gw.setEdgeWight(graph, 3, 9);
//
//	gw.graph_cut(graph);
//
//	saveCellLabel("../area5_full/plane_graph_cut/pred.txt", graph);
//	cv::Mat indoor_map = show_graph_label_on_map(graph, arr, map, info.map_resolution, info.map_origin);
//	cv::imshow("indoor", indoor_map);
//	cv::waitKey();
//
//	/*cv::Mat seg_map = cv::imread("gray_seg_map.png", 0);
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");*/
//}


//// 区域4
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../area4/info/pcInfo.txt", info);
//	readPCInfo("../area4/info/pcInfo.txt", wall_info);
//
//	cv::Mat map = cv::imread("../area4/wall_map.png", 0);
//	cv::Mat room_map = cv::imread("../area4/room_map.png", 0);
//	// 读取 Cell，将其转换为graph
//	std::string arr_path = "../area4/cell.dat";
//
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	/*cv::Mat adjacent_wall = show_all_adjacent_lines_on_map(graph, arr, map, wall_info);
//	cv::Mat up_sample;
//	cv::resize(adjacent_wall, up_sample, cv::Size(), 1, 1, cv::INTER_LINEAR);
//	cv::imshow("adjacent_wall", up_sample);
//	cv::waitKey();*/
//
//	show_arr_on_map(arr, map, wall_info, 1);
//	show_arr_on_map(arr, map,info);
//
//	float a, b;
//	std::cin >> a >> b;
//	std::cout << a << " " << b << std::endl;
//	GraphCut gw(room_map, map, info, wall_info, arr);
//	gw.setAlpha(a, b);
//	gw.setVertexLabel(graph);
//	saveCellArea("../area4/plane_graph_cut/area.txt",graph);
//	saveCellLabel("../area4/plane_graph_cut/gt.txt",graph);
//	cv::Mat gt = show_graph_label_on_map(graph, arr, map, wall_info.map_resolution, wall_info.map_origin, 1);
//	cv::imshow("truth", gt);
//	cv::waitKey();
//	gw.setVertexWeight(graph);
//	gw.setEdgeWight(graph, 3, 9);
//
//	gw.graph_cut(graph);
//
//	saveCellLabel("../area4/plane_graph_cut/pred.txt", graph);
//	cv::Mat indoor_map = show_graph_label_on_map(graph, arr, map, info.map_resolution, info.map_origin);
//	cv::imshow("indoor", indoor_map);
//	cv::waitKey();
//
//	/*cv::Mat seg_map = cv::imread("gray_seg_map.png", 0);
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");*/
//}


//// 区域3
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../area3/info/pcInfo.txt", info);
//	readPCInfo("../area3/info/pcInfo.txt", wall_info);
//
//	cv::Mat map = cv::imread("../area3/wall_map.png", 0);
//	cv::Mat room_map = cv::imread("../area3/room_map.png", 0);
//	// 读取 Cell，将其转换为graph
//	std::string arr_path = "../area3/cell.dat";
//
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	/*cv::Mat adjacent_wall = show_all_adjacent_lines_on_map(graph, arr, map, wall_info);
//	cv::Mat up_sample;
//	cv::resize(adjacent_wall, up_sample, cv::Size(), 1, 1, cv::INTER_LINEAR);
//	cv::imshow("adjacent_wall", up_sample);
//	cv::waitKey();*/
//
//	show_arr_on_map(arr, map, wall_info, 1);
//	show_arr_on_map(arr, map,info);
//
//	float a, b;
//	std::cin >> a >> b;
//	std::cout << a << " " << b << std::endl;
//	GraphCut gw(room_map, map, info, wall_info, arr);
//	gw.setAlpha(a, b);
//	gw.setVertexLabel(graph);
//	saveCellArea("../area3/plane_graph_cut/area.txt",graph);
//	saveCellLabel("../area3/plane_graph_cut/gt.txt",graph);
//	cv::Mat gt = show_graph_label_on_map(graph, arr, map, wall_info.map_resolution, wall_info.map_origin, 1);
//	cv::imshow("truth", gt);
//	cv::waitKey();
//	gw.setVertexWeight(graph);
//	gw.setEdgeWight(graph, 3, 9);
//
//	gw.graph_cut(graph);
//
//	saveCellLabel("../area3/plane_graph_cut/pred.txt", graph);
//	cv::Mat indoor_map = show_graph_label_on_map(graph, arr, map, info.map_resolution, info.map_origin);
//	cv::imshow("indoor", indoor_map);
//	cv::waitKey();
//
//	/*cv::Mat seg_map = cv::imread("gray_seg_map.png", 0);
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");*/
//}