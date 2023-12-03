#include "Graph.h"
#include "Graphcut.h"
#include "visualization.h"
#include "roomSeg.h"

void readSegments(std::string path, std::vector<Segment>& _lines) {
	std::ifstream in;
	in.open(path);
	Segment s;
	std::string oneLine;
	while (in >> s)
		_lines.push_back(s);
	in.close();
}

void readLabel(std::string path, std::vector<int>& label) {
	std::ifstream f(path);
	float i;
	while (f >> i)
		label.push_back(int(i));
	f.close();
}

void init_graph_label(ArrGraph& graph, const std::vector<int>& label) {
	for (int i = 0; i < graph.g.m_vertices.size(); i++)
	{
		graph.g.m_vertices[i].m_property.label = label[i];
	}
}

////区域6
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../data/pcInfo.txt", info);
//	readPCInfo("../data/pcInfo.txt", wall_info);
//
//	//cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);
//	cv::Mat map = cv::imread("../data/wall_map_multi.png", 0);
//	//cv::Mat room_map = cv::imread("../data/grid_with_door2.png", 0);
//	cv::Mat room_map = cv::imread("../data/room_map_chen1.png", 0);
//	// 读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../Plane_line2_simply/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	show_arr_on_map(arr, room_map, info);
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	
//	std::vector<int> label;
//	readLabel("../data/plot_labels.txt", label);
//	init_graph_label(graph, label);
//	show_arr_on_map(arr, map,info);
//	cv::Mat error_map = return_error_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("error", error_map);
//	cv::waitKey();
//	cv::imwrite("../data/error_map.png", error_map);
//}

////区域5
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
//
//	std::vector<int> label;
//	readLabel("../area5/plot_labels.txt", label);
//	init_graph_label(graph, label);
//	show_arr_and_vertex_on_map(arr, map, info,1);
//	cv::Mat error_map = return_error_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("error", error_map);
//	cv::waitKey();
//	//cv::imwrite("../area5/error_map.png", error_map);
//}

// 区域5房间分割
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
//
//	std::vector<int> label;
//	readLabel("../area5/plot_labels.txt", label);
//	init_graph_label(graph, label);
//	cv::Mat img = show_graph_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("sad", img);
//	cv::waitKey();
//	cv::Mat seg_map = cv::imread("../area5/room_seg/area5_gray_seg_map2.png", 0);
//	std::string wall_path = "./wall2.txt";
//	std::string floor_path = "./floor2.txt";
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");
//	/*roomseg.get_rooms_bound();
//	roomseg.show_rooms_bound();
//	roomseg.save_wall_and_floor(wall_path, floor_path);*/
//}

//// 区域6房间分割
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../data/pcInfo.txt", info);
//	readPCInfo("../data/pcInfo.txt", wall_info);
//
//	//cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);
//	cv::Mat map = cv::imread("../data/wall_map_multi.png", 0);
//	//cv::Mat room_map = cv::imread("../data/grid_with_door2.png", 0);
//	cv::Mat room_map = cv::imread("../data/room_map_chen1.png", 0);
//	// 读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../Plane_line2_simply/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//
//	std::vector<int> label;
//	readLabel("../data/plot_labels.txt", label);
//	init_graph_label(graph, label);
//	cv::Mat img = show_graph_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("sad", img);
//	cv::waitKey();
//	 //将cell投影到房间语义图上
//	/*cv::Mat seg_map = cv::imread("seg_map.png", 1);
//	show_arr_on_map(arr, seg_map, info);*/
//	cv::Mat seg_map = cv::imread("gray_seg_map.png", 0);
//	std::string wall_path = "./wall_A6.txt";
//	std::string floor_path = "./floor_A6.txt";
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");
//	roomseg.get_rooms_bound();
//	roomseg.show_rooms_bound();
//	//roomseg.save_wall_and_floor(wall_path, floor_path);
//}

// //区域5全景房间分割
//void main() {
//	/*std::vector<Segment> _lines;
//	readSegments("../area5_full/regular_lines.txt", _lines);*/
//	//show_Segments_on_map(_lines, map, info);
//	
//	 //读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../area5_full/info/pcInfo.txt", info);
//	readPCInfo("../area5_full/info/pcInfo.txt", wall_info);
//
//	cv::Mat map = cv::imread("../area5_full/wall_map.png", 0);
//	cv::Mat room_map = cv::imread("../area5_full/room_map.png", 0);
//
//	
//	//读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../area5_full/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//
//	std::vector<int> label;
//	readLabel("../area5_full/plane_graph_cut/pred.txt", label);
//	init_graph_label(graph, label);
//	show_arr_and_vertex_on_map(arr, map, info, 1);
//	cv::Mat img = show_graph_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("sad", img);
//	cv::waitKey();
//
//	cv::Mat seg_map = cv::imread("../area5_full/seg_map/area5_full_gray_seg_map.png", 0);
//	std::string wall_path = "../area5_full/seg_map/wall.txt";
//	std::string floor_path = "../area5_full/seg_map/floor.txt";
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");
//	roomseg.get_rooms_bound();
//	roomseg.show_rooms_bound();
//	roomseg.save_wall_and_floor(wall_path, floor_path);
//}

//区域5全景 可视化 error map
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../area5_full/info/pcInfo.txt", info);
//	readPCInfo("../area5_full/info/pcInfo.txt", wall_info);
//
//	cv::Mat map = cv::imread("../area5_full/wall_map.png", 0);
//	cv::Mat room_map = cv::imread("../area5_full/room_map.png", 0);
//	// 读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../area5_full/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	
//	std::vector<int> label;
//	/*readLabel("../area5_full/calAcc/plot_labels.txt", label);*/
//	readLabel("../area5_full/plane_graph_cut/pred.txt", label);
//	init_graph_label(graph, label);
//	show_arr_on_map(arr, map,info);
//	cv::Mat mm = show_graph_label_on_map(graph, arr, map, 0.05, info.map_origin);
//	cv::imshow("sadasd", mm);
//	cv::waitKey();
//	/*cv::Mat error_map = return_error_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("error", error_map);
//	cv::waitKey();
//	cv::imwrite("../area5_full/calAcc/error_map.png", error_map);*/
//
//	cv::Mat seg_map = cv::imread("../area5_full/seg_map/area5_full_gray_seg_map.png", 0);
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");
//}

//区域4全景房间分割
//void main() {
//	/*std::vector<Segment> _lines;
//	readSegments("../area5_full/regular_lines.txt", _lines);*/
//	//show_Segments_on_map(_lines, map, info);
//	
//	 //读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../area4/info/pcInfo.txt", info);
//	readPCInfo("../area4/info/pcInfo.txt", wall_info);
//
//	cv::Mat map = cv::imread("../area4/wall_map.png", 0);
//	cv::Mat room_map = cv::imread("../area4/room_map.png", 0);
//
//	
//	//读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../area4/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//
//	std::vector<int> label;
//	readLabel("../area4/plane_graph_cut/pred.txt", label);
//	init_graph_label(graph, label);
//	cv::Mat img = show_graph_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("sad", img);
//	cv::waitKey();
//
//	cv::Mat seg_map = cv::imread("../area4/seg_map/area4_full_gray_seg_map.png", 0);
//	std::string wall_path = "../area4/seg_map/wall.txt";
//	std::string floor_path = "../area4/seg_map/floor.txt";
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");
//	roomseg.get_rooms_bound();
//	roomseg.show_rooms_bound();
//	//roomseg.save_wall_and_floor(wall_path, floor_path);
//}

//区域4全景 可视化 error map
//void main() {
//	// 读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../area4/info/pcInfo.txt", info);
//	readPCInfo("../area4/info/pcInfo.txt", wall_info);
//
//	cv::Mat map = cv::imread("../area4/wall_map.png", 0);
//	cv::Mat room_map = cv::imread("../area4/room_map.png", 0);
//	// 读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../area4/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//	
//	std::vector<int> label;
//	readLabel("../area4/plane_graph_cut/plot_labels.txt", label);
//	//readLabel("../area4/plane_graph_cut/pred.txt", label);
//	init_graph_label(graph, label);
//	/*show_arr_on_map(arr, map,info);
//	cv::Mat mm = show_graph_label_on_map(graph, arr, map, 0.05, info.map_origin);
//	cv::imshow("sadasd", mm);
//	cv::waitKey();*/
//	cv::Mat error_map = return_error_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("error", error_map);
//	cv::waitKey();
//	cv::imwrite("../area5_full/calAcc/error_map.png", error_map);
//}


////区域4全景房间分割
//void main() {
//	/*std::vector<Segment> _lines;
//	readSegments("../area5_full/regular_lines.txt", _lines);*/
//	//show_Segments_on_map(_lines, map, info);
//	
//	 //读取 预备信息
//	pcInfo info, wall_info;
//	readPCInfo("../area3/info/pcInfo.txt", info);
//	readPCInfo("../area3/info/pcInfo.txt", wall_info);
//
//	cv::Mat map = cv::imread("../area3/wall_map.png", 0);
//	cv::Mat room_map = cv::imread("../area3/room_map.png", 0);
//
//	
//	//读取 Cell，将其转换为graph
//	//std::string arr_path = "../res/Cell/cell.dat";
//	std::string arr_path = "../area3/cell.dat";
//	ArrGraph graph;
//	Arrangement arr = graph.read_Arr(arr_path);
//	graph.Arr2Graph(arr);
//
//	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
//	std::cout << "边：" << graph.g.m_edges.size() << std::endl;
//
//	std::vector<int> label;
//	readLabel("../area3/plane_graph_cut/pred.txt", label);
//	init_graph_label(graph, label);
//	cv::Mat img = show_graph_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
//	cv::imshow("sad", img);
//	cv::waitKey();
//
//
//	cv::Mat seg_map = cv::imread("../area3/seg_map/area3_full_gray_seg_map.png", 0);
//	std::string wall_path = "../area3/seg_map/wall.txt";
//	std::string floor_path = "../area3/seg_map/floor.txt";
//	roomSeg roomseg(seg_map, info, arr);
//	roomseg.segment(graph);
//	roomseg.show_seg_map("seg.png");
//	roomseg.get_rooms_bound();
//	//roomseg.show_rooms_bound();
//	//roomseg.save_wall_and_floor(wall_path, floor_path);
//}


//区域4 error_map
void main() {
	/*std::vector<Segment> _lines;
	readSegments("../area5_full/regular_lines.txt", _lines);*/
	//show_Segments_on_map(_lines, map, info);

	 //读取 预备信息
	pcInfo info, wall_info;
	readPCInfo("../area3/info/pcInfo.txt", info);
	readPCInfo("../area3/info/pcInfo.txt", wall_info);

	cv::Mat map = cv::imread("../area3/wall_map.png", 0);
	cv::Mat room_map = cv::imread("../area3/room_map.png", 0);


	//读取 Cell，将其转换为graph
	//std::string arr_path = "../res/Cell/cell.dat";
	std::string arr_path = "../area3/cell.dat";
	ArrGraph graph;
	Arrangement arr = graph.read_Arr(arr_path);
	graph.Arr2Graph(arr);

	std::cout << "节点：" << graph.g.m_vertices.size() << std::endl;
	std::cout << "边：" << graph.g.m_edges.size() << std::endl;


	std::vector<int> label;
	readLabel("../area3/calAcc/plot_labels.txt", label);
	init_graph_label(graph, label);
	show_arr_on_map(arr, map,info);
	cv::Mat error_map = return_error_label_on_map(graph, arr, room_map, info.map_resolution, info.map_origin);
	cv::imshow("error", error_map);
	cv::waitKey();
	cv::imwrite("../area3/calAcc/error_map.png", error_map);
}