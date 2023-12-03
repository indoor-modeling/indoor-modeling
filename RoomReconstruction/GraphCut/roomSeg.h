#pragma once
#include "Graph.h"
#include "visualization.h"

int cal_max_label(std::vector<int> point_label) {
	std::map<int, int> key_list;
	for (auto iter = point_label.begin(); iter != point_label.end(); iter++) {
		key_list[*iter]++;
	}
	/*std::map<int, int>::iterator best
		= std::max_element(key_list.begin(), key_list.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b)->bool { return a.second < b.second; });
	return best->first;*/
	std::vector<std::pair<int, int>> sortMap;
	for (auto it : key_list)
		sortMap.push_back(std::make_pair(it.first, it.second));
	std::sort(sortMap.begin(), sortMap.end(),
		[](const std::pair<int, int>& x, const std::pair<int, int>& y) -> int {
			return x.second > y.second; });
	int maxlabel;
	if(sortMap[0].first!=0)
		maxlabel=  sortMap[0].first;
	else if (sortMap.size()>1 && (sortMap[0].second - sortMap[1].second)/ point_label.size()<0.3)
		maxlabel= sortMap[1].first;
	else
		maxlabel= sortMap[0].first;
	if (maxlabel == 255)
		return 0;
	else
		return maxlabel;	
}



class roomSeg {	
	cv::Mat room_map;  // 房间地图	
	pcInfo room_info;  // info
	cv::Mat seg_room;
	std::map<int, std::vector<int>> label_with_room;  // one label --> rooms
	std::map<int, int> room_with_label; // one room --> one label
	std::vector<int> room_index;
	std::map<int, std::vector<Segment>> rooms_bound;
	std::map<int, std::vector<cv::Point2d>> rooms_bound_polylines;

	Face_index_map index_map;
	
public:
	void segment(ArrGraph& graph);
	void show_seg_map(std::string path);
	void get_rooms_bound();
	void show_rooms_bound();
	void save_wall_and_floor(std::string wall_path, std::string floor_path);
private:
	int calVertexLabel(Face_handle f);
	void room_bound(std::vector<int> face_idx, std::vector<Segment>& room_out_ccb);

public:
	roomSeg(cv::Mat& multi_room , pcInfo& info, Arrangement& arr);
};

inline void roomSeg::segment(ArrGraph& graph)
{
	for (int i = 0; i < graph.g.m_vertices.size(); i++)
	{
		std::cout << "Face " << i << " ";
		if (graph.g[i].label != 0) {
			auto v_l = calVertexLabel(index_map.face(graph.g[i].index));
			label_with_room[v_l].push_back(graph.g[i].index);
			room_with_label[graph.g[i].index] = v_l;
		}	
		else {
			label_with_room[0].push_back(graph.g[i].index);
			room_with_label[graph.g[i].index] = 0;
		}
	}
}

inline void roomSeg::show_seg_map(std::string path)
{
	
	seg_room = gen_multi_rooms_on_map(index_map, room_map, room_info, label_with_room);
	
	cv::imshow("sdasda", seg_room);
	int active = cv::waitKey();
	if(active==86)	
		cv::imwrite(path, seg_room);
}

inline int roomSeg::calVertexLabel(Face_handle f)
{
	/*cv::Mat img = show_face_on_map(f, room_map, _info);
	cv::imshow("face on map", img);
	cv::waitKey();*/
	auto tri = Trianglution(f);
	//tri.showRandomPoints();

	auto point_in_map = point2map(tri.get_random_points(), room_info);
	std::cout << "points num:" << point_in_map.size() << std::endl;

	auto face_point_label = point_label(point_in_map, room_map);

	auto max_label = cal_max_label(face_point_label);

	return max_label;

	/////////// 可视化 面
	//auto map1 = show_face_on_map(f, room_map, _info);
	///*cv::imshow("1", map1);
	//cv::waitKey();*/
	//auto map2 = show_random_points_on_map(point_in_map, map1, _info);
	//cv::imshow("2", map2);
	//cv::waitKey();
	/////////
}

inline void roomSeg::room_bound(std::vector<int> face_idx, std::vector<Segment>& room_out_ccb) {
	
	// 查找到所有外边
	std::vector<Arrangement::Ccb_halfedge_const_circulator> bound_edge;
	for (auto idx : face_idx) {
		auto curr_f = index_map.face(idx);
		Arrangement::Ccb_halfedge_const_circulator outer_ccb = curr_f->outer_ccb();
		Arrangement::Ccb_halfedge_const_circulator curr = outer_ccb;
		do {
			Arrangement::Halfedge_const_handle e = curr;
			// 找到 每个 面 的 相邻面,查询两个面是否属于同一室内
			Face_const_handle neighbour_face = e->twin()->face();
			if (!neighbour_face->is_unbounded())
			{
				// const-->non const
				Face_handle f = index_map.arrangement()->non_const_handle(neighbour_face);
				auto adjacent_face_idx = index_map[f];
				if (room_with_label[idx] != room_with_label[adjacent_face_idx]) {
					bound_edge.push_back(e);
				}
			}
			else
				bound_edge.push_back(e);
		} while (++curr != outer_ccb);
	}
	
	// 为所有外边排序
	auto begin_edge = bound_edge[0];
	auto curr_edge = begin_edge;
	room_out_ccb.push_back(Segment(begin_edge->source()->point(), begin_edge->target()->point()));
	do {
		auto next_edge_begin_point = curr_edge->target()->point();
		for (auto edge : bound_edge)
		{
			if (edge->source()->point() == next_edge_begin_point)
			{
				curr_edge = edge;
				room_out_ccb.push_back(Segment(curr_edge->source()->point(), curr_edge->target()->point()));
				break;
			}
		}
	} while (curr_edge!= begin_edge);
	room_out_ccb.pop_back();
	//show_Segments_on_map(room_out_ccb, room_map, room_info);

}

inline void roomSeg::get_rooms_bound() {
	// 0是室外，跳过
	for (int i = 1; i < label_with_room.size(); i++) {
		if (label_with_room[i].size() != 0) {
			std::cout << "get room:" << i << " bound";
			std::vector<Segment> room_out_ccb;
			room_bound(label_with_room[i], room_out_ccb);
			rooms_bound[i] = room_out_ccb;
			std::cout << " num edge:" << room_out_ccb.size() << std::endl;
		}	
	}

	for (auto room : rooms_bound)
	{
		std::vector<cv::Point2d> polyline;
		for (auto s : room.second) {
			polyline.push_back(cv::Point2d(CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y())));
		}
		// 加入终点，加入起点
		//polyline.push_back(cv::Point(CGAL::to_double(room.second[room.second.size() - 1].target().x()), CGAL::to_double(room.second[room.second.size() - 1].target().y())));
		polyline.push_back(cv::Point2d(CGAL::to_double(room.second[0].source().x()), CGAL::to_double(room.second[0].source().y())));
		rooms_bound_polylines[room.first] = polyline;
	}
}

inline void roomSeg::show_rooms_bound()
{
	cv::Mat bound_map = return_rooms_boundary(room_map, room_info, rooms_bound_polylines);
	cv::imshow("rooms_bound", bound_map);
	cv::waitKey();
}

inline void roomSeg::save_wall_and_floor(std::string wall_path, std::string floor_path)
{
	std::ofstream wall_f(wall_path), floor_f(floor_path);
	for (int i = 1; i <= rooms_bound.size(); i++)
	{
		wall_f << "#begin" << std::endl;
		for (int j = 0; j < rooms_bound[i].size(); j++)
		{
			wall_f << rooms_bound[i][j].source().x() << " " << rooms_bound[i][j].source().y() << " ";
			wall_f << rooms_bound[i][j].target().x() << " " << rooms_bound[i][j].target().y() <<std::endl;
		}
		wall_f << "#end" << std::endl;

		floor_f << "#begin" << std::endl;
		for (int j = 0; j < rooms_bound_polylines[i].size(); j++)
		{
			floor_f << rooms_bound_polylines[i][j].x << " " << rooms_bound_polylines[i][j].y <<std::endl;
		}
		floor_f << "#end"  << std::endl;
	}
	wall_f.close();
	floor_f.close();
}

roomSeg::roomSeg(cv::Mat& multi_room, pcInfo& info, Arrangement& arr) {
	room_map = multi_room;
	room_info = info;
	index_map = Face_index_map(arr);
}