#pragma once
#include "include.h"
#include "utils.h"
#include "Graph.h"

//���ӻ� Face���ָ�ͼmap��
//---------------------------------------------------------------------------------
cv::Mat show_face_on_map(Face_handle f, cv::Mat map, pcInfo info);
//---------------------------------------------------------------------------------

//���ӻ� ���ڱ� ���ָ�ͼmap��
//---------------------------------------------------------------------------------
cv::Mat show_adjacent_on_map(Halfedge_handle edge, cv::Mat map, pcInfo info);
//---------------------------------------------------------------------------------

// ���ӻ� �� �� �ָ�ͼmap��
//---------------------------------------------------------------------------------
cv::Mat show_random_points_on_map(std::vector<Point> random_points, cv::Mat map, pcInfo info);
//---------------------------------------------------------------------------------

// ���ӻ� Arrangement�ڷָ�ͼmap��
//---------------------------------------------------------------------------------
void show_arr_on_map(Arrangement arr, cv::Mat map, pcInfo info, float resize=1.0);
//---------------------------------------------------------------------------------

// ���ӻ� �߶� �ָ�ͼmap��
//---------------------------------------------------------------------------------
void show_Segments_on_map(std::vector<Segment>& segments, cv::Mat map, pcInfo info);
//---------------------------------------------------------------------------------

// ���ӻ� Graph label���ָ�ͼmap��
//---------------------------------------------------------------------------------
cv::Mat show_graph_label_on_map(ArrGraph graph, Arrangement arr, cv::Mat map, float map_resolution, cv::Point2d map_origin, int resize = 1);
//---------------------------------------------------------------------------------

//���ӻ� �������ڱ� ���ָ�ͼmap��
//---------------------------------------------------------------------------------
cv::Mat show_all_adjacent_lines_on_map(ArrGraph& graph, Arrangement& arr, cv::Mat map, pcInfo info);
//---------------------------------------------------------------------------------

cv::Mat gen_multi_rooms_on_map(Face_index_map index_map , cv::Mat map, pcInfo info, std::map<int, std::vector<int>> label_with_rooms);

cv::Mat return_error_label_on_map(ArrGraph graph, Arrangement arr, cv::Mat map, float map_resolution, cv::Point2d map_origin);

void show_arr_and_vertex_on_map(Arrangement arr, cv::Mat map, pcInfo info, float resize);

cv::Mat return_rooms_boundary(cv::Mat map, pcInfo info, std::map<int, std::vector<Segment>> rooms_bound);

cv::Mat return_rooms_boundary(cv::Mat map, pcInfo info, std::map<int, std::vector<cv::Point2d>> rooms_bound);