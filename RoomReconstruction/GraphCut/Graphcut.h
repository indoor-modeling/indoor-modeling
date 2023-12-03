#pragma once
#include "Graph.h"
#include "visualization.h"
// 设置Graph的权重
//---------------------------------------------------------------------------------
class GraphCut
{
private:
	// 顶点初始标签
	int calVertexLabel(Face_handle f);
	// 顶点权重
	float calVertexWeight(Face_handle f, int flag);
	// 边权重
	float calEdgeWight(Face_handle source, Face_handle target, float sample_num_per_meter, int thickness = 1);

public:
	void setAlpha(float data_alpha, float smooth_alpha);
	void setVertexLabel(ArrGraph& graph);
	void setVertexWeight(ArrGraph& graph);
	/* 设置边权重
	 @param sample_num_per_meter 每米采样点数量
	 @param thickness 线条宽度
	*/
	void setEdgeWight(ArrGraph& graph, float sample_num_per_meter, int thickness = 1);

	void graph_cut(ArrGraph& graph);

public:
	float face_alpha = 0.0;
	float edge_alpha = 1;
	// 房间地图
	cv::Mat room_map;
	// 墙面地图
	cv::Mat wall_map;

	pcInfo _info;
	pcInfo _wall_info;
	// 地图分辨率
	float map_resolution;

	// 地图原点
	cv::Point2d map_origin;

	Face_index_map index_map;

	// 室内外阈值判断
	float thresold = 0.5;
public:
	GraphCut(const std::string& _room_path, const std::string& _wall_path, pcInfo info, pcInfo wall_info, Arrangement& arr);
	GraphCut(cv::Mat in_room_map, cv::Mat in_wall_map, pcInfo info, pcInfo wall_info, Arrangement& arr);
};

inline void saveCellLabel(std::string path,  ArrGraph& graph) 
{
	std::ofstream f(path);
	for (int i = 0; i < graph.g.m_vertices.size(); i++)
		f << graph.g.m_vertices[i].m_property.label << std::endl;
	f.close();

};
inline void saveCellArea(std::string path, ArrGraph& graph)
{
	std::ofstream f(path);
	for (int i = 0; i < graph.g.m_vertices.size(); i++)
		f << graph.g.m_vertices[i].m_property.area << std::endl;
	f.close();
};
//---------------------------------------------------------------------------------


