#pragma once
#include "Graph.h"
#include "visualization.h"
// ����Graph��Ȩ��
//---------------------------------------------------------------------------------
class GraphCut
{
private:
	// �����ʼ��ǩ
	int calVertexLabel(Face_handle f);
	// ����Ȩ��
	float calVertexWeight(Face_handle f, int flag);
	// ��Ȩ��
	float calEdgeWight(Face_handle source, Face_handle target, float sample_num_per_meter, int thickness = 1);

public:
	void setAlpha(float data_alpha, float smooth_alpha);
	void setVertexLabel(ArrGraph& graph);
	void setVertexWeight(ArrGraph& graph);
	/* ���ñ�Ȩ��
	 @param sample_num_per_meter ÿ�ײ���������
	 @param thickness �������
	*/
	void setEdgeWight(ArrGraph& graph, float sample_num_per_meter, int thickness = 1);

	void graph_cut(ArrGraph& graph);

public:
	float face_alpha = 0.0;
	float edge_alpha = 1;
	// �����ͼ
	cv::Mat room_map;
	// ǽ���ͼ
	cv::Mat wall_map;

	pcInfo _info;
	pcInfo _wall_info;
	// ��ͼ�ֱ���
	float map_resolution;

	// ��ͼԭ��
	cv::Point2d map_origin;

	Face_index_map index_map;

	// ��������ֵ�ж�
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


