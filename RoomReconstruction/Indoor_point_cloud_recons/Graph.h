#pragma once
#include "include.h"
#include "Trianglution.h"
#include "CGAL_utils.h"

// ͼGraph�Ľڵ�Vertex�ͱ�Edge����
struct Vertex_property
{
	int index; // ��¼��ӦArr�ڵ��index
	int label;	// �ڵ�ı�ǩ����Ӧ�ж��ٸ�����
	std::vector<double> cost;	//�ڵ�ĳɱ���cost.resize(label.size, 1)
};
struct Edge_property
{
	double weight;
};

// ����ͼ
using Graph = boost::adjacency_list <boost::setS,
	boost::vecS,
	boost::undirectedS,
	Vertex_property,
	Edge_property>;
using GT = boost::graph_traits<Graph>;
using vertex_descriptor = GT::vertex_descriptor;
using edge_descriptor = GT::edge_descriptor;


// Cellת����BGL��
// ʹ��ʾ��: ArrGraph graph;
//			Arrangement arr = graph.read_Arr(path);
//			graph.Arr2Graph(arr);
//			graph.g;
//---------------------------------------------------------------------------------
class ArrGraph
{
public:
	// ��ȡ�洢��Arr
	Arrangement read_Arr(std::string path);

	// ��Arrת����boost::graph
	void Arr2Graph(Arrangement& arr);
	// TODO------������������Ҫ��̬ȷ��

public:
	Graph g;
	// label ������Ŀ
	int num_room = 2;

	// ��Edge Ȩ��weight
	double EdgeWeight = 0.5;

	// ���������¼ �ڵ��Ƿ��Ѿ��洢
	// ͼ�ڵ�index��Arr��index
	std::vector<int> index_graph;
	std::vector<int> index_arr;
};
//---------------------------------------------------------------------------------

// ����Graph��Ȩ��
//---------------------------------------------------------------------------------
class GraphWeight
{
private:
	// �����ʼ��ǩ
	int calVertexLabel(Face_handle f);
	// ����Ȩ��
	float calVertexWeight(Face_handle f, int flag);
	// ��Ȩ��
	float calEdgeWight(Face_handle source, Face_handle target);

public:
	void setVertexLabel(ArrGraph graph);
	void setVertexWeight(ArrGraph graph);
	void setEdgeWight(ArrGraph graph);
public:
	float face_alpha = 0.3;
	float edge_alpha = 1 - face_alpha;
	// �����ͼ
	cv::Mat room_map;
	// ǽ���ͼ
	cv::Mat wall_map;

	pcInfo _info;
	// ��ͼ�ֱ���
	float map_resolution;

	// ��ͼԭ��
	cv::Point2d map_origin;

	Face_index_map index_map;

	// ��������ֵ�ж�
	float thresold = 0.5;
public:
	GraphWeight(const std::string& _room_path, const std::string& _wall_path, pcInfo info, Arrangement arr);
	GraphWeight(cv::Mat in_room_map, cv::Mat in_wall_map, pcInfo info, Arrangement arr);
};
//---------------------------------------------------------------------------------

// Graph cut �㷨
//---------------------------------------------------------------------------------
void graph_cut(ArrGraph graph);
//---------------------------------------------------------------------------------