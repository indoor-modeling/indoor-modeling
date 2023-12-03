#pragma once
#include "include.h"
#include "Trianglution.h"
#include "utils.h"

// ͼGraph�Ľڵ�Vertex�ͱ�Edge����
struct Vertex_property
{
	float area;
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


