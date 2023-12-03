#pragma once
#include "include.h"
#include "Trianglution.h"
#include "utils.h"

// 图Graph的节点Vertex和边Edge属性
struct Vertex_property
{
	float area;
	int index; // 记录对应Arr节点的index
	int label;	// 节点的标签，对应有多少个房间
	std::vector<double> cost;	//节点的成本，cost.resize(label.size, 1)
};
struct Edge_property
{
	double weight;
};

// 定义图
using Graph = boost::adjacency_list <boost::setS,
	boost::vecS,
	boost::undirectedS,
	Vertex_property,
	Edge_property>;
using GT = boost::graph_traits<Graph>;
using vertex_descriptor = GT::vertex_descriptor;
using edge_descriptor = GT::edge_descriptor;


// Cell转换到BGL中
// 使用示例: ArrGraph graph;
//			Arrangement arr = graph.read_Arr(path);
//			graph.Arr2Graph(arr);
//			graph.g;
//---------------------------------------------------------------------------------
class ArrGraph
{
public:
	// 读取存储的Arr
	Arrangement read_Arr(std::string path);

	// 将Arr转换到boost::graph
	void Arr2Graph(Arrangement& arr);
	

public:
	Graph g;
	// label 房间数目
	int num_room = 2;

	// 边Edge 权重weight
	double EdgeWeight = 0.5;

	// 创建数组记录 节点是否已经存储
	// 图节点index、Arr中index
	std::vector<int> index_graph;
	std::vector<int> index_arr;
};
//---------------------------------------------------------------------------------


