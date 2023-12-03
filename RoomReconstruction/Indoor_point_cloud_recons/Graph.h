#pragma once
#include "include.h"
#include "Trianglution.h"
#include "CGAL_utils.h"

// 图Graph的节点Vertex和边Edge属性
struct Vertex_property
{
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
	// TODO------两个参数均需要动态确定

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

// 设置Graph的权重
//---------------------------------------------------------------------------------
class GraphWeight
{
private:
	// 顶点初始标签
	int calVertexLabel(Face_handle f);
	// 顶点权重
	float calVertexWeight(Face_handle f, int flag);
	// 边权重
	float calEdgeWight(Face_handle source, Face_handle target);

public:
	void setVertexLabel(ArrGraph graph);
	void setVertexWeight(ArrGraph graph);
	void setEdgeWight(ArrGraph graph);
public:
	float face_alpha = 0.3;
	float edge_alpha = 1 - face_alpha;
	// 房间地图
	cv::Mat room_map;
	// 墙面地图
	cv::Mat wall_map;

	pcInfo _info;
	// 地图分辨率
	float map_resolution;

	// 地图原点
	cv::Point2d map_origin;

	Face_index_map index_map;

	// 室内外阈值判断
	float thresold = 0.5;
public:
	GraphWeight(const std::string& _room_path, const std::string& _wall_path, pcInfo info, Arrangement arr);
	GraphWeight(cv::Mat in_room_map, cv::Mat in_wall_map, pcInfo info, Arrangement arr);
};
//---------------------------------------------------------------------------------

// Graph cut 算法
//---------------------------------------------------------------------------------
void graph_cut(ArrGraph graph);
//---------------------------------------------------------------------------------