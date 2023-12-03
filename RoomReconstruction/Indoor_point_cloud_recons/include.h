#ifndef ARR_EXACT_CONSTRUCTION_SEGMENTS_H
#define ARR_EXACT_CONSTRUCTION_SEGMENTS_H

#include <CGAL/IO/Arr_iostream.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_linear_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/graph_traits_dual_arrangement_2.h>
#include <CGAL/Arr_face_index_map.h>

#include <CGAL/Polygon_2.h>
// 三角剖分
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_conformer_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/point_generators_2.h>
#include <list>

// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>


// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// boost graph 
#include <CGAL/boost/graph/alpha_expansion_graphcut.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::FT                                        Number_type;
typedef CGAL::Arr_linear_traits_2<Kernel>                 Traits;
typedef Traits::Point_2                                   Point;
typedef Traits::Segment_2                                 Segment;
typedef Traits::Ray_2                                     Ray;
typedef Traits::Line_2                                    Line;
typedef Traits::Iso_rectangle_2                           Rectangle_2;
typedef Traits::X_monotone_curve_2                        X_monotone_curve;
typedef CGAL::Arrangement_2<Traits>                       Arrangement;
typedef Arrangement::Vertex_handle                        Vertex_handle;
typedef Arrangement::Halfedge_handle                      Halfedge_handle;
typedef Arrangement::Face_handle                          Face_handle;
typedef Arrangement::Face_const_handle					  Face_const_handle;
typedef CGAL::Arr_face_index_map<Arrangement>			  Face_index_map;

// 三角剖分
typedef CGAL::Exact_predicates_tag													Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, CGAL::Default, Itag>		CDT;
//typedef CDT::Point_2														Point;
typedef CGAL::Partition_traits_2<Kernel>											PTraits;
typedef PTraits::Polygon_2															Polygon2;
typedef std::list<Polygon2>															Polygon_list;


typedef CGAL::Polygon_2<Kernel> WPolygon;



// 点云分割预备信息
struct pcInfo
{
	pcl::PointXYZ minP, maxP;
	cv::Point2d map_origin;
	float map_resolution;
	int width, height;
};

// 二维数组
using Vector2d = std::vector<std::vector<double>>;



#endif
