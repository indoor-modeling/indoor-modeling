#include "Trianglution.h"

unsigned findVertexIndex(Point pt, std::vector<Point> points)
{
	std::vector<Point>::iterator item = std::find(points.begin(), points.end(), pt);
	if (item != points.end())
	{
		unsigned pos = std::distance(points.begin(), item);
		return pos;
	}
	return -1;
}

TriIndices  GetTriIndices(std::vector<Point> points)
{
	// 先进行凹多边形划分凸多边形集合
	Polygon_list polys;

	CGAL::approx_convex_partition_2(points.begin(),
		points.end(), std::back_inserter(polys));

	//对于每个凸多边形
	CDT cdt;
	unsigned vertIndex;
	TriIndex triIndex;
	TriIndices triIndices;
	for (Polygon_list::iterator it = polys.begin(); it != polys.end(); ++it)
	{
		//约束三角剖分
		cdt.clear();
		cdt.insert(it->vertices_begin(), it->vertices_end());

		for (CDT::Finite_faces_iterator it = cdt.finite_faces_begin(); it != cdt.finite_faces_end(); ++it)
		{
			for (unsigned i = 0; i < 3; ++i)
			{
				Point point;
				point = it->vertex(i)->point();
				vertIndex = findVertexIndex(point, points);
				triIndex[i] = vertIndex;
			}
			triIndices.push_back(triIndex);
		}
	}
	return triIndices;
}

float Trianglution::area()
{
	float sum_area = 0;
	for (auto t : triangles)
	{
		sum_area += CGAL::to_double(t.area());
	}
	return sum_area;
}

void Trianglution::gen_random_points()
{
	float sum_area = area();
	int num_points = int(sum_area * points_per_area);
	if (num_points == 0)
		num_points = 1;
	CGAL::Random_points_in_triangles_2<Point> g(triangles);
	std::copy_n(g, num_points, std::back_inserter(random_points));
}

void Trianglution::triangle()
{
	TriIndices ind = GetTriIndices(cell_polygon);
	//std::cout << "Number of Face: " << ind.size() << std::endl;
	for (auto face : ind)
	{
		triangles.push_back(Kernel::Triangle_2(cell_polygon[face[0]], cell_polygon[face[1]], cell_polygon[face[2]]));
		//std::cout << triangles.back() << std::endl;
	}
}

std::vector<Point> Trianglution::get_random_points()
{
	return random_points;
}

void Trianglution::showTriangle()
{
	if (!triangles.empty())
	{
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("T Viewer"));
		int i = 0;
		for (auto triangle : triangles)
		{
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(CGAL::to_double(triangle[0].x()), CGAL::to_double(triangle[0].y()), 0),
				pcl::PointXYZ(CGAL::to_double(triangle[1].x()), CGAL::to_double(triangle[1].y()), 0),
				std::to_string(i++));
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(CGAL::to_double(triangle[1].x()), CGAL::to_double(triangle[1].y()), 0),
				pcl::PointXYZ(CGAL::to_double(triangle[2].x()), CGAL::to_double(triangle[2].y()), 0),
				std::to_string(i++));
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(CGAL::to_double(triangle[2].x()), CGAL::to_double(triangle[2].y()), 0),
				pcl::PointXYZ(CGAL::to_double(triangle[0].x()), CGAL::to_double(triangle[0].y()), 0),
				std::to_string(i++));
		}
		while (!viewer->wasStopped())
			viewer->spinOnce(100);
	}
}

void Trianglution::showRandomPoints()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("P Viewer"));
	if (!cell_polygon.empty())
	{
		int i = 0;
		for (int j = 0; j < cell_polygon.size(); j++) {
			if (j == cell_polygon.size() - 1)
			{
				viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(CGAL::to_double(cell_polygon[j].x() + 4), CGAL::to_double(cell_polygon[j].y() + 4), 0),
					pcl::PointXYZ(CGAL::to_double(cell_polygon[0].x() + 4), CGAL::to_double(cell_polygon[0].y() + 4), 0),
					1.0, 0.0, 0.0,
					"ori" + std::to_string(i++));
			}
			else
			{
				viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(CGAL::to_double(cell_polygon[j].x() + 4), CGAL::to_double(cell_polygon[j].y() + 4), 0),
					pcl::PointXYZ(CGAL::to_double(cell_polygon[j + 1].x() + 4), CGAL::to_double(cell_polygon[j + 1].y() + 4), 0),
					1.0, 0.0, 0.0,
					"ori" + std::to_string(i++));
			}
		}
	}
	if (!triangles.empty())
	{
		int i = 0;
		for (auto triangle : triangles)
		{
			auto color = pcl::getRandomColor();
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(CGAL::to_double(triangle[0].x()), CGAL::to_double(triangle[0].y()), 0),
				pcl::PointXYZ(CGAL::to_double(triangle[1].x()), CGAL::to_double(triangle[1].y()), 0),
				color.r / 255.0, color.g / 255.0, color.b / 255.0,
				std::to_string(i++));
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(CGAL::to_double(triangle[1].x()), CGAL::to_double(triangle[1].y()), 0),
				pcl::PointXYZ(CGAL::to_double(triangle[2].x()), CGAL::to_double(triangle[2].y()), 0),
				color.r / 255.0, color.g / 255.0, color.b / 255.0,
				std::to_string(i++));
			viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(CGAL::to_double(triangle[2].x()), CGAL::to_double(triangle[2].y()), 0),
				pcl::PointXYZ(CGAL::to_double(triangle[0].x()), CGAL::to_double(triangle[0].y()), 0),
				color.r / 255.0, color.g / 255.0, color.b / 255.0,
				std::to_string(i++));
		}
	}
	if (!random_points.empty())
	{
		int j = 0;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
		for (auto p : random_points)
		{
			pc->push_back(pcl::PointXYZ(CGAL::to_double(p.x()), CGAL::to_double(p.y()), 0));
		}
		viewer->addPointCloud<pcl::PointXYZ>(pc, "pc" + std::to_string(j++));
	}
	viewer->addText("Triangles: " + std::to_string(triangles.size()), 0, 50);
	viewer->addText("Random points: " + std::to_string(random_points.size()), 0, 100);
	viewer->addText("Area: " + std::to_string(area()), 0, 150);
	while (!viewer->wasStopped())
		viewer->spinOnce(100);
}

Trianglution::Trianglution(Face_handle f, int num_points_per_area)
{
	std::vector<Point> polygon_list;
	Arrangement::Ccb_halfedge_const_circulator outer_ccb = f->outer_ccb();
	Arrangement::Ccb_halfedge_const_circulator curr = outer_ccb;
	do {
		Arrangement::Halfedge_const_handle e = curr;
		polygon_list.push_back(e->source()->point());
	} while (++curr != outer_ccb);

	cell_polygon = polygon_list;
	points_per_area = num_points_per_area;

	triangle();
	gen_random_points();
}