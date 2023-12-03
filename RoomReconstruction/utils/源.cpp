#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/common/common.h>

#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointCloud<pcl::PointXYZL> pc;
void getFiles(const std::string& path, std::vector<std::string>& files)
{
	//�ļ����  
	long long hFile = 0;
	//�ļ���Ϣ��_finddata_t��Ҫio.hͷ�ļ�  
	struct _finddata_t fileinfo;
	std::string p;
	int i = 0;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//�����Ŀ¼,����֮  
			//�������,�����б�  
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				//if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					//getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

void getLabelPoints(pc& RoomPc, int label, pc& LabelPoints) {
	for (auto p : RoomPc.points)
	{
		if ((int)p.label == label)
			LabelPoints.push_back(p);
	}
}

void readRoomBound(std::string file, std::vector<pc>& bounds) {
	std::ifstream in(file);
	
	std::string line;
	while (getline(in, line)) {
		std::stringstream liness(line);
		double coord;
		std::vector<double> room;
		while (liness >> coord) {
			room.push_back(coord);
		}

		pc one_room;
		for (int i = 0; i < room.size()-1;i=i+2) {
			pcl::PointXYZL p;
			p.x = room[i];
			p.y = room[i + 1];
			p.z = 0;
			one_room.push_back(p);
		}
		bounds.push_back(one_room);
	}
	in.close();
}

void getRoomPc(pc& cloud, pc& bound, pc& RoomPc) {
	pcl::ConvexHull<pcl::PointXYZL> hull; //����һ��͹������
	hull.setInputCloud(bound.makeShared());           //�������úõ�͹����Χ
	hull.setDimension(2);				 //����͹����ά��
	std::vector<pcl::Vertices> polygons; //���ö�̬�������ڱ���͹������
	//���õ�����������͹������״
	pcl::PointCloud<pcl::PointXYZL>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZL>);
	hull.reconstruct(*surface_hull, polygons);	//����͹�����

	pcl::CropHull<pcl::PointXYZL> crop;	//����CropHull����
	crop.setDim(2);					    //����ά��
	crop.setInputCloud(cloud.makeShared());			//�����������
	crop.setHullIndices(polygons);		//�����ն���εĶ���
	crop.setHullCloud(surface_hull);	//�����ն���ε���״
	crop.filter(RoomPc);				//ִ��CropHull�˲����洢��Ӧ�Ľ��
	std::cout << "Cloud after crop has:" << RoomPc.size() << " data points." << std::endl;



	/*pc ceiling, floor;
	getLabelPoints(RoomPc, 0, ceiling);
	getLabelPoints(RoomPc, 1, floor);

	Eigen::Vector4f centroid_ceiling, centroid_floor;
	pcl::compute3DCentroid(ceiling, centroid_ceiling);
	pcl::compute3DCentroid(floor, centroid_floor);

	pcl::PointXYZL p1(centroid_ceiling[0], centroid_ceiling[1], centroid_ceiling[2]);
	pcl::PointXYZL p2(centroid_floor[0], centroid_floor[1], centroid_floor[2]);
	pcl::PointCloud<pcl::PointXYZL>::Ptr centroid_point(new pcl::PointCloud<pcl::PointXYZL>);
	centroid_point->push_back(p1); centroid_point->push_back(p2);*/

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("crophull display"));
	//view->setBackgroundColor(255, 255, 255);

	//int v1(0);
	//view->createViewPort(0.0, 0.0, 0.5, 1, v1);
	//view->setBackgroundColor(255, 255, 255, v1);
	//view->addPointCloud(cloud.makeShared(), "cloud", v1);
	//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
	//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	//view->addPolygon<pcl::PointXYZL>(surface_hull, 0, .069 * 255, 0.2 * 255, "backview_hull_polyLine1", v1);

	//int v2(0);
	//view->createViewPort(0.5, 0.0, 1, 1, v2);
	////view->setBackgroundColor(255, 255, 255, v2);
	//view->addPointCloud(RoomPc.makeShared(), "objects", v2);
	//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 1, "objects");
	//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "objects");
	///*view->addPointCloud(centroid_point, "centor", v2);
	//view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "centor");*/

	//while (!view->wasStopped())
	//{
	//	view->spinOnce(1000);
	//}

}

void getRoomHeight(){// ��ȡԭʼ����
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);//���ص�������
	pcl::PLYReader reader;
	reader.read("D:\\dev\\���ڵ��Ʒָ�\\CODE\\area3\\Area_3_sub_label.ply", *cloud);
	double floor_z = 0;
	pcl::PointXYZL minPt1, maxPt1;
	pcl::getMinMax3D(*cloud, minPt1, maxPt1);
	//floor_z = minPt1.z;
	floor_z = 0;

	// ��ȡÿ������߽�
	std::vector<pc> bounds;
	readRoomBound("D:\\dev\\���ڵ��Ʒָ�\\CODE\\area3\\seg_map\\area3.txt", bounds);

	// ����͹����ȡÿ�������Ӧ�ĵ���
	std::vector<pc> RoomsPc;
	for (auto bound : bounds) {
		pc roomPc;
		getRoomPc(*cloud, bound, roomPc);
		RoomsPc.push_back(roomPc);
	}
	/*pc roomPc;
	getRoomPc(*cloud, bounds[6], roomPc);
	RoomsPc.push_back(roomPc);*/

	// ��ȡÿ��������컨�塢�ذ岢�������
	std::vector<float> diss;
	for (auto room : RoomsPc) {
		pc ceiling, floor;
		getLabelPoints(room, 0, ceiling);
		// getLabelPoints(room, 1, floor);

		double ceiling_z = 0;
		pcl::PointXYZL minPt, maxPt;
		pcl::getMinMax3D(room, minPt, maxPt);

		if (!ceiling.empty())
		{
			for (auto p : ceiling)
				ceiling_z += p.z;
			ceiling_z /= ceiling.size();

		}
		if (ceiling_z < 2)
			ceiling_z = maxPt.z;
		/*if(!floor.empty()){
			for (auto p : floor)
				floor_z += p.z;
			floor_z /= floor.size();
		}
		else
		{
			pcl::PointXYZL minPt, maxPt;
			pcl::getMinMax3D(room, minPt, maxPt);
			floor_z = minPt.z;
		}*/

		double distance = std::abs(ceiling_z - floor_z);


		// ��������
		/*Eigen::Vector4f centroid_ceiling, centroid_floor;
		pcl::compute3DCentroid(ceiling, centroid_ceiling);
		pcl::compute3DCentroid(floor, centroid_floor);

		pcl::PointXYZ p1(centroid_ceiling[0], centroid_ceiling[1], centroid_ceiling[2]);
		pcl::PointXYZ p2(centroid_floor[0], centroid_floor[1], centroid_floor[2]);*/
		//auto distance = pcl::euclideanDistance(p1, p2);

		std::cout << distance << std::endl;
		diss.push_back(distance);
	}

	// ����ļ�
	std::ofstream out("D:\\dev\\���ڵ��Ʒָ�\\CODE\\area3\\seg_map\\area3full_with_height.txt");
	for (int i = 0; i < bounds.size(); i++)
	{
		for (auto p : bounds[i])
		{
			out << p.x << " " << p.y << " " << diss[i] << " ";
		}
		out << std::endl;
	}
	out.close();
}

void readPly() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//���ص�������
	pcl::PLYReader reader;
	reader.read("C:\\Users\\Van\\Desktop\\Enschededataset.ply", *cloud);
	std::cout << cloud->width << cloud->height << std::endl;


	// �������߹��ƶ���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	// ����һ���յ�KdTree���󣬲��������ݸ����߹��ƶ���
	// ���ڸ������������ݼ���KdTree��������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// �洢�������
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	// ʹ�ð뾶�ڲ�ѯ����Χ3���׷�Χ�ڵ������ٽ�Ԫ��
	ne.setRadiusSearch(0.03);

	// ���㷨��
	ne.compute(*cloud_normals);

	// �������
	for (size_t i = 0; i < cloud_normals->size(); ++i)
		std::cout << "Normal " << i << ": " << cloud_normals->points[i].normal_x << " "
		<< cloud_normals->points[i].normal_y << " "
		<< cloud_normals->points[i].normal_z << std::endl;

	// ���ӻ�
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}


void main()
{
	/*std::vector<std::string> files;
	getFiles("D:\\dev\\���ڵ��Ʒָ�\\CODE\\unbalance_small_scene\\line_pc", files);
	std::string out("D:\\dev\\���ڵ��Ʒָ�\\CODE\\unbalance_small_scene\\line_pcd\\");
	int i = 0;
	for (auto file : files) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::io::loadPLYFile<pcl::PointXYZRGB>(file, *cloud);
		for (auto p : cloud->points)
		{
			out_cloud->points.push_back(p);
		}
		pcl::io::savePLYFile<pcl::PointXYZRGB>(out+std::to_string(i)+".ply", *out_cloud);
		i++;
	}*/
	getRoomHeight();
	//readPly();
}