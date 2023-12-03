#pragma once

#include "include.h"
#include "MeanShift.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>  //ֱͨ�˲���ͷ�ļ�
#include <pcl/filters/voxel_grid.h>  //�����˲���ͷ�ļ�
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/statistical_outlier_removal.h>   //ͳ���˲���ͷ�ļ�
#include <pcl/filters/conditional_removal.h>    //�����˲���ͷ�ļ�
#include <pcl/filters/radius_outlier_removal.h>   //�뾶�˲���ͷ�ļ�
#include <pcl/ModelCoefficients.h> // ���ģ��ϵ�� ͷ�ļ�
#include <pcl/segmentation/sac_segmentation.h> // ransac�ָ�ͷ�ļ�
#include <pcl/filters/extract_indices.h> // �˲���ͷ�ļ�
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/project_inliers.h> // ͶӰͷ�ļ�
#include "..\refineLine\utils.hpp"


// �����Լ���pcl��������

struct EIGEN_ALIGN16 MyPointType    // enforce SSE padding for correct memory alignment
{
	PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
	float scalar_Label; // scalar_preds
	//float class; // scalar_class
	PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,		// here we assume a XYZ + "test" (as fields)  ע������ͺ�
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, scalar_Label, scalar_Label)
	//(float, class, class)
)


template<class PointT>
using Clouds_vector = std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<PointT> >;

using Coff_vector  = std::vector<pcl::ModelCoefficients>;


enum class Roomlabel
{
	ceiling,
	floor,
	outwall,
	window,
	door,
	clutter,
	roomwall,
	hallwall
};
enum class S3DISLabel
{
	ceiling,
	floor,
	wall,
	beam,
	column,
	window,
	door,
	table,
	chair,
	sofa,
	bookcase,
	board,
	clutter

};

// ����pcInfo
//---------------------------------------------------------------------------------
template<class T>
void setPCInfo(pcl::PointCloud<T>& cloud, pcInfo& info)
{
	T minp, maxp;
	pcl::getMinMax3D<T>(cloud, minp, maxp);
	info.minP.x = minp.x; info.minP.y = minp.y;
	info.maxP.x = maxp.x; info.maxP.y = maxp.y;
	info.map_origin = cv::Point2d(info.minP.x, info.minP.y);
	info.height = (int)(info.maxP.y - info.minP.y) / info.map_resolution;
	info.width = (int)(info.maxP.x - info.minP.x) / info.map_resolution;
}
void savePCInfo(std::string path, pcInfo& info);
void readPCInfo(std::string path, pcInfo& info);
//---------------------------------------------------------------------------------

// ��ȡ �ļ����������ļ���
//---------------------------------------------------------------------------------
void getFiles(const std::string& path, std::vector<std::string>& files);
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
//---------------------------------------------------------------------------------

// ��ȡ ply�ļ�
//---------------------------------------------------------------------------------
template<class T>
class PLY_IO {
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
public:
	static void read_ply(std::string file_path, PointCloudPtr cloud)
	{
		if (pcl::io::loadPLYFile(file_path, *cloud) < 0)
			PCL_ERROR("���ƶ�ȡʧ�ܣ�\n");
	}

	static void save_ply(std::string file_path, PointCloudPtr cloud) {
		if(pcl::io::savePLYFile(file_path, *cloud))
			PCL_ERROR("���Ʊ���ɹ���\n");
	}
};

//---------------------------------------------------------------------------------

// ����ͶӰ
//---------------------------------------------------------------------------------
template<class T>
class pcl_projection
{
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;

public:
	// ���캯�������� xoy�� ����ͶӰ
	pcl_projection()
	{
		/*
		* Ĭ���� xoy ƽ�� ͶӰ
		*/
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		coefficients->values.push_back(0.0);
		coefficients->values.push_back(0.0);
		coefficients->values.push_back(1.0);
		coefficients->values.push_back(0.0);
		this->coefficients = coefficients;
	}

	// ���캯������������ƽ�����ͶӰ
	pcl_projection(std::string plane)
	{
		if (plane == "xz")
		{
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
			coefficients->values.push_back(0.0);
			coefficients->values.push_back(1.0);
			coefficients->values.push_back(0.0);
			coefficients->values.push_back(0.0);
			this->coefficients = coefficients;
		}
		else if (plane == "yz")
		{
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
			coefficients->values.push_back(1.0);
			coefficients->values.push_back(0.0);
			coefficients->values.push_back(0.0);
			coefficients->values.push_back(0.0);
			this->coefficients = coefficients;
		}

	}

	// ���캯��������һƽ��ͶӰ
	pcl_projection(pcl::ModelCoefficients::Ptr coefficients)
	{
		this->coefficients = coefficients;
	}

public:
	void project(PointCloudPtr cloud, PointCloudPtr cloud_projected)
	{
		pcl::ProjectInliers<T> proj;
		proj.setInputCloud(cloud);
		proj.setModelCoefficients(coefficients);
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.filter(*cloud_projected);
	}
private:
	pcl::ModelCoefficients::Ptr coefficients;
};
//---------------------------------------------------------------------------------

// �߶���Ƭ
//---------------------------------------------------------------------------------
template <class PointT>
class PassThroughFilter
{
	typedef typename pcl::PointCloud<PointT>::Ptr pcPtr;
public:
	static void filter(pcPtr cloud, pcPtr filter_cloud, double thre_low, double thre_high, const bool& flag_in=false)
	{
		pcl::PassThrough<PointT> passthrough;
		passthrough.setInputCloud(cloud);//�������
		passthrough.setFilterFieldName("z");//��z����в���
		passthrough.setFilterLimits(thre_low, thre_high);//����ֱͨ�˲���������Χ
		passthrough.setFilterLimitsNegative(flag_in);//true��ʾ������Χ�⣬false��ʾ������Χ��
		passthrough.filter(*filter_cloud);//ִ���˲������˽�������� cloud_after_PassThrough
		std::cout << "ֱͨ�˲���������ݵ�����" << filter_cloud->size() << std::endl;
	}
};
//---------------------------------------------------------------------------------

// ȥ������----�뾶�˲�
//---------------------------------------------------------------------------------
template<class PointT>
void RadiusOutlierFilter(const pcl::PointCloud<PointT>& cloud, pcl::PointCloud<PointT>& filter_cloud,
						const double& radius, const int& thre_count);
template<class PointT>
void RadiusOutlierFilter(const pcl::PointCloud<PointT>& cloud, pcl::PointCloud<PointT>& filter_cloud, const double& radius, const int& thre_count)
{
	pcl::RadiusOutlierRemoval<PointT> radiusoutlier;  //�����˲���

	radiusoutlier.setInputCloud(cloud);    //�����������
	radiusoutlier.setRadiusSearch(radius);     //����radiusΪ100�ķ�Χ�����ٽ���
	radiusoutlier.setMinNeighborsInRadius(thre_count); //���ò�ѯ�������㼯��С��2��ɾ��

	radiusoutlier.filter(filter_cloud);
	std::cout << "�뾶�˲���������ݵ�����" << filter_cloud.size() << std::endl;
}
//---------------------------------------------------------------------------------

// �²���----�����˲�
//---------------------------------------------------------------------------------
template<class T>
class pcl_downSample
{
	typedef typename pcl::PointCloud<T>::Ptr PointCloudPtr;
public:
	/***************************************************************
		*  @brief     �����²���
		*  @param     cloud���������
		*  @param     cloud_filter���������
		*  @param     l/w/h���������س����
		*  @note      ��ע
		*  @Sample usage:    pcl_downSample<pcl::PointXYZ>::voxel_grid(cloud_project, cloud_sample, 0.2, 0.2, 0.2);
		**************************************************************/
	static void voxel_grid(PointCloudPtr cloud, PointCloudPtr cloud_filter, float l, float w, float h)
	{
		std::cout << "��ʼ�����˲�" << std::endl;
		std::cout << "�˲�ǰ������Ŀ��" << cloud->size() << std::endl;
		pcl::VoxelGrid<T> filter;
		filter.setInputCloud(cloud);
		// ��������դ��Ĵ�СΪ 1x1x1cm 0.01f
		// filter.setLeafSize(5.0f, 5.0f, 5.0f);
		filter.setLeafSize(l, w, h);
		filter.filter(*cloud_filter);
		std::cout << "�˲��������Ŀ��" << cloud_filter->size() << std::endl;
	}
};
//---------------------------------------------------------------------------------
 
// ���������ͼ.0-->�������壬1-->ǽ������
//---------------------------------------------------------------------------------
template<class PointT>
cv::Mat genLabelMap(pcl::PointCloud<PointT>& cloud, pcInfo info, int flag);
template<class PointT>
inline cv::Mat genLabelMap(pcl::PointCloud<PointT>& cloud, pcInfo info, int flag)
{
	cv::Mat map = cv::Mat::zeros(info.height, info.width, CV_8UC1);
	if (flag == 0)
	{
		for (int iter = 0; iter < cloud.size(); iter++)
		{
			int i = int((cloud[iter].x - info.minP.x) / info.map_resolution);
			if (i < 0 || i >= info.width) continue;

			int j = int((cloud[iter].y - info.minP.y) / info.map_resolution);
			if (j < 0 || j >= info.height - 1) continue;
			// if (cloud->points[iter].label != 0 && cloud->points[iter].label != 2 && cloud->points[iter].label != 3 && cloud->points[iter].label != 4)
			//if (cloud[iter].label == 2 || cloud[iter].label == 4 || cloud[iter].label == 5 || cloud[iter].label == 6)
			map.at<uchar>(j, i) = 255;
		}
	}
	else
	{
		for (int iter = 0; iter < cloud.size(); iter++)
		{
			int i = int((cloud[iter].x - info.minP.x) / info.map_resolution);
			if (i < 0 || i >= info.width) continue;

			int j = int((cloud[iter].y - info.minP.y) / info.map_resolution);
			if (j < 0 || j >= info.height - 1) continue;
			// if (cloud->points[iter].label != 0 && cloud->points[iter].label != 2 && cloud->points[iter].label != 3 && cloud->points[iter].label != 4)
			if (cloud[iter].label == 2 || cloud[iter].label == 4 || cloud[iter].label == 5 || cloud[iter].label == 6)
				map.at<uchar>(j, i) = 255;
		}
	}
	return map;
}
//---------------------------------------------------------------------------------

// ��ȡ�ض�����
//---------------------------------------------------------------------------------
void filterLabel(pcl::PointCloud<MyPointType>& in_cloud, pcl::PointCloud<MyPointType>& out_cloud, std::vector<int> labels);
//---------------------------------------------------------------------------------

// RANSAC ��ȡֱ��
//---------------------------------------------------------------------------------
template<class PointT>
class RANSACLines {
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
public:
	Clouds_vector<PointT> lines;
	Coff_vector coff;
public:
	void genWallLines(PointCloudPtr cloud, int maxIter, float maxMissPoint, float distanceThreshold)
	{
		PointCloudPtr outliers(new pcl::PointCloud<PointT>);
		pcl::copyPointCloud<PointT, PointT>(*cloud, *outliers);

		//-----------------------------���ֱ��-----------------------------
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());


		//�����ָ����
		pcl::SACSegmentation<PointT> seg;
		seg.setOptimizeCoefficients(true);

		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(maxIter);
		seg.setDistanceThreshold(distanceThreshold);

		//�����˲�������
		pcl::ExtractIndices<PointT> extract;
		int i = 0, nr_points = (int)cloud->size();


		//�����Ӵ��ı�׼����
		while (outliers->points.size() > maxMissPoint * nr_points)
		{
			// �����µĵ����зָ����ƽ����ɲ���
			seg.setInputCloud(outliers);
			seg.segment(*inliers, *coefficients);
			coff.push_back(*coefficients);
			if (inliers->indices.size() == 0 && i!=0)
			{
				std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			// �����ڲ�
			PointCloudPtr cloud_tmp(new pcl::PointCloud<PointT>);
			extract.setInputCloud(outliers);
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*cloud_tmp);

			lines.push_back(*cloud_tmp);
			// �����˲�������
			extract.setNegative(true);
			extract.filter(*cloud_tmp);
			*outliers = *cloud_tmp;
			std::cout << "��ȡ����" << i << "��ֱ��" << std::endl;
			i++;
		}
		std::cout << "�ָ�: " << i << "��ֱ��" << std::endl;
	}
	void readCoff(std::string path)
	{
		std::ifstream in;
		in.open(path, ios::in);
		pcl::ModelCoefficients coff_tmp;
		coff_tmp.values.resize(6);
		double x, y, z, a, b, c;
		while (in >> x >> y >> z >> a >> b >> c) {
			coff_tmp.values[0] = x; coff_tmp.values[1] = y; coff_tmp.values[2] = z;
			coff_tmp.values[3] = a; coff_tmp.values[4] = b; coff_tmp.values[5] = c;
			coff.push_back(coff_tmp);
		}
		in.close();
	}
	void readLines(std::string path) {
		std::vector<std::string> files;
		getFiles(path, files);
		auto it = files.begin();
		while (it != files.end())
		{
			PointCloudPtr cloud(new pcl::PointCloud<PointT>);
			if (pcl::io::loadPCDFile(*it, *cloud) < 0)
				std::cerr << *it << "* " << "load error" << std::endl;
			lines.push_back(*cloud);
			it++;
		}
	}
};
// void genWallLines(pcl::PointCloud<PointT>& cloud, Clouds_vector<PointT> lines, Coff_vector coff,  int maxIter, float maxMissPoint, float distanceThreshold);
//---------------------------------------------------------------------------------

// �����ȡ Clouds_vector
//---------------------------------------------------------------------------------
template<class PointT>
class Clouds_vector_IO {
public:
	static void save(std::string path, Clouds_vector<PointT>& lines)
	{
		int i = 0;
		for (auto line : lines)
			pcl::io::savePCDFileASCII(path + "/line_cloud_" + std::to_string(i++) + ".pcd", line);

	}

	static void read(std::string path, Clouds_vector<PointT>& lines)
	{
		typedef typename pcl::PointCloud<PointT>::Ptr pcPtr;
		std::vector<std::string> files;
		getFiles(path, files);
		auto it = files.begin();
		while (it != files.end())
		{
			pcPtr cloud(new pcl::PointCloud<PointT>);
			if (pcl::io::loadPCDFile(*it, *cloud) < 0)
				std::cerr << *it << "* " << "load error" << std::endl;
			lines.push_back(*cloud);
			it++;
		}

	}
};
//---------------------------------------------------------------------------------

// ����ֱ������Coff_vector
//---------------------------------------------------------------------------------
void saveCoff_vector(std::string path, Coff_vector coff);
void saveCoff_vector(std::string path, Coff_vector coff)
{
	std::ofstream out;
	out.open(path);
	for (auto c : coff)
		out << c.values[0] << " " << c.values[1] << " " << c.values[2] << " "
		<< c.values[3] << " " << c.values[4] << " " << c.values[5] << std::endl;
	out.close();

}
//---------------------------------------------------------------------------------

// ��ȡֱ������Coff_vector
//---------------------------------------------------------------------------------
void readCoff_vector(std::string path, Coff_vector& coff);
void readCoff_vector(std::string path, Coff_vector& coff)
{
	std::ifstream in;
	in.open(path, ios::in);
	pcl::ModelCoefficients coff_tmp;
	coff_tmp.values.resize(6);
	double x, y, z, a, b, c;
	while (in >> x >> y >> z >> a >> b >> c) {
		coff_tmp.values[0] = x; coff_tmp.values[1] = y; coff_tmp.values[2] = z;
		coff_tmp.values[3] = a; coff_tmp.values[4] = b; coff_tmp.values[5] = c;
		coff.push_back(coff_tmp);
	}
	in.close();

}
//---------------------------------------------------------------------------------

// �����ʺϵ� �����߶�, ����
//---------------------------------------------------------------------------------
template <typename PointT>
void genSegmens(Clouds_vector<PointT> lines, pcInfo info, std::vector<Segment>& res);
void genSegmens(std::vector<Segment>& lines, pcInfo info, std::vector<Segment>& res);
void saveSegmens(std::string path, std::vector<Segment>& res);
std::vector<Segment> readSegmens(std::string path);
//---------------------------------------------------------------------------------
 
// MeanShift ��ֱ��
//---------------------------------------------------------------------------------
//  Mean_Shift ms;
//	ms.setInputLines<PointT>(rc.load_lines);
//	ms.setKernel_bandwidth(5);
//	ms.fitLines();
//	ms.view<PointT>(cloud);
//  ms.saveLines("./res/sim_lines.txt");
//
class Mean_Shift
{
public:
	Mean_Shift();
	void setInputLines(std::vector<Segment> cloud_lines);
	void setKernel_bandwidth(double width) { kernel_bandwidth = width; };
	void fitLines();
	template<class PointT>
	void view(pcl::PointCloud<PointT> cloud);
	void saveLines(std::string filename);
public:
	

	MeanShift* msp;
	double kernel_bandwidth;
	// �򻯽����
	// model���򻯺�ֱ�߶˵� support��ԭֱ��
	std::vector<Cluster> shifted_source;
	std::vector<Cluster> shifted_target;
	std::vector <std::vector<double> > source, target;
	std::vector <std::vector<double> > source_shift, target_shift;
	std::vector <std::vector<double> > segment_simple;
	std::vector<Segment> seg;
};

Mean_Shift::Mean_Shift()
{
	msp = new MeanShift();
	kernel_bandwidth = 1;
}

void Mean_Shift::setInputLines(std::vector<Segment> cloud_lines)
{
	for (auto s : cloud_lines)
	{
		source.push_back(std::vector<double>{CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y())});
		target.push_back(std::vector<double>{CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y())});
	}
	
}
void Mean_Shift::fitLines()
{
	shifted_source = msp->cluster(source, kernel_bandwidth);
	shifted_target = msp->cluster(target, kernel_bandwidth);

	// �ҳ� ����� ԭ��� �� ���ĵ�
	for (int point = 0; point < source.size(); point++) {
		for (int cluster = 0; cluster < shifted_source.size(); cluster++) {
			auto source_res = std::find(shifted_source[cluster].original_points.begin(), shifted_source[cluster].original_points.end(), source[point]);
			if (source_res != shifted_source[cluster].original_points.end()) {
				source_shift.push_back(shifted_source[cluster].shifted_points[0]);
				break;
			}				
		}
	}
	// �ҳ� ����� ԭ�յ� �� ���ĵ�
	for (int point = 0; point < target.size(); point++) {
		for (int cluster = 0; cluster < shifted_target.size(); cluster++) {
			auto target_res = std::find(shifted_target[cluster].original_points.begin(), shifted_target[cluster].original_points.end(), target[point]);
			if (target_res != shifted_target[cluster].original_points.end()) {
				target_shift.push_back(shifted_target[cluster].shifted_points[0]);
				break;
			}
		}
	}


	for (int i = 0; i < source_shift.size(); i++) {
		std::vector<double> s{ source_shift[i][0], source_shift[i][1],target_shift[i][0], target_shift[i][1] };
		segment_simple.push_back(s);
	}
	std::sort(segment_simple.begin(), segment_simple.end());
	std::cout << "�߶�ȥ��ǰ��" << segment_simple.size()<< std::endl;
	segment_simple.erase(std::unique(segment_simple.begin(), segment_simple.end()), segment_simple.end());
	std::cout << "�߶�ȥ�غ�" << segment_simple.size() << std::endl;

	for (int i = 0; i < segment_simple.size(); i++) {
		seg.push_back(Segment(Point(segment_simple[i][0], segment_simple[i][1]), Point(segment_simple[i][2], segment_simple[i][3])));
	}

	// std::cout << "fit lines num: " << shifted_values.size() << std::endl;
}
template<class PointT>
void Mean_Shift::view(pcl::PointCloud<PointT> cloud)
{
	//----------------------------������ӻ�-----------------------------
	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Fit line Viewer"));

	//viewer->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), "cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

	//if (!shifted_values.empty())
	//{
	//	for (int i = 0; i < shifted_values.size(); i++)
	//	{
	//		//�������
	//		srand((unsigned)i);
	//		pcl::PointXYZ minP, maxP;
	//		minP.x = shifted_values[i].mode[0]; minP.y = shifted_values[i].mode[1];
	//		maxP.x = shifted_values[i].mode[2]; maxP.y = shifted_values[i].mode[3];
	//		std::cout << rand() / 255.0 << std::endl;
	//		viewer->addLine(minP, maxP, rand() / 255.0, rand() / 255.0, rand() / 255.0, "line_" + std::to_string(i), 0);
	//	}
	//}

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//}
}
void Mean_Shift::saveLines(std::string filename)
{
	/*std::ofstream outfile;
	outfile.open(filename);
	if (!shifted_values.empty())
	{
		for (int i = 0; i < shifted_values.size(); i++)
			outfile << shifted_values[i].mode[0] << " " << shifted_values[i].mode[1] << " " << shifted_values[i].mode[2] << " " << shifted_values[i].mode[3] << std::endl;
	}
	outfile.close();*/
}
//---------------------------------------------------------------------------------

// ���ӻ� Arrangement�ڷָ�ͼmap��
//---------------------------------------------------------------------------------
template <class PointT>
class pcl_vis {
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
public:
	static void show_Ransac_lines(PointCloudPtr cloud, Coff_vector lines) {
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("lines Viewer"));
		
		pcl::visualization::PointCloudColorHandlerRandom<PointT> rgb(cloud);
		viewer->addPointCloud(cloud, rgb,"cloud");
		int i = 0;
		for (auto c : lines) {
			viewer->addLine(c, "Line_" + std::to_string(i));
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Line_" + std::to_string(i));
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, 0, 0, "Line_" + std::to_string(i));
			i++;
		}
		
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
	}

	static void show_Ransac_lines(Clouds_vector<PointT> lines)
	{
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("lines Viewer"));
		int i = 0;
		for (auto line : lines)
		{
			pcl::visualization::PointCloudColorHandlerRandom<PointT> rgb(line.makeShared());
			viewer->addPointCloud(line.makeShared(), rgb, "line"+ std::to_string(i));
			i++;
		}
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
	}

	static void showPointCloud(PointCloudPtr cloud) {
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("cloud Viewer"));
		viewer->addPointCloud<PointT>(cloud, "cloud");
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
		}
	}
};
//---------------------------------------------------------------------------------

void savePCInfo(std::string path, pcInfo& info) {
	std::ofstream out(path);
	out << info.minP.x << " " << info.minP.y << " " << info.maxP.x << " " << info.maxP.y << std::endl;
	out << info.map_origin.x << " " << info.map_origin.y << std::endl;
	out << info.map_resolution << std::endl
		<< info.width << std::endl
		<< info.height << std::endl;
	out.close();
}

void readPCInfo(std::string path, pcInfo& info) {
	std::ifstream in(path);
	pcl::PointXYZ minp, maxp;
	cv::Point2d ori;
	in >> minp.x >> minp.y >> maxp.x >> maxp.y;
	info.minP = minp; info.maxP = maxp;
	in >> ori.x >> ori.y;
	info.map_origin = ori;
	in >> info.map_resolution >> info.width >> info.height;
	in.close();
}

template <typename PointT>
void genSegmens(Clouds_vector<PointT> lines, pcInfo info, std::vector<Segment>& res)
{
	std::vector<Line> CGAL_lines; // �����ֱ��
	std::vector<Segment> bbox; // ��Χ��
	std::vector<Segment> segments_intersection; // �ཻ���ֱ��

	// ת��ΪCGALֱ��
	for (auto line : lines)
	{
		PointT minP, maxP;
		pcl::getMinMax3D<PointT>(line, minP, maxP);
		CGAL_lines.push_back(Line(Point(minP.x, minP.y), Point(maxP.x, maxP.y)));
	}
		

	// ��ʼ����Χ��
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.maxP.x, info.minP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.maxP.x, info.minP.y)));

	// ����ÿ��ֱ�����Χ�еĽ��㣬��Ϊ���߶ζ˵�
	for (auto in_line : CGAL_lines)
	{
		std::vector<Point> seg;
		for (auto bbox_line : bbox)
		{
			auto result = CGAL::intersection(bbox_line, in_line);
			if (result) {
				/* not empty */
				if (const Point* p = boost::get<Point>(&*result)) {
					std::cout << "Points intersect:" << std::endl;
					std::cout << *p << std::endl;
					seg.push_back(*p);
				}
				else {
					std::cout << "ֱ�����Χ�й���" << std::endl;
					const Segment* s = boost::get<Segment>(&*result);
					std::cout << *s << std::endl;
				}
			}
			else {
				/* empty intersection */
				std::cout << "None intersection!" << std::endl;
			}
		}
		if (seg.size() == 2)
			segments_intersection.push_back(Segment(seg[0], seg[1]));
	}
	res = segments_intersection;
}

void genSegmens(std::vector<Segment>& lines, pcInfo info, std::vector<Segment>& res)
{
	std::vector<Line> CGAL_lines; // �����ֱ��
	std::vector<Segment> bbox; // ��Χ��
	std::vector<Segment> segments_intersection; // �ཻ���ֱ��

	
	// ת��ΪCGALֱ��
	for (auto line : lines)
		CGAL_lines.push_back(Line(line));


	// ��ʼ����Χ��
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.minP.x, info.minP.y), Point(info.maxP.x, info.minP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.minP.x, info.maxP.y)));
	bbox.push_back(Segment(Point(info.maxP.x, info.maxP.y), Point(info.maxP.x, info.minP.y)));

	// ����ÿ��ֱ�����Χ�еĽ��㣬��Ϊ���߶ζ˵�
	for (auto in_line : CGAL_lines)
	{
		std::vector<Point> seg;
		for (auto bbox_line : bbox)
		{
			auto result = CGAL::intersection(bbox_line, in_line);
			if (result) {
				/* not empty */
				if (const Point* p = boost::get<Point>(&*result)) {
					std::cout << "Points intersect:" << std::endl;
					std::cout << *p << std::endl;
					seg.push_back(*p);
				}
				else {
					std::cout << "ֱ�����Χ�й���" << std::endl;
					const Segment* s = boost::get<Segment>(&*result);
					std::cout << *s << std::endl;
				}
			}
			else {
				/* empty intersection */
				std::cout << "None intersection!" << std::endl;
			}
		}
		if (seg.size() == 2)
			segments_intersection.push_back(Segment(seg[0], seg[1]));
	}
	res = segments_intersection;
}

void saveSegmens(std::string path, std::vector<Segment>& res) {
	std::ofstream out(path);
	for (auto s : res)
		out << s.source() << " " << s.target() << std::endl;
	out.close();
}

std::vector<Segment> readSegmens(std::string path) {
	std::ifstream in(path);
	std::vector<Segment> segments;
	Point source, target;
	while (in >> source >> target)
		segments.push_back(Segment(source, target));
	in.close();
	return segments;
}



void filterLabel(pcl::PointCloud<MyPointType>& in_cloud, pcl::PointCloud<MyPointType>& out_cloud, std::vector<int> labels) {
	for (const auto p : in_cloud.points) {
		if (std::find(labels.begin(), labels.end(), (int)p.scalar_Label) != labels.end())
			out_cloud.push_back(p);
	}
}