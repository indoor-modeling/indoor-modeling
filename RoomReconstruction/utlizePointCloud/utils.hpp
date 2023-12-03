#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

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

// �����Լ���pcl��������

struct EIGEN_ALIGN16 MyPointType    // enforce SSE padding for correct memory alignment
{
	PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
	float scalar_class; // scalar_preds //scalar_Label
	//int label;
	//float class; // scalar_class
	PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,		// here we assume a XYZ + "test" (as fields)  ע������ͺ�
	(float, x, x)
	(float, y, y)
	(float, z, z)
	//(int, label, label)
	(float, scalar_class, scalar_class)
	//(float, class, class)
)

// ���Ʒָ�Ԥ����Ϣ
struct pcInfo
{
	pcl::PointXYZ minP, maxP;
	cv::Point2d map_origin;
	float map_resolution;
	int width, height;
};

template<class PointT>
using Clouds_vector = std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<PointT> >;

using Coff_vector = std::vector<pcl::ModelCoefficients>;

// �����ǩ
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

// ���ã����棬��ȡpcInfo
//---------------------------------------------------------------------------------
template<class T>
void setPCInfo(pcl::PointCloud<T>& cloud, pcInfo& info);
void savePCInfo(std::string path, pcInfo& info);
void readPCInfo(std::string path, pcInfo& info);
//---------------------------------------------------------------------------------

// ��ȡ �ļ����������ļ���
//---------------------------------------------------------------------------------
void getFiles(const std::string & path, std::vector<std::string>&files);
//---------------------------------------------------------------------------------

// ��ȡ������PLY�ļ�
//---------------------------------------------------------------------------------
template<class T>
void read_ply(std::string file_path, pcl::PointCloud<T>& cloud);
template<class T>
void save_ply(std::string file_path, pcl::PointCloud<T>& cloud);
//---------------------------------------------------------------------------------

// ����ͶӰ
//---------------------------------------------------------------------------------
class pcl_projection
{
public:
	// ���캯�������� xoy�� ����ͶӰ
	pcl_projection();
	// ���캯������������ƽ�����ͶӰ
	pcl_projection(std::string plane);
	// ���캯��������һƽ��ͶӰ
	pcl_projection(pcl::ModelCoefficients::Ptr coefficients);
	
	template<class T>
	void project(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& cloud_projected);
private:
	pcl::ModelCoefficients::Ptr coefficients;
	
};
//---------------------------------------------------------------------------------

// �߶���Ƭ
//---------------------------------------------------------------------------------
template <class T>
void PassThroughFilter(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& filter_cloud, double thre_low, double thre_high, const bool& flag_in = false);
//---------------------------------------------------------------------------------

// ȥ������----�뾶�˲�
//---------------------------------------------------------------------------------
template<class T>
void RadiusOutlierFilter(const pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& filter_cloud, const double& radius, const int& thre_count);
//---------------------------------------------------------------------------------

// �²���----�����˲�
//---------------------------------------------------------------------------------
template<class T>
void voxel_grid(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& cloud_filter, float l, float w, float h);
//---------------------------------------------------------------------------------

// ��ȡ�ض�����
//---------------------------------------------------------------------------------
void filterLabel(pcl::PointCloud<MyPointType>& in_cloud, pcl::PointCloud<MyPointType>& out_cloud, std::vector<int> labels);
//---------------------------------------------------------------------------------

// ���������ͼ
/*
* @param inverseLabel Ĭ��False����ȡlabels����֮��ȥ��labels��ͶӰʣ�������
*/
//---------------------------------------------------------------------------------
cv::Mat genSemanticMap(pcl::PointCloud<MyPointType>& in_cloud, pcInfo& info, std::vector<int> labels, bool inverseLabel=false);
//---------------------------------------------------------------------------------



template<class T>
void setPCInfo(pcl::PointCloud<T>& cloud, pcInfo& info)
{
	T minp, maxp;
	pcl::getMinMax3D<T>(cloud, minp, maxp);
	info.minP.x = minp.x; info.minP.y = minp.y;
	info.maxP.x = maxp.x; info.maxP.y = maxp.y;
	info.map_origin = cv::Point2d(info.minP.x, info.minP.y);
	info.height = (int)((info.maxP.y - info.minP.y) / info.map_resolution)+1;
	info.width = (int)((info.maxP.x - info.minP.x) / info.map_resolution)+1;
}

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

template<class T>
void read_ply(std::string file_path, pcl::PointCloud<T>& cloud) {
	if (pcl::io::loadPLYFile(file_path, cloud) < 0)
		PCL_ERROR("���ƶ�ȡʧ�ܣ�\n");
}
template<class T>
void save_ply(std::string file_path, pcl::PointCloud<T>& cloud) {
	if (pcl::io::savePLYFile(file_path, cloud))
		PCL_ERROR("���Ʊ���ɹ���\n");
}

template<class T>
inline void PassThroughFilter(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& filter_cloud, double thre_low, double thre_high, const bool& flag_in)
{
	pcl::PassThrough<T> passthrough;
	passthrough.setInputCloud(cloud);//�������
	passthrough.setFilterFieldName("z");//��z����в���
	passthrough.setFilterLimits(thre_low, thre_high);//����ֱͨ�˲���������Χ
	passthrough.setFilterLimitsNegative(flag_in);//true��ʾ������Χ�⣬false��ʾ������Χ��
	passthrough.filter(filter_cloud);//ִ���˲������˽�������� cloud_after_PassThrough
	std::cout << "ֱͨ�˲���������ݵ�����" << filter_cloud->size() << std::endl;
}

pcl_projection::pcl_projection() {
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

pcl_projection::pcl_projection(std::string plane) {
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
pcl_projection::pcl_projection(pcl::ModelCoefficients::Ptr coefficients)
{
		this->coefficients = coefficients;
}
template <class T>
void pcl_projection::project(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& cloud_projected) {
	
	pcl::ProjectInliers<T> proj;
	proj.setInputCloud(cloud.makeShared());
	proj.setModelCoefficients(coefficients);
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.filter(cloud_projected);
	
}

template<class T>
void RadiusOutlierFilter(const pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& filter_cloud, const double& radius, const int& thre_count) {
	pcl::RadiusOutlierRemoval<T> radiusoutlier;  //�����˲���

	radiusoutlier.setInputCloud(cloud);    //�����������
	radiusoutlier.setRadiusSearch(radius);     //����radiusΪ100�ķ�Χ�����ٽ���
	radiusoutlier.setMinNeighborsInRadius(thre_count); //���ò�ѯ�������㼯��С��2��ɾ��

	radiusoutlier.filter(filter_cloud);
	std::cout << "�뾶�˲���������ݵ�����" << filter_cloud.size() << std::endl;
}

template<class T>
inline void voxel_grid(pcl::PointCloud<T>& cloud, pcl::PointCloud<T>& cloud_filter, float l, float w, float h)
{
	std::cout << "��ʼ�����˲�" << std::endl;
	std::cout << "�˲�ǰ������Ŀ��" << cloud.size() << std::endl;
	pcl::VoxelGrid<T> filter;
	filter.setInputCloud(cloud.makeShared());
	// ��������դ��Ĵ�СΪ 1x1x1cm 0.01f
	// filter.setLeafSize(5.0f, 5.0f, 5.0f);
	filter.setLeafSize(l, w, h);
	filter.filter(cloud_filter);
	std::cout << "�˲��������Ŀ��" << cloud_filter.size() << std::endl;
}

void filterLabel(pcl::PointCloud<MyPointType>& in_cloud, pcl::PointCloud<MyPointType>& out_cloud, std::vector<int> labels) {
	for (const auto p : in_cloud.points) {
		if (std::find(labels.begin(), labels.end(), (int)p.scalar_class) != labels.end())
			out_cloud.push_back(p);
	}
}

inline cv::Mat genSemanticMap(pcl::PointCloud<MyPointType>& in_cloud, pcInfo& info, std::vector<int> labels, bool inverseLabel)
{
	cv::Mat map = cv::Mat::zeros(info.height, info.width, CV_8UC1);
	std::cout <<"�����ͼ�ߴ磨���ߣ���" << map.size() << std::endl;
	
	for (int iter = 0; iter < in_cloud.points.size(); iter++) {
		int i = int((in_cloud.points[iter].x - info.map_origin.x) / info.map_resolution);
		if (i < 0 || i >= info.width) continue;

		int j = int((in_cloud.points[iter].y - info.map_origin.y) / info.map_resolution);
		if (j < 0 || j >= info.height - 1) continue;

		if (inverseLabel == false) {
			if (!labels.empty()) {
				if (std::find(labels.begin(), labels.end(), in_cloud.points[iter].scalar_class) != labels.end())
					map.at<uchar>(j, i) = 255;
			}
			else
					map.at<uchar>(j, i) = 255;
		}
		else if (inverseLabel == true) {
			if (!labels.empty()) {
				if (std::find(labels.begin(), labels.end(), in_cloud.points[iter].scalar_class) == labels.end())
					map.at<uchar>(j, i) = 255;
			}
			else
					map.at<uchar>(j, i) = 255;
		}		
	}
	return map;
}
