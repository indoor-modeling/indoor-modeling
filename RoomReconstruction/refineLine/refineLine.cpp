#define PCL_NO_PRECOMPILE
#include "utils.hpp"
#include "MeanShift.h"


void show_Segments_on_map(std::vector<std::vector<double> >& segments, cv::Mat map, pcInfo info,int resize=1)
{
    cv::Mat segments_on_map(map.size(), CV_8UC3);
    if (map.type() == CV_8UC1)
    {
        // 构造等大小的彩色图
        for (int y = 0; y < map.rows; y++)
            for (int x = 0; x < map.cols; x++)
            {
                auto gray = map.at<uchar>(y, x);
                cv::Vec3b& pixel = segments_on_map.at<cv::Vec3b>(y, x);
                if (gray == 0) {
                    pixel[0] = cv::abs(0);
                    pixel[1] = cv::abs(0);
                    pixel[2] = cv::abs(0);
                }
                else
                {
                    pixel[0] = cv::abs(255);
                    pixel[1] = cv::abs(255);
                    pixel[2] = cv::abs(255);
                }
               
            }
    }
    else
        segments_on_map = map.clone();
    for (auto s : segments) {
        auto source = cv::Point2d(((s[0] - info.map_origin.x) / info.map_resolution),
            ((s[1] - info.map_origin.y) / info.map_resolution));
        auto target = cv::Point2d(((s[2] - info.map_origin.x) / info.map_resolution),
            (((s[3]) - info.map_origin.y) / info.map_resolution));
        cv::line(segments_on_map, source, target, cv::Scalar(0, 0, 255));
    }
    cv::Mat up_sample;
    cv::resize(segments_on_map, up_sample, cv::Size(), resize, resize, cv::INTER_LINEAR);
    cv::imshow("segments_on_map", up_sample);
    cv::waitKey();

}

cv::Mat return_Segments_on_map(std::vector<std::vector<double> >& segments, cv::Mat map, pcInfo info, int resize = 1)
{
    cv::Mat segments_on_map(map.size(), CV_8UC3);
    if (map.type() == CV_8UC1)
    {
        // 构造等大小的彩色图
        for (int y = 0; y < map.rows; y++)
            for (int x = 0; x < map.cols; x++)
            {
                auto gray = map.at<uchar>(y, x);
                cv::Vec3b& pixel = segments_on_map.at<cv::Vec3b>(y, x);
                if (gray == 0) {
                    pixel[0] = cv::abs(255);
                    pixel[1] = cv::abs(255);
                    pixel[2] = cv::abs(255);
                }
                else
                {
                    pixel[0] = cv::abs(106);
                    pixel[1] = cv::abs(55);
                    pixel[2] = cv::abs(66);
                }

            }
    }
    else
        segments_on_map = map.clone();
    for (auto s : segments) {
        auto source = cv::Point2d(((s[0] - info.map_origin.x) / info.map_resolution),
            ((s[1] - info.map_origin.y) / info.map_resolution));
        auto target = cv::Point2d(((s[2] - info.map_origin.x) / info.map_resolution),
            (((s[3]) - info.map_origin.y) / info.map_resolution));
        cv::line(segments_on_map, source, target, cv::Scalar(92, 193, 252));
    }
    cv::Mat up_sample;
    cv::resize(segments_on_map, up_sample, cv::Size(), resize, resize, cv::INTER_LINEAR);
    return up_sample;

}

void readLines(const std::string& path, std::vector<std::vector<double>>& lines) {
    std::vector<std::string> files_name;
    getFiles(path, files_name);
    for (int i = 0; i < files_name.size(); i++)
    {
        std::ifstream in(files_name[i]);
        std::vector<double> all_data;
        std::vector<double> line;
        double tmp;
        while (in >> tmp)
            all_data.push_back(tmp);
        for (int i = 0; i < 3; i++) {
            line.push_back(all_data[i]);
        }
        for (int i = 20; i > 17; i--)
            line.push_back(all_data[i]);
        lines.push_back(line);
    }   
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



//void main() {
//    // 读取基本信息
//	pcInfo info;
//	readPCInfo("../data/pcInfo.txt", info);
//    // 读取 底图
//    //cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);
//    cv::Mat map = cv::imread("../res/wall_point.png", 0);
//    // 选择保存路径
//    ofstream out("../res/refineLine/simple_lines.txt");
//
//	// 读取 直线点云
//	Clouds_vector<MyPointType> lines;
//	read_Cloud_vector<MyPointType>("../res/linePC_new", lines);
//
//	// 将点云生成在包围盒内的线段
//	std::vector<Segment> segments;
//	genSegmens<MyPointType>(lines, info, segments);
//
//	// 将线段转换为MS算法可用的格式
//	std::vector<std::vector<double> > segments_in_ms;
//	for (auto s : segments)
//		segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//									CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//	MeanShift* msp = new MeanShift();
//	double kernel_bandwidth = 0.2;
//
//	std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//	printf("\n====================\n");
//	printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//	printf("简化之后 %lu 条线段\n", clusters.size());
//	printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    show_Segments_on_map(segments_in_ms, map, info);
//    show_Segments_on_map(segments_res, map, info);
//
//    for (int cluster = 0; cluster < clusters.size(); cluster++) {
//        printf("Cluster %i:\n", cluster);
//        for (int point = 0; point < clusters[cluster].original_points.size(); point++) {
//            for (int dim = 0; dim < clusters[cluster].original_points[point].size(); dim++) {
//                printf("%f ", clusters[cluster].original_points[point][dim]);
//            }
//            printf(" -> ");
//            for (int dim = 0; dim < clusters[cluster].shifted_points[point].size(); dim++) {
//                printf("%f ", clusters[cluster].shifted_points[point][dim]);
//            }
//            printf("\n");
//        }
//        printf("\n");
//        out << clusters[cluster].shifted_points[0][0] << " " << clusters[cluster].shifted_points[0][1] << " "
//            << clusters[cluster].shifted_points[0][2] << " " << clusters[cluster].shifted_points[0][3] << " " << std::endl;
//    }
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//    /*auto regualrSegments = regularSegementDirection(seg_last, 5);
//    for (int i = 0; i < regualrSegments.size(); i++) {
//        if (regualrSegments[i] != seg_last[i])
//            std::cout << "ori:" << seg_last[i] << " --> " << "after:" << regualrSegments[i] << std::endl;
//    }*/
//    //saveSegmens("../res1/refineLine/simple_lines_3.txt", seg_last);
//    out.close();
//}

//按平面提取的直线的简化
//void main() {
//    // 读取基本信息
//    pcInfo info;
//    readPCInfo("../data/pcInfo.txt", info);
//
//    // 读取 底图
//    //cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);//绘制在有横梁的墙面图
//     cv::Mat map = cv::imread("../res/wall_point.png", 0);// 绘制在线的点云上
//    // 读取 直线
//    std::string path = "../Plane_line2";
//    // 选择保存路径
//    //ofstream out("../Plane_line2_simply/simple_lines.txt");
//
//    // 读取的直线并延长与包围盒相交
//    std::vector<std::vector<double>> lines;
//    readLines(path, lines);
//    std::vector<Segment> segments;
//    genSegments(lines, info, segments);
//
//    // 将线段转换为MS算法可用的格式
//    std::vector<std::vector<double> > segments_in_ms;
//    for (auto s : segments)
//        segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//    MeanShift* msp = new MeanShift();
//    double kernel_bandwidth = 0.1;
//
//    std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//    printf("\n====================\n");
//    printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//    printf("简化之后 %lu 条线段\n", clusters.size());
//    printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    cv::Mat ori = return_Segments_on_map(segments_in_ms, map, info);
//    cv::Mat simply = return_Segments_on_map(segments_res, map, info);
//    cv::imwrite("../Plane_line2_simply/ori_lines.png", ori);
//    cv::imwrite("../Plane_line2_simply/simply_lines.png", simply);
//
//    for (int cluster = 0; cluster < clusters.size(); cluster++) {
//        printf("Cluster %i:\n", cluster);
//        for (int point = 0; point < clusters[cluster].original_points.size(); point++) {
//            for (int dim = 0; dim < clusters[cluster].original_points[point].size(); dim++) {
//                printf("%f ", clusters[cluster].original_points[point][dim]);
//            }
//            printf(" -> ");
//            for (int dim = 0; dim < clusters[cluster].shifted_points[point].size(); dim++) {
//                printf("%f ", clusters[cluster].shifted_points[point][dim]);
//            }
//            printf("\n");
//        }
//        printf("\n");
//        /*out << clusters[cluster].shifted_points[0][0] << " " << clusters[cluster].shifted_points[0][1] << " "
//            << clusters[cluster].shifted_points[0][2] << " " << clusters[cluster].shifted_points[0][3] << " " << std::endl;*/
//    }
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//
//    //saveSegmens("../Plane_line2_simply/simple_lines_3.txt", seg_last);
//   // out.close();
//}

//  毕业论文绘制简化直线效果图
//void main(){
//    // 读取基本信息
//    pcInfo info;
//    readPCInfo("../data/pcInfo.txt", info);
//    // 绘制空白底图  
//    cv::Mat map = cv::imread("../seg_map.png");
//    //cv::Mat map = cv::Mat(info.height, info.width, CV_8UC3, cv::Scalar(255, 255, 255));
//    // 读取 直线点云
//    Clouds_vector<MyPointType> lines;
//    read_Cloud_vector<MyPointType>("../res/linePC", lines);
//
//    // 将点云生成在包围盒内的线段
//    std::vector<Segment> segments;
//    genSegmens<MyPointType>(lines, info, segments);
//
//    // 将线段转换为MS算法可用的格式
//    std::vector<std::vector<double> > segments_in_ms;
//    for (auto s : segments)
//        segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//    MeanShift* msp = new MeanShift();
//    double kernel_bandwidth = 0.2;
//
//    std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//    printf("\n====================\n");
//    printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//    printf("简化之后 %lu 条线段\n", clusters.size());
//    printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    cv::Mat ori = return_Segments_on_map(segments_in_ms, map, info);
//    cv::Mat aft = return_Segments_on_map(segments_res, map, info);
//    cv::imwrite("../ori_line2.png", ori);
//    cv::imwrite("../refine_line2.png", aft);
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//    /*auto regualrSegments = regularSegementDirection(seg_last, 5);
//    for (int i = 0; i < regualrSegments.size(); i++) {
//        if (regualrSegments[i] != seg_last[i])
//            std::cout << "ori:" << seg_last[i] << " --> " << "after:" << regualrSegments[i] << std::endl;
//    }*/
//    //saveSegmens("../res1/refineLine/simple_lines_3.txt", seg_last);
//}


//  小场景实验
//void main() {
//    // 读取基本信息
//    pcInfo info;
//    readPCInfo("../unbalance_small_scene/room_info.txt", info);
//    // 绘制空白底图  
//    cv::Mat map = cv::Mat(info.height, info.width, CV_8UC3, cv::Scalar(255, 255, 255));
//    // 读取 直线点云
//    Clouds_vector<MyPointType> lines;
//    read_Cloud_vector<MyPointType>("../unbalance_small_scene/line_pc", lines);
//
//    // 将点云生成在包围盒内的线段
//    std::vector<Segment> segments;
//    genSegmens<MyPointType>(lines, info, segments);
//
//    // 将线段转换为MS算法可用的格式
//    std::vector<std::vector<double> > segments_in_ms;
//    for (auto s : segments)
//        segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//    MeanShift* msp = new MeanShift();
//    double kernel_bandwidth = 0.2;
//
//    std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//    printf("\n====================\n");
//    printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//    printf("简化之后 %lu 条线段\n", clusters.size());
//    printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    cv::Mat ori = return_Segments_on_map(segments_in_ms, map, info);
//    cv::Mat aft = return_Segments_on_map(segments_res, map, info);
//    cv::imwrite("../ori_line.png", ori);
//    cv::imwrite("../refine_line.png", aft);
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//    /*auto regualrSegments = regularSegementDirection(seg_last, 5);
//    for (int i = 0; i < regualrSegments.size(); i++) {
//        if (regualrSegments[i] != seg_last[i])
//            std::cout << "ori:" << seg_last[i] << " --> " << "after:" << regualrSegments[i] << std::endl;
//    }*/
//    saveSegmens("../unbalance_small_scene/simple_lines.txt", seg_last);
//}

// //区域5按平面提取的直线的简化
//void main() {
//    // 读取基本信息
//    pcInfo info;
//    readPCInfo("../area5/pcInfo.txt", info);
//
//    // 读取 底图
//    //cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);//绘制在有横梁的墙面图
//    cv::Mat map = cv::imread("../area5/wall_map.png", 0);// 绘制在线的点云上
//   // 读取 直线
//    std::string path = "../area5/plane_vertex_v2";
//    // 选择保存路径
//    //ofstream out("../Plane_line2_simply/simple_lines.txt");
//
//    // 读取的直线并延长与包围盒相交
//    std::vector<std::vector<double>> lines;
//    readLines(path, lines);
//    std::vector<Segment> segments;
//    genSegments(lines, info, segments);
//
//    // 将线段转换为MS算法可用的格式
//    std::vector<std::vector<double> > segments_in_ms;
//    for (auto s : segments)
//        segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//    MeanShift* msp = new MeanShift();
//    double kernel_bandwidth = 0.2;
//
//    std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//    printf("\n====================\n");
//    printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//    printf("简化之后 %lu 条线段\n", clusters.size());
//    printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    cv::Mat ori = return_Segments_on_map(segments_in_ms, map, info);
//    cv::Mat simply = return_Segments_on_map(segments_res, map, info);
//    cv::imwrite("../area5/ori_lines.png", ori);
//    cv::imwrite("../area5/simply_lines.png", simply);
//
//    for (int cluster = 0; cluster < clusters.size(); cluster++) {
//        printf("Cluster %i:\n", cluster);
//        for (int point = 0; point < clusters[cluster].original_points.size(); point++) {
//            for (int dim = 0; dim < clusters[cluster].original_points[point].size(); dim++) {
//                printf("%f ", clusters[cluster].original_points[point][dim]);
//            }
//            printf(" -> ");
//            for (int dim = 0; dim < clusters[cluster].shifted_points[point].size(); dim++) {
//                printf("%f ", clusters[cluster].shifted_points[point][dim]);
//            }
//            printf("\n");
//        }
//        printf("\n");
//        /*out << clusters[cluster].shifted_points[0][0] << " " << clusters[cluster].shifted_points[0][1] << " "
//            << clusters[cluster].shifted_points[0][2] << " " << clusters[cluster].shifted_points[0][3] << " " << std::endl;*/
//    }
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//
//    saveSegmens("../area5/simple_lines.txt", seg_last);
//   // out.close();
//}

 //区域5 按 切片直线简化

//void main() {
//    // 读取基本信息
//	pcInfo info;
//	readPCInfo("../area5_part2/info/pcInfo.txt", info);
//    // 读取 底图
//    //cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);
//    cv::Mat map = cv::imread("../area5/wall_map.png", 0);
//
//	// 读取 直线点云
//	Clouds_vector<MyPointType> lines;
//	read_Cloud_vector<MyPointType>("../area5/line_pc", lines);
//
//	// 将点云生成在包围盒内的线段
//	std::vector<Segment> segments;
//	genSegmens<MyPointType>(lines, info, segments);
//
//	// 将线段转换为MS算法可用的格式
//	std::vector<std::vector<double> > segments_in_ms;
//	for (auto s : segments)
//		segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//									CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//	MeanShift* msp = new MeanShift();
//	double kernel_bandwidth = 0.1;
//
//	std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//	printf("\n====================\n");
//	printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//	printf("简化之后 %lu 条线段\n", clusters.size());
//	printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    show_Segments_on_map(segments_in_ms, map, info);
//    show_Segments_on_map(segments_res, map, info);
//
// 
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//    /*auto regualrSegments = regularSegementDirection(seg_last, 5);
//    for (int i = 0; i < regualrSegments.size(); i++) {
//        if (regualrSegments[i] != seg_last[i])
//            std::cout << "ori:" << seg_last[i] << " --> " << "after:" << regualrSegments[i] << std::endl;
//    }*/
//    //saveSegmens("../res1/refineLine/simple_lines_3.txt", seg_last);
//}

//区域5part2 按 切片直线简化
//void main() {
//    // 读取基本信息
//    pcInfo info;
//    readPCInfo("../area5_part2/info/pcInfo.txt", info);
//    // 读取 底图
//    //cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);
//    cv::Mat map = cv::imread("../area5_part2/wall_map.png", 0);
//
//    // 读取 直线点云
//    Clouds_vector<MyPointType> lines;
//    read_Cloud_vector<MyPointType>("../area5_part2/line_pc2", lines);
//
//    // 将点云生成在包围盒内的线段
//    std::vector<Segment> segments;
//    genSegmens<MyPointType>(lines, info, segments);
//
//    // 将线段转换为MS算法可用的格式
//    std::vector<std::vector<double> > segments_in_ms;
//    for (auto s : segments)
//        segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//    MeanShift* msp = new MeanShift();
//    double kernel_bandwidth = 0.05;
//
//    std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//    printf("\n====================\n");
//    printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//    printf("简化之后 %lu 条线段\n", clusters.size());
//    printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    show_Segments_on_map(segments_in_ms, map, info);
//    show_Segments_on_map(segments_res, map, info);
//
//
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//    /*auto regualrSegments = regularSegementDirection(seg_last, 5);
//    for (int i = 0; i < regualrSegments.size(); i++) {
//        if (regualrSegments[i] != seg_last[i])
//            std::cout << "ori:" << seg_last[i] << " --> " << "after:" << regualrSegments[i] << std::endl;
//    }*/
//    //saveSegmens("../res1/refineLine/simple_lines_3.txt", seg_last);
//}


////区域5全景
//void main() {
//    // 读取基本信息
//    pcInfo info;
//    readPCInfo("../area5_full/info/pcInfo.txt", info);
//    // 读取 底图
//    //cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);
//    cv::Mat map = cv::imread("../area5_full/wall_map.png", 0);
//
//    // 读取 直线点云
//    Clouds_vector<MyPointType> lines;
//    read_Cloud_vector<MyPointType>("../area5_full/line_pc2", lines);
//
//    // 将点云生成在包围盒内的线段
//    std::vector<Segment> segments;
//    genSegmens<MyPointType>(lines, info, segments);
//
//    // 将线段转换为MS算法可用的格式
//    std::vector<std::vector<double> > segments_in_ms;
//    for (auto s : segments)
//        segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//    MeanShift* msp = new MeanShift();
//    double kernel_bandwidth = 0.05;
//
//    std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//    printf("\n====================\n");
//    printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//    printf("简化之后 %lu 条线段\n", clusters.size());
//    printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    //show_Segments_on_map(segments_in_ms, map, info);
//    //show_Segments_on_map(segments_res, map, info);
//
//
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//
//    saveSegmens("../area5_full/refine_lines.txt", seg_last);
//}

// //区域5全景 按平面提取的直线的简化
//void main() {
//    // 读取基本信息
//    pcInfo info;
//    readPCInfo("../area5_full/info/pcInfo.txt", info);
//
//    // 读取 底图
//    //cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);//绘制在有横梁的墙面图
//    cv::Mat map = cv::imread("../area5_full/wall_map.png", 0);// 绘制在线的点云上
//   // 读取 直线
//    std::string path = "../area5_full/plane_line";
//    // 选择保存路径
//    //ofstream out("../Plane_line2_simply/simple_lines.txt");
//
//    // 读取的直线并延长与包围盒相交
//    std::vector<std::vector<double>> lines;
//    readLines(path, lines);
//    std::vector<Segment> segments;
//    genSegments(lines, info, segments);
//
//    // 将线段转换为MS算法可用的格式
//    std::vector<std::vector<double> > segments_in_ms;
//    for (auto s : segments)
//        segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//    MeanShift* msp = new MeanShift();
//    double kernel_bandwidth = 0.1;
//
//    std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//    printf("\n====================\n");
//    printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//    printf("简化之后 %lu 条线段\n", clusters.size());
//    printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    auto regular_lines = readSegmens("../area5_full/regular_lines.txt");
//    std::vector<std::vector<double> > regular_segments;
//    for (auto s : regular_lines)
//        regular_segments.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//
//    cv::Mat ori = return_Segments_on_map(segments_in_ms, map, info);
//    cv::Mat simply = return_Segments_on_map(regular_segments, map, info);
//    
//    cv::imwrite("../area5_full/ori_lines.png", ori);
//    cv::imwrite("../area5_full/simply_lines.png", simply);
//
//    //for (int cluster = 0; cluster < clusters.size(); cluster++) {
//    //    printf("Cluster %i:\n", cluster);
//    //    for (int point = 0; point < clusters[cluster].original_points.size(); point++) {
//    //        for (int dim = 0; dim < clusters[cluster].original_points[point].size(); dim++) {
//    //            printf("%f ", clusters[cluster].original_points[point][dim]);
//    //        }
//    //        printf(" -> ");
//    //        for (int dim = 0; dim < clusters[cluster].shifted_points[point].size(); dim++) {
//    //            printf("%f ", clusters[cluster].shifted_points[point][dim]);
//    //        }
//    //        printf("\n");
//    //    }
//    //    printf("\n");
//    //    /*out << clusters[cluster].shifted_points[0][0] << " " << clusters[cluster].shifted_points[0][1] << " "
//    //        << clusters[cluster].shifted_points[0][2] << " " << clusters[cluster].shifted_points[0][3] << " " << std::endl;*/
//    //}
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//
//    //saveSegmens("../area5_full/simple_lines.txt", seg_last);
//   // out.close();
//}


////区域4全景 按平面提取的直线的简化
//void main() {
//    // 读取基本信息
//    pcInfo info;
//    readPCInfo("../area4/info/pcInfo.txt", info);
//
//    // 读取 底图
//    //cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);//绘制在有横梁的墙面图
//    cv::Mat map = cv::imread("../area4/wall_map.png", 0);// 绘制在线的点云上
//   // 读取 直线
//    std::string path = "../area4/plane_line";
//    // 选择保存路径
//    //ofstream out("../Plane_line2_simply/simple_lines.txt");
//
//    // 读取的直线并延长与包围盒相交
//    std::vector<std::vector<double>> lines;
//    readLines(path, lines);
//    std::vector<Segment> segments;
//    genSegments(lines, info, segments);
//
//    // 将线段转换为MS算法可用的格式
//    std::vector<std::vector<double> > segments_in_ms;
//    for (auto s : segments)
//        segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//    MeanShift* msp = new MeanShift();
//    double kernel_bandwidth = 0.1;
//
//    std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);
//
//    printf("\n====================\n");
//    printf("简化之前 %lu 条线段\n", segments_in_ms.size());
//    printf("简化之后 %lu 条线段\n", clusters.size());
//    printf("====================\n\n");
//
//    std::vector<std::vector<double> > segments_res;
//    for (auto cluter : clusters)
//    {
//        segments_res.push_back(cluter.shifted_points[0]);
//    }
//    auto regular_lines = readSegmens("../area4/regular_lines.txt");
//    std::vector<std::vector<double> > regular_segments;
//    for (auto s : regular_lines)
//        regular_segments.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
//                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
//
//    cv::Mat ori = return_Segments_on_map(segments_in_ms, map, info);
//    cv::Mat simply = return_Segments_on_map(regular_segments, map, info);
//
//    cv::imwrite("../area4/ori_lines.png", ori);
//    cv::imwrite("../area4/simply_lines.png", simply);
//
//    //for (int cluster = 0; cluster < clusters.size(); cluster++) {
//    //    printf("Cluster %i:\n", cluster);
//    //    for (int point = 0; point < clusters[cluster].original_points.size(); point++) {
//    //        for (int dim = 0; dim < clusters[cluster].original_points[point].size(); dim++) {
//    //            printf("%f ", clusters[cluster].original_points[point][dim]);
//    //        }
//    //        printf(" -> ");
//    //        for (int dim = 0; dim < clusters[cluster].shifted_points[point].size(); dim++) {
//    //            printf("%f ", clusters[cluster].shifted_points[point][dim]);
//    //        }
//    //        printf("\n");
//    //    }
//    //    printf("\n");
//    //    /*out << clusters[cluster].shifted_points[0][0] << " " << clusters[cluster].shifted_points[0][1] << " "
//    //        << clusters[cluster].shifted_points[0][2] << " " << clusters[cluster].shifted_points[0][3] << " " << std::endl;*/
//    //}
//
//    // 简化后的线段可能变短，需要再次与包围盒相交
//    std::vector<Segment> seg_refine;
//    std::vector<Segment> seg_last;
//    for (auto s : segments_res)
//        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
//    genSegmens(seg_refine, info, seg_last);
//
//    //saveSegmens("../area5_full/simple_lines.txt", seg_last);
//   // out.close();
//}


//区域3全景 按平面提取的直线的简化
void main() {
    // 读取基本信息
    pcInfo info;
    readPCInfo("../area3/info/pcInfo.txt", info);

    // 读取 底图
    //cv::Mat map = cv::imread("../data/wall_map_with_3.png", 0);//绘制在有横梁的墙面图
    cv::Mat map = cv::imread("../area3/wall_map.png", 0);// 绘制在线的点云上
   // 读取 直线
    std::string path = "../area3/plane_line";
    // 选择保存路径
    //ofstream out("../Plane_line2_simply/simple_lines.txt");

    // 读取的直线并延长与包围盒相交
    std::vector<std::vector<double>> lines;
    readLines(path, lines);
    std::vector<Segment> segments;
    genSegments(lines, info, segments);

    // 将线段转换为MS算法可用的格式
    std::vector<std::vector<double> > segments_in_ms;
    for (auto s : segments)
        segments_in_ms.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });
    MeanShift* msp = new MeanShift();
    double kernel_bandwidth = 0.1;

    std::vector<Cluster> clusters = msp->cluster(segments_in_ms, kernel_bandwidth);

    printf("\n====================\n");
    printf("简化之前 %lu 条线段\n", segments_in_ms.size());
    printf("简化之后 %lu 条线段\n", clusters.size());
    printf("====================\n\n");

    std::vector<std::vector<double> > segments_res;
    for (auto cluter : clusters)
    {
        segments_res.push_back(cluter.shifted_points[0]);
    }
    auto regular_lines = readSegmens("../area3/regular_lines1.txt");
    std::vector<std::vector<double> > regular_segments;
    for (auto s : regular_lines)
        regular_segments.push_back({ CGAL::to_double(s.source().x()), CGAL::to_double(s.source().y()),
                                    CGAL::to_double(s.target().x()), CGAL::to_double(s.target().y()) });

    cv::Mat ori = return_Segments_on_map(segments_in_ms, map, info);
    cv::Mat simply = return_Segments_on_map(regular_segments, map, info);

    cv::imwrite("../area3/ori_lines2.png", ori);
    cv::imwrite("../area3/simply_lines2.png", simply);


    // 简化后的线段可能变短，需要再次与包围盒相交
    std::vector<Segment> seg_refine;
    std::vector<Segment> seg_last;
    for (auto s : segments_res)
        seg_refine.push_back(Segment(Point(s[0], s[1]), Point(s[2], s[3])));
    genSegmens(seg_refine, info, seg_last);

    //saveSegmens("../area5_full/simple_lines.txt", seg_last);
   // out.close();
}