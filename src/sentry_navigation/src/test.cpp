#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdlib>  // 对于 rand() 和 RAND_MAX
#include <cmath>    // 对于除法操作

int main(int argc, char *argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_in->width = 100;
    cloud_in->height = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for(int i = 0; i < cloud_in->points.size(); ++i)
    {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0);
        // cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0);
    }

    float d_yaw = 30;
    float d_x = 1;
    float d_y = 2;
    float s = sin(d_yaw * M_PI / 180);
    float c = cos(d_yaw * M_PI / 180);
    Eigen::Matrix3f T;
    T << c, -s, d_x,
         s, c, d_y,
         0, 0, 1;
    std::cout << T << std::endl;

    *cloud_out  = *cloud_in;
    for(int i = 0; i < cloud_out->points.size(); ++i)
    {
        Eigen::Vector3f point_in, point_out;
        point_in << cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z;
        point_out = T * point_in;
        cloud_out->points[i].x = point_out[0];
        cloud_out->points[i].y = point_out[1];
        cloud_out->points[i].z = point_out[2];
    }
    
    //求两个点云的几何中心
    int num_points = cloud_in->points.size();
    Eigen::Vector3f sum_point_in = Eigen::Vector3f::Zero();
    Eigen::Vector3f sum_point_out = Eigen::Vector3f::Zero();

    for(int i = 0; i < num_points; ++i)
    {
        sum_point_in[0] += cloud_in->points[i].x;
        sum_point_in[1] += cloud_in->points[i].y;
        sum_point_in[2] += cloud_in->points[i].z;

        sum_point_out[0] += cloud_out->points[i].x;
        sum_point_out[1] += cloud_out->points[i].y;
        sum_point_out[2] += cloud_out->points[i].z;
    }

    Eigen::Vector3f u_point_in, u_point_out;
    //源点云几何中心
    u_point_in[0] = sum_point_in[0] / num_points;
    u_point_in[1] = sum_point_in[1] / num_points;
    u_point_in[2] = sum_point_in[2] / num_points;
    //目标点云几何中心
    u_point_out[0] = sum_point_out[0] / num_points;
    u_point_out[1] = sum_point_out[1] / num_points;
    u_point_out[2] = sum_point_out[2] / num_points;

    //点云去中心化
    for(int i = 0; i < num_points; ++i)
    {
        cloud_in->points[i].x -= u_point_in[0];
        cloud_in->points[i].y -= u_point_in[1];
        cloud_in->points[i].z -= u_point_in[2];

        cloud_out->points[i].x -= u_point_out[0];
        cloud_out->points[i].y -= u_point_out[1];
        cloud_out->points[i].z -= u_point_out[2];
    }

    //求W矩阵
    Eigen::Matrix3f W = Eigen::Matrix3f::Zero();
    for(int i = 0; i < num_points; ++i)
    {
        Eigen::Vector3f point_in, point_out;
        point_in << cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z;
        point_out << cloud_out->points[i].x, cloud_out->points[i].y, cloud_out->points[i].z;
        W += point_out * point_in.transpose();
    }
    // SVD分解W，得到U和V
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    Eigen::Matrix3f R = U * V.transpose();
    std::cout << u_point_out << std::endl << R * u_point_in << std::endl;
    Eigen::Vector3f t = u_point_out - R * u_point_in;
    std::cout << "R" << std::endl << R << std::endl;
    std::cout << "t" << std::endl << t << std::endl;

    return 0;
}
