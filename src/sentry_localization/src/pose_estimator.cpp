#include <pose_system.hpp>
#include <odom_system.hpp>
#include <pose_estimator.hpp>
#include <unscented_kalman_filter.hpp>

//pcl
#include <pcl/voxel_grid.h>

namespace sentry_localization{

PoseEstimator::PoseEstimator(pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr& registration, const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat, double time_duration);
  : registration(registration), time_duration(time_duration) 
{
    last_observation = Eigen::MatrixXf::Identity(16, 16);
    last_observation.block<3, 3>(0, 0) = quat.toRotationMatrix();
    last_observation.block<3, 1>(0, 3) = pos;

    process_noise = Eigen::Matrix4f::Identity(16, 16);
    process_noise.middleRows(0, 3) *= 1.0;
    process_noise.middleRows(3, 3) *= 1.0;
    process_noise.middleRows(6, 4) *= 0.5;
    process_noise.middleRows(10, 3) *= 1e-6;
    process_noise.middleRows(13, 3) *= 1e-6;

    Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(7, 7);
    measurement_noise.middleRows(0, 3) *= 0.01;
    measurement_noise.middleRows(3, 4) *= 0.001;

    Eigen::VectorXf mean(16);
    mean.middleRows(0, 3) = pos;
    mean.middleRows(3, 3).setZero();
    mean.middleRows(6, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z()).normalized();
    mean.middleRows(10, 3).setZero();
    mean.middleRows(13, 3).setZero();

    Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(16, 16) * 0.01;

    PoseSystem system;
    ukf.reset(new sentry_localization::UnsentedKalmanFilterX<float, PoseSystem>(system, 16, 6, 7, process_noise, measurement_noise, mean, cov));
}

PoseEstimator::~PoseEstimator();

void PoseEstimator::predict(const ros::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro)
{
  if(init_stamp.is_zero()){
    init_stamp = stamp;
  }

  if((stamp - init_stamp).toSec() < time_duration || prev_stamp.is_zero() || prev_stamp == stamp){
    prev_stamp = stamp;
    return;
  }

  double dt = (stamp - prev_stamp).toSec();
  prev_stamp = stamp;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  Eigen::VectorXf control(6);
  control.head<3>() = acc;
  control.tail<3>() = gyro;
  ukf->predict(control);
}

void PoseEstimator::predict_odom(const Eigen::Matrix4f& odom_delta)
{
  if(!odom_ukf)
  {
    Eigen::MatriXf  odom_process_noise = Eigen::MatrixXf::Identity(7, 7);
    Eigen::MatrixXf odom_measurement = Eigen::MatrixXf::Identity(7, 7) * 1e-3;

    Eigen::VectorXf odom_mean(7);
    odom_mean.block<3, 1>(0, 0) = Eigen::Vector3f(ukf->mean[0], ukf->mean[1], ukf->mean[2]);
    odom_mean.block<4, 1>(3, 0) = Eigen::Vector4f(ukf->mean[6], ukf->mean[7], ukf->mean[8], ukf->mean[9]);
    Eigen::MatrixXf odom_cov = Eigen::MatrixXf::Identity(7, 7) * 1e-2;

    OdomSystem odom_system;
    odom_ukf.reset(new sentry_localization::UnsentedKalmanFilterX<float, OdomSystem>(odom_system, 7, 7, 7, odom_process_noise, odom_measurement_noise, odom_mean, odom_cov));
  }

  Eigen::Quaternionf quat(odom_delta.block<3, 3>(0, 0));
  if(odom_delta().coeffs().dot(quat.coeffs()) < 0.0){
    quat.coeffs() *= -1.0f;
  }

  Eigen::VectorXf control(7);
  control.middleRows(0, 3) = odom_delta.block<3, 3>(0, 0);
  control.middleRows(3, 4) = Eigen::Vector4f(quat.w(), quat.x(), quat.y(), quat.z());

  Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(7, 7);
  process_noise.topLeftCorner(3, 3) = Eigen::matrix3f::Identity() * odom_delta.block<3, 1>(0, 3).norm() + Eigen::Matrix3f::Identity() * 1e-3;
  process_noise.bottomRightCorner(4, 4) = Eigen::Matrix4f::Identity() * (1- std::abs(quat.w()) + Eigen::Matrix4f::Identity() * 1e-3);

  odom_ukf->setProcessNoiseCov(process_noise);
  odom_ukf->predict(control); 
}




}