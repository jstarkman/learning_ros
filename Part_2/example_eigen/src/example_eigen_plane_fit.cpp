#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

double g_noise_gain = 0.1;

#define ROS_INFO printf
#define ROS_INFO_STREAM(...) std::cout << __VA_ARGS__ << std::endl

int main()
{
  Eigen::Vector3d normal_vec(1, 2, 3);
  ROS_INFO("creating example noisy, planar data...");
  std::cout << "normal: " << normal_vec.transpose() << std::endl;
  normal_vec /= normal_vec.norm();
  std::cout << "unit length normal: " << normal_vec.transpose() << std::endl;
  double dist = 1.23;

  std::cout << "plane distance from origin: " << dist << std::endl;

  Eigen::Vector3d v1, v2;

  Eigen::Matrix3d Rot_z;
  Rot_z.row(0) << 0, -1, 0;
  Rot_z.row(1) << 1, 0, 0;
  Rot_z.row(2) << 0, 0, 1;
  std::cout << "Rot_z: " << std::endl;

  std::cout << Rot_z << std::endl;

  v1 = Rot_z * normal_vec;

  ROS_INFO_STREAM("v1: " << v1.transpose() << std::endl);

  double dotprod = v1.dot(normal_vec);
  double dotprod2 = v1.transpose() * normal_vec;

  std::cout << "v1 dot normal: " << dotprod << "; v1.transpose()*normal_vec: " << dotprod2 << std::endl;
  std::cout << "(should be identical)" << std::endl;

  v2 = v1.cross(normal_vec);
  v2 /= v2.norm();
  std::cout << "v2: " << v2.transpose() << std::endl;

  dotprod = v2.dot(normal_vec);
  std::cout << "v2 dot normal_vec = " << dotprod << "  (should be zero)" << std::endl;

  v1 = v2.cross(normal_vec);

  std::cout << "v1= " << v1.transpose() << std::endl;
  std::cout << " v1 dot v2 = " << v1.dot(v2) << "; v1 dot normal_vec = " << v1.dot(normal_vec) << std::endl;
  std::cout << "(these should also be zero)" << std::endl;

  int npts = 10;
  Eigen::MatrixXd points_mat(3, npts);

  Eigen::Vector3d point;
  Eigen::Vector2d rand_vec;

  for (int ipt = 0; ipt < npts; ipt++)
  {
    rand_vec.setRandom(2, 1);
    point = rand_vec(0) * v1 + rand_vec(1) * v2 + dist * normal_vec;
    points_mat.col(ipt) = point;
  }

  std::cout << "random points on plane (in columns): " << std::endl;
  std::cout << points_mat << std::endl;

  Eigen::MatrixXd Noise = Eigen::MatrixXd::Random(3, npts);

  std::cout << "noise_gain = " << g_noise_gain << "; edit this as desired" << std::endl;

  points_mat = points_mat + Noise * g_noise_gain;
  std::cout << "random points on plane (in columns) w/ noise: " << std::endl;
  std::cout << points_mat << std::endl;

  std::cout << std::endl << std::endl;
  ROS_INFO("starting identification of plane from data: ");

  Eigen::Vector3d centroid = Eigen::MatrixXd::Zero(3, 1);

  npts = points_mat.cols();
  std::cout << "matrix has ncols = " << npts << std::endl;
  for (int ipt = 0; ipt < npts; ipt++)
  {
    centroid += points_mat.col(ipt);
  }
  centroid /= npts;
  std::cout << "centroid: " << centroid.transpose() << std::endl;

  Eigen::MatrixXd points_offset_mat = points_mat;
  for (int ipt = 0; ipt < npts; ipt++)
  {
    points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid;
  }

  Eigen::Matrix3d CoVar;
  CoVar = points_offset_mat * (points_offset_mat.transpose());
  std::cout << "covariance: " << std::endl;
  std::cout << CoVar << std::endl;

  Eigen::EigenSolver<Eigen::Matrix3d> es3d(CoVar);

  Eigen::VectorXd evals;

  std::cout << "The eigenvalues of CoVar are:" << std::endl << es3d.eigenvalues().transpose() << std::endl;
  std::cout << "(these should be real numbers, and one of them should be zero)" << std::endl;
  std::cout << "The matrix of eigenvectors, V, is:" << std::endl;
  std::cout << es3d.eigenvectors() << std::endl << std::endl;
  std::cout << "(these should be real-valued vectors)" << std::endl;

  evals = es3d.eigenvalues().real();
  std::cout << "real parts of evals: " << evals.transpose() << std::endl;

  double min_lambda = evals[0];
  Eigen::Vector3cd complex_vec;
  Eigen::Vector3d est_plane_normal;
  complex_vec = es3d.eigenvectors().col(0);

  est_plane_normal = complex_vec.real();

  double lambda_test;
  int i_normal = 0;

  for (int ivec = 1; ivec < 3; ivec++)
  {
    lambda_test = evals[ivec];
    if (lambda_test < min_lambda)
    {
      min_lambda = lambda_test;
      i_normal = ivec;
      est_plane_normal = es3d.eigenvectors().col(ivec).real();
    }
  }

  std::cout << "min eval is " << min_lambda << ", corresponding to component " << i_normal << std::endl;
  std::cout << "corresponding evec (est plane normal): " << est_plane_normal.transpose() << std::endl;
  std::cout << "correct answer is: " << normal_vec.transpose() << std::endl;
  double est_dist = est_plane_normal.dot(centroid);
  std::cout << "est plane distance from origin = " << est_dist << std::endl;
  std::cout << "correct answer is: " << dist << std::endl;
  std::cout << std::endl << std::endl;

  ROS_INFO("2ND APPROACH b = A*x SOLN");
  Eigen::VectorXd ones_vec = Eigen::MatrixXd::Ones(npts, 1);
  Eigen::MatrixXd A = points_mat.transpose();

  Eigen::Vector3d x_soln = A.fullPivLu().solve(ones_vec);

  double dist_est2 = 1.0 / x_soln.norm();
  x_soln *= dist_est2;
  std::cout << "normal vec, 2nd approach: " << x_soln.transpose() << std::endl;
  std::cout << "plane distance = " << dist_est2 << std::endl;

  return 0;
}
