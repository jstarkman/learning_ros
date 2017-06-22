

#include <pcl-1.7/pcl/PCLHeader.h>
#include <pcl_utils/pcl_utils.h>

PclUtils::PclUtils(ros::NodeHandle *nodehandle)
  : nh_(*nodehandle)
  , pclKinect_ptr_(new PointCloud<pcl::PointXYZ>)
  , pclKinect_clr_ptr_(new PointCloud<pcl::PointXYZRGB>)
  , pclTransformed_ptr_(new PointCloud<pcl::PointXYZ>)
  , pclSelectedPoints_ptr_(new PointCloud<pcl::PointXYZ>)
  , pclSelectedPtsClr_ptr_(new PointCloud<pcl::PointXYZRGB>)
  , pclTransformedSelectedPoints_ptr_(new PointCloud<pcl::PointXYZ>)
  , pclGenPurposeCloud_ptr_(new PointCloud<pcl::PointXYZ>)
{
  initializeSubscribers();
  initializePublishers();
  got_kinect_cloud_ = true;
  got_selected_points_ = false;
  take_snapshot_ = false;
}

int PclUtils::read_clr_pcd_file(string fname)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fname, *pclKinect_clr_ptr_) == -1)
  {
    ROS_ERROR("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded " << pclKinect_clr_ptr_->width * pclKinect_clr_ptr_->height << " data points from file " << fname
            << std::endl;
  return (0);
}

int PclUtils::read_pcd_file(string fname)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(fname, *pclKinect_ptr_) == -1)
  {
    ROS_ERROR("Couldn't read file \n");
    return (-1);
  }
  std::cout << "Loaded " << pclKinect_ptr_->width * pclKinect_ptr_->height << " data points from file " << fname
            << std::endl;
  return (0);
}

Eigen::Affine3f PclUtils::make_affine_from_plane_params(Eigen::Vector3f plane_normal, double plane_dist)
{
  Eigen::Vector3f xvec, yvec, zvec;
  Eigen::Matrix3f R_transform;
  Eigen::Affine3f A_transform;
  Eigen::Vector3f plane_origin;

  for (int i = 0; i < 3; i++)
    zvec(i) = plane_normal(i);
  if (zvec(2) > 0)
    zvec *= -1.0;

  xvec << 1, 0, 0;
  xvec = xvec - zvec * (zvec.dot(xvec));
  xvec /= xvec.norm();
  yvec = zvec.cross(xvec);
  R_transform.col(0) = xvec;
  R_transform.col(1) = yvec;
  R_transform.col(2) = zvec;

  if (plane_dist > 0)
    plane_dist *= -1.0;
  A_transform.linear() = R_transform;
  plane_origin = zvec * plane_dist;
  A_transform.translation() = plane_origin;
  return A_transform;
}

Eigen::Affine3f PclUtils::make_affine_from_plane_params(Eigen::Vector4f plane_parameters)
{
  Eigen::Vector3f plane_normal;
  double plane_dist;
  plane_normal(0) = plane_parameters(0);
  plane_normal(1) = plane_parameters(1);
  plane_normal(2) = plane_parameters(2);
  plane_dist = plane_parameters(3);
  return (make_affine_from_plane_params(plane_normal, plane_dist));
}

Eigen::Affine3f PclUtils::make_affine_from_plane_params(Eigen::Vector4f plane_parameters, Eigen::Vector3f centroid)
{
  Eigen::Vector3f plane_normal;
  Eigen::Affine3f A_transform;
  double plane_dist;
  plane_normal(0) = plane_parameters(0);
  plane_normal(1) = plane_parameters(1);
  plane_normal(2) = plane_parameters(2);
  plane_dist = plane_parameters(3);
  A_transform = make_affine_from_plane_params(plane_normal, plane_dist);
  A_transform.translation() = centroid;
  return A_transform;
}

void PclUtils::fit_points_to_plane(Eigen::MatrixXf points_mat, Eigen::Vector3f &plane_normal, double &plane_dist)
{
  int npts = points_mat.cols();

  centroid_ = Eigen::MatrixXf::Zero(3, 1);

  for (int ipt = 0; ipt < npts; ipt++)
  {
    centroid_ += points_mat.col(ipt);
  }
  centroid_ /= npts;
  cout << "centroid: " << centroid_.transpose() << endl;

  Eigen::MatrixXf points_offset_mat = points_mat;
  for (int ipt = 0; ipt < npts; ipt++)
  {
    points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid_;
  }

  Eigen::Matrix3f CoVar;
  CoVar = points_offset_mat * (points_offset_mat.transpose());

  Eigen::EigenSolver<Eigen::Matrix3f> es3f(CoVar);

  Eigen::VectorXf evals;

  evals = es3f.eigenvalues().real();

  double min_lambda = evals[0];
  double max_lambda = evals[0];

  plane_normal = es3f.eigenvectors().col(0).real();
  major_axis_ = es3f.eigenvectors().col(0).real();

  double lambda_test;
  int i_normal = 0;
  int i_major_axis = 0;

  for (int ivec = 1; ivec < 3; ivec++)
  {
    lambda_test = evals[ivec];
    if (lambda_test < min_lambda)
    {
      min_lambda = lambda_test;
      i_normal = ivec;
      plane_normal = es3f.eigenvectors().col(i_normal).real();
    }
    if (lambda_test > max_lambda)
    {
      max_lambda = lambda_test;
      i_major_axis = ivec;
      major_axis_ = es3f.eigenvectors().col(i_major_axis).real();
    }
  }

  if (plane_normal(2) > 0)
    plane_normal = -plane_normal;

  plane_dist = plane_normal.dot(centroid_);

  ROS_INFO("major_axis: %f, %f, %f", major_axis_(0), major_axis_(1), major_axis_(2));
  ROS_INFO("plane normal: %f, %f, %f", plane_normal(0), plane_normal(1), plane_normal(2));
}

void PclUtils::fit_points_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, Eigen::Vector3f &plane_normal,
                                   double &plane_dist)
{
  Eigen::MatrixXf points_mat;
  Eigen::Vector3f cloud_pt;

  int npts = input_cloud_ptr->points.size();
  points_mat.resize(3, npts);

  for (int i = 0; i < npts; ++i)
  {
    cloud_pt = input_cloud_ptr->points[i].getVector3fMap();
    points_mat.col(i) = cloud_pt;
  }
  fit_points_to_plane(points_mat, plane_normal, plane_dist);
}

Eigen::Vector3f PclUtils::compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr)
{
  Eigen::Vector3f centroid;
  Eigen::Vector3f cloud_pt;
  int npts = input_cloud_ptr->points.size();
  centroid << 0, 0, 0;

  for (int ipt = 0; ipt < npts; ipt++)
  {
    cloud_pt = input_cloud_ptr->points[ipt].getVector3fMap();
    centroid += cloud_pt;
  }
  centroid /= npts;
  return centroid;
}

Eigen::Vector3f PclUtils::compute_centroid(pcl::PointCloud<pcl::PointXYZ> &input_cloud)
{
  Eigen::Vector3f centroid;
  Eigen::Vector3f cloud_pt;
  int npts = input_cloud.points.size();
  centroid << 0, 0, 0;

  for (int ipt = 0; ipt < npts; ipt++)
  {
    cloud_pt = input_cloud.points[ipt].getVector3fMap();
    centroid += cloud_pt;
  }
  centroid /= npts;
  return centroid;
}

/*
void PclUtils::fit_xformed_selected_pts_to_plane(Eigen::Vector3f &plane_normal, double &plane_dist) {
    fit_points_to_plane(pclTransformedSelectedPoints_ptr_, plane_normal, plane_dist);






    patch_params_msg.offset = plane_dist;
    patch_params_msg.centroid.resize(3);
    patch_params_msg.normal_vec.resize(3);
    for (int i=0;i<3;i++) {
        patch_params_msg.normal_vec[i]=plane_normal[i];
        patch_params_msg.centroid[i]= centroid_[i];
    }
    patch_params_msg.frame_id = "torso";

}
*/

Eigen::Affine3f PclUtils::transformTFToEigen(const tf::Transform &t)
{
  Eigen::Affine3f e;

  for (int i = 0; i < 3; i++)
  {
    e.matrix()(i, 3) = t.getOrigin()[i];
    for (int j = 0; j < 3; j++)
    {
      e.matrix()(i, j) = t.getBasis()[i][j];
    }
  }

  for (int col = 0; col < 3; col++)
    e.matrix()(3, col) = 0;
  e.matrix()(3, 3) = 1;
  return e;
}

/**here is a function that transforms a cloud of points into an alternative frame;
 * it assumes use of pclKinect_ptr_ from kinect sensor as input, to pclTransformed_ptr_ , the cloud in output frame
 *
 * @param A [in] supply an Eigen::Affine3f, such that output_points = A*input_points
 */
void PclUtils::transform_kinect_cloud(Eigen::Affine3f A)
{
  transform_cloud(A, pclKinect_ptr_, pclTransformed_ptr_);
  /*
  pclTransformed_ptr_->header = pclKinect_ptr_->header;
  pclTransformed_ptr_->is_dense = pclKinect_ptr_->is_dense;
  pclTransformed_ptr_->width = pclKinect_ptr_->width;
  pclTransformed_ptr_->height = pclKinect_ptr_->height;
  int npts = pclKinect_ptr_->points.size();
  cout << "transforming npts = " << npts << endl;
  pclTransformed_ptr_->points.resize(npts);


  for (int i = 0; i < npts; ++i) {
      pclTransformed_ptr_->points[i].getVector3fMap() = A * pclKinect_ptr_->points[i].getVector3fMap();
  }
   * */
}

void PclUtils::transform_selected_points_cloud(Eigen::Affine3f A)
{
  transform_cloud(A, pclSelectedPoints_ptr_, pclTransformedSelectedPoints_ptr_);
}

void PclUtils::get_transformed_selected_points(pcl::PointCloud<pcl::PointXYZ> &outputCloud)
{
  int npts = pclTransformedSelectedPoints_ptr_->points.size();
  outputCloud.header = pclTransformedSelectedPoints_ptr_->header;
  outputCloud.is_dense = pclTransformedSelectedPoints_ptr_->is_dense;
  outputCloud.width = npts;
  outputCloud.height = 1;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloud.points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloud.points[i].getVector3fMap() = pclTransformedSelectedPoints_ptr_->points[i].getVector3fMap();
  }
}
void PclUtils::get_copy_selected_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud)
{
  int npts = pclSelectedPoints_ptr_->points.size();
  outputCloud->header = pclSelectedPoints_ptr_->header;
  outputCloud->header.frame_id = "camera_depth_optical_frame";
  outputCloud->is_dense = pclSelectedPoints_ptr_->is_dense;
  outputCloud->width = npts;
  outputCloud->height = 1;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloud->points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloud->points[i].getVector3fMap() = pclSelectedPoints_ptr_->points[i].getVector3fMap();
  }
}

void PclUtils::get_kinect_points(pcl::PointCloud<pcl::PointXYZ> &outputCloud)
{
  int npts = pclKinect_ptr_->points.size();
  outputCloud.header = pclKinect_ptr_->header;
  outputCloud.is_dense = pclKinect_ptr_->is_dense;
  outputCloud.width = npts;
  outputCloud.height = 1;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloud.points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloud.points[i].getVector3fMap() = pclKinect_ptr_->points[i].getVector3fMap();
  }
}

void PclUtils::get_kinect_points(pcl::PointCloud<pcl::PointXYZRGB> &outputCloud)
{
  int npts = pclKinect_clr_ptr_->points.size();
  outputCloud.header = pclKinect_clr_ptr_->header;
  outputCloud.is_dense = pclKinect_clr_ptr_->is_dense;
  outputCloud.width = npts;
  outputCloud.height = 1;

  cout << "get_kinect_points xyzrgb, copying cloud w/ npts =" << npts << endl;
  outputCloud.points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloud.points[i] = pclKinect_clr_ptr_->points[i];
  }
}

void PclUtils::get_kinect_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloudPtr)
{
  int npts = pclKinect_clr_ptr_->points.size();

  outputCloudPtr->header = pclKinect_clr_ptr_->header;
  outputCloudPtr->is_dense = pclKinect_clr_ptr_->is_dense;
  cout << "setting width: " << endl;
  outputCloudPtr->width = npts;
  cout << "setting height" << endl;
  outputCloudPtr->height = 1;

  outputCloudPtr->points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloudPtr->points[i].getVector3fMap() = pclKinect_clr_ptr_->points[i].getVector3fMap();

    outputCloudPtr->points[i].r = pclKinect_clr_ptr_->points[i].r;
    outputCloudPtr->points[i].g = pclKinect_clr_ptr_->points[i].g;
    outputCloudPtr->points[i].b = pclKinect_clr_ptr_->points[i].b;
  }
}

void PclUtils::get_kinect_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloudPtr)
{
  int npts = pclKinect_ptr_->points.size();
  outputCloudPtr->header = pclKinect_ptr_->header;
  outputCloudPtr->is_dense = pclKinect_ptr_->is_dense;
  outputCloudPtr->width = npts;
  outputCloudPtr->height = 1;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloudPtr->points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloudPtr->points[i].getVector3fMap() = pclKinect_ptr_->points[i].getVector3fMap();
  }
}

void PclUtils::get_gen_purpose_cloud(pcl::PointCloud<pcl::PointXYZ> &outputCloud)
{
  int npts = pclGenPurposeCloud_ptr_->points.size();
  outputCloud.header = pclGenPurposeCloud_ptr_->header;
  outputCloud.is_dense = pclGenPurposeCloud_ptr_->is_dense;
  outputCloud.width = npts;
  outputCloud.height = 1;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloud.points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloud.points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap();
  }
}

void PclUtils::get_selected_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloudPtr)
{
  int npts = pclSelectedPoints_ptr_->points.size();
  outputCloudPtr->header = pclSelectedPoints_ptr_->header;
  outputCloudPtr->is_dense = pclSelectedPoints_ptr_->is_dense;
  outputCloudPtr->width = npts;
  outputCloudPtr->height = 1;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloudPtr->points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloudPtr->points[i].getVector3fMap() = pclSelectedPoints_ptr_->points[i].getVector3fMap();
  }
}

void PclUtils::get_selected_points(pcl::PointCloud<pcl::PointXYZ> &outputCloud)
{
  int npts = pclSelectedPoints_ptr_->points.size();
  outputCloud.header = pclSelectedPoints_ptr_->header;
  outputCloud.is_dense = pclSelectedPoints_ptr_->is_dense;
  outputCloud.width = npts;
  outputCloud.height = 1;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloud.points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloud.points[i].getVector3fMap() = pclSelectedPoints_ptr_->points[i].getVector3fMap();
  }
}

void PclUtils::example_pcl_operation()
{
  int npts = pclTransformedSelectedPoints_ptr_->points.size();
  copy_cloud(pclTransformedSelectedPoints_ptr_, pclGenPurposeCloud_ptr_);
  Eigen::Vector3f offset;
  offset << 0, 0, 0.05;
  for (int i = 0; i < npts; ++i)
  {
    pclGenPurposeCloud_ptr_->points[i].getVector3fMap() = pclGenPurposeCloud_ptr_->points[i].getVector3fMap() + offset;
  }
}

void PclUtils::copy_indexed_pts_to_output_cloud(vector<int> &indices, PointCloud<pcl::PointXYZRGB> &outputCloud)
{
  int npts = indices.size();
  outputCloud.header = pclKinect_clr_ptr_->header;
  outputCloud.is_dense = pclKinect_clr_ptr_->is_dense;
  outputCloud.width = npts;
  outputCloud.height = 1;
  int i_index;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloud.points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    i_index = indices[i];
    outputCloud.points[i].getVector3fMap() = pclKinect_clr_ptr_->points[i_index].getVector3fMap();
    outputCloud.points[i].r = pclKinect_clr_ptr_->points[i_index].r;
    outputCloud.points[i].g = pclKinect_clr_ptr_->points[i_index].g;
    outputCloud.points[i].b = pclKinect_clr_ptr_->points[i_index].b;
    /*
        std::cout <<i_index
          << "    " << (int) pclKinect_clr_ptr_->points[i_index].r
          << " "    << (int) pclKinect_clr_ptr_->points[i_index].g
          << " "    << (int) pclKinect_clr_ptr_->points[i_index].b << std::endl;
     */
  }
}

Eigen::Vector3d PclUtils::find_avg_color()
{
  Eigen::Vector3d avg_color;
  Eigen::Vector3d pt_color;
  Eigen::Vector3d ref_color;
  indices_.clear();
  ref_color << 147, 147, 147;
  int npts = pclKinect_clr_ptr_->points.size();
  int npts_colored = 0;
  for (int i = 0; i < npts; i++)
  {
    pt_color(0) = (double)pclKinect_clr_ptr_->points[i].r;
    pt_color(1) = (double)pclKinect_clr_ptr_->points[i].g;
    pt_color(2) = (double)pclKinect_clr_ptr_->points[i].b;

    if ((pt_color - ref_color).norm() > 1)
    {
      avg_color += pt_color;
      npts_colored++;
      indices_.push_back(i);
    }
  }
  ROS_INFO("found %d points with interesting color", npts_colored);
  avg_color /= npts_colored;
  ROS_INFO("avg interesting color = %f, %f, %f", avg_color(0), avg_color(1), avg_color(2));
  return avg_color;
}

Eigen::Vector3d PclUtils::find_avg_color_selected_pts(vector<int> &indices)
{
  Eigen::Vector3d avg_color;
  Eigen::Vector3d pt_color;

  int npts = indices.size();
  int index;

  for (int i = 0; i < npts; i++)
  {
    index = indices[i];
    pt_color(0) = (double)pclKinect_clr_ptr_->points[index].r;
    pt_color(1) = (double)pclKinect_clr_ptr_->points[index].g;
    pt_color(2) = (double)pclKinect_clr_ptr_->points[index].b;
    avg_color += pt_color;
  }
  avg_color /= npts;
  ROS_INFO("avg color = %f, %f, %f", avg_color(0), avg_color(1), avg_color(2));
  return avg_color;
}

void PclUtils::find_indices_color_match(vector<int> &input_indices, Eigen::Vector3d normalized_avg_color,
                                        double color_match_thresh, vector<int> &output_indices)
{
  Eigen::Vector3d pt_color;

  int npts = input_indices.size();
  output_indices.clear();
  int index;
  int npts_matching = 0;

  for (int i = 0; i < npts; i++)
  {
    index = input_indices[i];
    pt_color(0) = (double)pclKinect_clr_ptr_->points[index].r;
    pt_color(1) = (double)pclKinect_clr_ptr_->points[index].g;
    pt_color(2) = (double)pclKinect_clr_ptr_->points[index].b;
    pt_color = pt_color / pt_color.norm();
    if ((normalized_avg_color - pt_color).norm() < color_match_thresh)
    {
      output_indices.push_back(index);
      npts_matching++;
    }
  }
  ROS_INFO("found %d color-match points from indexed set", npts_matching);
}

void PclUtils::filter_cloud_z(double z_nom, double z_eps, double radius, Eigen::Vector3f centroid, vector<int> &indices)
{
  filter_cloud_z(pclTransformed_ptr_, z_nom, z_eps, radius, centroid, indices);
}

void PclUtils::find_coplanar_pts_z_height(double plane_height, double z_eps, vector<int> &indices)
{
  filter_cloud_z(pclTransformed_ptr_, plane_height, z_eps, indices);
}

int PclUtils::box_filter_z_transformed_cloud(double z_min, double z_max, vector<int> &indices)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pclTransformed_ptr_);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter(indices);
  int npts = indices.size();
  ROS_DEBUG("number of points passing the filter = %d", npts);
  return npts;
}

double PclUtils::find_table_height(double z_min, double z_max, double dz)
{
  vector<int> indices;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pclTransformed_ptr_);
  pass.setFilterFieldName("z");
  double z_table = 0.0;
  int npts_max = 0;
  int npts;
  for (double z = z_min; z < z_max; z += dz)
  {
    pass.setFilterLimits(z, z + dz);
    pass.filter(indices);
    npts = indices.size();
    ROS_INFO("z=%f; npts = %d", z, npts);
    if (npts > npts_max)
    {
      npts_max = npts;
      z_table = z + 0.5 * dz;
    }
  }
  return z_table;
}

double PclUtils::find_table_height(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max,
                                   double dz_tol)
{
  vector<int> indices;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pclTransformed_ptr_);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*cloud_filtered);
  int n_filtered = cloud_filtered->points.size();
  ROS_INFO("num x-filtered pts = %d", n_filtered);

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min, y_max);
  pass.filter(*cloud_filtered);
  n_filtered = cloud_filtered->points.size();
  ROS_INFO("num y-filtered pts = %d", n_filtered);

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter(*cloud_filtered);
  n_filtered = cloud_filtered->points.size();
  ROS_INFO("num z-filtered pts = %d", n_filtered);

  pass.setInputCloud(cloud_filtered);
  double z_table = 0.0;
  int npts_max = 0;
  int npts;
  for (double z = z_min; z < z_max; z += dz_tol)
  {
    pass.setFilterLimits(z, z + dz_tol);
    pass.filter(indices);
    npts = indices.size();
    ROS_INFO("z=%f; npts = %d", z, npts);
    if (npts > npts_max)
    {
      npts_max = npts;
      z_table = z + 0.5 * dz_tol;
    }
    ROS_DEBUG("number of points passing the filter = %d", npts);
  }
  return z_table;
}

const int min_n_filtered = 100;
bool PclUtils::find_plane_fit(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max,
                              double dz_tol, Eigen::Vector3f &plane_normal, double &plane_dist,
                              Eigen::Vector3f &major_axis, Eigen::Vector3f &centroid)
{
  vector<int> indices;
  bool ans_valid = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pclTransformed_ptr_);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*cloud_filtered);
  int n_filtered = cloud_filtered->points.size();
  ROS_INFO("num x-filtered pts = %d", n_filtered);

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min, y_max);
  pass.filter(*cloud_filtered);
  n_filtered = cloud_filtered->points.size();
  ROS_INFO("num y-filtered pts = %d", n_filtered);

  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  pass.filter(*cloud_filtered);
  n_filtered = cloud_filtered->points.size();
  ROS_INFO("num z-filtered pts = %d", n_filtered);
  if (n_filtered < min_n_filtered)
  {
    ans_valid = false;
  }
  fit_points_to_plane(cloud_filtered, plane_normal, plane_dist);
  major_axis = major_axis_;
  centroid = centroid_;
  return ans_valid;
}

void PclUtils::filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps,
                              vector<int> &indices)
{
  int npts = inputCloud->points.size();
  Eigen::Vector3f pt;
  indices.clear();
  double dz;
  int ans;
  for (int i = 0; i < npts; ++i)
  {
    pt = inputCloud->points[i].getVector3fMap();

    dz = pt[2] - z_nom;
    if (fabs(dz) < z_eps)
    {
      indices.push_back(i);
    }
  }
  int n_extracted = indices.size();
  cout << " number of points in range = " << n_extracted << endl;
}

void PclUtils::filter_cloud_z(PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, double z_nom, double z_eps,
                              vector<int> &indices)
{
  int npts = inputCloud->points.size();
  Eigen::Vector3f pt;
  indices.clear();
  double dz;
  int ans;
  for (int i = 0; i < npts; ++i)
  {
    pt = inputCloud->points[i].getVector3fMap();

    dz = pt[2] - z_nom;
    if (fabs(dz) < z_eps)
    {
      indices.push_back(i);
    }
  }
  int n_extracted = indices.size();
  cout << " number of points in range = " << n_extracted << endl;
}

void PclUtils::filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud, double z_nom, double z_eps, double radius,
                              Eigen::Vector3f centroid, vector<int> &indices)
{
  int npts = inputCloud->points.size();
  Eigen::Vector3f pt;
  indices.clear();
  double dz;
  int ans;
  for (int i = 0; i < npts; ++i)
  {
    pt = inputCloud->points[i].getVector3fMap();

    dz = pt[2] - z_nom;
    if (fabs(dz) < z_eps)
    {
      if ((pt - centroid).norm() < radius)
      {
        indices.push_back(i);
      }
    }
  }
  int n_extracted = indices.size();
  cout << " number of points in range = " << n_extracted << endl;
}

void PclUtils::box_filter(PointCloud<pcl::PointXYZ>::Ptr inputCloud, Eigen::Vector3f pt_min, Eigen::Vector3f pt_max,
                          vector<int> &indices)
{
  int npts = inputCloud->points.size();
  Eigen::Vector3f pt;
  indices.clear();
  double dz;
  int ans;
  for (int i = 0; i < npts; ++i)
  {
    pt = inputCloud->points[i].getVector3fMap();

    if ((pt[0] > pt_min[0]) && (pt[0] < pt_max[0]) && (pt[1] > pt_min[1]) && (pt[1] < pt_max[1]) &&
        (pt[2] > pt_min[2]) && (pt[2] < pt_max[2]))
    {
      indices.push_back(i);
    }
  }
  int n_extracted = indices.size();
  cout << " number of points in range = " << n_extracted << endl;
}

void PclUtils::box_filter(Eigen::Vector3f pt_min, Eigen::Vector3f pt_max, vector<int> &indices)
{
  box_filter(pclTransformed_ptr_, pt_min, pt_max, indices);
}

void PclUtils::analyze_selected_points_color()
{
  int npts = pclTransformedSelectedPoints_ptr_->points.size();

  int npts_clr = pclSelectedPtsClr_ptr_->points.size();
  cout << "color pts size = " << npts_clr << endl;
  pcl::PointXYZRGB p;

  uint32_t rgb = *reinterpret_cast<int *>(&p.rgb);
  uint8_t r, g, b;
  int r_int;

  for (int i = 0; i < npts; ++i)
  {
    p = pclSelectedPtsClr_ptr_->points[i];
    r = (rgb >> 16) & 0x0000ff;
    r_int = (int)r;

    cout << "r_int: " << r_int << endl;
    cout << "r1: " << r << endl;
    r = pclSelectedPtsClr_ptr_->points[i].r;
    cout << "r2 = " << r << endl;
  }
  cout << "done combing through selected pts" << endl;
  got_kinect_cloud_ = false;
}

void PclUtils::copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud)
{
  int npts = inputCloud->points.size();
  outputCloud->header = inputCloud->header;
  outputCloud->is_dense = inputCloud->is_dense;
  outputCloud->width = npts;
  outputCloud->height = 1;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloud->points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloud->points[i].getVector3fMap() = inputCloud->points[i].getVector3fMap();
  }
}

void PclUtils::copy_cloud_xyzrgb_indices(PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, vector<int> &indices,
                                         PointCloud<pcl::PointXYZRGB>::Ptr outputCloud)
{
  int npts = indices.size();
  outputCloud->header = inputCloud->header;
  outputCloud->is_dense = inputCloud->is_dense;
  outputCloud->width = npts;
  outputCloud->height = 1;

  cout << "copying cloud w/ npts =" << npts << endl;
  outputCloud->points.resize(npts);
  for (int i = 0; i < npts; ++i)
  {
    outputCloud->points[i] = inputCloud->points[indices[i]];
  }
}

void PclUtils::transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr)
{
  output_cloud_ptr->header = input_cloud_ptr->header;
  output_cloud_ptr->is_dense = input_cloud_ptr->is_dense;
  output_cloud_ptr->width = input_cloud_ptr->width;
  output_cloud_ptr->height = input_cloud_ptr->height;
  int npts = input_cloud_ptr->points.size();
  cout << "transforming npts = " << npts << endl;
  output_cloud_ptr->points.resize(npts);

  for (int i = 0; i < npts; ++i)
  {
    output_cloud_ptr->points[i].getVector3fMap() = A * input_cloud_ptr->points[i].getVector3fMap();
  }
}

void PclUtils::transform_cloud(Eigen::Affine3f A, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr)
{
  output_cloud_ptr->header = input_cloud_ptr->header;
  output_cloud_ptr->is_dense = input_cloud_ptr->is_dense;
  output_cloud_ptr->width = input_cloud_ptr->width;
  output_cloud_ptr->height = input_cloud_ptr->height;
  int npts = input_cloud_ptr->points.size();
  cout << "transforming npts = " << npts << endl;
  output_cloud_ptr->points.resize(npts);

  float xval;
  pcl::PointXYZRGB pcl_pt;
  Eigen::Vector3f pt1, pt2;
  for (int i = 0; i < npts; ++i)
  {
    pt1 = input_cloud_ptr->points[i].getVector3fMap();

    pt2 = A * pt1;

    pcl_pt.x = pt2(0);
    pcl_pt.y = pt2(1);
    pcl_pt.z = pt2(2);
    pcl_pt.rgb = input_cloud_ptr->points[i].rgb;

    output_cloud_ptr->points[i] = pcl_pt;
  }
  int npts_out = output_cloud_ptr->points.size();
}

void PclUtils::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");

  pointcloud_subscriber_ = nh_.subscribe("/kinect/depth/points", 1, &PclUtils::kinectCB, this);
  real_kinect_subscriber_ = nh_.subscribe("/camera/depth_registered/points", 1, &PclUtils::kinectCB, this);

  selected_points_subscriber_ =
      nh_.subscribe<sensor_msgs::PointCloud2>("/selected_points", 1, &PclUtils::selectCB, this);
}

void PclUtils::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_pointcloud", 1, true);
}

/**
 * callback fnc: receives transmissions of Kinect data; if got_kinect_cloud is false, copy current transmission to
 * internal variable
 * @param cloud [in] messages received from Kinect
 */

void PclUtils::kinectCB(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  if (!got_kinect_cloud_)
  {
    pcl::fromROSMsg(*cloud, *pclKinect_ptr_);
    pcl::fromROSMsg(*cloud, *pclKinect_clr_ptr_);
    ROS_INFO("kinectCB: got cloud with %d * %d points", (int)pclKinect_ptr_->width, (int)pclKinect_ptr_->height);
    got_kinect_cloud_ = true;

    int npts_clr = pclKinect_clr_ptr_->points.size();
    cout << "Kinect color pts size = " << npts_clr << endl;
    avg_color_ = find_avg_color();
    /*
     for (size_t i = 0; i < pclKinect_clr_ptr_->points.size (); ++i)
     std::cout << " " << (int) pclKinect_clr_ptr_->points[i].r
              << " "    << (int) pclKinect_clr_ptr_->points[i].g
              << " "    << (int) pclKinect_clr_ptr_->points[i].b << std::endl;






        cout<<"done combing through selected pts"<<endl;
     *     */
  }
}

void PclUtils::selectCB(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  pcl::fromROSMsg(*cloud, *pclSelectedPoints_ptr_);

  ROS_INFO("RECEIVED NEW PATCH w/  %d * %d points", pclSelectedPoints_ptr_->width, pclSelectedPoints_ptr_->height);

  std::cout << "frame_id =" << pclSelectedPoints_ptr_->header.frame_id << endl;
  Eigen::Vector3f plane_normal;
  double plane_dist;
  fit_points_to_plane(pclSelectedPoints_ptr_, plane_normal, plane_dist);
  ROS_INFO("plane dist = %f", plane_dist);
  ROS_INFO("plane normal = (%f, %f, %f)", plane_normal(0), plane_normal(1), plane_normal(2));

  patch_normal_ = plane_normal;
  patch_dist_ = plane_dist;

  /*
  ROS_INFO("Color version has  %d * %d points", pclSelectedPtsClr_ptr_->width, pclSelectedPtsClr_ptr_->height);

  for (size_t i = 0; i < pclSelectedPtsClr_ptr_->points.size (); ++i) {
  std::cout <<i<<": "
            << "    " << (int) pclSelectedPtsClr_ptr_->points[i].r
            << " "    << (int) pclSelectedPtsClr_ptr_->points[i].g
            << " "    << (int) pclSelectedPtsClr_ptr_->points[i].b << std::endl;
  }
   * */
  ROS_INFO("done w/ selected-points callback");

  got_selected_points_ = true;
}