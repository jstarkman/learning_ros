// pcl+AF8-utils.h header file+ADs- doxygen comments follow //
/// wsn+ADs- Oct, 2015.
/// Include this file in +ACI-pcl+AF8-utils.cpp+ACI-, and in any main that uses this library.
/// This class provides example functions using the Point Cloud Library to operate
/// on point-cloud data

+ACM-ifndef PCL+AF8-UTILS+AF8-H+AF8-
+ACM-define PCL+AF8-UTILS+AF8-H+AF8-

+ACM-define +AF8-USE+AF8-MATH+AF8-DEFINES
+ACM-include +ADw-math.h+AD4-
+ACM-include +ADw-ros/ros.h+AD4-  //generic C stuff
+ACM-include +ADw-stdlib.h+AD4-
+ACM-include +ADw-iostream+AD4-
+ACM-include +ADw-vector+AD4-

+ACM-include +ADw-Eigen/Dense+AD4-
+ACM-include +ADw-Eigen/Eigen+AD4-  //for the Eigen library
+ACM-include +ADw-Eigen/Eigenvalues+AD4-
+ACM-include +ADw-Eigen/Geometry+AD4-

+ACM-include +ADw-geometry+AF8-msgs/PointStamped.h+AD4-
+ACM-include +ADw-geometry+AF8-msgs/TransformStamped.h+AD4-
+ACM-include +ADw-sensor+AF8-msgs/PointCloud2.h+AD4-  //useful ROS message types

//+ACM-include +ADw-cwru+AF8-msgs/PatchParams.h+AD4-

+ACM-include +ADw-tf/transform+AF8-broadcaster.h+AD4-
+ACM-include +ADw-tf/transform+AF8-listener.h+AD4-  // transform listener headers

+ACM-include +ADw-pcl/features/normal+AF8-3d.h+AD4-
+ACM-include +ADw-pcl/filters/extract+AF8-indices.h+AD4-
+ACM-include +ADw-pcl/io/io.h+AD4-
+ACM-include +ADw-pcl/io/pcd+AF8-io.h+AD4-  //point-cloud library headers+ADs- likely don't need all these
+ACM-include +ADw-pcl/point+AF8-cloud.h+AD4-
+ACM-include +ADw-pcl/point+AF8-types.h+AD4-
+ACM-include +ADw-pcl/ros/conversions.h+AD4-
+ACM-include +ADw-pcl+AF8-ros/point+AF8-cloud.h+AD4-
+ACM-include +ADw-pcl+AF8-ros/transforms.h+AD4-
+ACM-include +ADw-pcl-1.7/pcl/impl/point+AF8-types.hpp+AD4-

+ACM-include +ADw-pcl/filters/passthrough.h+AD4-
+ACM-include +ADw-pcl/filters/voxel+AF8-grid.h+AD4-

using namespace std+ADs-  // just to avoid requiring std::, Eigen:: ...
using namespace Eigen+ADs-
using namespace pcl+ADs-
using namespace pcl::io+ADs-

// define a class, including a constructor, member variables and member functions
class PclUtils
+AHs-
public:
  PclUtils(ros::NodeHandle +ACo-nodehandle)+ADs-  // constructor

  // insert doxygen documentation of member fncs+ADs-  run +ACI-doxywizard+ACI- to create documentation

  // this fnc is a copy of plane-fitter from exmaple+AF8-ros+AF8-library
  /+ACoAKg-provide an array of 3-D points (in columns), and this function will use and eigen-vector approach to find the
  +ACo- best-fit plane
  +ACo- It returns the plane's normal vector and the plane's (signed) distance from the origin.
  +ACo- +AEA-param points+AF8-array input: points+AF8-array is a matrix of 3-D points to be plane-fitted+ADs- coordinates are in columns
  +ACo- +AEA-param plane+AF8-normal output: this function will compute components of the plane normal here
  +ACo- +AEA-param plane+AF8-dist output: scalar (signed) distance of the plane from the origin
  +ACo-/
  void fit+AF8-points+AF8-to+AF8-plane(Eigen::MatrixXf points+AF8-array, Eigen::Vector3f +ACY-plane+AF8-normal, double +ACY-plane+AF8-dist)+ADs-
  Eigen::Vector3f compute+AF8-centroid(pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr input+AF8-cloud+AF8-ptr)+ADs-
  Eigen::Vector3f compute+AF8-centroid(pcl::PointCloud+ADw-pcl::PointXYZ+AD4- +ACY-input+AF8-cloud)+ADs-

  void fit+AF8-points+AF8-to+AF8-plane(pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr input+AF8-cloud+AF8-ptr, Eigen::Vector3f +ACY-plane+AF8-normal,
                           double +ACY-plane+AF8-dist)+ADs-
  // void fit+AF8-xformed+AF8-selected+AF8-pts+AF8-to+AF8-plane(Eigen::Vector3f +ACY-plane+AF8-normal, double +ACY-plane+AF8-dist)+ADs-

  //
  //
  //
  /+ACoAKg-a utility fnc to convert tf::Transform type into an Eigen::Affine3f
   +ACo- Affine type +ACI-f+ACI- is needed for use with point clouds, since +ACI-floats+ACI- not +ACI-doubles+ACI- are more practical, considering
   +ACo- the expected (limited) resolution of the sensor, as well as the large size of point clouds
   +ACo- +AEA-param t  +AFs-in+AF0- provide a transform, e.g. per:
   +ACo-     g+AF8-tfListenerPtr-+AD4-lookupTransform(+ACI-torso+ACI-, +ACI-kinect+AF8-pc+AF8-frame+ACI-, ros::Time(0), tf+AF8-sensor+AF8-frame+AF8-to+AF8-torso+AF8-frame)+ADs-
   +ACo- +AEA-return an Eigen Affine object, A, such that point+AF8-in+AF8-new+AF8-frame +AD0- A+ACo-point+AF8-in+AF8-original+AF8-frame
   +ACo-/
  Eigen::Affine3f transformTFToEigen(const tf::Transform +ACY-t)+ADs-

  /+ACoAKg- function to create an Eigen-style Affine transform based on construction of a coordinate frame
   +ACo- placed on the surface of a plane
   +ACo-/
  Eigen::Affine3f make+AF8-affine+AF8-from+AF8-plane+AF8-params(Eigen::Vector3f plane+AF8-normal, double plane+AF8-dist)+ADs-
  /+ACoAKg-
   +ACo- returns an Eigen::Affine transform to a coordinate frame constructed on a plane defined by
   +ACo- plane+AF8-parameters (normal+AF8-x, normal+AF8-y, normal+AF8-z, distance)
   +ACo- useful for transforming data to find planar surfaces with z-axis vertical
   +ACo-/
  Eigen::Affine3f make+AF8-affine+AF8-from+AF8-plane+AF8-params(Eigen::Vector4f plane+AF8-parameters)+ADs-
  Eigen::Affine3f make+AF8-affine+AF8-from+AF8-plane+AF8-params(Eigen::Vector4f plane+AF8-parameters, Eigen::Vector3f centroid)+ADs-

  void transform+AF8-kinect+AF8-cloud(Eigen::Affine3f A)+ADs-
  void transform+AF8-selected+AF8-points+AF8-cloud(Eigen::Affine3f A)+ADs-
  void transform+AF8-cloud(Eigen::Affine3f A, pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr input+AF8-cloud+AF8-ptr,
                       pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr output+AF8-cloud+AF8-ptr)+ADs-
  // color version:
  void transform+AF8-cloud(Eigen::Affine3f A, pcl::PointCloud+ADw-pcl::PointXYZRGB+AD4-::Ptr input+AF8-cloud+AF8-ptr,
                       pcl::PointCloud+ADw-pcl::PointXYZRGB+AD4-::Ptr output+AF8-cloud+AF8-ptr)+ADs-
  void reset+AF8-got+AF8-kinect+AF8-cloud()
  +AHs-
    got+AF8-kinect+AF8-cloud+AF8- +AD0- false+ADs-
  +AH0AOw-
  void reset+AF8-got+AF8-selected+AF8-points()
  +AHs-
    got+AF8-selected+AF8-points+AF8- +AD0- false+ADs-
  +AH0AOw-
  // void take+AF8-snapshot() +AHs-take+AF8-snapshot+AF8APQ- true+ADsAfQA7-
  bool got+AF8-kinect+AF8-cloud()
  +AHs-
    return got+AF8-kinect+AF8-cloud+AF8AOw-
  +AH0AOw-
  bool got+AF8-selected+AF8-points()
  +AHs-
    return got+AF8-selected+AF8-points+AF8AOw-
  +AH0AOw-
  void save+AF8-kinect+AF8-snapshot()
  +AHs-
    pcl::io::savePCDFileASCII(+ACI-kinect+AF8-snapshot.pcd+ACI-, +ACo-pclKinect+AF8-ptr+AF8-)+ADs-
  +AH0AOw-  // B/W
  int read+AF8-pcd+AF8-file(string fname)+ADs-
  int read+AF8-clr+AF8-pcd+AF8-file(string fname)+ADs-

  // alternative +ACI-save+ACI- fnc: save as a colored pointcloud
  void save+AF8-kinect+AF8-clr+AF8-snapshot()
  +AHs-
    pcl::io::savePCDFileASCII(+ACI-kinect+AF8-clr+AF8-snapshot.pcd+ACI-, +ACo-pclKinect+AF8-clr+AF8-ptr+AF8-)+ADs-
  +AH0AOw-
  int save+AF8-kinect+AF8-clr+AF8-snapshot+AF8-binary()
  +AHs-
    return (pcl::io::savePCDFile(+ACI-kinect+AF8-clr+AF8-snapshot+AF8-bin.pcd+ACI-, +ACo-pclKinect+AF8-clr+AF8-ptr+AF8-, true))+ADs-
  +AH0AOw-

  void save+AF8-transformed+AF8-kinect+AF8-snapshot()
  +AHs-
    pcl::io::savePCDFileASCII(+ACI-xformed+AF8-kinect+AF8-snapshot.pcd+ACI-, +ACo-pclTransformed+AF8-ptr+AF8-)+ADs-
  +AH0AOw-
  void get+AF8-transformed+AF8-selected+AF8-points(pcl::PointCloud+ADw-pcl::PointXYZ+AD4- +ACY-outputCloud)+ADs-
  void get+AF8-copy+AF8-selected+AF8-points(pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr +ACY-outputCloud)+ADs-

  void copy+AF8-cloud(PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr inputCloud, PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr outputCloud)+ADs-
  void copy+AF8-cloud+AF8-xyzrgb+AF8-indices(PointCloud+ADw-pcl::PointXYZRGB+AD4-::Ptr inputCloud, vector+ADw-int+AD4- +ACY-indices,
                                 PointCloud+ADw-pcl::PointXYZRGB+AD4-::Ptr outputCloud)+ADs-

  void get+AF8-indices(vector+ADw-int+AD4- +ACY-indices)
  +AHs-
    indices +AD0- indices+AF8AOw-
  +AH0AOw-
  // same as above, but assumes
  void copy+AF8-indexed+AF8-pts+AF8-to+AF8-output+AF8-cloud(vector+ADw-int+AD4- +ACY-indices, PointCloud+ADw-pcl::PointXYZRGB+AD4- +ACY-outputCloud)+ADs-

  void get+AF8-gen+AF8-purpose+AF8-cloud(pcl::PointCloud+ADw-pcl::PointXYZ+AD4- +ACY-outputCloud)+ADs-
  void get+AF8-kinect+AF8-points(pcl::PointCloud+ADw-pcl::PointXYZ+AD4- +ACY-outputCloud)+ADs-
  void get+AF8-kinect+AF8-points(pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr +ACY-outputCloudPtr)+ADs-
  void get+AF8-kinect+AF8-points(pcl::PointCloud+ADw-pcl::PointXYZRGB+AD4- +ACY-outputCloudPtr)+ADs-
  void get+AF8-kinect+AF8-points(pcl::PointCloud+ADw-pcl::PointXYZRGB+AD4-::Ptr +ACY-outputCloud)+ADs-
  void get+AF8-selected+AF8-points(pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr +ACY-outputCloudPtr)+ADs-
  void get+AF8-selected+AF8-points(pcl::PointCloud+ADw-pcl::PointXYZ+AD4- +ACY-outputCloud)+ADs-

  void example+AF8-pcl+AF8-operation()+ADs-
  // operate on transformed Kinect data and identify point indices within z+AF8-eps of specified height
  void filter+AF8-cloud+AF8-z(PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr inputCloud, double z+AF8-nom, double z+AF8-eps, vector+ADw-int+AD4- +ACY-indices)+ADs-
  void filter+AF8-cloud+AF8-z(PointCloud+ADw-pcl::PointXYZRGB+AD4-::Ptr inputCloud, double z+AF8-nom, double z+AF8-eps, vector+ADw-int+AD4- +ACY-indices)+ADs-
  // as above, specifically for transformed kinect cloud:
  void find+AF8-coplanar+AF8-pts+AF8-z+AF8-height(double plane+AF8-height, double z+AF8-eps, vector+ADw-int+AD4- +ACY-indices)+ADs-
  // find pts within  z+AF8-eps of z+AF8-height, AND within +ACI-radius+ACI- of +ACI-centroid+ACI-
  void filter+AF8-cloud+AF8-z(PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr inputCloud, double z+AF8-nom, double z+AF8-eps, double radius,
                      Eigen::Vector3f centroid, vector+ADw-int+AD4- +ACY-indices)+ADs-
  // same as above, but specifically operates on transformed kinect cloud
  void filter+AF8-cloud+AF8-z(double z+AF8-nom, double z+AF8-eps, double radius, Eigen::Vector3f centroid, vector+ADw-int+AD4- +ACY-indices)+ADs-
  int box+AF8-filter+AF8-z+AF8-transformed+AF8-cloud(double z+AF8-min, double z+AF8-max, vector+ADw-int+AD4- +ACY-indices)+ADs-

  // using passthrough filter is MUCH faster+ACEAIQ-
  double find+AF8-table+AF8-height(double z+AF8-min, double z+AF8-max, double dz)+ADs-  // op on xformed cloud+ADs- uses pcl passthru filter
  // another fnc using passthru filter--with x, y and z limits
  double find+AF8-table+AF8-height(double x+AF8-min, double x+AF8-max, double y+AF8-min, double y+AF8-max, double z+AF8-min, double z+AF8-max,
                           double dz+AF8-tol)+ADs-

  void box+AF8-filter(PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr inputCloud, Eigen::Vector3f pt+AF8-min, Eigen::Vector3f pt+AF8-max,
                  vector+ADw-int+AD4- +ACY-indices)+ADs-
  void box+AF8-filter(Eigen::Vector3f pt+AF8-min, Eigen::Vector3f pt+AF8-max, vector+ADw-int+AD4- +ACY-indices)+ADs-
  bool find+AF8-plane+AF8-fit(double x+AF8-min, double x+AF8-max, double y+AF8-min, double y+AF8-max, double z+AF8-min, double z+AF8-max, double dz+AF8-tol,
                      Eigen::Vector3f +ACY-plane+AF8-normal, double +ACY-plane+AF8-dist, Eigen::Vector3f +ACY-major+AF8-axis,
                      Eigen::Vector3f +ACY-centroid)+ADs-

  void analyze+AF8-selected+AF8-points+AF8-color()+ADs-

  Eigen::Vector3f get+AF8-centroid()
  +AHs-
    return centroid+AF8AOw-
  +AH0AOw-
  Eigen::Vector3f get+AF8-major+AF8-axis()
  +AHs-
    return major+AF8-axis+AF8AOw-
  +AH0AOw-
  Eigen::Vector3f get+AF8-patch+AF8-normal()
  +AHs-
    return patch+AF8-normal+AF8AOw-
  +AH0AOw-
  double get+AF8-patch+AF8-dist()
  +AHs-
    return patch+AF8-dist+AF8AOw-
  +AH0AOw-
  Eigen::Vector3d find+AF8-avg+AF8-color()+ADs-
  Eigen::Vector3d find+AF8-avg+AF8-color+AF8-selected+AF8-pts(vector+ADw-int+AD4- +ACY-indices)+ADs-
  void find+AF8-indices+AF8-color+AF8-match(vector+ADw-int+AD4- +ACY-input+AF8-indices, Eigen::Vector3d normalized+AF8-avg+AF8-color,
                                double color+AF8-match+AF8-thresh, vector+ADw-int+AD4- +ACY-output+AF8-indices)+ADs-

private:
  ros::NodeHandle nh+AF8AOw-
  // some objects to support subscriber, service, and publisher
  ros::Subscriber pointcloud+AF8-subscriber+AF8AOw-   // use this to subscribe to a pointcloud topic
  ros::Subscriber real+AF8-kinect+AF8-subscriber+AF8AOw-  // use this to subscribe to a physical kinect device

  ros::Subscriber selected+AF8-points+AF8-subscriber+AF8AOw-  // this to subscribe to +ACI-selectedPoints+ACI- topic from Rviz

  // ros::ServiceServer minimal+AF8-service+AF8AOw- //maybe want these later
  ros::Publisher pointcloud+AF8-publisher+AF8AOw-
  ros::Publisher patch+AF8-publisher+AF8AOw-

  pcl::PointCloud+ADw-pcl::PointXYZRGB+AD4-::Ptr pclKinect+AF8-clr+AF8-ptr+AF8AOw-      // pointer for color version of pointcloud
  pcl::PointCloud+ADw-pcl::PointXYZRGB+AD4-::Ptr pclSelectedPtsClr+AF8-ptr+AF8AOw-  // pointer for color version of pointcloud

  pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr pclKinect+AF8-ptr+AF8AOw-  //(new PointCloud+ADw-pcl::PointXYZ+AD4-)+ADs-
  pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr pclTransformed+AF8-ptr+AF8AOw-
  pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr pclSelectedPoints+AF8-ptr+AF8AOw-
  pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr pclTransformedSelectedPoints+AF8-ptr+AF8AOw-
  pcl::PointCloud+ADw-pcl::PointXYZ+AD4-::Ptr pclGenPurposeCloud+AF8-ptr+AF8AOw-
  pcl::PassThrough+ADw-pcl::PointXYZ+AD4- pass+ADs-  // create a pass-through object
  bool got+AF8-kinect+AF8-cloud+AF8AOw-
  bool got+AF8-selected+AF8-points+AF8AOw-
  bool take+AF8-snapshot+AF8AOw-
  // member methods as well:
  void initializeSubscribers()+ADs-  // we will define some helper methods to encapsulate the gory details of initializing
                                 // subscribers, publishers and services
  void initializePublishers()+ADs-
  // void initializeServices()+ADs-

  void kinectCB(const sensor+AF8-msgs::PointCloud2ConstPtr +ACY-cloud)+ADs-  // prototype for callback fnc
  void selectCB(const sensor+AF8-msgs::PointCloud2ConstPtr +ACY-cloud)+ADs-  // callback for selected points

  Eigen::Vector3f major+AF8-axis+AF8-, centroid+AF8AOw-
  Eigen::Vector3d avg+AF8-color+AF8AOw-
  Eigen::Vector3f patch+AF8-normal+AF8AOw-
  double patch+AF8-dist+AF8AOw-
  vector+ADw-int+AD4- indices+AF8AOw-  // put interesting indices here
                         // prototype for example service
  // bool serviceCallback(example+AF8-srv::simple+AF8-bool+AF8-service+AF8-messageRequest+ACY- request,
  // example+AF8-srv::simple+AF8-bool+AF8-service+AF8-messageResponse+ACY- response)+ADs-

+AH0AOw-  // note: a class definition requires a semicolon at the end of the definition

+ACM-endif  // this closes the header-include trick...ALWAYS need one of these to match +ACM-ifndef
