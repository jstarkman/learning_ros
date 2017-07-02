
#include <arm7dof_traj_as/arm7dof_traj_as.h>

Arm7dof_traj_streamer::Arm7dof_traj_streamer(ros::NodeHandle* nodehandle)
{
  initializeSubscribers();
  initializePublishers();

  qdot_max_vec_ << q0dotmax, q1dotmax, q2dotmax, q3dotmax, q4dotmax, q5dotmax, q6dotmax;
  qdot_max_vec_ *= SPEED_SCALE_FACTOR;

  q_vec_Xd_.resize(7);
}

void Arm7dof_traj_streamer::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");
  joint_state_sub_ = nh_.subscribe("arm7dof/joint_states", 1, &Arm7dof_traj_streamer::jointStatesCb, this);
}

void Arm7dof_traj_streamer::initializePublishers()
{
  ROS_INFO("Initializing Publishers");

  traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("trajActionServer", 1);
}

void Arm7dof_traj_streamer::map_arm_joint_indices(vector<string> joint_names)
{
  joint_indices_.clear();

  int index;
  int n_jnts = joint_names.size();
  cout << "num jnt names = " << n_jnts << endl;
  std::string j_name;

  for (int j = 0; j < arm7dof_NJNTS; j++)
  {
    j_name = g_arm7dof_jnt_names[j];
    for (int i = 0; i < n_jnts; i++)
    {
      if (j_name.compare(joint_names[i]) == 0)
      {
        index = i;
        joint_indices_.push_back(index);
        break;
      }
    }
  }
  cout << "indices of arm joints: " << endl;
  for (int i = 0; i < arm7dof_NJNTS; i++)
  {
    cout << joint_indices_[i] << ", ";
  }
}

void Arm7dof_traj_streamer::jointStatesCb(const sensor_msgs::JointState& js_msg)
{
  joint_states_ = js_msg;
  map_arm_joint_indices(js_msg.name);
  for (int i = 0; i < arm7dof_NJNTS; i++)
  {
    q_vec_[i] = js_msg.position[joint_indices_[i]];
    q_vec_Xd_[i] = q_vec_[i];
  }
}

double Arm7dof_traj_streamer::transition_time(Vectorq7x1 dqvec)
{
  double t_max = fabs(dqvec[0]) / qdot_max_vec_[0];

  double ti;
  for (int i = 1; i < 7; i++)
  {
    ti = fabs(dqvec[i]) / qdot_max_vec_[i];
    if (ti > t_max)
      t_max = ti;
  }
  return t_max;
}

double Arm7dof_traj_streamer::transition_time(Eigen::VectorXd dqvec)
{
  double t_max = fabs(dqvec[0]) / qdot_max_vec_[0];

  double ti;
  for (int i = 1; i < 7; i++)
  {
    ti = fabs(dqvec[i]) / qdot_max_vec_[i];
    if (ti > t_max)
      t_max = ti;
  }
  return t_max;
}

void Arm7dof_traj_streamer::stuff_trajectory(std::vector<Eigen::VectorXd> qvecs,
                                             trajectory_msgs::JointTrajectory& new_trajectory)
{
  trajectory_msgs::JointTrajectoryPoint trajectory_point1;

  trajectory_point1.positions.clear();

  new_trajectory.points.clear();
  new_trajectory.joint_names.clear();

  new_trajectory.joint_names.push_back("joint0");
  new_trajectory.joint_names.push_back("joint1");
  new_trajectory.joint_names.push_back("joint2");
  new_trajectory.joint_names.push_back("joint3");
  new_trajectory.joint_names.push_back("joint4");
  new_trajectory.joint_names.push_back("joint5");
  new_trajectory.joint_names.push_back("joint6");

  new_trajectory.header.stamp = ros::Time::now();
  Eigen::VectorXd q_start, q_end, dqvec;
  double del_time;
  double net_time = 0.0;
  q_start = qvecs[0];
  q_end = qvecs[0];

  trajectory_point1.time_from_start = ros::Duration(net_time);
  for (int i = 0; i < 7; i++)
  {
    trajectory_point1.positions.push_back(q_start[i]);
  }
  new_trajectory.points.push_back(trajectory_point1);

  for (int iq = 1; iq < qvecs.size(); iq++)
  {
    q_start = q_end;
    q_end = qvecs[iq];
    dqvec = q_end - q_start;

    del_time = transition_time(dqvec);
    if (del_time < dt_traj)
      del_time = dt_traj;

    net_time += del_time;

    for (int i = 0; i < 7; i++)
    {
      trajectory_point1.positions[i] = q_end[i];
    }

    trajectory_point1.time_from_start = ros::Duration(net_time);
    new_trajectory.points.push_back(trajectory_point1);
  }
}

void Arm7dof_traj_streamer::pub_arm_trajectory(trajectory_msgs::JointTrajectory& new_trajectory)
{
  traj_pub_.publish(new_trajectory);
  cout << "publishing arm trajectory with npts = " << new_trajectory.points.size() << endl;
}

void Arm7dof_traj_streamer::pub_arm_trajectory_init()
{
  std::vector<Eigen::VectorXd> qvec;
  trajectory_msgs::JointTrajectory new_trajectory;

  qvec.push_back(q_vec_Xd_);
  qvec.push_back(q_vec_Xd_);
  stuff_trajectory(qvec, new_trajectory);
  traj_pub_.publish(new_trajectory);
}
