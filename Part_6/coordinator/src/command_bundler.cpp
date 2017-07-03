

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigator/navigatorAction.h>
#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <actionlib/server/simple_action_server.h>

#include <coordinator/ManipTaskAction.h>

#include <generic_gripper_services/genericGripperInterface.h>
#include <object_manipulation_properties/object_ID_codes.h>

class TaskActionServer
{
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<coordinator::ManipTaskAction> as_;

  coordinator::ManipTaskGoal goal_;
  coordinator::ManipTaskResult result_;
  coordinator::ManipTaskFeedback feedback_;

  actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac_;
  actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac_;

  object_finder::objectFinderGoal object_finder_goal_;
  object_grabber::object_grabberGoal object_grabber_goal_;
  void objectGrabberDoneCb_(const actionlib::SimpleClientGoalState& state,
                            const object_grabber::object_grabberResultConstPtr& result);
  void objectFinderDoneCb_(const actionlib::SimpleClientGoalState& state,
                           const object_finder::objectFinderResultConstPtr& result);
  int object_grabber_return_code_;
  int found_object_code_;

  geometry_msgs::PoseStamped pickup_pose_;
  geometry_msgs::PoseStamped dropoff_pose_;
  int goal_action_code_, object_code_, perception_source_;
  int vision_object_code_;
  double surface_height_;
  bool found_surface_height_;

  bool working_on_task_;
  int status_code_;
  int action_code_, pickup_action_code_, dropoff_action_code_;
  ros::Publisher pose_publisher_;

public:
  TaskActionServer();

  ~TaskActionServer(void)
  {
  }

  void executeCB(const actionlib::SimpleActionServer<coordinator::ManipTaskAction>::GoalConstPtr& goal);
};

TaskActionServer::TaskActionServer()
  : as_(nh_, "manip_task_action_service", boost::bind(&TaskActionServer::executeCB, this, _1), false)
  , object_finder_ac_("object_finder_action_service", true)
  , object_grabber_ac_("object_grabber_action_service", true)
{
  ROS_INFO("in constructor of TaskActionServer...");

  as_.start();

  action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
  status_code_ = coordinator::ManipTaskFeedback::NO_CURRENT_TASK;
  working_on_task_ = false;

  ROS_INFO("waiting for object-grabber action server: ");
  bool server_exists = false;
  while ((!server_exists) && (ros::ok()))
  {
    server_exists = object_grabber_ac_.waitForServer(ros::Duration(0.5));
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("retrying...");
  }
  ROS_INFO("connected to object_grabber action server");

  ROS_INFO("attempting to connect to object-finder action server");
  server_exists = false;
  while ((!server_exists) && (ros::ok()))
  {
    server_exists = object_finder_ac_.waitForServer(ros::Duration(0.5));
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    ROS_INFO("retrying...");
  }
  ROS_INFO("connected to object_finder action server");

  pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
  found_surface_height_ = false;
}

void TaskActionServer::objectFinderDoneCb_(const actionlib::SimpleClientGoalState& state,
                                           const object_finder::objectFinderResultConstPtr& result)
{
  ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());

  ROS_INFO("got object code response = %d; ", result->found_object_code);
  if (result->found_object_code == object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED)
  {
    ROS_WARN("object code not recognized");
  }
  else if (result->found_object_code == object_finder::objectFinderResult::OBJECT_FOUND)
  {
    ROS_INFO("found object!");
    pickup_pose_ = result->object_pose;
    result_.object_pose = pickup_pose_;
    ROS_INFO("got pose x,y,z = %f, %f, %f", pickup_pose_.pose.position.x, pickup_pose_.pose.position.y,
             pickup_pose_.pose.position.z);
    pose_publisher_.publish(pickup_pose_);
  }
  else
  {
    ROS_WARN("object not found!");
  }

  result_.object_finder_return_code = found_object_code_;
  found_object_code_ = result->found_object_code;
}

void TaskActionServer::objectGrabberDoneCb_(const actionlib::SimpleClientGoalState& state,
                                            const object_grabber::object_grabberResultConstPtr& result)
{
  ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());

  ROS_INFO("got result output = %d; ", result->return_code);

  result_.object_grabber_return_code = object_grabber_return_code_;
  object_grabber_return_code_ = result->return_code;
}

void TaskActionServer::executeCB(const actionlib::SimpleActionServer<coordinator::ManipTaskAction>::GoalConstPtr& goal)
{
  ROS_INFO("in executeCB: received manipulation task");

  goal_action_code_ = goal->action_code;
  action_code_ = goal->action_code;
  ROS_INFO("requested action code is: %d", goal_action_code_);

  if (goal_action_code_ == coordinator::ManipTaskGoal::DROPOFF_OBJECT)
  {
    object_code_ = goal->object_code;
  }
  else if (goal_action_code_ == coordinator::ManipTaskGoal::GRAB_OBJECT)
  {
    object_code_ = goal->object_code;
    ROS_INFO("object code is: %d", object_code_);

    if (goal->perception_source == coordinator::ManipTaskGoal::BLIND_MANIP)
    {
      ROS_INFO("blind manipulation; using provided pick-up pose");
      pickup_pose_ = goal->pickup_frame;
    }
  }
  else if (goal_action_code_ == coordinator::ManipTaskGoal::STRADDLE_OBJECT)
  {
    object_code_ = goal->object_code;
    ROS_INFO("object code is: %d", object_code_);
    pickup_pose_ = goal->pickup_frame;
  }
  else if (goal_action_code_ == coordinator::ManipTaskGoal::GET_PICKUP_POSE)
  {
    ROS_INFO("object code is: %d", object_code_);
    ROS_INFO("perception_source is: %d", goal->perception_source);
    object_code_ = goal->object_code;
    perception_source_ = goal->perception_source;
    vision_object_code_ = object_code_;
  }

  status_code_ = coordinator::ManipTaskFeedback::RECEIVED_NEW_TASK;
  working_on_task_ = true;

  while (working_on_task_)
  {
    feedback_.feedback_status = status_code_;
    as_.publishFeedback(feedback_);

    if (as_.isPreemptRequested())
    {
      ROS_WARN("goal cancelled!");
      result_.manip_return_code = coordinator::ManipTaskResult::ABORTED;
      action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
      status_code_ = coordinator::ManipTaskFeedback::ABORTED;
      working_on_task_ = false;
      as_.setAborted(result_);
      return;
    }

    switch (action_code_)
    {
      case coordinator::ManipTaskGoal::FIND_TABLE_SURFACE:
        ROS_INFO("serving request to find table surface");
        found_object_code_ = object_finder::objectFinderResult::OBJECT_FINDER_BUSY;
        object_finder_goal_.object_id = ObjectIdCodes::TABLE_SURFACE;

        object_finder_goal_.known_surface_ht = false;

        object_finder_ac_.sendGoal(object_finder_goal_,
                                   boost::bind(&TaskActionServer::objectFinderDoneCb_, this, _1, _2));
        action_code_ = coordinator::ManipTaskGoal::WAIT_FIND_TABLE_SURFACE;
        ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
        ROS_INFO("waiting on perception");
        break;
      case coordinator::ManipTaskGoal::WAIT_FIND_TABLE_SURFACE:
        if (found_object_code_ == object_finder::objectFinderResult::OBJECT_FOUND)
        {
          ROS_INFO("surface-finder success");
          surface_height_ = pickup_pose_.pose.position.z;
          found_surface_height_ = true;
          ROS_INFO("found table ht = %f", surface_height_);
          as_.setSucceeded(result_);
          return;
        }
        else if (found_object_code_ == object_finder::objectFinderResult::OBJECT_FINDER_BUSY)
        {
        }
        else
        {
          ROS_WARN("object-finder failure; aborting");
          action_code_ = coordinator::ManipTaskGoal::ABORT;
          result_.manip_return_code = coordinator::ManipTaskResult::FAILED_PERCEPTION;
          found_surface_height_ = false;
        }

        break;

      case coordinator::ManipTaskGoal::GET_PICKUP_POSE:
        ROS_INFO("establishing pick-up pose");
        if (perception_source_ == coordinator::ManipTaskGoal::BLIND_MANIP)
        {
          ROS_INFO("blind manipulation; using provided pick-up pose");
          pickup_pose_ = goal->pickup_frame;
          result_.object_pose = pickup_pose_;

          found_object_code_ = object_finder::objectFinderResult::OBJECT_FOUND;
          action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_FINDER;
          status_code_ = coordinator::ManipTaskFeedback::PERCEPTION_BUSY;
        }
        else if (perception_source_ == coordinator::ManipTaskGoal::PCL_VISION)
        {
          ROS_INFO("invoking object finder");
          found_object_code_ = object_finder::objectFinderResult::OBJECT_FINDER_BUSY;
          ROS_INFO("instructing finder to locate object %d", vision_object_code_);
          object_finder_goal_.object_id = vision_object_code_;
          if (found_surface_height_)
          {
            object_finder_goal_.known_surface_ht = true;
            object_finder_goal_.surface_ht = surface_height_;
            ROS_INFO("using surface ht = %f", surface_height_);
          }
          else
          {
            object_finder_goal_.known_surface_ht = false;
          }

          ROS_INFO("sending object-finder goal: ");

          object_finder_ac_.sendGoal(object_finder_goal_,
                                     boost::bind(&TaskActionServer::objectFinderDoneCb_, this, _1, _2));

          action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_FINDER;
          ROS_INFO("waiting on perception");
        }
        else
        {
          ROS_WARN("unrecognized perception mode; quitting");
          action_code_ = coordinator::ManipTaskGoal::ABORT;
          result_.manip_return_code = coordinator::ManipTaskResult::FAILED_PERCEPTION;
        }

        ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
        break;

      case coordinator::ManipTaskGoal::WAIT_FOR_FINDER:
        if (found_object_code_ == object_finder::objectFinderResult::OBJECT_FOUND)
        {
          ROS_INFO("object-finder success");

          /* if (goal_action_code_ == coordinator::ManipTaskGoal::MANIP_OBJECT) {
                        action_code_ = coordinator::ManipTaskGoal::GRAB_OBJECT;
                        status_code_ = coordinator::ManipTaskFeedback::DROPOFF_PLANNING_BUSY;
                    } else*/ {
            working_on_task_ = false;
            action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
            status_code_ = coordinator::ManipTaskFeedback::COMPLETED_MOVE;
            result_.manip_return_code = coordinator::ManipTaskResult::MANIP_SUCCESS;

            as_.setSucceeded(result_);
            return;
          }

          object_grabber_return_code_ = object_grabber::object_grabberResult::PENDING;
        }
        else if (found_object_code_ == object_finder::objectFinderResult::OBJECT_FINDER_BUSY)
        {
        }
        else
        {
          ROS_WARN("object-finder failure; aborting");
          action_code_ = coordinator::ManipTaskGoal::ABORT;
          result_.manip_return_code = coordinator::ManipTaskResult::FAILED_PERCEPTION;
        }
        break;

      case coordinator::ManipTaskGoal::GRAB_OBJECT:
        status_code_ = coordinator::ManipTaskFeedback::PICKUP_MOTION_BUSY;
        ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);

        object_grabber_goal_.action_code = object_grabber::object_grabberGoal::GRAB_OBJECT;
        object_grabber_goal_.object_frame = pickup_pose_;
        object_grabber_goal_.object_id = object_code_;
        object_grabber_goal_.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY;

        ROS_INFO("sending goal to grab object: ");
        object_grabber_ac_.sendGoal(object_grabber_goal_,
                                    boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

        action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_GRAB_OBJECT;
        status_code_ = coordinator::ManipTaskFeedback::PICKUP_MOTION_BUSY;

        object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;

        break;

      case coordinator::ManipTaskGoal::WAIT_FOR_GRAB_OBJECT:
        if (object_grabber_return_code_ == object_grabber::object_grabberResult::OBJECT_ACQUIRED)
        {
          ROS_INFO("acquired object");

          /*if (goal_action_code_ == coordinator::ManipTaskGoal::MANIP_OBJECT) {
                        
                        action_code_ = coordinator::ManipTaskGoal::DROPOFF_OBJECT;
                        status_code_ = coordinator::ManipTaskFeedback::PICKUP_SUCCESSFUL;
                    } else*/ {
            working_on_task_ = false;
            action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
            status_code_ = coordinator::ManipTaskFeedback::COMPLETED_MOVE;
            result_.manip_return_code = coordinator::ManipTaskResult::MANIP_SUCCESS;

            as_.setSucceeded(result_);
            return;
          }
        }
        else if (object_grabber_return_code_ == object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY)
        {
        }
        else
        {
          ROS_WARN("trouble with acquiring object");
          action_code_ = coordinator::ManipTaskGoal::ABORT;
          result_.manip_return_code = coordinator::ManipTaskResult::FAILED_PICKUP;
        }
        break;

      case coordinator::ManipTaskGoal::CART_MOVE_TO_GRIPPER_POSE:
        status_code_ = coordinator::ManipTaskFeedback::MOVE_BUSY;
        ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
        object_grabber_goal_.action_code = object_grabber::object_grabberGoal::CART_MOVE_CURRENT_TO_CART_GOAL;
        object_grabber_goal_.object_frame = goal->gripper_goal_frame;

        ROS_INFO("sending goal to move gripper along Cartesian path to specified destination: ");
        object_grabber_ac_.sendGoal(object_grabber_goal_,
                                    boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

        action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_MOVE;
        status_code_ = coordinator::ManipTaskFeedback::MOVE_BUSY;

        object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;

        break;

      case coordinator::ManipTaskGoal::STRADDLE_OBJECT:
        status_code_ = coordinator::ManipTaskFeedback::MOVE_BUSY;
        ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);

        object_grabber_goal_.action_code = object_grabber::object_grabberGoal::STRADDLE_OBJECT;
        object_grabber_goal_.object_frame = pickup_pose_;
        object_grabber_goal_.object_id = object_code_;
        object_grabber_goal_.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY;

        ROS_INFO("sending goal to straddle object: ");
        object_grabber_ac_.sendGoal(object_grabber_goal_,
                                    boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

        action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_MOVE;
        status_code_ = coordinator::ManipTaskFeedback::MOVE_BUSY;

        object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;

        break;

      case coordinator::ManipTaskGoal::DROPOFF_OBJECT:
        status_code_ = coordinator::ManipTaskFeedback::DROPOFF_MOTION_BUSY;
        ROS_INFO("executeCB: action_code, status_code = %d, %d", action_code_, status_code_);
        object_grabber_goal_.action_code = object_grabber::object_grabberGoal::DROPOFF_OBJECT;
        object_grabber_goal_.object_id = object_code_;
        dropoff_pose_ = goal->dropoff_frame;
        object_grabber_goal_.object_frame = dropoff_pose_;
        object_grabber_goal_.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY;

        ROS_INFO("sending goal to drop off object: ");
        object_grabber_ac_.sendGoal(object_grabber_goal_,
                                    boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

        action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_DROPOFF_OBJECT;

        object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;

        break;

      case coordinator::ManipTaskGoal::WAIT_FOR_DROPOFF_OBJECT:

        if (object_grabber_return_code_ == object_grabber::object_grabberResult::SUCCESS)
        {
          ROS_INFO("switch/case happiness!  dropped off object; manip complete");
          working_on_task_ = false;
          action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
          status_code_ = coordinator::ManipTaskFeedback::COMPLETED_DROPOFF;

          result_.manip_return_code = coordinator::ManipTaskResult::MANIP_SUCCESS;
          as_.setSucceeded(result_);
          return;
        }
        else if (object_grabber_return_code_ == object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY)
        {
        }
        else
        {
          ROS_WARN("trouble with acquiring object");
          action_code_ = coordinator::ManipTaskGoal::ABORT;
          result_.manip_return_code = coordinator::ManipTaskResult::FAILED_DROPOFF;
        }
        break;

      case coordinator::ManipTaskGoal::MOVE_TO_PRE_POSE:
        status_code_ = coordinator::ManipTaskFeedback::PREPOSE_MOVE_BUSY;
        object_grabber_goal_.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
        ROS_INFO("sending goal to move to pre-pose: ");
        object_grabber_ac_.sendGoal(object_grabber_goal_,
                                    boost::bind(&TaskActionServer::objectGrabberDoneCb_, this, _1, _2));

        action_code_ = coordinator::ManipTaskGoal::WAIT_FOR_MOVE;

        object_grabber_return_code_ = object_grabber::object_grabberResult::OBJECT_GRABBER_BUSY;
        break;
      case coordinator::ManipTaskGoal::WAIT_FOR_MOVE:
        if (object_grabber_return_code_ == object_grabber::object_grabberResult::SUCCESS)
        {
          ROS_INFO("completed move");
          working_on_task_ = false;
          action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
          status_code_ = coordinator::ManipTaskFeedback::COMPLETED_MOVE;
          result_.manip_return_code = coordinator::ManipTaskResult::MANIP_SUCCESS;
          as_.setSucceeded(result_);
          return;
        }
        else if (object_grabber_return_code_ == object_grabber::object_grabberResult::FAILED_CANNOT_REACH)
        {
          ROS_WARN("unreachable");
          working_on_task_ = false;
          action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
          status_code_ = coordinator::ManipTaskFeedback::COMPLETED_MOVE;
          result_.manip_return_code = coordinator::ManipTaskResult::FAILED_MOVE;
          as_.setSucceeded(result_);
          return;
        }
        else
        {
          ROS_INFO("object_grabber_return_code_ = %d", object_grabber_return_code_);
          ros::Duration(0.5).sleep();
        }

        break;

      case coordinator::ManipTaskGoal::ABORT:
        ROS_WARN("aborting goal...");

        action_code_ = coordinator::ManipTaskGoal::NO_CURRENT_TASK;
        status_code_ = coordinator::ManipTaskFeedback::ABORTED;
        working_on_task_ = false;
        as_.setAborted(result_);
        return;

      default:
        ROS_WARN("executeCB: error--case not recognized");
        working_on_task_ = false;
        break;
    }
  }
  ROS_INFO("executeCB: I should not be here...");

  result_.manip_return_code = coordinator::ManipTaskResult::ABORTED;
  as_.setAborted(result_);
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "coordinator");
  ros::NodeHandle nh;

  TaskActionServer taskActionServer;

  ROS_INFO("main going into loop");
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return 0;
}
