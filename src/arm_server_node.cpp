
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_server/SimplePickAction.h>
#include <arm_server/SimplePlaceAction.h>
#include <arm_server/SimpleTargetAction.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define TABLE_NAME "table"

// server wrapper for pick & place goals
typedef actionlib::SimpleActionServer<arm_server::SimplePickAction> pick_server_t;
typedef actionlib::SimpleActionServer<arm_server::SimplePlaceAction> place_server_t;
typedef actionlib::SimpleActionServer<arm_server::SimpleTargetAction> target_server_t;

// moveit client
typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> pick_client_t;
typedef actionlib::SimpleActionClient<moveit_msgs::PlaceAction> place_client_t;



moveit::planning_interface::PlanningSceneInterface *planning_scene_ptr = nullptr;

moveit::planning_interface::MoveGroupInterface *group_ptr = nullptr;

tf::TransformListener *transformer = nullptr;

// planning scene cylinder object name
std::string obj_name = "";

// distance from gripper link to object
double distance = 0.0;

// is execution is in process
bool executing_pick = false;
bool executing_place = false;
bool executing_target = false;


// adding cylinder to planning scene
void addCylinderToScene(std::string name,
                        double x,
                        double y,
                        double z,
                        double height,
                        double width)
{
    std::vector<moveit_msgs::CollisionObject> col_objects;
    moveit_msgs::CollisionObject target_obj;
    target_obj.id = name;

    // define object primitives (size and shape)
    shape_msgs::SolidPrimitive cylinder_primitives;
    cylinder_primitives.type = cylinder_primitives.CYLINDER;
    cylinder_primitives.dimensions.resize(2);
    cylinder_primitives.dimensions[0] = height;
    cylinder_primitives.dimensions[1] = width;

    target_obj.primitives.push_back(cylinder_primitives);

    // define object position and orientation
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

    target_obj.primitive_poses.push_back(target_pose);

    target_obj.operation = target_obj.ADD;
    target_obj.header.frame_id = "/base_footprint";

    col_objects.push_back(target_obj);
    planning_scene_ptr->addCollisionObjects(col_objects);
}

// adding cylinder to planning scene
void addBoxToScene(std::string name,
                        double x,
                        double y,
                        double z,
                        double height,
                        double width,
                        double depth)
{
    std::vector<moveit_msgs::CollisionObject> col_objects;
    moveit_msgs::CollisionObject target_obj;
    target_obj.id = name;

    // define box primitives (size and shape)
    shape_msgs::SolidPrimitive box_primitives;
    box_primitives.type = box_primitives.BOX;
    box_primitives.dimensions.resize(3);
    box_primitives.dimensions[0] = height;
    box_primitives.dimensions[1] = width;
    box_primitives.dimensions[2] = depth;

    target_obj.primitives.push_back(box_primitives);

    // define object position and orientation
    geometry_msgs::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);

    target_obj.primitive_poses.push_back(target_pose);

    target_obj.operation = target_obj.ADD;
    target_obj.header.frame_id = "/base_footprint";

    col_objects.push_back(target_obj);
    planning_scene_ptr->addCollisionObjects(col_objects);
}

moveit_msgs::PickupGoal buildPickGoal(const std::string& obj_name)
{
    moveit_msgs::PickupGoal pu_goal;
    pu_goal.target_name = obj_name;
    pu_goal.group_name = "arm";
    pu_goal.support_surface_name = TABLE_NAME;
    pu_goal.end_effector = "eef";
    pu_goal.allowed_planning_time = 15.0;
    pu_goal.planner_id = "RRTConnectkConfigDefault";
    pu_goal.minimize_object_distance = true;

    pu_goal.planning_options.replan_delay = 2.0;
    pu_goal.planning_options.planning_scene_diff.is_diff = true;
    pu_goal.planning_options.planning_scene_diff.robot_state.is_diff = true;
    pu_goal.planning_options.replan=true;
    pu_goal.planning_options.replan_attempts=5;

    moveit_msgs::Grasp g;
    g.max_contact_force = 1.0;

    g.pre_grasp_approach.direction.header.frame_id = "/base_footprint";
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.min_distance = 0.01;
    g.pre_grasp_approach.desired_distance = 0.2;

    g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(g.pre_grasp_posture.joint_names.size());
    g.pre_grasp_posture.points[0].positions[0] = 0.14;

    g.grasp_pose.header.frame_id = pu_goal.target_name;
    g.grasp_pose.pose.position.x = -0.02;
    g.grasp_pose.pose.position.y = 0.0;
    g.grasp_pose.pose.position.z = 0.0;
    g.grasp_pose.pose.orientation.x = 0.0;
    g.grasp_pose.pose.orientation.y = 0.0;
    g.grasp_pose.pose.orientation.z = 0.0;
    g.grasp_pose.pose.orientation.w = 1.0;

    g.grasp_posture.joint_names = g.pre_grasp_posture.joint_names;
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].positions[0] = 0.01;
    g.grasp_posture.points[0].effort.resize(g.grasp_posture.joint_names.size());
    g.grasp_posture.points[0].effort[0] = 0.6;

    g.post_grasp_retreat.direction.header.frame_id = "/base_footprint";
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;

    pu_goal.possible_grasps.push_back(g);

    return pu_goal;
}

moveit_msgs::PlaceGoal buildPlaceGoal(double x,
                                      double y,
                                      double z,
                                      const std::string& obj_name)
{

    moveit_msgs::PlaceGoal place_goal;
    place_goal.group_name = "arm";
    place_goal.attached_object_name = obj_name;
    place_goal.place_eef = false;
    place_goal.support_surface_name = TABLE_NAME;
    place_goal.planner_id = "RRTConnectkConfigDefault";
    place_goal.allowed_planning_time = 15.0; //////////////////////////////////////////TODO: TEST LOWER TIMES
    place_goal.planning_options.replan = true;
    place_goal.planning_options.replan_attempts = 5;
    place_goal.planning_options.replan_delay = 2.0; //////////////////////////////////////////TODO: TEST LOWER TIMES
    place_goal.planning_options.planning_scene_diff.is_diff = true;
    place_goal.planning_options.planning_scene_diff.robot_state.is_diff = true;

    std::vector<moveit_msgs::PlaceLocation> locations;
    moveit_msgs::PlaceLocation location;
    location.pre_place_approach.direction.header.frame_id = "/base_footprint";
    location.pre_place_approach.direction.vector.z = -1.0;
    location.pre_place_approach.min_distance = 0.1;
    location.pre_place_approach.desired_distance = 0.2;

    location.post_place_retreat.direction.header.frame_id = "/gripper_link";
    location.post_place_retreat.direction.vector.x = -1.0;
    location.post_place_retreat.min_distance = 0.0;
    location.post_place_retreat.desired_distance = 0.2;

    location.place_pose.header.frame_id = "/base_footprint";
    location.place_pose.pose.position.x = x;
    location.place_pose.pose.position.y = y;
    location.place_pose.pose.position.z = z;
    location.place_pose.pose.orientation.w = 1.0;

    locations.push_back(location);
    place_goal.place_locations = locations;

    return place_goal;

}

/**************************TARGET SERVER CALLBACKS****************************/
void executeTargetCB(const arm_server::SimpleTargetGoalConstPtr& goal, target_server_t* as)
{
    if (executing_pick || executing_place)
    {
        arm_server::SimpleTargetResult result;
        as->setAborted(result, "tried to execute target, while some other goal");
        return;
    }

    executing_target = true;


    // get original goal point
    geometry_msgs::PointStamped origin_goal;
    origin_goal.header.frame_id = goal->frame_id;
    origin_goal.point.x = goal->x;
    origin_goal.point.y = goal->y;
    origin_goal.point.z = goal->z;

    if(goal->frame_id[0] != '/'){
      ROS_INFO("[target_server]: moving to pre-defined pose");
      moveit::planning_interface::MoveGroupInterface group("arm");

      group.setPlanningTime(10.0);
      group.setNumPlanningAttempts(50);
      group.setPlannerId("RRTConnectkConfigDefault");
      group.setPoseReferenceFrame("base_footprint");
      group.setStartStateToCurrentState();
      group.setNamedTarget(goal->frame_id);
      moveit::planning_interface::MoveGroupInterface::Plan startPosPlan;
      if(group.plan(startPosPlan)) { //Check if plan is valid
          group.execute(startPosPlan);
      }
      // else{
      //   ROS_WARN("failed moving to %s pose",goal->frame_id);
      // }
    executing_target = false;
    return;
  }

    // transfer original goal to in relation to base footprint
    geometry_msgs::PointStamped transformed_goal;
    try
    {
        transformer->transformPoint("/base_footprint", origin_goal, transformed_goal);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "/base_footprint";
    goal_pose.pose.position.x = transformed_goal.point.x;
    goal_pose.pose.position.y = transformed_goal.point.y;
    goal_pose.pose.position.z = transformed_goal.point.z;
    goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    group_ptr->setPoseTarget(goal_pose);
    moveit::planning_interface::MoveItErrorCode err = group_ptr->plan(plan);
    if ( err == moveit::planning_interface::MoveItErrorCode::SUCCESS )
    {
        ROS_INFO("[target_server]: plan found, executing...");

        // Execute the plan
        ros::Time start = ros::Time::now();
        group_ptr->move();
    }
    else
        ROS_WARN("[target_server]: plan failed");

    executing_target = false;
}
/***************************************************************************/


/**************************PICK SERVER CALLBACKS****************************/
void executePickCB(const arm_server::SimplePickGoalConstPtr& goal, pick_server_t* as)
{
    if (executing_place || executing_target)
    {
        arm_server::SimplePickResult result;
        as->setAborted(result, "tried to execute target, while some other goal");
        return;
    }

    executing_pick = true;

    distance = 0.0;

    pick_client_t pick_client("pickup", true);
    ROS_INFO("[arm_server]: waiting for Moveit pickup server");
    pick_client.waitForServer();
    ROS_INFO("[arm_server]: got Moveit pickup server");

    obj_name = goal->obj_name;

    // get original goal point
    geometry_msgs::PointStamped origin_goal;
    origin_goal.header.frame_id = goal->frame_id;
    origin_goal.point.x = goal->x;
    origin_goal.point.y = goal->y;
    origin_goal.point.z = goal->z;

    // transfer original goal to in relation to base footprint
    geometry_msgs::PointStamped transformed_goal;
    try
    {
        transformer->transformPoint("/base_footprint", origin_goal, transformed_goal);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    // visualize object in planning scene
    addCylinderToScene(goal->obj_name,
                       transformed_goal.point.x,
                       transformed_goal.point.y,
                       transformed_goal.point.z,
                       goal->h,
                       goal->w);

    double table_z = transformed_goal.point.z - (goal->h / 2.0)/* - 0.01*/;

    addBoxToScene(TABLE_NAME,
                  transformed_goal.point.x,
                  transformed_goal.point.y,
                  table_z,
                  0.3,
                  0.3,
                  0.01);

    // build and execute pick
    moveit_msgs::PickupGoal pick_goal = buildPickGoal(goal->obj_name);
    actionlib::SimpleClientGoalState pick_status = pick_client.sendGoalAndWait(pick_goal);

    // send result and last gripper_link position to client
    geometry_msgs::PoseStamped eef_pose = group_ptr->getCurrentPose("gripper_link");
    arm_server::SimplePickResult result;
    result.x = eef_pose.pose.position.x;
    result.y = eef_pose.pose.position.y;
    result.z = eef_pose.pose.position.z;

    if(pick_status == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("[arm_server]: goal execution succeeded");
        as->setSucceeded(result, pick_status.getText());
    }
    else
    {
        ROS_WARN("[arm_server]: goal execution failed");
        as->setAborted(result, pick_status.getText());
    }

    executing_pick = false;

}
/***************************************************************************/

/**************************PLACE SERVER CALLBACKS***************************/
void executePlaceCB(const arm_server::SimplePlaceGoalConstPtr& goal, place_server_t* as)
{
    if (executing_pick || executing_target)
    {
        arm_server::SimplePlaceResult result;
        as->setAborted(result, "tried to execute target, while some other goal");
        return;
    }

    executing_place = true;

    distance = 0.0;

    place_client_t place_client("place", true);
    ROS_INFO("[arm_server]: waiting for Moveit place server");
    place_client.waitForServer();
    ROS_INFO("[arm_server]: got Moveit place server");

    obj_name = goal->obj_name;

    // get original goal point
    geometry_msgs::PointStamped origin_goal;
    origin_goal.header.frame_id = goal->frame_id;
    origin_goal.point.x = goal->x;
    origin_goal.point.y = goal->y;
    origin_goal.point.z = goal->z;

    // transfer original goal to in relation to base footprint
    geometry_msgs::PointStamped transformed_goal;
    try
    {
        transformer->transformPoint("/base_footprint", origin_goal, transformed_goal);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    // build and execute pick
    moveit_msgs::PlaceGoal place_goal = buildPlaceGoal(transformed_goal.point.x,
                                                        transformed_goal.point.y,
                                                        transformed_goal.point.z,
                                                        goal->obj_name);
    actionlib::SimpleClientGoalState place_status = place_client.sendGoalAndWait(place_goal);

    // send result and last gripper_link position to client
    geometry_msgs::PoseStamped eef_pose = group_ptr->getCurrentPose("gripper_link");
    arm_server::SimplePlaceResult result;
    result.x = eef_pose.pose.position.x;
    result.y = eef_pose.pose.position.y;
    result.z = eef_pose.pose.position.z;

    if(place_status == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("[arm_server]: goal execution succeeded");
        as->setSucceeded(result, place_status.getText());
    }
    else
    {
        ROS_WARN("[arm_server]: goal execution failed");
        as->setAborted(result, place_status.getText());
    }

    executing_place = false;


}
/***************************************************************************/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_server_node");
    ros::NodeHandle nh;

    // use async spinner when working with moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // we need move group to get gripper current point, and for target server
    moveit::planning_interface::MoveGroupInterface group("arm");
    group_ptr = &group;
    group.setStartStateToCurrentState();
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPoseReferenceFrame("/base_footprint");
    group.setMaxVelocityScalingFactor(1.0);
    group.setNumPlanningAttempts(5);
    group.setPlanningTime(5.0);

    //It is important to provide some tolerance
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(0.02);
    //group_arm_torso.setGoalJointTolerance(0.01);
    //group_arm_torso.setGoalTolerance(0.05);

    pick_server_t pick_server(nh, "simple_pick", boost::bind(&executePickCB, _1, &pick_server), false);
    pick_server.start();

    place_server_t place_server(nh, "simple_place", boost::bind(&executePlaceCB, _1, &place_server), false);
    place_server.start();

    target_server_t target_server(nh, "simple_target", boost::bind(&executeTargetCB, _1, &target_server), false);
    target_server.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_ptr = &planning_scene_interface;

    tf::TransformListener tf_trans;
    transformer = &tf_trans;

    ROS_INFO("[arm_server]: ready");

    while (ros::ok())
    {
        // get current gripper position
        geometry_msgs::PoseStamped eef_pose = group.getCurrentPose("gripper_link");

        // get current object position
        std::vector<std::string> object_ids;
        object_ids.push_back(obj_name);
        std::map<std::string, geometry_msgs::Pose> objects_map = planning_scene_interface.getObjectPoses(object_ids);

        // if during pick excution, send feedback of distance
        // between gripper and object to client
        if (objects_map.size() > 0)
        {
            geometry_msgs::Pose obj_pose = objects_map[obj_name];
            double delta_x = fabs(obj_pose.position.x - eef_pose.pose.position.x);
            double delta_y = fabs(obj_pose.position.x - eef_pose.pose.position.x);
            double delta_z = fabs(obj_pose.position.x - eef_pose.pose.position.x);
            distance = sqrt( pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2) );
            //ROS_INFO("distance: %f", distance);


            if (executing_place)
            {
                arm_server::SimplePlaceFeedback feedback;
                feedback.distance = distance;
                feedback.x = eef_pose.pose.position.x;
                feedback.y = eef_pose.pose.position.y;
                feedback.z = eef_pose.pose.position.z;
                place_server.publishFeedback(feedback);
            }
            else if (executing_pick)
            {
                arm_server::SimplePickFeedback feedback;
                feedback.distance = distance;
                feedback.x = eef_pose.pose.position.x;
                feedback.y = eef_pose.pose.position.y;
                feedback.z = eef_pose.pose.position.z;
                pick_server.publishFeedback(feedback);
            }
            else if (executing_target)
            {
                arm_server::SimpleTargetFeedback feedback;
                feedback.distance = distance;
                feedback.x = eef_pose.pose.position.x;
                feedback.y = eef_pose.pose.position.y;
                feedback.z = eef_pose.pose.position.z;
                target_server.publishFeedback(feedback);
            }
        }


        ros::Rate(10).sleep();
        ros::spinOnce();
    }

    return 0;
}
