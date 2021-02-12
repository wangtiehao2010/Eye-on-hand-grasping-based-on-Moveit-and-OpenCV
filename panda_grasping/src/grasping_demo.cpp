#include "panda_grasping/grasping_demo.h"

GraspingDemo::GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length,
                           float breadth) :
        it_(n_),
        armgroup("panda_arm"),
        grippergroup("hand"),
        vMng_(length, breadth) {
    this->nh_ = n_;

    grasp_running = false;
    cameraStandby = false;

    this->pregrasp_x = pregrasp_x;
    this->pregrasp_y = pregrasp_y;
    this->pregrasp_z = pregrasp_z;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::WallDuration(5.0).sleep();
    ROS_INFO_STREAM("Getting into the Grasping Position....");
    attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);
    ros::WallDuration(5.0).sleep();
    cameraStandby = true;

    updateCameraCoordinate();

    // Subscribe to input video feed and publish object location
    image_sub_ = it_.subscribe("/panda/camera/image_raw", 1, &GraspingDemo::imageCb, this);
}

void GraspingDemo::imageCb(const sensor_msgs::ImageConstPtr &msg) {
    if (!grasp_running && cameraStandby) {
        ROS_INFO_STREAM("Processing the Image to locate the Object...");
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // ROS_INFO("Image Message Received");
        float obj_x, obj_y;
        vMng_.get2DLocation(cv_ptr->image, obj_x, obj_y);

        // Temporary Debugging
        std::cout << " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
        std::cout << " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

//    obj_camera_frame.setZ(-obj_y);
//    obj_camera_frame.setY(-obj_x);
        obj_camera_frame.setZ(-obj_y);
        obj_camera_frame.setY(-obj_x);
        obj_camera_frame.setX(0.319314);

        updateCameraCoordinate();

        obj_robot_frame = camera_to_robot_ * obj_camera_frame;
        grasp_running = true;

        // Temporary Debugging
        std::cout << " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
        std::cout << " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
        std::cout << " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
    }
}

void GraspingDemo::attainPosition(float x, float y, float z) {
    // ROS_INFO("The attain position function called");

    // For getting the pose
    geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = currPose.pose.orientation;

    // Starting Postion before picking
    target_pose1.position.x = x;
    target_pose1.position.y = y;
    target_pose1.position.z = z;
    target_pose1.orientation.x = 0; //x = Kx * sin(theta/2), 当绕x轴旋转，Kx取1
    target_pose1.orientation.y = 1; //y = Ky * sin(theta/2), 当绕y轴旋转，Ky取1
    target_pose1.orientation.z = 0; //z = Kz * sin(theta/2), 当绕z轴旋转，Kz取1
    target_pose1.orientation.w = 0; //w = cos(theta/2)
    armgroup.setPoseTarget(target_pose1);

    armgroup.move();
//  exit(0);
}

void GraspingDemo::attainObject() {
    // ROS_INFO("The attain Object function called");
//  attainPosition(obj_robot_frame.getX(), obj_robot_frame.getY(), obj_robot_frame.getZ()+0.04);
    double slideDownDistance = 0.13;
    double gripperLength = 0;
    attainPosition(obj_robot_frame.getX(), obj_robot_frame.getY(),
                   obj_robot_frame.getZ() + slideDownDistance + gripperLength);
    std::cout << "Camera to robot" << camera_to_robot_.getOrigin().getX() << "        "
              << camera_to_robot_.getOrigin().getY() << "        " << camera_to_robot_.getOrigin().getZ() << std::endl;
    std::cout << "object to robot" << obj_robot_frame.getX() << "        " << obj_robot_frame.getY() << "        "
              << obj_robot_frame.getZ() << std::endl;


    // Open Gripper
    ros::WallDuration(1.0).sleep();
    grippergroup.setNamedTarget("open");
    grippergroup.move();

    // Slide down the Object
    geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
    geometry_msgs::Pose target_pose1;

    target_pose1.orientation = currPose.pose.orientation;
    target_pose1.position = currPose.pose.position;

//  target_pose1.position.z = obj_robot_frame.getZ() - 0.02;
    target_pose1.position.z -= slideDownDistance;
    armgroup.setPoseTarget(target_pose1);
    armgroup.move();
//    exit(0);
}

void GraspingDemo::grasp() {
    // ROS_INFO("The Grasping function called");

    ros::WallDuration(1.0).sleep();
    grippergroup.setNamedTarget("close");
    grippergroup.move();
}

void GraspingDemo::lift() {
    double slideDownDistance = 0.13;
    // ROS_INFO("The lift function called");

    // For getting the pose
    geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = currPose.pose.orientation;
    target_pose1.position = currPose.pose.position;

    // Starting Postion after picking
    //target_pose1.position.z = target_pose1.position.z + 0.06;

    if (rand() % 2) {
//      target_pose1.position.z = target_pose1.position.z + 0.06;
        target_pose1.position.y = target_pose1.position.y + 0.02;
    } else {
//      target_pose1.position.z = target_pose1.position.z + 0.06;
//      target_pose1.orientation.x = 0; //x = Kx * sin(theta/2), 当绕x轴旋转，Kx取1
//      target_pose1.orientation.y = 0.707; //y = Ky * sin(theta/2), 当绕y轴旋转，Ky取1
//      target_pose1.orientation.z = 0.707; //z = Kz * sin(theta/2), 当绕z轴旋转，Kz取1
//      target_pose1.orientation.w = 0; //w = cos(theta/2)
        target_pose1.position.y = target_pose1.position.y - 0.02;
    }

    armgroup.setPoseTarget(target_pose1);
    armgroup.move();

    // Open Gripper
    ros::WallDuration(1.0).sleep();
    grippergroup.setNamedTarget("open");
    grippergroup.move();

    target_pose1.position.z = target_pose1.position.z + slideDownDistance;
    armgroup.setPoseTarget(target_pose1);
    armgroup.move();
}

void GraspingDemo::goHome() {
    geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

    // Go to Home Position
    attainPosition(homePose.pose.position.x, homePose.pose.position.y, homePose.pose.position.z);
}

void GraspingDemo::initiateGrasping() {
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::WallDuration(3.0).sleep();

    homePose = armgroup.getCurrentPose();

    ROS_INFO_STREAM("Approaching the Object....");
    attainObject();
//  exit(0);

    ROS_INFO_STREAM("Attempting to Grasp the Object now..");
    grasp();

    ROS_INFO_STREAM("Lifting the Object....");
    lift();

    ROS_INFO_STREAM("Going back to home position....");
    goHome();

    cameraStandby = true;
    grasp_running = false;
}

void GraspingDemo::updateCameraCoordinate()
{
    try {
        this->tf_camera_to_robot.waitForTransform("/panda_link0", "/camera_link", ros::Time(0), ros::Duration(50.0));
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
        ros::Duration(1.0).sleep();
    }

    try {
        this->tf_camera_to_robot.lookupTransform("/panda_link0", "/camera_link", ros::Time(0),
                                                 (this->camera_to_robot_));
    }

    catch (tf::TransformException &ex) {
        ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_grasping");
    float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
    ros::NodeHandle n;

    if (!n.getParam("graspingDemo_panda/table_length", length))
        length = 0.3;
    if (!n.getParam("graspingDemo_panda/table_breadth", breadth))
        breadth = 0.3;
    if (!n.getParam("graspingDemo_panda/pregrasp_x", pregrasp_x))
        pregrasp_x = 0;
    if (!n.getParam("graspingDemo_panda/pregrasp_y", pregrasp_y))
        pregrasp_y = 0.4;
    if (!n.getParam("graspingDemo_panda/pregrasp_z", pregrasp_z))
        pregrasp_z = 0.35;

    GraspingDemo simGrasp(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth);
    ROS_INFO_STREAM("Waiting for five seconds..");

    ros::WallDuration(3.0).sleep();
    while (ros::ok()) {
        // Process image callback
        ros::spinOnce();

        simGrasp.initiateGrasping();
    }
    return 0;
}
