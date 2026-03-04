/*

This is the final demo task that has to be performed....


*/


#include <calib_util.hpp>
#include <RealSense.hpp>

#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>


#include <atls_msgs_srvs/GetImage.h>
#include <atls_msgs_srvs/GetSubtask.h>
#include <atls_msgs_srvs/UI.h>
#include <atls_msgs_srvs/GetPose.h>
#include <atls_msgs_srvs/UR5Goal.h>
#include <atls_msgs_srvs/ExecuteURJointPos.h>
#include <atls_msgs_srvs/UIUser.h>

#include <samlibs.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>

#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_dual_quaternion.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/pcl_config.h>
#include <trac_ik_solver/FKIK2.h>
#include <boost/lexical_cast.hpp>
#include <unistd.h>
#include <signal.h>


#include <../../atls_motion_planner/include/atls_motion_planner/RG2Controll.hpp>
#include <../../atls_motion_planner/include/atls_motion_planner/ur_motion_planning_utils.hpp>



#define NUM_JOINT 6

#define nl '\n'


bool GRIPPERCONTROL=true;
using namespace std;
using namespace cv;



Sam::Point cornerA(0.0643354,   0.530454, -0.068);
Sam::Point cornerB(0.475345,  -0.538934, -0.068);





// Utils here ------------------------------------------------------
// calculates norm of a vector.
double norm(Eigen::Vector3d v){
    return sqrt(v(0)*v(0)+v(1)*v(1)+v(2)*v(2));
}

// does cross product of two vectors
Eigen::Vector3d crossProduct(Eigen::Vector3d a, Eigen::Vector3d b) {
    Eigen::Vector3d result;
    result(0) = a(1) * b(2) - a(2) * b(1);
    result(1) = a(2) * b(0) - a(0) * b(2);
    result(2) = a(0) * b(1) - a(1) * b(0);
    // std::cout << "cross product result is :" << result << "\n";
    return result;
}

// takes projection of a vector v on u
Eigen::Vector3d projection(Eigen::Vector3d v, Eigen::Vector3d u){
    double vu = v.dot(u);
    double vv = v.dot(v);
    cout << vv << " " << vu << "\n";

    return (vu/vv)*v;
}



// this calculates rotational matrix from clicked points by the groundingDino from rcnl PC. the approach vector is fixed.
void create_Rvert_to_robot_frame(Eigen::Matrix4d& T, Eigen::Vector4d& tmp_c, Eigen::Vector4d& tmp_s, Eigen::Matrix3d& RotM, Eigen::Vector3d& t_eef){
    tmp_c = T*tmp_c;
    tmp_s = T*tmp_s;
    // now in robot frame
    Eigen::Vector3d  t_s, s, a, target_eef, n;

    t_eef << tmp_c(0), tmp_c(1), tmp_c(2);
    t_s << tmp_s(0), tmp_s(1), tmp_s(2);

    s = t_s-t_eef;

    double s_norm = norm(s);

    s = s/s_norm;
    a << -0.0272563,0.00571188,-0.999612;

    Eigen::Vector3d proj = projection(a,s);

    s = (s-proj);
    s = s/norm(s);
    n = crossProduct(s,a);
    double n_norm = norm(n);
    n = n/n_norm;

    RotM.col(0)=n;
    RotM.col(1)=s;
    RotM.col(2)=a;
    return;
}



void create_pick_stops(Eigen::Vector3d& stop_a, Eigen::Vector3d& stop_b, Eigen::Vector3d& t_eef, double gripper_len, double object_offset, double threshold, Eigen::Matrix3d RotM, double table_cheat){
    Eigen::Vector3d eef_thresh;
    eef_thresh << 0,0,-(gripper_len+object_offset);

    stop_a = RotM* eef_thresh + t_eef;

    eef_thresh << 0,0,-(gripper_len+threshold);

    stop_b = RotM*eef_thresh + t_eef;
    stop_b(2) = table_cheat;
}

void create_place_stops(Eigen::Vector3d& stop_a, Eigen::Vector3d& stop_b, Eigen::Vector4d& centroid, Eigen::Matrix4d T, double gripper_len){
    centroid = T*centroid;
    stop_a << centroid(0), centroid(1), centroid(2)+ 0.25 + gripper_len;
    stop_b << centroid(0), centroid (1), centroid(2) + 0.05+gripper_len;
}



// Utils end here----------------------------------------------




// init_jt is for initial joint angles and 
// inter_jt is for intermediate joint angles between tasks... This was studied and hardcoded....

std::vector<double> init_jt ={1.46523, -1.37435, 1.97646, -1.92476, -1.55294, 0};
// std::vector<double> inter_jt ={0.43446, -1.51437, 1.85685, -1.77112, -1.67054, -1.22327};


void print_subtask(string object1, string object2){
    ROS_INFO("Generated subtasks of your task instruction from VLM.");
    std::cout << "1. Pick up the " << object1 << " from the table.\n";
    std::cout << "2. Move the " << object1 << " near the " << object2 << ".\n";
    std::cout << "3. Place the " << object1 << " on the " << object2 << ".\n";
    return;
}



// robot functions------------------------------------------------------------------------------------------

int executeJointPos_Python(ros::ServiceClient& client_ur_mPlanner, atls_msgs_srvs::ExecuteURJointPos& target_jt_req,
                     const std::vector<double>& pos, FKIK2& fkik, double acceleration, double velocity, double gripper_len ){

	Eigen::Matrix3d R; Eigen::Vector3d p;
    Eigen::Vector3d eef_vector;
    eef_vector << 0,0,gripper_len;
	p = fkik.getFK_offline(pos, R, "tool0", eef_vector);
	// std::cout<<"Target EEF Position: "<< p.transpose()<<"\n\n";

// 	int d1= Sam::checkLOC(cornerA, cornerB, Sam::Point(p(0),p(1), p(2)));


// 	if(d1<=0  ){
// //		std::cout<<"Target location is in collition space, distance is "<< d1 << "  "<<d2 << "  "<<d3 << "  "<<d4 << "  "<<
// //				" ... check again!!\n";
// 		std::cout<<"Target location is in collition space, distance is "<< d1 <<  		" ... check again!!\n";
// 		// return -1;
// 	}

	target_jt_req.request.target_jt.data.clear();

	// target_jt_req.request.movement_type.data="movej";
	target_jt_req.request.vel_in_deg.data = velocity;
	target_jt_req.request.acc_in_deg.data = acceleration;

    // std::cout<<"In execute .... joint position received: \n";
    // Sam::display<double>(pos,"JOint pos in rad");
        Sam::vec_type j_deg(6);
        for(size_t i=0; i<6; i++){
        	j_deg[i]=Sam::radToDeg<double>(pos[i]);
        }

        // Sam::display<double>(j_deg, "Joint pos in Deg:");

    for(int j=0; j<NUM_JOINT; j++)
    	target_jt_req.request.target_jt.data.push_back(pos[j]);

    // std::string ans=Sam::waitForEnter("Sending Robot!!");
    // if(!ans.compare("n"))
    // 	return -1;
//    cv::destroyAllWindows();

    if(client_ur_mPlanner.call(target_jt_req))
    {
        // std::cout << "Reached commended position" << std::endl;
        sleep(2);
        return 1;

    }
    else
    {
        std::cout << "Unable to reach commanded position" << std::endl;
        // success = false;
        return -1;
    }
}

void showPoints(cv::Point2i p_clik_cen, cv::Point2i p_clik_edge, cv::Mat color_image){
    cv::circle(color_image, p_clik_cen, 3, cv::Scalar(255,255,255),cv::FILLED, 8,0);
    cv::circle(color_image, p_clik_edge, 3, cv::Scalar(255,255,255),cv::FILLED, 8,0);
    cv::imshow("Point Display", color_image);
    cv::waitKey(0);
    //cv::destroyAllWindows();
}


/*

This robot demo server is integrated with the robot ui

*/




class RobotDemoServer{

private:
  // declare the variables here
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  ros::ServiceClient get_pose_client_;
  ros::ServiceClient get_subtask_client_;
  ros::ServiceClient client_ur_mPlanner;
  ros::ServiceServer confirmation_service_;

  double checker;

  Eigen::Vector3d eef_thresh, t_eef;
  Eigen::Matrix3d R_vert;
  
  Eigen::Vector3d stop_1, stop_2, stop_3, stop_4;
  double threshold, acceleration, velocity, gripper_len, object_offset, table_cheat;
  atls_msgs_srvs::ExecuteURJointPos target_jt_req;
  RG2Controll rg2;
  std::string gr_command;

  std::string chain_start;
  std::string chain_end;
  std::string urdf_param;
  std::string j_limits;

  std::string path;
  // path is path to workspace of ros1.
  
  bool calculated;
  

public:
RobotDemoServer(ros::NodeHandle& nh) : nh_(nh) {

    path = "/home/rcnl/Documents/ROS_Workspace/graspdetect_ws";
    // constructor gives values to the declared variables.
    chain_start = "base_link";
    chain_end = "tool0";
    urdf_param = "/robot_description";
    j_limits = path + "/config/joint_limit.txt";
    
    checker = 32.2;
    calculated = false;

    service_ = nh_.advertiseService("robot_demo_server", &RobotDemoServer::robot_callback, this);
    confirmation_service_ = nh_.advertiseService("robot_execution_server", &RobotDemoServer::user_callback, this);
    get_pose_client_ = nh_.serviceClient<atls_msgs_srvs::GetPose>("get_pose");
    get_subtask_client_ = nh_.serviceClient<atls_msgs_srvs::GetSubtask>("subtask");
    client_ur_mPlanner =  nh_.serviceClient<atls_msgs_srvs::ExecuteURJointPos>("ur_motion_planner/execute_ur_joint_pos_service");

  }




bool robot_callback(atls_msgs_srvs::UI::Request& req, atls_msgs_srvs::UI::Response& res){
    // this callback function is for the robot algo pipeline to calculate all the stops and 
    // object frame images and send it to the user for confirmation to execute robot action.

    int scale=1;
    int dictionaryId = 16;

    
    signal(SIGINT, Sam::signal_callback_handler);


    std::string T_fname = path + "/config/transformation_cam_to_rob";
    std::string conf_file = path + "/config/rosTopics.txt";
    std::string R_file = path + "/config/rotation.txt";


    std::string io_fname = path + "/config/io_files.txt";
    std::string param_fname = path+ "/config/parameters.txt";

    std::map<std::string, std::string> io_files;
    Sam::readConfigFile(io_fname, io_files);
    std::string read_fname(io_files["read_file"]);
    std::string cam_rob_write_fname(io_files["write_file"]);
    std::string cam_rob_write_fname2(io_files["write_file_opt"]);


    std::map<std::string, std::string> param_files;
    Sam::readConfigFile(param_fname, param_files);
    std::string read_thold(param_files["threshold"]);    // (gripper_len_closed - gripper_len_open) = threshold
    std::string read_acc(param_files["acceleration"]);   // robot accn
    std::string read_vel(param_files["velocity"]);       // robot jt velocity
    std::string read_gripp_len(param_files["gripperlength"]);   // length of gripper
    std::string read_object_offset(param_files["object_offset"]);   // the z distance above the object the gripper will stop to pick up the object.
    std::string read_table_cheat(param_files["table_cheat"]); // for small table 0.050744 and large table 0.140744
    // table cheat for TCS table only.

    threshold = boost::lexical_cast<double>(read_thold);
    acceleration = boost::lexical_cast<double>(read_acc);
    velocity = boost::lexical_cast<double>(read_vel);
    gripper_len = boost::lexical_cast<double>(read_gripp_len);
    object_offset = boost::lexical_cast<double>(read_object_offset);
    table_cheat = boost::lexical_cast<double>(read_table_cheat);


    size_t num_smp = 20;


    Eigen::Matrix4d T;
    Sam::mat_type T_mat = Sam::readFile<double>(T_fname);
    T = Sam::makeEigenMat<double>(T_mat);




    std::map<std::string, std::string> topic_config;
    Sam::readConfigFile(conf_file, topic_config);
    std::string cam_info_tpoic(topic_config["cam_info"]), color_img_topic(topic_config["rgb_img"]),
                    depth_img_topic(topic_config["depth_img"]), depth_img_topic2("/camera/depth/image_rect_raw") ;


    // fkik class declared here.
    FKIK2 fkik(chain_start, chain_end, urdf_param, nh_,j_limits);





    // realsense object created to get rgb-d image
    Sam::RealSense rs(nh_,"get_image");



    

    // goto initial pose

    target_jt_req.request.movement_type.data = "movej";

    int reached=executeJointPos_Python(client_ur_mPlanner, target_jt_req, init_jt, fkik, acceleration, velocity, gripper_len);

    ROS_INFO("Moved the robot to initial position to avoid camera view obstruction.");

    

    // return true;

    std::string task_message, first_object, second_object;
    std::string subtask_list;

    // std::cout << "\n";
    // std::cout << "\n";
    // std::cout << "Please enter a task you want to perform : ";

    task_message = req.task.data;
 
    // std::getline(std::cin, task_message);
    // std::cout << "\n";
    // std::cout << "\n";
    ROS_INFO("Your task instruction has been accepted.");
    cout << "Your task is : " << task_message << nl;



    // ros::ServiceClient get_subtask_client = nh_.serviceClient<atls_msgs_srvs::GetSubtask>("/subtask");

    atls_msgs_srvs::GetSubtask subtask_srv;

    subtask_srv.request.task.data = task_message;

    


    ROS_INFO("Generating subtasks from your task instruction.");
    ros::service::waitForService("subtask");

    if(get_subtask_client_.call(subtask_srv)){
        if(!subtask_srv.response.subtasks.empty()){
            first_object = subtask_srv.response.subtasks[0];
            second_object = subtask_srv.response.subtasks[1];
            subtask_list = subtask_srv.response.subtask_response;
            // std::cout << "first obj: " << first_object << std::endl;
            // std::cout << "second obj : " << second_object << std::endl;
            // two objects. first object to pick up and second object to the first object into for example tray.
        }
    }
    
    else{
        ROS_ERROR("Falied to call subtask call");
        return false;
    }

    // if (ros::service::waitForService("/get_subtask", ros::Duration(5.0))) {
    //     if (get_subtask_client_.call(subtask_srv)) {
    //         if (!subtask_srv.response.subtasks.empty()) {
    //             first_object = subtask_srv.response.subtasks[0];
    //             second_object = subtask_srv.response.subtasks[1];
    //             subtask_list = subtask_srv.response.subtask_response;

    //             // std::cout << "first obj: " << first_object << std::endl;
    //             // std::cout << "second obj : " << second_object << std::endl;
    //         }
    //     } else {
    //         ROS_ERROR("Service call to /get_subtask failed.");
    //     }
    // } else {
    //     ROS_ERROR("Service /get_subtask not available after waiting for 5 seconds.");
    // }
    cout << nl;
    cout << nl;
    print_subtask(first_object,second_object);
    cout << nl;
    cout << nl;

    // ros::ServiceClient get_pose_client = nh_.serviceClient<atls_msgs_srvs::GetPose>("/get_pose");
    
    atls_msgs_srvs::GetPose getpose_srv;
    sensor_msgs::Image source_frame, target_frame;

    getpose_srv.request.source_object.data = first_object;
    getpose_srv.request.target_object.data = second_object;
    getpose_srv.request.task.data = task_message;


    cv::Point2i p_clik_cen, p_clik_edge;
    Eigen::Vector4d centroid2;
    ROS_INFO("Fetching robot pose to perform the task.");
    if(get_pose_client_.call(getpose_srv)){
        if(!getpose_srv.response.source_centroid.empty()){
            p_clik_cen.y = getpose_srv.response.source_centroid[0];
            p_clik_cen.x = getpose_srv.response.source_centroid[1];
            p_clik_edge.y = getpose_srv.response.grasp_point[0];
            p_clik_edge.x = getpose_srv.response.grasp_point[1];
            centroid2 << getpose_srv.response.target_centroid[0], getpose_srv.response.target_centroid[1], getpose_srv.response.target_centroid[2], 1;        
            source_frame = getpose_srv.response.source_frame;
            target_frame = getpose_srv.response.target_frame;
        }
    }
    else{
        ROS_ERROR("Falied to call get_pose service. Check if system JishnuPc server node is running.");
        return false;
    }



    
    Sam::RealSense::RGBD_IMAGE image = rs.getRgbAndDepthImage();


    // showPoints(p_clik_cen,p_clik_edge, image[0]);    // to show image and the click points of the object in openCV

    cv::Point3d p1,p2;
    p1 = rs.compute3DPoint(p_clik_cen.x,p_clik_cen.y);
    p2 = rs.compute3DPoint(p_clik_edge.x,p_clik_edge.y);

    // std::cout << "3d point compute end\n";
    //tmp_c is the centroid point , tmp_s is the s vector....

    Eigen::Vector4d tmp_c, tmp_s;


    tmp_c << p1.x, p1.y, p1.z, 1;
    tmp_s << p2.x, p2.y, p2.z, 1;

    // std::cout << "Transformation matrix :" << T << std::endl;


    create_Rvert_to_robot_frame(T, tmp_c, tmp_s, R_vert, t_eef);

    // std::cout<<"Computed Rot:\n"<<R_vert<<"\n";

   create_pick_stops(stop_1, stop_2,  t_eef, gripper_len, object_offset, threshold, R_vert, table_cheat);

    // target_eef<<t_eef(0),t_eef(1), t_eef(2)+gripper_len;


   create_place_stops(stop_3, stop_4, centroid2, T, gripper_len);

////////////////////////////////////editing here/////////////////


    res.source_frame = source_frame;
    res.target_frame = target_frame;
    res.subtasks = subtask_list;
    res.success.data = true;

    calculated = true;
    return true;


}


bool user_callback(atls_msgs_srvs::UIUser::Request& req, atls_msgs_srvs::UIUser::Response& res){
    
    FKIK2 fkik(chain_start, chain_end, urdf_param, nh_,j_limits);
    Sam::vec_type target_jt(6);

    if(!calculated){
        res.success.data = false;
        return true;
    }
    calculated = false;
    std::cout << "doing robot tasks" << nl;

    // return true;
    std::vector<double> J=fkik.getCurrentJointAngle();
    std::vector<double> result = fkik.getIK(R_vert,stop_1,J);

    ///////////////////////////////////////////////////////////////////////////////////

    // Eigen::Vector3d ep=fkik.getEFPos();
    // std::cout<<"current rob_p: "<<ep.transpose()<<"\n";
    // Eigen::Vector3d pos_f; Eigen::Matrix3d R_f;
    // fkik.getFK_offline(res, R_f, pos_f);

    // std::cout<<"\n"<<pos_f.transpose()<<"\n";


    // string s1=Sam::waitForEnter("Move robot??(y/n):");
    for(size_t i=0; i<6; i++){
        target_jt[i]=result[i];
    }
    // cout << "wrist joint 3 prev" << target_jt[5] << endl;
    target_jt[5] = target_jt[5]+0.802;
    // cout << "wrist joint 3 later" << target_jt[5] << endl;



    
    // ros client to control the robot...

    target_jt_req.request.movement_type.data = "movej";

    // string s1=Sam::waitForEnter("Move robot??(y/n):");
    // if(s1=="y"){
    int reached=executeJointPos_Python(client_ur_mPlanner, target_jt_req, target_jt, fkik, acceleration, velocity, gripper_len);
                                    // std::cout<<"REACHED? : "<<reached<<"\n";
        if(reached == 1){
            ROS_INFO("Ready to pick up object.");
        }
    // }
    gr_command = "open";
    if(gr_command=="open"){
        // std::cout<<"Opening gripper!\n";
        rg2.openGripper();
    }



    J=fkik.getCurrentJointAngle();
    result = fkik.getIK(R_vert,stop_2,J);

    ///////////////////////////////////////////////////////////////////////////////////

    // ep=fkik.getEFPos();
    // std::cout<<"current rob_p: "<<ep.transpose()<<"\n";
    // fkik.getFK_offline(res, R_f, pos_f);

    // std::cout<<"\n"<<pos_f.transpose()<<"\n";


    // s1=Sam::waitForEnter("Move robot??(y/n):");
    for(size_t i=0; i<6; i++){
        target_jt[i]=result[i];
    }
    // cout << "wrist joint 3 prev" << target_jt[5] << endl;
    target_jt[5] = target_jt[5]+0.802;
    // cout << "wrist joint 3 later" << target_jt[5] << endl;

    target_jt_req.request.movement_type.data = "movel";

    // if(s1=="y"){
        reached=executeJointPos_Python(client_ur_mPlanner, target_jt_req, target_jt, fkik, acceleration, velocity, gripper_len);
        if(reached == 1){
            ROS_INFO("Picking up object.");
        }
    // }




    gr_command = "close";
    if(gr_command=="close"){
        // std::cout<<"Closing gripper!\n";
        rg2.closeGripper();
    }
    sleep(0.5);

    // s1=Sam::waitForEnter("Move robot??(y/n):");

// 0.391414;-1.2404;1.63489
// -0.202299;-1.5886;1.92075
    // target_jt[0]=0.391414;
    // target_jt[1]=-1.2404;
    // target_jt[2]=1.63489;

    J=fkik.getCurrentJointAngle();
    result = fkik.getIK(R_vert,stop_1,J);

    ///////////////////////////////////////////////////////////////////////////////////

    // Eigen::Vector3d ep=fkik.getEFPos();
    // std::cout<<"current rob_p: "<<ep.transpose()<<"\n";
    // Eigen::Vector3d pos_f; Eigen::Matrix3d R_f;
    // fkik.getFK_offline(res, R_f, pos_f);

    // std::cout<<"\n"<<pos_f.transpose()<<"\n";


    // string s1=Sam::waitForEnter("Move robot??(y/n):");
    for(size_t i=0; i<6; i++){
        target_jt[i]=result[i];
    }
    // cout << "wrist joint 3 prev" << target_jt[5] << endl;
    target_jt[5] = target_jt[5]+0.802;
    
    
    target_jt_req.request.movement_type.data = "movej";
    // if(s1=="y"){
        reached=executeJointPos_Python(client_ur_mPlanner, target_jt_req, target_jt, fkik, acceleration, velocity, gripper_len);
        if(reached == 1){
            ROS_INFO("Ready to place object.");
        }
    // }







    J=fkik.getCurrentJointAngle();
    result = fkik.getIK(R_vert,stop_3,J);



    for(size_t i=0; i<6; i++){
        target_jt[i]=result[i];
    }
    // cout << "wrist joint 3 prev" << target_jt[5] << endl;
    target_jt[5] = target_jt[5]+0.802;
    // cout << "wrist joint 3 later" << target_jt[5] << endl;

    // s1=Sam::waitForEnter("Move robot??(y/n):");
    target_jt_req.request.movement_type.data = "movej";
    reached=executeJointPos_Python(client_ur_mPlanner, target_jt_req, target_jt, fkik, acceleration, velocity, gripper_len);
    if(reached == 1){
        ROS_INFO("Over target object");
    }





    J=fkik.getCurrentJointAngle();
    result = fkik.getIK(R_vert,stop_4,J);



    for(size_t i=0; i<6; i++){
        target_jt[i]=result[i];
    }
    // cout << "wrist joint 3 prev" << target_jt[5] << endl;
    target_jt[5] = target_jt[5]+0.802;
    // cout << "wrist joint 3 later" << target_jt[5] << endl;

    // s1=Sam::waitForEnter("Move robot??(y/n):");
    target_jt_req.request.movement_type.data = "movel";
    reached=executeJointPos_Python(client_ur_mPlanner, target_jt_req, target_jt, fkik, acceleration, velocity, gripper_len);
    if(reached == 1){
        ROS_INFO("Placing object.");
    }



    gr_command = "open";
    if(gr_command=="open"){
        // std::cout<<"Opening gripper!\n";
        rg2.openGripper();
    }




    J=fkik.getCurrentJointAngle();
    result = fkik.getIK(R_vert,stop_3,J);



    for(size_t i=0; i<6; i++){
        target_jt[i]=result[i];
    }
    // cout << "wrist joint 3 prev" << target_jt[5] << endl;
    target_jt[5] = target_jt[5]+0.802;
    // cout << "wrist joint 3 later" << target_jt[5] << endl;

    // s1=Sam::waitForEnter("Move robot??(y/n):");
    target_jt_req.request.movement_type.data = "movel";
    reached=executeJointPos_Python(client_ur_mPlanner, target_jt_req, target_jt, fkik, acceleration, velocity, gripper_len);
    if(reached == 1){
        ROS_INFO("Over target object");
    }


    sleep(0.5);

    // s1=Sam::waitForEnter("Move robot??(y/n):");
    ROS_INFO("Your task has been executed returning to init");
    target_jt_req.request.movement_type.data = "movej";
    reached=executeJointPos_Python(client_ur_mPlanner, target_jt_req, init_jt, fkik, acceleration, velocity, gripper_len);
                            // std::cout<<"REACHED? : "<<reached<<"\n";
    if(reached == 1){
        ROS_INFO("Reached init joint");
    }

    gr_command = "close";
    if(gr_command=="close"){
        // std::cout<<"Closing gripper!\n";
        rg2.closeGripper();
    }
    
    res.success.data = true;
    return true;
}


};






int main(int argc, char **argv)
{
  ros::init(argc, argv, "Demo");
  ros::NodeHandle nh;

  RobotDemoServer server(nh);

  ROS_INFO("Ready to do perform tasks");
  ros::spin();

  return 0;
}
