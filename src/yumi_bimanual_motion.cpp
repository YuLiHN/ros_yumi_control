#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <boost/lexical_cast.hpp>

// global variables
/*
#define JOINT_VELOCITY_LIMIT 0.05
#define VELOCITY_CONST 2.0 //1.0
#define ROTATION_CONST 2.0 //1.0
#define MAX_POS_ERR 0.05
#define MAX_ROT_ERR 0.05
#define ALPHA 0.7
*/

// functions declarations


class YumiArmController {
public:
    // Constructor
	YumiArmController(ros::NodeHandle& nh, int arm_id) : nh_(nh), arm_id_(arm_id) {

		robot_ready = 0;

		arm_joint_positions.resize(7);
		arm_joint_velocity.resize(7);
		arm_joint_velcmd.resize(7);
		velocity_command_pub.resize(7);

		joint_subscriber = nh_.subscribe("/yumi/joint_states", 1, &YumiArmController::joint_state_callback, this);
		
		// set joints velocity commands and gripper topics
		int urdf_order[7] = {1,2,7,3,4,5,6};
		std::string gripper_effor_topic_name;
		if (arm_id_ == 1) { // left arm
			for(int i = 0; i < 7; i++) {
				command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_l/command";
				velocity_command_pub[i] = nh_.advertise<std_msgs::Float64>(command_topic.c_str(), 10);
			}
			gripper_effor_topic_name = "/yumi/gripper_l_effort_cmd";
			gripper_command_pub = nh_.advertise<std_msgs::Float64>(gripper_effor_topic_name, 2);
		} else { // right arm
			for(int i = 0; i < 7; i++) {
				command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_r/command";
				velocity_command_pub[i] = nh_.advertise<std_msgs::Float64>(command_topic.c_str(), 10);
			}
			gripper_effor_topic_name = "/yumi/gripper_r_effort_cmd";
			gripper_command_pub = nh_.advertise<std_msgs::Float64>(gripper_effor_topic_name, 2);
		}
		

		nh_.getParam("JOINT_VELOCITY_LIMIT", JOINT_VELOCITY_LIMIT);
		nh_.getParam("VELOCITY_CONST", VELOCITY_CONST);
		nh_.getParam("ROTATION_CONST", ROTATION_CONST);
		nh_.getParam("MAX_POS_ERR", MAX_POS_ERR);
		nh_.getParam("MAX_ROT_ERR", MAX_ROT_ERR);
		nh_.getParam("ALPHA", ALPHA);
		// get wanted initial configuration of the joints
		std::vector<double> vect;
		if (arm_id_ == 1)
			nh_.getParam("/initial_joint_position/left_arm", vect);
		else
			nh_.getParam("/initial_joint_position/right_arm", vect);
		init_joint_position.push_back(vect);
		if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
			ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");
		arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);

		std::cout << "yumi_node started" << std::endl;

		// reset position arm
		while(ros::ok()) {

			ros::spinOnce();

			if (robot_ready == 1) {
				reset_arm();
				return;
			}

			ros::Duration(0.1).sleep();

		}

		std::cout << "arm resetted" << std::endl;
		
    
    }

    // Destructor
    ~YumiArmController() {
        // reset_arm();
		cmd.data = 0;
		for (int i = 0; i < 7; i++)
			velocity_command_pub[i].publish(cmd);
    }

	void main_loop() {

		for (int i = 0; i < 5; i++) 
		{
			ros::spinOnce();
			ros::Duration(0.1).sleep();
		}

		/*
		int flag = 0;		
		while(ros::ok()) {
			ros::spinOnce();

			arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);

			double angle1 = 0.0;
			double angle2 = 0.0;
			double angle3 = 0.0;
			current_pose.M.GetRPY(angle1, angle2, angle3);

			std::cout << angle1 << " " << angle2 << " " << angle3 << std::endl;


			if (flag == 0) {
				for (int i = 0; i < 20; i++) {

					arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);

					double angle1 = 0.0;
					double angle2 = 0.0;
					double angle3 = 0.0;
					current_pose.M.GetRPY(angle1, angle2, angle3);

					ros::spinOnce();
					KDL::Twist desired_vel;
					desired_vel.vel = KDL::Vector(0.0, 0.0, 0.0);
					desired_vel.rot = KDL::Vector(0.0, 0.0, 0.1);
					arm_kdl_wrapper.ik_solver_vel->CartToJnt(arm_joint_positions, desired_vel, arm_joint_velcmd);

					//double prevCmd = 0.0;
					std::vector<double> prevCmd(7, 0.0);
					for(int i = 0; i < 7; i++)
					{
						cmd.data = VELOCITY_CONST * arm_joint_velcmd(i);
						cmd.data = limit_joint_velcmd(cmd.data, i);
						cmd.data = lowPassFilter(cmd.data, prevCmd[i], ALPHA);
						velocity_command_pub[i].publish(cmd);
						prevCmd[i] = cmd.data;
					}
					std::cout << angle1 << " " << angle2 << " " << angle3 << std::endl;
					ros::Duration(0.1).sleep();
				}
				flag = 1;
			}

			cmd.data = 0;
			for (int i = 0; i < 7; i++)
				velocity_command_pub[i].publish(cmd);


			ros::Duration(1.0).sleep();
		}
		*/

		//for (int i = 0; i < 5; i++) {
		
		ros::spinOnce();
		KDL::Frame desired_pose;
		arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		// desired_pose.p = current_pose.p + KDL::Vector(0.25, 0.35, -0.1); //KDL::Vector(0.0, 0.0, 0.0);
		double z = current_pose.p(2);
		desired_pose.p = KDL::Vector(0.475, -0.160, z-0.05);
		desired_pose.M = current_pose.M; //KDL::Rotation();
		// go_to_position_waypoints(desired_pose, VELOCITY_CONST);
		go_to_position_waypoints(desired_pose, VELOCITY_CONST);
		pick();
		close_gripper();
		get_up();
		open_gripper();
		//}

		// cmd.data = 0;
		// for (int i = 0; i < 7; i++)
		// 	velocity_command_pub[i].publish(cmd);
		

		
		
		// ros::spinOnce();
		// KDL::Frame desired_pose;
		// arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		// desired_pose.p = current_pose.p + KDL::Vector(0.2, 0.4, 0.0); //KDL::Vector(0.0, 0.0, 0.0);
		// desired_pose.M = current_pose.M; //KDL::Rotation();
		// go_to_position(desired_pose, VELOCITY_CONST);
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// pick();
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// close_gripper();
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// get_up();
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		// desired_pose.p = current_pose.p + KDL::Vector(-0.1, 0.0, 0.0); //KDL::Vector(0.0, 0.0, 0.0);
		// desired_pose.M = current_pose.M; //KDL::Rotation();
		// go_to_position(desired_pose, VELOCITY_CONST);
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// open_gripper();
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// reset_arm();
		// ros::Duration(1.0).sleep();
	}
		
		

	void reset_arm() {
		
		// go to intial joint position
		bool all_fine = false;
		while(all_fine == false) {
			ros::spinOnce();
			all_fine = true;
			for (int i = 0; i < 7; i++) {
				cmd.data = 0.3*(init_joint_position[0][i]-arm_joint_positions(i));
				cmd.data = limit_joint_velcmd(cmd.data, i);
				velocity_command_pub[i].publish(cmd);
				if(std::abs(init_joint_position[0][i]-arm_joint_positions(i))>0.02)
					all_fine = false;
			}
			ros::Duration(0.1).sleep();
		}

		// reset 0 velocity to all joints
		cmd.data = 0;
		for (int i = 0; i < 7; i++)
			velocity_command_pub[i].publish(cmd);

		// open gripper (for the right arm this is a position not an effort)
		gripper_cmd.data = 20.0;
		gripper_command_pub.publish(gripper_cmd);
		ros::Duration(0.1).sleep();

	}

	void open_gripper() {
		if (arm_id_ == 1) {  // left arm
			// for the left arm this is an effort: -20 max opening force, 20 max closing force
			gripper_cmd.data = -10.0;
			gripper_command_pub.publish(gripper_cmd);
			ros::Duration(1.0).sleep();
			gripper_cmd.data = 0.0;
			gripper_command_pub.publish(gripper_cmd);
		} else { // right arm
			// for the right arm this is a position: 0 closed, 20 open
			gripper_cmd.data = 20.0;
			gripper_command_pub.publish(gripper_cmd);
			ros::Duration(1.0).sleep();
		}
	}

	void close_gripper() {
		if (arm_id_ == 1) {  // left arm
			// for the left arm this is an effort: -20 max opening force, 20 max closing force
			gripper_cmd.data = 10.0;
			gripper_command_pub.publish(gripper_cmd);
			ros::Duration(1.0).sleep();
			gripper_cmd.data = 0.0;
			gripper_command_pub.publish(gripper_cmd);
		} else { // right arm
			// for the right arm this is a position: 0 closed, 20 open
			gripper_cmd.data = 0.0;
			gripper_command_pub.publish(gripper_cmd);
			ros::Duration(1.0).sleep();
		}
	}

	void go_to_position(KDL::Frame desired_pose, double vel_constant, double max_pos_error=0.01) {
		// ros::spinOnce();
		// arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		// std::vector<KDL::Vector> waypoints = interpolateWaypoints(current_pose.p, desired_pose.p, 0.03);

		bool reached = false;
		
		while (reached == false) {

			ros::spinOnce();

			arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
			

			KDL::Twist desired_vel;
			desired_vel.vel = desired_pose.p - current_pose.p;
			double angle1_des = 0.0;
			double angle2_des = 0.0;
			double angle3_des = 0.0;
			double angle1_cur = 0.0;
			double angle2_cur = 0.0;
			double angle3_cur = 0.0;
			desired_pose.M.GetRPY(angle1_des, angle2_des, angle3_des);
			current_pose.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);

			double angle1_diff = angle1_des - angle1_cur;
			double angle2_diff = angle2_des - angle2_cur;
			double angle3_diff = angle3_des - angle3_cur;
			angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
			angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
			angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;

			desired_vel.rot = ROTATION_CONST * KDL::Vector(angle1_diff, angle2_diff, angle3_diff);
			arm_kdl_wrapper.ik_solver_vel->CartToJnt(arm_joint_positions, desired_vel, arm_joint_velcmd);

			//double prevCmd = 0.0;
			std::vector<double> prevCmd(7, 0.0);
			for(int i = 0; i < 7; i++)
			{
				cmd.data = vel_constant * arm_joint_velcmd(i);
				cmd.data = limit_joint_velcmd(cmd.data, i);
				cmd.data = lowPassFilter(cmd.data, prevCmd[i], ALPHA);
				velocity_command_pub[i].publish(cmd);
				prevCmd[i] = cmd.data;
			}

			// std::cout << "pos: " << current_pose.p(0) << " " << current_pose.p(1) << " " << current_pose.p(2) << std::endl;
			// std::cout << "err pos: " << std::abs(current_pose.p(0) - desired_pose.p(0)) << " " << std::abs(current_pose.p(1) - desired_pose.p(1)) << " " << std::abs(current_pose.p(2) - desired_pose.p(2)) << std::endl;
			// std::cout << "err rot: " << angle1_diff << " " << angle2_diff << " " << angle3_diff << std::endl;

			if (std::abs(current_pose.p(0) - desired_pose.p(0)) < max_pos_error &&
				std::abs(current_pose.p(1) - desired_pose.p(1)) < max_pos_error &&
				std::abs(current_pose.p(2) - desired_pose.p(2)) < max_pos_error &&
				angle1_diff < MAX_ROT_ERR && angle2_diff < MAX_ROT_ERR && angle3_diff < MAX_ROT_ERR) {
				cmd.data = 0;
				for (int i = 0; i < 7; i++)
					velocity_command_pub[i].publish(cmd);
				reached = true;
			}

			ros::Duration(0.01).sleep();

		}
	}
	
	std::vector<KDL::Vector> interpolateWaypoints(KDL::Vector& start, KDL::Vector& end, double min_step_size) 
	{
		std::vector<KDL::Vector> waypoints;

		// Calculate the distance between start and end positions
		double distance = (end - start).Normalize(1e-5);

		// Calculate the number of waypoints based on the minimum step size
		int numWaypoints = static_cast<int>(distance / min_step_size) + 1;

		std::cout<<"numWaypoints: "<<numWaypoints<<std::endl;

		// Generate waypoints
		for (int i = 0; i < numWaypoints; ++i) {
			double ratio = static_cast<double>(i) / (numWaypoints - 1); // Interpolation ratio from 0 to 1
			KDL::Vector interpolatedPoint = start + ratio * (end - start);
			waypoints.push_back(interpolatedPoint);

			// std::cout<<"ratio: "<<ratio<<std::endl;
			// std::cout<<"x: "<<interpolatedPoint.x()<<" y: "<<interpolatedPoint.y()<<" z: "<<interpolatedPoint.z()<<std::endl;
		}
	
		return waypoints;
	}
	
	void go_to_position_waypoints(KDL::Frame desired_pose, double vel_constant, double min_step_size=0.01) 
	{

		ros::spinOnce();

		arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);

		std::vector<KDL::Vector> waypoints = interpolateWaypoints(current_pose.p, desired_pose.p, min_step_size);

		int i=0;
		for (const auto& wp : waypoints)
		{
			std::cout<<"x: "<<wp.x()<<" y: "<<wp.y()<<" z: "<<wp.z()<<std::endl;

			if (i == waypoints.size() - 1) {
				go_to_position(KDL::Frame(current_pose.M , wp),VELOCITY_CONST, MAX_POS_ERR);
				auto err = current_pose.p - desired_pose.p;
				std::cout<<"final err x: "<<err(0)<<" y: "<<err(1)<<" z: "<<err(2)<<std::endl;
			}
			else{
				go_to_position(KDL::Frame(current_pose.M , wp),VELOCITY_CONST, 0.01);
			}

			
			++i;
		}

 
	}

	void pick() {

		KDL::Frame desired_pose;
		desired_pose.p = current_pose.p;
		desired_pose.p(2) = 0.215;
		desired_pose.M = current_pose.M;

		go_to_position(desired_pose, 0.5);

		return;
	}

	void get_up() {

		KDL::Frame desired_pose;
		desired_pose.p = current_pose.p;
		desired_pose.p(2) = 0.42;
		desired_pose.M = current_pose.M;

		go_to_position(desired_pose, 0.5);

		return;
	}

	void joint_state_callback(const sensor_msgs::JointState & msg) {
		if (arm_id_ == 1) {
			int arm_indecis[7] = {0,2,12,4,6,8,10};
			for(int i = 0; i < 7; i++) {
				arm_joint_positions(i) = msg.position[arm_indecis[i]];
				arm_joint_velocity[i] = msg.velocity[arm_indecis[i]];
			}
		} else {
			int arm_indecis[7] = {1,3,13,5,7,9,11};
			for(int i = 0; i < 7; i++) {
				arm_joint_positions(i) = msg.position[arm_indecis[i]];
				arm_joint_velocity[i] = msg.velocity[arm_indecis[i]];
			}
		}

		robot_ready = 1;
	}

	double lowPassFilter(double current, double previous, double alpha) {
    	return alpha * current + (1.0 - alpha) * previous;
	}

	double limit_joint_velcmd(double cmd, int joint) {
		double limited_cmd = cmd;
		double joint_vel = arm_joint_velocity[joint];
		if((cmd - joint_vel) > JOINT_VELOCITY_LIMIT)
			limited_cmd = JOINT_VELOCITY_LIMIT;
		else if((cmd - joint_vel) < (-JOINT_VELOCITY_LIMIT))
			limited_cmd = -JOINT_VELOCITY_LIMIT;
		return limited_cmd;
	}

	// Katharina code ##################################################################

	double calc_cond_for_q(KDL::JntArray current_q){
 
        KDL::Jacobian jac;
        arm_kdl_wrapper.jnt_jac_solver->JntToJac(current_q, jac); //confirm method exists
 
        // svd process
        Eigen::MatrixXd jac_eigen(6, jac.columns()); // confirm kdl jac is 6 rows
        for (unsigned int i = 0; i < jac.rows(); ++i) {
            for (unsigned int j = 0; j < jac.columns(); ++j) {
                jac_eigen(i, j) = jac(i, j);
            }
        }
        // Calculate the condition number using the singular value decomposition (SVD)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac_eigen);
        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1); //check division feasibility
        return cond;
    }

	// function needs: parametrized discretized curve (array of waypoints) and current pose
    // but trajectory is like std::vector<KDL::Vector> waypoints;
    double get_cond_from_rollout(KDL::JntArray q_initial, std::vector<KDL::Frame> trajectory, double vel_constant) { // first val traj = t+1
 
        KDL::Frame pose_t;
        KDL::JntArray q_t;
        KDL::JntArray dq_t;
 
        double max_pos_error = 0.01;
        double stepsize = 0.01; //1e-2;
        KDL::Frame p_des;
        KDL::Twist desired_vel;
 
        double max_cond = 1.0;
 
        q_t = q_initial;
 
        for (int i = 0; i < trajectory.size(); i++) {

			arm_kdl_wrapper.fk_solver_pos->JntToCart(q_t, pose_t, -1); // maybe move outside for cycle
 
            p_target = trajectory[i]; 
 
            bool pose_reached = false;
            while(pose_reached == false) {

				// 1) assuming we start with zero velocity and velocity controllers are ideal, 3) limit vel commands (and low pass)
                
                desired_vel = compute_desired_vel(p_des, pose_t);


                arm_kdl_wrapper.ik_solver_vel->CartToJnt(q_t, desired_vel, dq_t);
				for(int i = 0; i < 7; i++)
				{
					dq_t(i) = vel_constant * dq_t(i);
					dq_t(i) = limit_joint_velcmd(dq_t(i), i);
				}
 
                q_t += stepsize * dq_t; // maybe put in for loop as q_t(1) += ...
                
                arm_kdl_wrapper.fk_solver_pos->JntToCart(q_t, pose_t, -1);

				double angle1_des = 0.0;
				double angle2_des = 0.0;
				double angle3_des = 0.0;
				double angle1_cur = 0.0;
				double angle2_cur = 0.0;
				double angle3_cur = 0.0;
				p_target.M.GetRPY(angle1_des, angle2_des, angle3_des);
				pose_t.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);
		
				double angle1_diff = angle1_des - angle1_cur;
				double angle2_diff = angle2_des - angle2_cur;
				double angle3_diff = angle3_des - angle3_cur;
				angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
				angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
				angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;
 
                //see about angle_diff
                if (std::abs(pose_t.p(0) - p_target.p(0)) < max_pos_error &&
					std::abs(pose_t.p(1) - p_target.p(1)) < max_pos_error &&
					std::abs(pose_t.p(2) - p_target.p(2)) < max_pos_error &&
					angle1_diff < MAX_ROT_ERR && angle2_diff < MAX_ROT_ERR && angle3_diff < MAX_ROT_ERR) {
	
					pose_reached = true;
				}
			}

			// get cond(jac) for this point
			double current_cond = calc_cond_for_q(q_t);
			// check cond and if inf or something abort mission
			// add to total or whatever

			if (max_cond < current_cond)
				max_cond = current_cond;
			
		}

		return max_cond; //cond_total;
	}
 
    KDL::Twist compute_desired_vel(KDL::Frame p_des, KDL::Frame p_t) {
 
        KDL::Twist desired_vel;
 
        // compute cartesian reference command
        desired_vel.vel = p_des.p - p_t.p;
        double angle1_des = 0.0;
        double angle2_des = 0.0;
        double angle3_des = 0.0;
        double angle1_cur = 0.0;
        double angle2_cur = 0.0;
        double angle3_cur = 0.0;
        p_des.M.GetRPY(angle1_des, angle2_des, angle3_des);
        p_t.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);
 
        double angle1_diff = angle1_des - angle1_cur;
        double angle2_diff = angle2_des - angle2_cur;
        double angle3_diff = angle3_des - angle3_cur;
        angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
        angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
        angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;
        desired_vel.rot = ROTATION_CONST * KDL::Vector(angle1_diff, angle2_diff, angle3_diff);
 
        return desired_vel;
    }

	void test_V() {

		KDL::JntArray q0;
		KDL::Frame x1 = KDL::Frame(KDL::Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.0, 0.0));
		KDL::Frame x2 = KDL::Frame(KDL::Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.1, 0.0));
		KDL::Frame x3 = KDL::Frame(KDL::Rotation(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), KDL::Vector(0.0, 0.2, 0.0));
		
		arm_kdl_wrapper.fk_solver_pos->JntToCart(q0, x1, -1); // TODO: get q0 without IK ???

		std::vector<KDL::Frame> trajectory = {x2, x3};
		double v = get_cond_from_rollout(q0, trajectory, 2.);
		std::cout << v << std::endl;

	}

private:

	ros::NodeHandle nh_;
	int arm_id_;

	KDL::JntArray arm_joint_positions;
	std::vector<double> arm_joint_velocity;
	KDLWrapper arm_kdl_wrapper;
	KDL::Twist arm_cart_velocity;
	KDL::JntArray arm_joint_velcmd;
	KDL::Frame current_pose;

	ros::Subscriber joint_subscriber;
	std::vector<ros::Publisher> velocity_command_pub;
	ros::Publisher gripper_command_pub;

	std_msgs::Float64 cmd;
	std_msgs::Float64 gripper_cmd;
	std::string command_topic;
	int robot_ready;

	// constants
	std::vector<std::vector<double>> init_joint_position;
	double JOINT_VELOCITY_LIMIT;
	double VELOCITY_CONST;
	double ROTATION_CONST;
	double MAX_POS_ERR;
	double MAX_ROT_ERR;
	double ALPHA;

	};

int main(int argc, char** argv) {

    ros::init(argc, argv, "yumi_arm_controller");

    ros::NodeHandle nh;
	int arm_id = 2; // using the same convention of KTH code. 1: left arm, 2: right arm

    YumiArmController arm_controller(nh, arm_id);

    //ros::spin();

	// arm_controller.main_loop();

	arm_controller.test_V();

    return 0;
}






















