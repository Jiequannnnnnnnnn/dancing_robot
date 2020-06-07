// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/toro.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

int state = JOINT_CONTROLLER;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;
std::string JOINT_DESIRED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

int main() {

    JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
    JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
    JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
    
    JOINT_DESIRED_KEY = "joints_desired";


    // start redis client
    auto redis_client = RedisClient();
    redis_client.connect();

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    auto robot = new Sai2Model::Sai2Model(robot_file, false);
    robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
    VectorXd initial_q = robot->_q;
    robot->updateModel();

    // prepare controller
    int dof = robot->dof();
    VectorXd command_torques = VectorXd::Zero(dof);
    MatrixXd N_prec = MatrixXd::Identity(dof, dof);

    // pose task
    const string control_link1 = "RL_foot";
    const Vector3d control_point1 = Vector3d(0,0,0);
    auto posori_task1 = new Sai2Primitives::PosOriTask(robot, control_link1, control_point1);
    Eigen::Vector3d ee_position1 = Eigen::Vector3d::Zero();
    robot->position(ee_position1, control_link1, control_point1);
    Eigen::Vector3d ee_init1 = ee_position1;
    
    const string control_link2 = "LL_foot";
    const Vector3d control_point2 = Vector3d(0,0,0);
    auto posori_task2 = new Sai2Primitives::PosOriTask(robot, control_link2, control_point2);
    Eigen::Vector3d ee_position2 = Eigen::Vector3d::Zero();
    robot->position(ee_position2, control_link2, control_point2);
    Eigen::Vector3d ee_init2 = ee_position2;
    
    const string control_link3 = "trunk";
    const Vector3d control_point3 = Vector3d(0,0,0);
    auto posori_task3 = new Sai2Primitives::PosOriTask(robot, control_link3, control_point3);
    Eigen::Vector3d ee_position3 = Eigen::Vector3d::Zero();
    robot->position(ee_position3, control_link3, control_point3);
    Eigen::Vector3d ee_init3 = ee_position3;
    
    const string control_link4 = "ra_link6";
    const Vector3d control_point4 = Vector3d(0,0,0);
    auto posori_task4 = new Sai2Primitives::PosOriTask(robot, control_link4, control_point4);
    Eigen::Vector3d ee_position4 = Eigen::Vector3d::Zero();
    robot->position(ee_position4, control_link4, control_point4);
    Eigen::Vector3d ee_init4 = ee_position4;
    
    const string control_link5 = "la_link6";
    const Vector3d control_point5 = Vector3d(0,0,0);
    auto posori_task5 = new Sai2Primitives::PosOriTask(robot, control_link5, control_point5);
    Eigen::Vector3d ee_position5 = Eigen::Vector3d::Zero();
    robot->position(ee_position5, control_link5, control_point5);
    Eigen::Vector3d ee_init5 = ee_position5;
    
    // desiredt task configuration
    posori_task1->_desired_position = ee_init1;
    posori_task2->_desired_position = ee_init2;
    posori_task1->_desired_position(1) = -0.3;
    posori_task2->_desired_position(1) = 0.3;
    //cout << ee_init1(0) << ee_init1(1) << ee_init1(2) << endl;
    //cout << ee_init2(0) << ee_init2(1) << ee_init2(2) << endl;
    posori_task3->_desired_position = ee_init3;
    posori_task4->_desired_position = ee_init4;
    posori_task5->_desired_position = ee_init5;

#ifdef USING_OTG
    posori_task1->_use_interpolation_flag = true;
    posori_task2->_use_interpolation_flag = true;
    posori_task3->_use_interpolation_flag = true;
    posori_task4->_use_interpolation_flag = true;
    posori_task5->_use_interpolation_flag = true;
#else
    posori_task1->_use_velocity_saturation_flag = true;
    posori_task2->_use_velocity_saturation_flag = true;
    posori_task3->_use_velocity_saturation_flag = true;
    posori_task4->_use_velocity_saturation_flag = true;
    posori_task5->_use_velocity_saturation_flag = true;
#endif
    
    VectorXd posori_task_torques1 = VectorXd::Zero(dof);
    VectorXd posori_task_torques2 = VectorXd::Zero(dof);
    VectorXd posori_task_torques3 = VectorXd::Zero(dof);
    VectorXd posori_task_torques4 = VectorXd::Zero(dof);
    VectorXd posori_task_torques5 = VectorXd::Zero(dof);
    
    posori_task1->_kp_pos = 225.0;
    posori_task1->_kv_pos = 30.0;
    posori_task1->_kp_ori = 225.0;
    posori_task1->_kv_ori = 30.0;
    
    posori_task2->_kp_pos = 225.0;
    posori_task2->_kv_pos = 30.0;
    posori_task2->_kp_ori = 225.0;
    posori_task2->_kv_ori = 30.0;
    
    posori_task3->_kp_pos = 225.0;
    posori_task3->_kv_pos = 30.0;
    posori_task3->_kp_ori = 225.0;
    posori_task3->_kv_ori = 30.0;
    
    posori_task4->_kp_pos = 200.0;
    posori_task4->_kv_pos = 20.0;
    posori_task4->_kp_ori = 200.0;
    posori_task4->_kv_ori = 20.0;
    
    posori_task5->_kp_pos = 200.0;
    posori_task5->_kv_pos = 20.0;
    posori_task5->_kp_ori = 200.0;
    posori_task5->_kv_ori = 20.0;

    // joint task
    auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
    joint_task->_use_interpolation_flag = true;
#else
    joint_task->_use_velocity_saturation_flag = true;
#endif

    VectorXd joint_task_torques = VectorXd::Zero(dof);
    joint_task->_kp = 225; //250.0;
    joint_task->_kv = 30; //15.0;
    
    joint_task->_otg->setMaxVelocity(3*M_PI);
    joint_task->_otg->setMaxAcceleration(6*M_PI);
    joint_task->_otg->setMaxJerk(9*M_PI);

    MatrixXd q_desired;
//    VectorXd q_init_desired_1 = initial_q;
//    q_init_desired_1 << 0,0,0,0,1,0,0,-1,0,0,0,0,0,-1,0,0,0,0,0,1,-0.1,-0.5,2,-0.7,0,-0.4,0,0,0,0,0,0,1;
//    q_desired.conservativeResize(q_desired.rows()+1, dof);
//    q_desired.row(q_desired.rows()-1) = q_init_desired_1;
//    VectorXd q_init_desired_2 = initial_q;
//    q_init_desired_2 << 0,0,0,0,1,0,0,-1,0,0,0,0,0,-1,0,0,0,0,0,1,-0.1,-0.5,2,-0.7,0,-0.4,0,0,0,0,0,0,-1;
//    q_desired.conservativeResize(q_desired.rows()+1, dof);
//    q_desired.row(q_desired.rows()-1) = q_init_desired_2;
    
    joint_task->_desired_position = initial_q;
    joint_task->_desired_position(20) = 0.6;
    joint_task->_desired_position(26) = 0.6;
    joint_task->_desired_position(21) = -1.57;
    joint_task->_desired_position(27) = -1.57;
    joint_task->_desired_position(22) = 0.6;
    joint_task->_desired_position(28) = 0.6;
    joint_task->_desired_position(32) = 0.7;

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000);
    double start_time = timer.elapsedTime(); //secs
    bool fTimerDidSleep = true;
    
    ofstream myfile;
    myfile.open ("joints.csv");

    int step = 1;
    int trunk_control = 0;
    int count = 0;
    double lastTime = start_time;

    while (runloop) {
    
    	cout << step << endl;
    
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime() - start_time;

        // read robot state from redis
        robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
        robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
        
        if (step == 1){
        	if (robot->_dq.norm() < 0.5 && (robot->_q - joint_task->_desired_position).norm() < 0.5){
        		step++;  
        		lastTime = time;	
        	}
        }
        else if (step == 2){
        	posori_task3->_desired_position(1) = 0.25*cos(0.5*M_PI*(time-lastTime) + M_PI/2);
        	posori_task3->_desired_position(0) = 0.1;
      		//posori_task3->_desired_position(0) += 0.01;
        	//posori_task3->_desired_position(0) = 0.05*sin(0.25*M_PI*(time-lastTime));
        	//joint_task->_desired_position(1) = 0.2*sin(0.5*M_PI*(time-lastTime));
        	//joint_task->_desired_position(0) = 0.1*sin(0.5*M_PI*(time-lastTime));
        	//joint_task->_desired_position(0) += 0.01;
        	
        	if (posori_task3->_desired_position(1) > 0){
        		trunk_control = 3;
        		joint_task->_desired_position(31) = 0.4;
        		joint_task->_desired_position(6) = -0.3;
        		if (posori_task2->_desired_position(0) - posori_task1->_desired_position(0) < 0.05){
        			posori_task2->_desired_position(0) += 0.05;
        			posori_task3->_desired_position(0) += 0.05;
        		}
        		//posori_task1->_desired_position(0) = 0;
        	}
        	else{
        		trunk_control = 4;
        		joint_task->_desired_position(31) = -0.4;
        		joint_task->_desired_position(12) = 0.3;
        		if (posori_task2->_desired_position(0) - posori_task1->_desired_position(0) >= 0.05){
        			posori_task1->_desired_position(0) += 0.05;
        			posori_task3->_desired_position(0) += 0.05;
        		}
        		//posori_task2->_desired_position(0) = 0;
        	}
        }
        
        
        // read desired joints configuration
//        joint_task->_desired_position = q_desired.row(dance_move);
//        if (robot->_dq.norm() < 1 && (robot->_q - joint_task->_desired_position).norm() < 1 && dance_move < q_desired.rows()-1) dance_move++;
        
        // update model
        robot->updateModel();

        // update task model and set hierarchy
        N_prec.setIdentity();
        if (trunk_control == 0){
        posori_task1->updateTaskModel(N_prec);
        N_prec = posori_task1->_N;
        posori_task2->updateTaskModel(N_prec);
        N_prec = posori_task2->_N;
        joint_task->updateTaskModel(N_prec);
        
        // compute torques
        posori_task1->computeTorques(posori_task_torques1);
        posori_task2->computeTorques(posori_task_torques2);
        joint_task->computeTorques(joint_task_torques);

        command_torques = posori_task_torques1 + posori_task_torques2 + joint_task_torques;
        }
        else if (trunk_control == 1){
        posori_task1->updateTaskModel(N_prec);
        N_prec = posori_task1->_N;
        posori_task2->updateTaskModel(N_prec);
        N_prec = posori_task2->_N;
        posori_task3->updateTaskModel(N_prec);
        N_prec = posori_task3->_N;
        //posori_task4->updateTaskModel(N_prec);
        //N_prec = posori_task4->_N;
        //posori_task5->updateTaskModel(N_prec);
        //N_prec = posori_task5->_N;
        joint_task->updateTaskModel(N_prec);
        
        joint_task->computeTorques(joint_task_torques);
        posori_task1->computeTorques(posori_task_torques1);
        posori_task2->computeTorques(posori_task_torques2);
        posori_task3->computeTorques(posori_task_torques3);
        //posori_task4->computeTorques(posori_task_torques4);
        //posori_task5->computeTorques(posori_task_torques5);
        
        command_torques = joint_task_torques + posori_task_torques1 + posori_task_torques2 + posori_task_torques3;// + posori_task_torques4 + posori_task_torques5;
        }
        else if (trunk_control == 2){
        	joint_task->updateTaskModel(N_prec);
        	
        	joint_task->computeTorques(joint_task_torques);
        	
        	command_torques = joint_task_torques;
        }
        else if (trunk_control == 3){
        	posori_task2->updateTaskModel(N_prec);
        	N_prec = posori_task2->_N;
        	posori_task3->updateTaskModel(N_prec);
        	N_prec = posori_task3->_N;
        	joint_task->updateTaskModel(N_prec);
        	
        	joint_task->computeTorques(joint_task_torques);
        	posori_task2->computeTorques(posori_task_torques2);
        	posori_task3->computeTorques(posori_task_torques3);
        	
        	command_torques = joint_task_torques + posori_task_torques2 + posori_task_torques3;
        }
        else if (trunk_control == 4){
        	posori_task1->updateTaskModel(N_prec);
        	N_prec = posori_task1->_N;
        	posori_task3->updateTaskModel(N_prec);
        	N_prec = posori_task3->_N;
        	joint_task->updateTaskModel(N_prec);
        	
        	joint_task->computeTorques(joint_task_torques);
        	posori_task1->computeTorques(posori_task_torques1);
        	posori_task3->computeTorques(posori_task_torques3);
        	
        	command_torques = joint_task_torques + posori_task_torques1 + posori_task_torques3;
        }

    
        /*
        if(state == JOINT_CONTROLLER)
        {
            
            //
            N_prec.setIdentity();
            joint_task->updateTaskModel(N_prec);
            // compute torques
            joint_task->computeTorques(joint_task_torques);
            command_torques = joint_task_torques;
            
            if( (robot->_q - q_init_desired).norm() < 0.15 )
            {
                posori_task->reInitializeTask();
                posori_task->_desired_position += Vector3d(-0.1,0.1,0.1);
                posori_task->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task->_desired_orientation;
                joint_task->reInitializeTask();
                joint_task->_kp = 0;
                state = POSORI_CONTROLLER;
            }
            
            
        }
        else if(state == POSORI_CONTROLLER)
        {
            // update task model and set hierarchy
            N_prec.setIdentity();
            posori_task->updateTaskModel(N_prec);
            N_prec = posori_task->_N;
            joint_task->updateTaskModel(N_prec);
            // compute torques
            posori_task->computeTorques(posori_task_torques);
            joint_task->computeTorques(joint_task_torques);
            command_torques = posori_task_torques + joint_task_torques;
        }
        */
            
        
        
        // send to redis
        redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
        controller_counter++;
        myfile << time;
        for (int i = 0; i < robot->_q.size(); i++) {
            myfile << "," << robot->_q(i);
        }
        myfile << endl;
    }

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    myfile.close();
    return 0;
}
