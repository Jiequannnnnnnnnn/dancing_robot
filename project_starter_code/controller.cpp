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

double sin_wave(double lower, double upper, double T, double t) {
    // generate a wave that starts from lower limit
    return (lower + upper) / 2.0 - (upper - lower) / 2.0 * cos(2 * M_PI / T * t);
}

double square_wave(double lower, double upper, double T, double t) {
    // generate a square wave (piecewise constant)
    if (((int)(t / (T / 2.0))) % 2 == 0)
        return lower;
    else
        return upper;
}

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
    
    // desiredt task configuration
    posori_task1->_desired_position = ee_init1;
    posori_task2->_desired_position = ee_init2;

#ifdef USING_OTG
    posori_task1->_use_interpolation_flag = true;
    posori_task2->_use_interpolation_flag = true;
#else
    posori_task1->_use_velocity_saturation_flag = true;
    posori_task2->_use_velocity_saturation_flag = true;
#endif
    
    VectorXd posori_task_torques1 = VectorXd::Zero(dof);
    VectorXd posori_task_torques2 = VectorXd::Zero(dof);
    
    posori_task1->_kp_pos = 200.0;
    posori_task1->_kv_pos = 20.0;
    posori_task1->_kp_ori = 200.0;
    posori_task1->_kv_ori = 20.0;
    
    posori_task2->_kp_pos = 200.0;
    posori_task2->_kv_pos = 20.0;
    posori_task2->_kp_ori = 200.0;
    posori_task2->_kv_ori = 20.0;

    // joint task
    auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
    joint_task->_use_interpolation_flag = true;
#else
    joint_task->_use_velocity_saturation_flag = true;
#endif

    VectorXd joint_task_torques = VectorXd::Zero(dof);
    joint_task->_kp = 500;
    joint_task->_kv = 30;
    joint_task->_otg->setMaxVelocity(10 * M_PI);
    joint_task->_otg->setMaxAcceleration(30*M_PI);
    joint_task->_otg->setMaxJerk(60*M_PI);

    MatrixXd q_desired;
    VectorXd q_init_desired_1 = initial_q;
    q_init_desired_1 << 0,0,0,0,1,0,0,-1,0,0,0,0,0,-1,0,0,0,0,0,1,-0.1,-0.5,2,-0.7,0,-0.4,0,0,0,0,0,0,1;
    q_desired.conservativeResize(q_desired.rows()+1, dof);
    q_desired.row(q_desired.rows()-1) = q_init_desired_1;
    VectorXd q_init_desired_2 = initial_q;
    q_init_desired_2 << 0,0,0,0,1,0,0,-1,0,0,0,0,0,-1,0,0,0,0,0,1,-0.1,-0.5,2,-0.7,0,-0.4,0,0,0,0,0,0,-1;
    q_desired.conservativeResize(q_desired.rows()+1, dof);
    q_desired.row(q_desired.rows()-1) = q_init_desired_2;

    // create a timer
    LoopTimer timer;
    timer.initializeTimer();
    timer.setLoopFrequency(1000);
    double start_time = timer.elapsedTime(); //secs
    bool fTimerDidSleep = true;
    
    ofstream myfile;
    myfile.open ("joints.csv");

    int dance_move = 0;

    int state = 0; // -1: reset; other: corresponding dance move

    while (runloop) {
        // cout << dance_move << endl;
        // wait for next scheduled loop
        cout << state << " " << dance_move << endl;
        timer.waitForNextLoop();
        double time = timer.elapsedTime() - start_time;

        // read robot state from redis
        robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
        robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
        
        // read desired joints configuration
        // joint_task->_desired_position = q_desired.row(dance_move);
        // if (robot->_dq.norm() < 0.1 && (robot->_q - joint_task->_desired_position).norm() < 1 && dance_move < q_desired.rows()-1) dance_move++;
        // overide above: read desired from redis
		// joint_task->_desired_position = redis_client.getEigenMatrixJSON(JOINT_DESIRED_KEY) / 180.0 * M_PI;
        
        VectorXd desired_angles(33);

        if (state == -1) {
            joint_task->_desired_position << 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
            if (robot->_dq.norm() < 0.1 && (robot->_q - joint_task->_desired_position).norm() < 1) {
                state = ++dance_move;
                start_time = timer.elapsedTime(); // reset start time
            }
        }
        else if (state == 0) {
            // wave actions
            desired_angles << 
            0,0,0,0,0,0, // whole body
            0,0,0,0,0,0, // left leg
            0,0,0,0,0,0, // right leg
            0, // waist
            20, sin_wave(100, 150, 2, time), 90, sin_wave(50, 70, 2, time), 0, 0, // left arm
            20, sin_wave(150, 100, 2, time), 90, sin_wave(70, 50, 2, time), 0, 0, // right arm
            sin_wave(-30, 30, 2, time), 0; // neck
            joint_task->_desired_position << (desired_angles / 180.0 * M_PI);
            if (time > 3*2) {
                state = -1; // reset
            }
        }
        else if (state == 1) {
            // gangnan style
            desired_angles << 
            0,0,0,0,0,0, // whole body
            0, sin_wave(0, -60, 1, time), 0, sin_wave(0, -60, 1, time), 30, 0, // left leg
            0, sin_wave(0, -60, 1, time), 0, sin_wave(0, -60, 1, time), 30, 0, // right leg
            0, // waist
            -30, 60, -60, 120, 0, 0, // left arm
            20, sin_wave(120, 170, 1, time), sin_wave(50, 120, 1, time), 60, 0, 0, // right arm
            sin_wave(-30, 30, 1, time), 0; // neck
            joint_task->_desired_position << (desired_angles / 180.0 * M_PI);
            if (time > 3*1) {
                state = -1; // reset
            }
        }
        else if (state == 2) {
            // kungfu actions
            joint_task->_kp = 500;
            joint_task->_kv = 30;
            joint_task->_otg->setMaxVelocity(100 * M_PI);
            joint_task->_otg->setMaxAcceleration(300*M_PI);
            joint_task->_otg->setMaxJerk(600*M_PI);
            desired_angles <<
            0,0,0,0,0,0,
            0,-60,0,-60,30,0,
            0,-60,0,-60,30,0,
            0,
            square_wave(-90, 70, 2, time), square_wave(40, 20, 2, time), square_wave(-40, 20, 2, time), square_wave(120, 0, 2, time) ,0,0,
            square_wave(70, -90, 2, time), square_wave(20, 40, 2, time), square_wave(20, -40, 2, time), square_wave(0, 120, 2, time) ,0,0,
            0,0;
            joint_task->_desired_position << (desired_angles / 180.0 * M_PI);
            if (time > 3*2) {
                state = -1; // reset
            }
        }
        else if (state == 3) {
            // fly wave
            desired_angles <<
            0,0,0,0,0,0,
            0,sin_wave(-100, 0, 1, time),0,sin_wave(100, 0, 1, time),0,0,
            0,sin_wave(-100, 0, 1, time),0,sin_wave(100, 0, 1, time),0,0,
            0,
            0,sin_wave(50, 110, 1, time),90,sin_wave(-20, 20, 1, time),0,0,
            0,sin_wave(50, 110, 1, time),90,sin_wave(-20, 20, 1, time),0,0,
            0,sin_wave(-30, 30, 1, time);
            joint_task->_desired_position << (desired_angles / 180.0 * M_PI);
            // if (time > 3*1) {
            //     state = -1; // reset
            // }
        }
        
        // update model
        robot->updateModel();

        // update task model and set hierarchy
        N_prec.setIdentity();
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
