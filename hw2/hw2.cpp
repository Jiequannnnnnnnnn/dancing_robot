#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>
#include <math.h>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

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
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.1);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
    
    ofstream myfile;
    myfile.open ("test.csv");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_1;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
            VectorXd q_desired = initial_q;
            q_desired(dof-1) = 0.1; // set the desired robot joint angles

            for (int i = 0; i < dof; i++) {
            double Kp, Kv;
            if(i == dof-1) {
            Kp = 50;
            Kv = -0.299;
            } else {
            Kp = 400;
            Kv = 50;
            }
            command_torques(i) = -Kp*(robot->_q(i) - q_desired(i)) - Kv*robot->_dq(i);
            } // calculate the control torques without coriolis, centrifugal, and gravity compensations

            VectorXd gravity(dof);
            robot->gravityVector(gravity); // calculate the joint space gravity compensation vector

            VectorXd coriolis(dof);
            robot->coriolisForce(coriolis); // calculate the joint space coriolis and centrifugal force compensation vector

            command_torques += coriolis + gravity;  // calculate the control torques


            
            myfile << time << "," << robot->_q(6)/M_PI*180 << endl;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
            double kp = 200;
            double kv = 27;
            double kvj = 21;
            
            VectorXd x_desired(3);
            x_desired << 0.3, 0.1, 0.5;
            Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
            robot->position(ee_position, link_name, pos_in_link);
            Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
            robot->linearVelocity(ee_velocity, link_name, pos_in_link);
			Eigen::MatrixXd ee_jacobian(3,dof);
            robot->Jv(ee_jacobian, link_name, pos_in_link);
            
            robot->operationalSpaceMatrices(Lambda, J_bar, N, Jv);
            
            Eigen::VectorXd g(dof);
            robot->gravityVector(g);
            
            MatrixXd F = Lambda * (-kp * (ee_position-x_desired) - kv * ee_velocity);
            command_torques = ee_jacobian.transpose() * F + g - N.transpose() * robot->_M * (kvj * robot->_dq);
            
            myfile << time << "," << robot->_q(0)/M_PI*180 << "," << robot->_q(1)/M_PI*180 << "," << robot->_q(2)/M_PI*180 << "," << robot->_q(3)/M_PI*180
            << "," << robot->_q(4)/M_PI*180 << "," << robot->_q(5)/M_PI*180 << "," << robot->_q(6)/M_PI*180
            << "," << ee_position(0) << "," << ee_position(1) << "," << ee_position(2) << endl;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 200;
            double kv = 27;
            double kvj = 21;
            
            VectorXd x_desired(3);
            x_desired << 0.3, 0.1, 0.5;
            Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
            robot->position(ee_position, link_name, pos_in_link);
            Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
            robot->linearVelocity(ee_velocity, link_name, pos_in_link);
            robot->Jv(Jv, link_name, pos_in_link);
            robot->operationalSpaceMatrices(Lambda, J_bar, N, Jv);
            robot->dynConsistentInverseJacobian(J_bar, Jv);

            Eigen::VectorXd g(dof);
            robot->gravityVector(g);
            
            MatrixXd F = Lambda * (kp * (x_desired-ee_position) - kv * ee_velocity) + J_bar.transpose() * g;
            command_torques = Jv.transpose() * F - N.transpose() * robot->_M * (kvj * robot->_dq);
            
            myfile << time << "," << robot->_q(0)/M_PI*180 << "," << robot->_q(1)/M_PI*180 << "," << robot->_q(2)/M_PI*180 << "," << robot->_q(3)/M_PI*180
            << "," << robot->_q(4)/M_PI*180 << "," << robot->_q(5)/M_PI*180 << "," << robot->_q(6)/M_PI*180
            << "," << ee_position(0) << "," << ee_position(1) << "," << ee_position(2) << endl;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			double kp = 200;
            double kv = 27;
            double kvj = 21;
            
            VectorXd x_desired(3);
            x_desired << 0.3 + 0.1*sin(M_PI*time), 0.1 + 0.1*cos(M_PI*time), 0.5;
            
            Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
            robot->position(ee_position, link_name, pos_in_link);
            Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
            robot->linearVelocity(ee_velocity, link_name, pos_in_link);
            robot->Jv(Jv, link_name, pos_in_link);
            robot->operationalSpaceMatrices(Lambda, J_bar, N, Jv);
            robot->dynConsistentInverseJacobian(J_bar, Jv);

            Eigen::VectorXd g(dof);
            robot->gravityVector(g);
            Eigen::VectorXd b(dof);
            robot->coriolisForce(b);
            
            double kp_i = 100.0;
            double kv_i = 13;
            
            VectorXd q_desired(dof);
            q_desired << 0, 0, 0, 0, 0, 0, 0;
            VectorXd q_err(dof);
            q_err = robot->_q - q_desired;
            VectorXd pd_torques = VectorXd::Zero(dof);
            pd_torques(0) = -kp_i*q_err(0) - kv_i*robot->_dq(0);
            pd_torques(1) = -kp_i*q_err(1) - kv_i*robot->_dq(1);
            pd_torques(2) = -kp_i*q_err(2) - kv_i*robot->_dq(2);
            pd_torques(3) = -kp_i*q_err(3) - kv_i*robot->_dq(3);
            pd_torques(4) = -kp_i*q_err(4) - kv_i*robot->_dq(4);
            pd_torques(5) = -kp_i*q_err(5) - kv_i*robot->_dq(5);
            pd_torques(6) = -kp_i*q_err(6) - kv_i*robot->_dq(6);
            
            pd_torques += g + b;
            pd_torques = robot->_M * pd_torques;
            
            MatrixXd F = Lambda * (kp * (x_desired-ee_position) - kv * ee_velocity) + J_bar.transpose() * g;
            command_torques = Jv.transpose() * F + pd_torques + g;
            
            myfile << time << "," << robot->_q(0)/M_PI*180 << "," << robot->_q(1)/M_PI*180 << "," << robot->_q(2)/M_PI*180 << "," << robot->_q(3)/M_PI*180
            << "," << robot->_q(4)/M_PI*180 << "," << robot->_q(5)/M_PI*180 << "," << robot->_q(6)/M_PI*180
            << "," << ee_position(0) << "," << ee_position(1) << "," << ee_position(2) << endl;
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    myfile.close();
	return 0;
}
