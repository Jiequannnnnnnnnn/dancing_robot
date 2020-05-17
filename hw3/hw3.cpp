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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
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
    myfile.open ("4b.csv");
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
		int controller_number = QUESTION_4;

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 100;
            double kv = 20;
            double kpj = 50;
            double kvj = 14;
            
            VectorXd x_desired(3);
            x_desired << 0.3 + 0.1*sin(M_PI*time), 0.1 + 0.1*cos(M_PI*time), 0.5;
            VectorXd xd_desired(3);
            xd_desired << 0.1*M_PI*cos(M_PI*time), -0.1*M_PI*sin(M_PI*time), 0;
            VectorXd xdd_desired(3);
            xdd_desired << -0.1*pow(M_PI,2)*sin(M_PI*time), -0.1*pow(M_PI,2)*cos(M_PI*time), 0;
            
            VectorXd q_desired(dof);
            q_desired << 0, 0, 0, 0, 0, 0, 0;
            
            Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
            robot->position(ee_position, link_name, pos_in_link);
            Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
            robot->linearVelocity(ee_velocity, link_name, pos_in_link);
            Eigen::MatrixXd ee_jacobian(3,dof);
            robot->Jv(ee_jacobian, link_name, pos_in_link);
            robot->taskInertiaMatrix(Lambda, ee_jacobian);
            robot->dynConsistentInverseJacobian(J_bar, ee_jacobian);
            robot->nullspaceMatrix(N, ee_jacobian);
            
            Eigen::VectorXd g(dof);
            robot->gravityVector(g);
            
            
            MatrixXd F = Lambda * (-kp * (ee_position-x_desired) - kv * ee_velocity);
            command_torques = ee_jacobian.transpose() * F + N.transpose() * (-kpj * (robot->_q - q_desired) - kvj * robot->_dq) + g;
            myfile << time << "," << ee_position(0) << "," << ee_position(1) << "," << ee_position(2) << endl;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 100;
            double kv = 20;
            double kmid = 25;
            double kdamp = 14;
            
            VectorXd x_desired(3);
            x_desired << -0.65, -0.45, 0.7;
            
            Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
            robot->position(ee_position, link_name, pos_in_link);
            Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
            robot->linearVelocity(ee_velocity, link_name, pos_in_link);
            Eigen::MatrixXd ee_jacobian(3,dof);
            robot->Jv(ee_jacobian, link_name, pos_in_link);
            robot->taskInertiaMatrix(Lambda, ee_jacobian);
            robot->dynConsistentInverseJacobian(J_bar, ee_jacobian);
            robot->nullspaceMatrix(N,ee_jacobian);
            
            Eigen::VectorXd g(dof);
            robot->gravityVector(g);
            
            VectorXd q_l(dof);
            q_l << -165, -100, -165, -170, -165, 0, -165;
            q_l = q_l / 180 * M_PI;
            VectorXd q_u(dof);
            q_u << 165, 100, 165, -30, 165, 210, 165;
            q_u = q_u / 180 * M_PI;
            
            MatrixXd F = Lambda * (-kp*(ee_position-x_desired) - kv*ee_velocity);
            VectorXd command_mid = (-2) * kmid * (robot->_q - 0.5*(q_l+q_u));
            command_torques = ee_jacobian.transpose()*F + N.transpose()*(-kdamp*robot->_dq) + command_mid + g;
            myfile << time << "," << robot->_q(3)/M_PI*180 << "," << robot->_q(5)/M_PI*180 << "," << ee_position(0) << "," << ee_position(1) << "," << ee_position(2) << endl;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
            double kp = 80;
            double kv = 20;
            double kvj = 14;
            
            Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
            robot->position(ee_position, link_name, pos_in_link);
            Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
            robot->linearVelocity(ee_velocity, link_name, pos_in_link);
            Eigen::Vector3d ee_w = Eigen::Vector3d::Zero();
            robot->angularVelocity(ee_w, link_name, pos_in_link);
            Eigen::MatrixXd jv(3,dof);
            Eigen::MatrixXd j0(3,dof);
            robot->Jv(jv, link_name, pos_in_link);
            robot->J_0(j0, link_name, pos_in_link);
            robot->taskInertiaMatrix(Lambda, j0);
            robot->dynConsistentInverseJacobian(J_bar, jv);
            robot->nullspaceMatrix(N,jv);
            Eigen::VectorXd g(dof);
            robot->gravityVector(g);
            
            Eigen::Matrix3d R;
            robot->rotation(R, link_name);
            
            Matrix3d Rd = Matrix3d::Zero(3, 3);
            Rd(0,0) = cos(M_PI/3); Rd(0,1) = 0; Rd(0,2) = sin(M_PI/3);
            Rd(1,0) = 0; Rd(1,1) = 1; Rd(1,2) = 0;
            Rd(2,0) = -sin(M_PI/3); Rd(2,1) = 0; Rd(2,2) = cos(M_PI/3);
            
            VectorXd x_desired(3);
            x_desired << 0.6, 0.3, 0.5;
            
            Eigen::VectorXd dphi(3);
            for (int i = 0; i < 3; i++) {
                dphi = dphi + R.col(i).cross(Rd.col(i));
            }
            dphi = -0.5 * dphi;
            
            MatrixXd A = kp * (x_desired - ee_position) - kv * ee_velocity;
            MatrixXd B = kp * (-dphi) - kv * ee_w;
            MatrixXd D(A.rows()+B.rows(), A.cols());
            D << A, B;
            
            MatrixXd F = Lambda * D;
            
            command_torques = j0.transpose()*F - N.transpose()*kvj*robot->_dq + g;
            myfile << time << "," << ee_position(0) << "," << ee_position(1) << "," << ee_position(2) << "," << dphi(0) << "," << dphi(1) << "," << dphi(2) << endl;
            
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
            double kp = 200;
            double kv = 20;
            double kpj = 50;
            double kvj = 14;
            
            VectorXd x_desired(3);
            x_desired << 0.6, 0.3, 0.4;
            
            VectorXd q_desired(dof);
            q_desired << 0, 0, 0, 0, 0, 0, 0;
            
            Eigen::Vector3d ee_position = Eigen::Vector3d::Zero();
            robot->position(ee_position, link_name, pos_in_link);
            Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
            robot->linearVelocity(ee_velocity, link_name, pos_in_link);
            Eigen::MatrixXd ee_jacobian(3,dof);
            robot->Jv(ee_jacobian, link_name, pos_in_link);
            robot->taskInertiaMatrix(Lambda, ee_jacobian);
            robot->dynConsistentInverseJacobian(J_bar, ee_jacobian);
            robot->nullspaceMatrix(N, ee_jacobian);
            
            Eigen::VectorXd g(dof);
            robot->gravityVector(g);
            
            VectorXd xd_desired = (kp/kv) * (x_desired-ee_position);
            double x = 0.1 / sqrt(pow(xd_desired(0),2) + pow(xd_desired(1),2) + pow(xd_desired(2),2));
            double v;
            if (abs(x) < 1) {
                v = x;
            } else {
                v = (x < 0) ? -1 : (x > 0);
            }
            MatrixXd F = Lambda * (-kv * (ee_velocity-v*xd_desired));
            command_torques = ee_jacobian.transpose() * F + N.transpose() * (-kpj * (robot->_q - q_desired) - kvj * robot->_dq) + g;
            myfile << time << "," << ee_position(0) << "," << ee_position(1) << "," << ee_position(2) << "," << ee_velocity(0) << "," << ee_velocity(1) << "," << ee_velocity(2) << endl;
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
