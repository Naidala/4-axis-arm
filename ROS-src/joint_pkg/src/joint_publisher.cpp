using namespace std;

#include <cmath>
#include <cstring>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <gcode_pkg/Position_EE.h>
#include <joint_pkg/Data_Joints_EE.h>

#include <tf/transform_broadcaster.h>
#include <iostream>
#include <visualization_msgs/Marker.h>

// Constants
#define a1 				0		// mm
#define a2 				240
#define a3				160
#define d1				50

#define Ts 				0.05
#define rate				1/Ts
#define K				2
#define vel				15		// mm/s
#define round_to_zero			1e-4

#define J2_MIN					-1	// rad
#define J2_MAX 					0.8   
#define J3_MIN					0 
#define J3_MAX					1.3   

#define DEG2RAD			M_PI/180

float th[] = {0*DEG2RAD, 90*DEG2RAD, -90*DEG2RAD};	// init q0
float EE[3] = {0.0};
float dq[3] = {0.0};
float des_E[3] = {0.0};
float old_E[3] = {0.0};
string conv = "";					// type of convergence

/************************/
/*********UTILITY********/
/************************/

void add_vector(float v1[], float v2[], float v[])
{
	v[0] = v1[0] + v2[0];
	v[1] = v1[1] + v2[1];
	v[2] = v1[2] + v2[2];
}


void err_vector(float v1[], float v2[], float v[])
{
	v[0] = v1[0] - v2[0];
	v[1] = v1[1] - v2[1];
	v[2] = v1[2] - v2[2];
}

/*
void copy_vector(float from[], float to[])
{
	to[0] = from[0];
	to[1] = from[1];
	to[2] = from[2];
}
*/

float norm(float v[])
{
	return sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));
}


void constant_product(float v[], float k, float tmp[])
{
	tmp[0]=v[0]*k;
	tmp[1]=v[1]*k;
	tmp[2]=v[2]*k;
}


/*****Inverse Matrix*****/
/******* J^(-1)*e *******/
void compute_dq(const float J[9], const float e[3], float dq[3])
{
  float A[9];
  int r1;
  int r2;
  int r3;
  float maxval;
  float a21;
  int rtemp;
  std::memcpy(&A[0], &J[0], 9U * sizeof(float));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(J[0]);
  a21 = std::abs(J[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(J[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = J[r2] / J[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r2] * A[r1 + 3];
  A[r3 + 3] -= A[r3] * A[r1 + 3];
  A[r2 + 6] -= A[r2] * A[r1 + 6];
  A[r3 + 6] -= A[r3] * A[r1 + 6];
  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  dq[1] = e[r2] - e[r1] * A[r2];
  dq[2] = (e[r3] - e[r1] * A[r3]) - dq[1] * A[r3 + 3];
  dq[2] /= A[r3 + 6];
  dq[0] = e[r1] - dq[2] * A[r1 + 6];
  dq[1] -= dq[2] * A[r2 + 6];
  dq[1] /= A[r2 + 3];
  dq[0] -= dq[1] * A[r1 + 3];
  dq[0] /= A[r1];
}


/************************/
/***Inverse Kinematics***/
/************************/

void direct_kynematics(float th[], float EE[])
{
	float L1[3] = {-a1*sin(th[0]), a1*cos(th[0]), d1};
	float L1_L2[3] = {-a2*cos(th[1])*sin(th[0]), a2*cos(th[1])*cos(th[0]), a2*sin(th[1])};
	float L2[3] = {0.0};
	add_vector(L1, L1_L2, L2);
	float L2_L3[3] = {-a3*cos(th[1]+th[2])*sin(th[0]), a3*cos(th[1]+th[2])*cos(th[0]), a3*sin(th[1]+th[2])};
	add_vector(L2, L2_L3, EE);
}


void jacobian(float th[], float Ja[])
{ 
	//ordinata per colonne
	Ja[0] = -cos(th[0])*(a1+a2*cos(th[1])+a3*cos(th[1]+th[2]));
	Ja[1] = -sin(th[0])*(a1+a2*cos(th[1])+a3*cos(th[1]+th[2]));
	Ja[2] = 0.0;
	Ja[3] = sin(th[0])*(a2*sin(th[1])+a3*sin(th[1]+th[2]));
	Ja[4] = -cos(th[0])*(a2*sin(th[1])+a3*sin(th[1]+th[2]));
	Ja[5] = a2*cos(th[1])+a3*cos(th[1]+th[2]);
	Ja[6] = sin(th[0])*a3*sin(th[1]+th[2]);
	Ja[7] = -cos(th[0])*a3*sin(th[1]+th[2]);
	Ja[8] = a3*cos(th[1]+th[2]);
}


void clik(float des_E[], float th[])
{
	
	float err[3] = {0.0};
	float Ja[9] = {0.0};

	direct_kynematics(th, EE);
	//printf("des = %f\t%f\t%f\n", des_E[0], des_E[1], des_E[2]);
	//printf("EE = %f\t%f\t%f\n", EE[0], EE[1], EE[2]);
	//printf("\n\n");

	jacobian(th, Ja);
	err_vector(des_E, EE, err);
	compute_dq(Ja, err, dq);
	constant_product(dq, K, dq);

	if ((dq[0] < round_to_zero)&&(dq[0] > -round_to_zero))
		dq[0] = 0.0;
	if ((dq[1] < round_to_zero)&&(dq[1] > -round_to_zero))
		dq[1] = 0.0;
	if ((dq[2] < round_to_zero)&&(dq[2] > -round_to_zero))
		dq[2] = 0.0;

	//printf("vel = %f\t%f\t%f\n", dq[0], dq[1], dq[2]);
	//printf("\n\n");
	float tmp[3] = {0.0};	// To discretize speed
	constant_product(dq, Ts, tmp);
	add_vector(th, tmp, th);
}


void path(float des_E[], float th[])
{

	float Ja[9] = {0.0};
	float S[3] = {0.0};

	direct_kynematics(th, EE);
	//printf("des = %f\t%f\t%f\n", des_E[0], des_E[1], des_E[2]);
	//printf("EE = %f\t%f\t%f\n", EE[0], EE[1], EE[2]);
	//printf("\n\n");
	//S[0] = des_E[0]-old_E[0]; 
	//S[1] = des_E[1]-old_E[1]; 
	//S[2] = des_E[2]-old_E[2];
	S[0] = des_E[0]-EE[0]; 
	S[1] = des_E[1]-EE[1]; 
	S[2] = des_E[2]-EE[2];

	jacobian(th, Ja);
	compute_dq(Ja, S, dq);
	constant_product(dq, vel/norm(S), dq);

	if ((dq[0] < round_to_zero)&&(dq[0] > -round_to_zero))
		dq[0] = 0.0;
	if ((dq[1] < round_to_zero)&&(dq[1] > -round_to_zero))
		dq[1] = 0.0;
	if ((dq[2] < round_to_zero)&&(dq[2] > -round_to_zero))
		dq[2] = 0.0;

	float tmp[3] = {0.0};	// To discretize speed
	constant_product(dq, Ts, tmp);	
	add_vector(th, tmp, th);
}



/****Subscriber Callback Function****/
void coord_Callback(const gcode_pkg::Position_EE& msg)
{
	//printf("Coordinate received!\n");

/*	float new_E[3] = {0.0};
	new_E[0] = msg.p.x;
	new_E[1] = msg.p.y;
	new_E[2] = msg.p.z;
	conv = msg.convergence;

	if ((conv == "path") && ((new_E[0] != des_E[0])||(new_E[1] != des_E[1])||(new_E[2] != des_E[2])))
		copy_vector(des_E, old_E);
	copy_vector(new_E, des_E);*/

	des_E[0] = msg.p.x;
	des_E[1] = msg.p.y;
	des_E[2] = msg.p.z;
	conv = msg.convergence;
}



int main(int argc, char** argv) {

    ros::init(argc, argv, "joint_publisher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("coordinates", 1, coord_Callback);
    ros::Publisher data_pub  = n.advertise<joint_pkg::Data_Joints_EE>("joint_states", 1);
    ros::Rate loop_rate(rate);

    joint_pkg::Data_Joints_EE data;

    while (ros::ok()) {

        data.joint_state.header.stamp = ros::Time::now();
        data.joint_state.name.resize(4);
        data.joint_state.position.resize(4);
        data.joint_state.velocity.resize(4);

	if (conv == "clik")
		clik(des_E, th);
	else if (conv == "path")
		path(des_E, th);
        
       	data.joint_state.name[0] ="joint_1";
        data.joint_state.position[0] = th[0];
	data.joint_state.velocity[0] = dq[0];

        data.joint_state.name[1] ="joint_2";
        data.joint_state.position[1] = th[1];
	data.joint_state.velocity[1] = dq[1];

       	data.joint_state.name[2] ="joint_3";
       	data.joint_state.position[2] = th[2];
	data.joint_state.velocity[2] = dq[2];

	data.joint_state.name[3] ="joint_ee";
       	data.joint_state.position[3] = th[2] - th[1];
	data.joint_state.velocity[3] = dq[2] - dq[1];

	data.ee.x = EE[0];
	data.ee.y = EE[1];
	data.ee.z = EE[2];

        data_pub.publish(data);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
