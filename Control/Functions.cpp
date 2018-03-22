#include "Functions.h"
#include "Global.h"
#include <random>
#include <stdio.h>
#include <stdlib.h>
//#include "SpaceMouse.h"
using namespace std;
using namespace CxUtils;
//using namespace arbitraryNamspace;
bool fu_flag = false;
void printPos(float* pos, int locx, int locy);
default_random_engine generator;
normal_distribution<double>distribution(0, 1);
//=============================================================================
// Maths functions.
//=============================================================================
// This gets the transformation gripper to world using joint angles.
Matrix<4, 4> GWj_Transformation(float *joint)
{
	float L1 = 99.2, L2 = 396.7, L3 = 323.4, L4 = 0;
	float angle1 = 0, angle2 = (-M_PI / 2), angle3 = 0, angle4 = (M_PI / 2), angle5 = (-M_PI / 2), angle6 = (M_PI / 2);
	for (int i = 0; i < 8; i++)
	{
		joint[i] = joint[i] * M_PI / 180;
	}

	Matrix<4, 4> A1;
	A1 = cos(joint[0]), (-1)*sin(joint[0]), 0, 0,
		sin(joint[0])*cos(angle1), cos(joint[0])*cos(angle1), (-1)*sin(angle1), 0,
		sin(joint[0])*sin(angle1), cos(joint[0])*sin(angle1), cos(angle1), 0,
		0, 0, 0, 1;

	Matrix<4, 4> A2;
	A2 = cos(joint[1]), (-1)*sin(joint[1]), 0, 0,
		0, 0, -sin(angle2), 0,
		sin(joint[1])*sin(angle2), cos(joint[1])*sin(angle2), 0, 0,
		0, 0, 0, 1;

	Matrix<4, 4> A3;
	A3 = cos(joint[2]), -1 * sin(joint[2]), 0, L2,
		sin(joint[2])*cos(angle3), cos(joint[2])*cos(angle3), -1 * sin(angle3), -1 * L1*sin(angle3),
		sin(joint[2])*sin(angle3), cos(joint[2])*sin(angle3), cos(angle3), L1*cos(angle3),
		0, 0, 0, 1;

	Matrix<4, 4> A4;
	A4 = cos(joint[3]), -1 * sin(joint[3]), 0, 0,
		0, 0, -sin(angle4), -1 * L3*sin(angle4),
		sin(joint[3])*sin(angle4), cos(joint[3])*sin(angle4), 0, 0,
		0, 0, 0, 1;

	Matrix<4, 4> A5;
	A5 = cos(joint[4]), -1 * sin(joint[4]), 0, 0,
		0, 0, -1 * sin(angle5), 0,
		sin(joint[4])*sin(angle5), cos(joint[4])*sin(angle5), 0, 0,
		0, 0, 0, 1;

	Matrix<4, 4> A6;
	A6 = cos(joint[5]), -1 * sin(joint[5]), 0, 0,
		0, 0, -1 * sin(angle6), -1 * L4*sin(angle6),
		sin(joint[5])*sin(angle6), cos(joint[5])*sin(angle6), 0, 0,
		0, 0, 0, 1;

	Matrix<4, 4> T06;
	T06 = A1 * A2 * A3 * A4 * A5 * A6;

	return T06;
}

// Get Rc2w rotational transform.
Matrix<3, 3> C2W_transform(float *position)
{
	// get Rc2w transform from YPR
	// 010308 - KIM
	Matrix<3, 3> Ree2w, Rc2ee, Rc2w;
	Ree2w = EE2C_transform(position);
	Rc2ee = 0, 0, 1, 1, 0, 0, 0, 1, 0;
	Rc2w = Ree2w * Rc2ee;

	ColumnVector<3> Rm;
	Rm = sqrt(Rc2w(1, 1) * Rc2w(1, 1) + Rc2w(2, 1) * Rc2w(2, 1) + Rc2w(3, 1) * Rc2w(3, 1)),
		sqrt(Rc2w(1, 2) * Rc2w(1, 2) + Rc2w(2, 2) * Rc2w(2, 2) + Rc2w(3, 2) * Rc2w(3, 2)),
		sqrt(Rc2w(1, 3) * Rc2w(1, 3) + Rc2w(2, 3) * Rc2w(2, 3) + Rc2w(3, 3) * Rc2w(3, 3));

	Rc2w = Rc2w(1, 1) / Rm(1), Rc2w(1, 2) / Rm(2), Rc2w(1, 3) / Rm(3),
		Rc2w(2, 1) / Rm(1), Rc2w(2, 2) / Rm(2), Rc2w(2, 3) / Rm(3),
		Rc2w(3, 1) / Rm(1), Rc2w(3, 2) / Rm(2), Rc2w(3, 3) / Rm(3);

	return Rc2w;
}

// Get Ree2w rotational transform.
Matrix<3, 3> EE2C_transform(float *position)
{
	Matrix<3, 3> Ry, Rp, Rr, Rc2w;

	register float c_y = position[3] * M_PI / 180.0f;
	register float c_p = position[4] * M_PI / 180.0f;
	register float c_r = position[5] * M_PI / 180.0f;
	Ry = cos(c_y), -sin(c_y), 0, sin(c_y), cos(c_y), 0, 0, 0, 1;
	Rp = cos(c_p), 0, -sin(c_p), 0, 1, 0, sin(c_p), 0, cos(c_p);
	Rr = 1, 0, 0, 0, cos(c_r), sin(c_r), 0, -sin(c_r), cos(c_r);
	Rc2w = Ry * Rp * Rr;




	ColumnVector<3> Rm;
	Rm = sqrt(Rc2w(1, 1) * Rc2w(1, 1) + Rc2w(2, 1) * Rc2w(2, 1) + Rc2w(3, 1) * Rc2w(3, 1)),
		sqrt(Rc2w(1, 2) * Rc2w(1, 2) + Rc2w(2, 2) * Rc2w(2, 2) + Rc2w(3, 2) * Rc2w(3, 2)),
		sqrt(Rc2w(1, 3) * Rc2w(1, 3) + Rc2w(2, 3) * Rc2w(2, 3) + Rc2w(3, 3) * Rc2w(3, 3));

	Rc2w = Rc2w(1, 1) / Rm(1), Rc2w(1, 2) / Rm(2), Rc2w(1, 3) / Rm(3),
		Rc2w(2, 1) / Rm(1), Rc2w(2, 2) / Rm(2), Rc2w(2, 3) / Rm(3),
		Rc2w(3, 1) / Rm(1), Rc2w(3, 2) / Rm(2), Rc2w(3, 3) / Rm(3);

	return Rc2w;
}


// Get Ree2w rotational transform.  zc test
Matrix<3, 3> EE2w_transform3(float *position)
{
	Matrix<3, 3> Ry, Rp, Rr, Rc2w;

	register float c_y = position[3] * M_PI / 180.0f;
	register float c_p = -position[4] * M_PI / 180.0f;
	register float c_r = sign(position[5])*(180 - abs(position[5])) * M_PI / 180.0f;
	Ry = cos(c_y), -sin(c_y), 0, sin(c_y), cos(c_y), 0, 0, 0, 1;
	Rp = cos(c_p), 0, sin(c_p), 0, 1, 0, -sin(c_p), 0, cos(c_p);
	Rr = 1, 0, 0, 0, cos(c_r), -sin(c_r), 0, sin(c_r), cos(c_r);
	Rc2w = Ry * Rp * Rr;

	ColumnVector<3> Rm;
	Rm = sqrt(Rc2w(1, 1) * Rc2w(1, 1) + Rc2w(2, 1) * Rc2w(2, 1) + Rc2w(3, 1) * Rc2w(3, 1)),
		sqrt(Rc2w(1, 2) * Rc2w(1, 2) + Rc2w(2, 2) * Rc2w(2, 2) + Rc2w(3, 2) * Rc2w(3, 2)),
		sqrt(Rc2w(1, 3) * Rc2w(1, 3) + Rc2w(2, 3) * Rc2w(2, 3) + Rc2w(3, 3) * Rc2w(3, 3));

	Rc2w = Rc2w(1, 1) / Rm(1), Rc2w(1, 2) / Rm(2), Rc2w(1, 3) / Rm(3),
		Rc2w(2, 1) / Rm(1), Rc2w(2, 2) / Rm(2), Rc2w(2, 3) / Rm(3),
		Rc2w(3, 1) / Rm(1), Rc2w(3, 2) / Rm(2), Rc2w(3, 3) / Rm(3);

	return Rc2w;
}
// Get Ree2w rotational transform.  zc test
Matrix<3, 3> EE2w_transform2(float *position)
{
	Matrix<3, 3> Ry, Rp, Rr, Rc2w;

	register float c_y = position[3] * M_PI / 180.0f;
	register float c_p = position[4] * M_PI / 180.0f;
	register float c_r = -position[5] * M_PI / 180.0f;
	Ry = cos(c_y), -sin(c_y), 0, sin(c_y), cos(c_y), 0, 0, 0, 1;
	Rp = cos(c_p), 0, -sin(c_p), 0, 1, 0, sin(c_p), 0, cos(c_p);
	Rr = 1, 0, 0, 0, cos(c_r), sin(c_r), 0, -sin(c_r), cos(c_r);
	Rc2w = Ry * Rp * Rr;

	//ColumnVector<3> Rm;
	//Rm = sqrt(Rc2w(1, 1) * Rc2w(1, 1) + Rc2w(2, 1) * Rc2w(2, 1) + Rc2w(3, 1) * Rc2w(3, 1)),
	//	sqrt(Rc2w(1, 2) * Rc2w(1, 2) + Rc2w(2, 2) * Rc2w(2, 2) + Rc2w(3, 2) * Rc2w(3, 2)),
	//	sqrt(Rc2w(1, 3) * Rc2w(1, 3) + Rc2w(2, 3) * Rc2w(2, 3) + Rc2w(3, 3) * Rc2w(3, 3));

	//Rc2w = Rc2w(1, 1) / Rm(1), Rc2w(1, 2) / Rm(2), Rc2w(1, 3) / Rm(3),
	//	Rc2w(2, 1) / Rm(1), Rc2w(2, 2) / Rm(2), Rc2w(2, 3) / Rm(3),
	//	Rc2w(3, 1) / Rm(1), Rc2w(3, 2) / Rm(2), Rc2w(3, 3) / Rm(3);

	return Rc2w;
}

// Get Rc2w rotational transform for yaw angle only - for relative L/R/F/B.
Matrix<3, 3> C2W_transform2(float *position)
{
	// get Rc2w transform from YPR
	// 010308 - KIM
	Matrix<3, 3> Ree2w, Rc2ee, Rc2w;
	Ree2w = EE2C_transform2(position);
	Rc2ee = 0, 0, 1, 1, 0, 0, 0, 1, 0;
	Rc2w = Ree2w * Rc2ee;

	ColumnVector<3> Rm;
	Rm = sqrt(Rc2w(1, 1) * Rc2w(1, 1) + Rc2w(2, 1) * Rc2w(2, 1) + Rc2w(3, 1) * Rc2w(3, 1)),
		sqrt(Rc2w(1, 2) * Rc2w(1, 2) + Rc2w(2, 2) * Rc2w(2, 2) + Rc2w(3, 2) * Rc2w(3, 2)),
		sqrt(Rc2w(1, 3) * Rc2w(1, 3) + Rc2w(2, 3) * Rc2w(2, 3) + Rc2w(3, 3) * Rc2w(3, 3));

	Rc2w = Rc2w(1, 1) / Rm(1), Rc2w(1, 2) / Rm(2), Rc2w(1, 3) / Rm(3),
		Rc2w(2, 1) / Rm(1), Rc2w(2, 2) / Rm(2), Rc2w(2, 3) / Rm(3),
		Rc2w(3, 1) / Rm(1), Rc2w(3, 2) / Rm(2), Rc2w(3, 3) / Rm(3);

	return Rc2w;
}

//Ree2w for just yaw angle - used for relative L/R/F/B
Matrix<3, 3> EE2C_transform2(float *position)
{
	Matrix<3, 3> Ry, Rp, Rr, Rc2w;

	register float c_y = position[3] * M_PI / 180.0f;

	Ry = cos(c_y), -sin(c_y), 0, sin(c_y), cos(c_y), 0, 0, 0, 1;
	Rp = 1, 0, 0, 0, 1, 0, 0, 0, 1;
	Rr = 1, 0, 0, 0, -1, 0, 0, 0, -1;
	Rc2w = Ry * Rp * Rr;

	ColumnVector<3> Rm;
	Rm = sqrt(Rc2w(1, 1) * Rc2w(1, 1) + Rc2w(2, 1) * Rc2w(2, 1) + Rc2w(3, 1) * Rc2w(3, 1)),
		sqrt(Rc2w(1, 2) * Rc2w(1, 2) + Rc2w(2, 2) * Rc2w(2, 2) + Rc2w(3, 2) * Rc2w(3, 2)),
		sqrt(Rc2w(1, 3) * Rc2w(1, 3) + Rc2w(2, 3) * Rc2w(2, 3) + Rc2w(3, 3) * Rc2w(3, 3));

	Rc2w = Rc2w(1, 1) / Rm(1), Rc2w(1, 2) / Rm(2), Rc2w(1, 3) / Rm(3),
		Rc2w(2, 1) / Rm(1), Rc2w(2, 2) / Rm(2), Rc2w(2, 3) / Rm(3),
		Rc2w(3, 1) / Rm(1), Rc2w(3, 2) / Rm(2), Rc2w(3, 3) / Rm(3);

	return Rc2w;
}

// Normalize rotation matrix.
Matrix<3, 3> NormalizeRotationMatrix(Matrix<3, 3> Rx)
{
	Matrix<3, 3> Ry;
	ColumnVector<3> Rm;

	Rm = sqrt(Rx(1, 1) * Rx(1, 1) + Rx(2, 1) * Rx(2, 1) + Rx(3, 1) * Rx(3, 1)),
		sqrt(Rx(1, 2) * Rx(1, 2) + Rx(2, 2) * Rx(2, 2) + Rx(3, 2) * Rx(3, 2)),
		sqrt(Rx(1, 3) * Rx(1, 3) + Rx(2, 3) * Rx(2, 3) + Rx(3, 3) * Rx(3, 3));

	Ry = Rx(1, 1) / Rm(1), Rx(1, 2) / Rm(2), Rx(1, 3) / Rm(3),
		Rx(2, 1) / Rm(1), Rx(2, 2) / Rm(2), Rx(2, 3) / Rm(3),
		Rx(3, 1) / Rm(1), Rx(3, 2) / Rm(2), Rx(3, 3) / Rm(3);

	return Ry;
}
/////____________________Mushtaq _______
  //////// finding the distance between the camera and link3 functions
/////////////////////////////////////////////////////////////////////////
ColumnVector<3> inverse_kinematics_MANUS_q1q2q3(float x, float y, float z)
{
	float L2 = 396.7, d2 = 99.2, d4 = 323.4;
	float q1, q2, q3, A, B, s1, c1, s3, c3;
	q1 = atan2(y, x) - atan2(d2, -sqrt(x*x + y * y - d2 * d2));
	s1 = sin(q1);
	c1 = cos(q1);
	s3 = ((c1*x + s1 * y)*(c1*x + s1 * y) + z * z - d4 * d4 - L2 * L2) / (2 * d4*L2);
	c3 = sqrt(1 - s3 * s3);
	q3 = atan2(s3, c3);
	s3 = sin(q3);
	c3 = cos(q3);

	A = (c1*x + s1 * y)*d4*c3 - (d4*s3 + L2)*z;
	B = d4 * c3*z + (d4*s3 + L2)*(c1*x + s1 * y);
	q2 = atan2(A, B);

	ColumnVector<3> q;
	q = q1, q2, q3;

	return q;
}
// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2


float dist3D_Segment_to_Segment(ColumnVector<3> P1, ColumnVector<3> P2, ColumnVector<3> P3, ColumnVector<3> P4)
{
	float SMALL_NUM = 0.00000001;
	ColumnVector<3> u, v, w;
	u = P2 - P1;
	v = P4 - P3;
	w = P1 - P3;
	float    a = dotProduct(u, u);         // always >= 0
	float    b = dotProduct(u, v);
	float    c = dotProduct(v, v);         // always >= 0
	float    d = dotProduct(u, w);
	float    e = dotProduct(v, w);
	float    D = a * c - b * b;        // always >= 0
	float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
	float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

								   // compute the line parameters of the two closest points
	if (D < SMALL_NUM) { // the lines are almost parallel
		sN = 0.0;         // force using point P0 on segment S1
		sD = 1.0;         // to prevent possible division by 0.0 later
		tN = e;
		tD = c;
	}
	else {                 // get the closest points on the infinite lines
		sN = (b*e - c * d);
		tN = (a*e - b * d);
		if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
			sN = 0.0;
			tN = e;
			tD = c;
		}
		else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
			sN = sD;
			tN = e + b;
			tD = c;
		}
	}

	if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
		tN = 0.0;
		// recompute sc for this edge
		if (-d < 0.0)
			sN = 0.0;
		else if (-d > a)
			sN = sD;
		else {
			sN = -d;
			sD = a;
		}
	}
	else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
		tN = tD;
		// recompute sc for this edge
		if ((-d + b) < 0.0)
			sN = 0;
		else if ((-d + b) > a)
			sN = sD;
		else {
			sN = (-d + b);
			sD = a;
		}
	}
	// finally do the division to get sc and tc
	sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
	tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

	// get the difference of the two closest points
	ColumnVector<3> dP;
	dP = w(1) + sc * u(1) - tc * v(1), w(2) + sc * u(2) - tc * v(2), w(3) + sc * u(3) - tc * v(3);  // =  S1(sc) - S2(tc)

	float dist = sqrt(dotProduct(dP, dP));
	return dist;   // return the closest distance
}

ColumnVector<3> link3_1st_end_postion(float q1, float q2, float q3)
{
	float L2 = 396.7, d2 = 99.2, d4 = 323.4;
	Matrix<4, 4> A1;
	A1 = cos(q1), -sin(q1), 0, 0, sin(q1), cos(q1), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

	Matrix<4, 4> A2;
	A2 = cos(q2), -sin(q2), 0, 0, 0, 0, 1, 0, -sin(q2), -cos(q2), 0, 0, 0, 0, 0, 1;

	Matrix<4, 4> A3;
	A3 = cos(q3), -sin(q3), 0, L2, sin(q3), cos(q3), 0, 0, 0, 0, 1, d2, 0, 0, 0, 1;


	Matrix<4, 4> T03;
	T03 = A1 * A2*A3;

	ColumnVector<3> P1;
	P1 = T03(1, 4), T03(2, 4), T03(3, 4);
	return P1;
}

float DistanceBetween_Camera_Link3(float *position)
{
	float x = position[0];
	float y = position[1];
	float z = position[2];

	float q1, q2, q3;
	float dist;
	int collision = 0;
	float r = 55 / 2;     //radius of link3
	ColumnVector<3> p1, p2, p3_c, p4_c, p3, p4, Q;
	p3_c = 82, -62, 82;  //  first end point coordinate of camera edge in end-effector frame
	p4_c = -95, -62, 82; // second end point coordinate of camera edge in end-effector frame
	Q = inverse_kinematics_MANUS_q1q2q3(x, y, z);
	q1 = Q(1);
	q2 = Q(2);
	q3 = Q(3);
	p1 = link3_1st_end_postion(q1, q2, q3); //first end point coordinate of link3 arm in base frame
	p2 = x, y, z;     //second end point coordinate of link3 arm in basec frame
	Matrix<3, 3> R06;  // rotation matrix from the end-effector frame to the base frame
	R06 = C2W_transform(position);
	p3 = R06 * p3_c + p2;
	p4 = R06 * p4_c + p2;

	dist = dist3D_Segment_to_Segment(p1, p2, p3, p4);
	if ((dist < r) || (dist == r)) {
		collision = 1;
	}

	else {
		collision = 0;
	}


	return dist;

}

///////////////////////////

//=============================================================================
// Communication functions.
//=============================================================================
// Communication thread.
void sthread_CAN(void *arg)
{
	// Server configurations.
	char host_str[256];
	ifstream host_file;
	host_file.open("C:\\MANUS\\CommonSpace\\Setting\\IpAddress.txt");
	host_file >> host_str;
	host_file.close();

	// Control log file.
	ofstream command_signal;
	sprintf(user_str, "%scontrol.log", debug_path);
	command_signal.open(user_str);

	packet_CAN.SetByteOrder(CX_PACKET_BIG_ENDIAN);
	if (!client_CAN.InitializeSocket(host_str, PORT_CAN))
	{
		cout << "[Error!]: Can not connect to server!" << endl;
		ShowStatus("STAT: server error");
		cin.get();
		return;
	}
	else
	{
		cout << "[Info]: Connect to server!" << endl;
	}

	while (client_CAN.Recv(packet_CAN, BUFFER_SIZE, 0) > 0 && !grasp_control)
	{
		packet_CAN.Read(source, 0);
		packet_CAN.Read(destination, 1);
		packet_CAN.Read(command, 2);

		if (command == WRITE_GO_VEL2)
			goto Write_Go_Vel2;

		if (destination == CAN)
		{
			if (source == GUI /*|| source == SLP*/)
			{
				if (command == GUI_COMMAND)
				{
					char val;
					packet_CAN.Read(val, 3);

					if (val == ' ')
						ResetAll();
					else
					{
						if (fu_flag) {

							int num = rand() % 17;
							val = reliable(num, val);
						}
						myRcv.key.writeLock();
						myRcv.command = val;
						myRcv.key.unlock();
					}

					if (!block_all_motions && (val != ' '))
					{
						adjust_queue_index(+1);
						for (int i = 0; i < 8; ++i)
							good_pos[pos_queue_index][i] = pos[i];
					}
				}
				else if (command == READ_POS)
				{
					packet_CAN.Clear();
					packet_CAN.Write(CAN, 0);
					packet_CAN.Write(GUI, 1);
					packet_CAN.Write(READ_POS_DONE, 2);
					packet_CAN.SetWritePos(3);

					char encoder_reading[256];
					sprintf(encoder_reading, "%1c%6.2f%1c%6.2f%1c%6.2f%1c%6.2f%1c%6.2f%1c%6.2f",
						((pos[0] > 0) ? '+' : '-'), fabs(pos[0]), ((pos[1] > 0) ? '+' : '-'), fabs(pos[1]),
						((pos[2] > 0) ? '+' : '-'), fabs(pos[2]), ((pos[3] > 0) ? '+' : '-'), fabs(pos[3]),
						((pos[4] > 0) ? '+' : '-'), fabs(pos[4]), ((pos[5] > 0) ? '+' : '-'), fabs(pos[5]));

					for (int i = 0; i < 42; ++i)
						packet_CAN.Write(encoder_reading[i], 3 + i);

					client_CAN.Send(packet_CAN);
					packet_CAN.Clear();
				}
				else if (command == FINAL_GRAB)
				{
					speed_mode = 0;
					UpdatePos(pos);
					SendCommand(CAN, GUI, UPDATE_SPEED, 0);
					for (int i = 0; i < 9; ++i)
						rpos[i] = pos[i];

					int packet_pos = 3;

					for (int i = 0; i < 1; ++i)
						packet_pos += packet_CAN.Read(user_str[i], packet_pos);

					user_str[1] = '\0';
					gripper_closedness = (int)atof((const char *)user_str);
					myRcv.key.writeLock();
					myRcv.command = 'j';
					myRcv.key.unlock();
					SendCommand(CAN, FSR, FSR_START, EMPTY_MESSAGE);
				}
				else if ((command == FSR_END) || (command == TP_END))
				{
					ResetAll();
					spaceMouse_stop = true;
				}
				else if ((command == AS_ON))
				{
					assistant_flag = true;
					move_arm = 0;
					oneclick_mode = 1;
				}
				else if ((command == AS_OFF))
				{
					assistant_flag = false;
					oneclick_mode = 0;
					move_arm = -1;
				}
			}
			else if (source == SPC)
			{
				if (command == SPC_COMMAND)
				{
					char val;
					packet_CAN.Read(val, 3);
					if (val == ' ')
						ResetAll();
					else {
						myRcv.key.writeLock();
						myRcv.command = val;
						myRcv.key.unlock();
					}
				}
				else if (command == TTS_SPEAK_DONE)
					tts_in_progress = false;
			}
			else if (source == FSR)
			{
				if ((command == FSR_END) || (command == TP_END))
				{
					ResetAll();
					spaceMouse_stop = true;
				}
			}
			else if (source == SLP) {
				if (command == SLIP) {
					ResetAll();
					myRcv.key.writeLock();
					myRcv.command = 'j';
					myRcv.key.unlock();
				}
			}
			else if (source == VIS)
			{
				if (command == STOP_MOTION)
				{
					ResetAll();
				}
				else if (command == GUI_COMMAND)
				{
					char val;
					packet_CAN.Read(val, 3);

					if (val == ' ')
						ResetAll();
					else {
						myRcv.key.writeLock();
						myRcv.command = val;
						myRcv.key.unlock();
					}
				}
				else if (command == FINAL_GRAB)
				{
					speed_mode = 0;
					UpdatePos(pos);
					SendCommand(CAN, GUI, UPDATE_SPEED, 0);

					int packet_pos = 3;

					for (int i = 0; i < 3; i++)
						packet_pos += packet_CAN.Read(user_str[i], packet_pos);

					user_str[3] = '\0';
					gripper_closedness = (int)atof((const char *)user_str);

					myRcv.key.writeLock();
					myRcv.command = 'j';
					myRcv.key.unlock();

					SendCommand(CAN, FSR, FSR_START, EMPTY_MESSAGE);
				}
				else if (command == FINAL_APPROACH)
				{
					speed_mode = 1;
					myRcv.key.writeLock();
					myRcv.command = 'n';
					myRcv.key.unlock();
				}
				// Retreat movement before grab
				else if (command == 'm')
				{
					speed_mode = 0;
					myRcv.key.writeLock();
					myRcv.command = 'm';
					myRcv.key.unlock();
				}
				// Move left before approach
				else if (command == 'w')
				{
					speed_mode = 0;
					myRcv.key.writeLock();
					myRcv.command = 'w';
					myRcv.key.unlock();
				}
				else if (command == 'e')
				{
					speed_mode = 0;
					myRcv.key.writeLock();
					myRcv.command = 'e';
					myRcv.key.unlock();
				}
				// Zhao Wang end 08/06/2010
				else if (command == SET_SPEED_MODE)
				{
					int packet_pos = 3;

					for (int i = 0; i < 3; ++i)
						packet_pos += packet_CAN.Read(user_str[i], packet_pos);

					user_str[3] = '\0';
					speed_mode = (int)atof((const char *)user_str);

					if (speed_mode > 4)
						speed_mode = 4;
					else if (speed_mode < 0)
						speed_mode = 0;
				}
				else if (command == READ_POS)
				{
					packet_CAN.Clear();
					packet_CAN.Write(CAN, 0);
					packet_CAN.Write(VIS, 1);
					packet_CAN.Write(READ_POS_DONE, 2);
					packet_CAN.SetWritePos(3);
					for (int i = 0; i < 6; ++i)
						packet_CAN.Write((const float)pos[i]);
					client_CAN.Send(packet_CAN);
					packet_CAN.Clear();
				}
				else if (command == READ_POS_SPEED)
				{
					packet_CAN.Clear();
					packet_CAN.Write(CAN, 0);
					packet_CAN.Write(VIS, 1);
					packet_CAN.Write(READ_POS_SPEED_DONE, 2);
					packet_CAN.SetWritePos(3);
					for (int i = 0; i < 6; ++i)
						packet_CAN.Write((const float)pos[i]);
					for (int i = 0; i < 6; ++i)
						packet_CAN.Write((const float)speed[i + 1]);
					client_CAN.Send(packet_CAN);
					packet_CAN.Clear();
				}
				else if (command == WRITE_SET_POS)	//
				{
					for (int i = 0; i < 6; ++i)
						packet_CAN.Read(pd[i], 3 + i * sizeof(float));
					packet_CAN.Read(speed_factor, 3 + 6 * sizeof(float));

					set_points_control = true;

					if (myRcv.command == NULL)
					{
						myRcv.key.writeLock();
						myRcv.command = 'l';
						myRcv.key.unlock();
					}
				}
				else if (command == WRITE_GO_VEL)	// Fine Motion
				{
					//if (debug_on) 
					//	timing << "38 " << TimeCheck() << "\n";

					Matrix<3, 1> Vc, Vw, Ww, Wc, Wypr;
					Matrix<3, 3> Rc2w, Rxyz2ypr, nRxyz2ypr;
					Matrix<3, 3> Ree2w, Rypr2xyz, nRypr2xyz;
					Matrix<3, 1> We;
					Matrix<3, 3> Rc2ee, Rc2eep, nRc2eep;
					ColumnVector<3> Dypr;

					if (!init_fine_motion)
					{
						for (int i = 0; i < 8; ++i)
							opos[i] = pos[i];
						potime = TimeCheck();
						for (int i = 0; i < 3; ++i)
							oypr[i] = 0.0f;
						init_fine_motion = true;
					}

					float tmp;

					int packet_pos = 3;
					float comm_sum = 0.0f;

					for (int i = 0; i < 3; ++i)
					{
						packet_pos += packet_CAN.Read(tmp, packet_pos);
						Vc(i + 1, 1) = tmp;
						comm_sum += fabs(tmp);
					}

					for (int i = 0; i < 3; ++i)
					{
						packet_pos += packet_CAN.Read(tmp, packet_pos);
						Wc(i + 1, 1) = tmp;
						comm_sum += fabs(tmp);
					}

					if (comm_sum == 0.0f)
					{
						// Stop the robot.
						myRcv.key.writeLock();
						myRcv.command = ' ';
						myRcv.key.unlock();
					}
					else
					{
						command_signal << TimeCheck() << " ";

						for (int i = 0; i < 6; ++i)
							command_signal << pos[i] << " ";

						float control_input;
						float w_lim = 10.0f, v_lim = 100.0f;
						float r2d_const = 180.0f / M_PI;
						float dt, dy, dp, dr;

						Rc2w = C2W_transform(pos);
						Ree2w = EE2C_transform(pos);

						Rypr2xyz = 0, Ree2w(2, 1), Ree2w(1, 1),
							0, -Ree2w(1, 1), Ree2w(2, 1),
							1, 0, Ree2w(3, 1);

						// WC -> YPRC [rad]
						Wypr = inverse(Rypr2xyz) * Rc2w * Wc;

						// Wc & Ww
						command_signal << transpose(Wc) << " " << transpose(Ww) << " " << transpose(Wypr) << " ";

						// Limit angular velocity.
						for (int i = 0; i < 3; ++i)
						{
							// [rad] -> [deg]
							control_input = Wypr(i + 1, 1) * r2d_const;
							Wypr(i + 1, 1) = (fabs(control_input) > w_lim) ? (float)(sign(control_input) * w_lim) : (float)control_input;
						}

						pctime = TimeCheck();
						dt = fabs((float)(pctime - potime));

						dy = (pos[3] - opos[3]);
						dp = (pos[4] - opos[4]);

						float tmp1 = (180.0f - pos[5]) - (180.0f - opos[5]);
						dr = (tmp1 >= 0.0f) ? ((tmp1 <= 180.0f) ? tmp1 : tmp1 - 360.0f) : ((tmp1 > -180.0f) ? tmp1 : 360.0f + tmp1);

						cypr[0] = 20.0f*dy / dt;	cypr[1] = 20.0f*dp / dt;	cypr[2] = 20.0f*dr / dt;

						for (int i = 0; i < 3; ++i) {
							dypr[i] = oypr[i] + (cypr[i] - oypr[i]) * (dt / 1000.0f) * 5.0f;
							Dypr(i + 1) = (fabs(dypr[i]) > 0) ? dypr[i] : 0.0f;
						}

						potime = pctime;
						for (int i = 0; i < 3; ++i) {
							opos[i + 3] = pos[i + 3];
							oypr[i] = dypr[i];
						}

						We = transpose(Ree2w) * Rypr2xyz * Dypr;	// [deg]

						Matrix<3, 1> Rde;
						// Cross product of We & d_c_ee.	
						Rde = We(2, 1) * d_c_ee(3, 1) - We(3, 1) * d_c_ee(2, 1),
							We(3, 1) * d_c_ee(1, 1) - We(1, 1) * d_c_ee(3, 1),
							We(1, 1) * d_c_ee(2, 1) - We(2, 1) * d_c_ee(1, 1);

						for (int i = 0; i < 3; ++i)	Rde(i + 1, 1) = Rde(i + 1, 1);

						Matrix<3, 1> WeRde;
						WeRde = Ree2w * Rde;

						Vw = Rc2w * Vc - WeRde;

						// Ww ~ Vw
						command_signal << transpose(Wypr) << " "
							<< transpose(We) << " " << transpose(Rde) << " "
							<< transpose(WeRde) << " " << transpose(Vc) << " "
							<< transpose(Vw) << " ";

						// Limit linear velocity.
						for (int i = 0; i < 3; ++i) {
							control_input = Vw(i + 1, 1);
							Vw(i + 1, 1) = (fabs(control_input) > v_lim) ? (float)(sign(control_input)*v_lim) : (float)control_input;
						}

						// Set speed variables of MANUS ARM.
						for (int i = 0; i < 3; i++)
							speed[i + 1] = (int)Vw(i + 1, 1);

						for (int i = 0; i < 3; i++)
							speed[i + 4] = (int)Wypr(i + 1, 1);

						for (int i = 0; i < 6; i++)
							command_signal << speed[i + 1] << " ";

						command_signal << endl;
					}

					new_status = true;
					//if (debug_on) {
					//	timing << "39 " << TimeCheck() 
					//		<< " " << transpose(Vc) << " " << transpose(Wc) 
					//		<< " " << transpose(Vw) << " " << transpose(Wypr) 
					//		<< " " << pos[0] << " " << pos[1] << " " << pos[2] 
					//	<< " " << pos[3] << " " << pos[4] << " " << pos[5] 
					//	<< "\n";
					//}
				}
				else if (command == WRITE_GO_VEL2)	//	Gross Motion.
				{
				Write_Go_Vel2:
					int packet_pos = 3;
					float tmp;
					for (int i = 0; i < 6; ++i)
					{
						packet_pos += packet_CAN.Read(tmp, packet_pos);
						pd[i] = tmp;
					}
					start_time_set_point_vel = TimeCheck();
					set_point_vel = true;		// look inside pdcontrol2()

					if (myRcv.command == NULL)
					{
						myRcv.key.writeLock();
						myRcv.command = 'l';
						myRcv.key.unlock();
					}
				}
			}
		}
	}
	command_signal.close();
	return;
}

// Send status info through socket communication.
void ClientSocketComm(unsigned char src, unsigned char dst, unsigned char comm, float* param)
{
	Packet packet;

	packet.Clear();
	packet.Write(src, 0);
	packet.Write(dst, 1);
	packet.Write(comm, 2);
	packet.SetWritePos(3);
	client_CAN.Send(packet);
	packet.Clear();
}

// Send commands to certain program.
void SendCommand(const unsigned char source, const unsigned char destination, const unsigned char command, const unsigned char data)
{
	packet_CAN.Clear();
	packet_CAN.Write(source, 0);
	packet_CAN.Write(destination, 1);
	packet_CAN.Write(command, 2);
	packet_CAN.SetWritePos(3);
	packet_CAN.Write(data, 3);
	client_CAN.Send(packet_CAN);
	packet_CAN.Clear();
	//SleepMs( 30 );//zc was 100
	stt = TimeCheck();
	ofstream test_data;
	test_data.open("C:\\MANUS\\CommonSpace\\Sensor_Data\\t_data.txt", ios::app);
	while ((TimeCheck() - stt) < 30)
	{
		ReadForce(cur_force);
		ReadPosit(cur_position);
		ReadVel();
		SleepMs(2);
		test_data << TimeCheck() << ", " << cur_velocity_f << ", " << cur_force << ", " << cur_position << ", " << cur_pos_nf << ", " << cur_velocity << ", " << grasp_start << ", " << grasp_end << ", " << speed[7] << ", " << grasp_npos << "\n";//grasp_npos-grasp_inipos
	}
}

void SendCommand2(unsigned char target, unsigned char command, unsigned char snd_data)
{
	packet_CAN.Clear();       //  Clear packet and encode data.
	packet_CAN.Write(CAN, 0);
	packet_CAN.Write(target, 1);
	packet_CAN.Write(command, 2);
	packet_CAN.SetWritePos(3);
	packet_CAN.Write(snd_data, 3);

	int response = client_CAN.Send(packet_CAN);
	//cout<< response<<endl;
	//return true;


	packet_CAN.Clear();

}


// Send voice command to SPC.
void TTSSpeak(unsigned char seq)
{
	SendCommand(CAN, SPC, SPEECH_DISABLE, EMPTY_MESSAGE);
	SendCommand(CAN, SPC, TTS_SPEAK, seq);
	tts_in_progress = true;
	while (tts_in_progress);
	SendCommand(CAN, SPC, SPEECH_ENABLE, EMPTY_MESSAGE);
}

//=============================================================================
// Information display.
//=============================================================================
// Go to certain position on the screen.
void gotoxy(int x, int y)
{
	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD point;
	point.X = x - 1;
	point.Y = y - 1;
	SetConsoleCursorPosition(hConsole, point);
	return;
}

//Display a menu with all key functions.
void Help(void)
{
	printf("type '?' for help\n");
	printf("type 'o' for quit or exit program\n");
	printf("type 'p' for printing of the positions\n");
	printf("type 'i' for manus status info\n");
	printf("type '1' for cartesian control\n");
	printf("type '4' for joint control\n");
	printf("type '5' for fold out\n");
	printf("type '6' for fold in\n\n");
	printf("type 'l' for auto mode\n\n");
	printf("type 'c' for grab visual information\n\n");

	printf("type 'q/a' for A1 or X (depending the mode)\n");
	printf("type 'w/s' for A2 or Y (depending the mode)\n");
	printf("type 'e/d' for A3 or Z (depending the mode)\n");
	printf("type 'r/f' for A4 or yaw (depending the mode)\n");
	printf("type 't/g' for A5 or pitch (depending the mode)\n");
	printf("type 'y/h' for A6 or roll (depending the mode)\n");
	printf("type 'u/j' for gripper open/close (A7)\n\n");
	printf("type any other key('space' most often used) for STOP movements\n");
}

// Update the pos information in shared memory.
void UpdatePos(const float *pos)
{
	register char pos_str[256];

	sprintf_s(pos_str, "%1c%06.2f%1c%06.2f%1c%06.2f%1c%06.2f%1c%06.2f%1c%06.2f%1c%06.0f%1d",
		((pos[0] > 0) ? '+' : '-'), fabs(pos[0]), ((pos[1] > 0) ? '+' : '-'), fabs(pos[1]),
		((pos[2] > 0) ? '+' : '-'), fabs(pos[2]), ((pos[3] > 0) ? '+' : '-'), fabs(pos[3]),
		((pos[4] > 0) ? '+' : '-'), fabs(pos[4]), ((pos[5] > 0) ? '+' : '-'), fabs(pos[5]),
		((pos[6] > 0) ? '+' : '-'), fabs(pos[6]), speed_mode);

	robot_pos->SetWritePos(0);
	robot_pos.Lock();
	robot_pos->Write((unsigned char*)pos_str, 50 * sizeof(BYTE), 0);
	robot_pos.Unlock();
};
void UpdateSpaceMouse(const int *spacemouse)
{
	register char pos_str[256];
	register char pos_str2[256];

	sprintf_s(pos_str, "%d%d%d%d%3d%3d%3d%3d%3d%3d%d%d%d%d%d%3d%4d",
		spacemouse[0], spacemouse[1], spacemouse[2], spacemouse[3],
		int(speed[1]), int(speed[2]), int(speed[3]), int(speed[4]), int(speed[5]), int(speed[6]), int(speed_mode), int(cam_cls), int(init_stop), int(spm_gripper), int(oneclick_mode), int(suggestedMotion), int(spm_operation));

	//cout << pos_str << endl;
	sprintf_s(pos_str2, "%d%d%d%d%3d%3d%3d%3d%3d%3d%d",
		spacemouse[0], spacemouse[1], spacemouse[2], spacemouse[3],
		int(speed[1]), int(speed[2]), int(speed[3]), int(speed[4]), int(speed[5]), int(speed[6]), int(init_stop));
	gotoxy(50, 30);
	cout << pos_str2 << endl;
	spaceMouseValues->SetWritePos(0);
	spaceMouseValues.Lock();
	spaceMouseValues->Write((unsigned char*)pos_str, 40 * sizeof(BYTE), 0);
	spaceMouseValues.Unlock();
};
// Display speed information.
void DisplaySpeed(void)
{
	gotoxy(1, 22);
	printf("vw: %.0f %.0f %.0f %.0f %.0f %.0f %.0f %.0f \n", speed[0], speed[1], speed[2], speed[3], speed[4], speed[5], speed[6], speed[7]);
	gotoxy(1, 23);
	printf("T: %.2f %.2f %.2f %.2f %.2f %.2f \n", pd[0], pd[1], pd[2], pd[3], pd[4], pd[5]);
	gotoxy(1, 25);
	printf("e: %.2f %.2f %.2f %.2f %.2f %.2f \n", eprev1[0], eprev1[1], eprev1[2], eprev1[3], eprev1[4], eprev1[5]);
}

// This function will display position in either two formats. X,Y,Z,Yaw,Pitch,roll and grip 
// when Cartesian mode is selected, and Joint angle 1,2,3,4,5,6 in joint mode.
void DisplayPos(float* pos)
{
	gotoxy(1, 24);
	ofstream posit;
	posit.open("C:\\MANUS\\CommonSpace\\movement_files\\pos.txt", ios::app);
	float dist;
	dist = sqrt((double)((pos[0]) * (pos[0]) + (pos[1]) * (pos[1]) + (pos[2]) * (pos[2])));

	printf("[%d]  %06.2f  %06.2f  %06.2f  %06.2f  %06.2f  %06.2f %06.2f(%1d) \n",
		speed_mode, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], speed[7], (block_all_motions ? 1 : 0));
	int time_check = TimeCheck();
	posit << time_check << "  " << pos[0] << " " << pos[1] << " " << pos[2] << " " << pos[3] << " " << pos[4] << " " << pos[5] << " " << speed[7] << "\n";

	if ((fabs(task_goal_pos[0] - pos[0]) < 50.0f) && (fabs(task_goal_pos[1] - pos[1]) < 50.0f) && (robot_in_out_of_range == false))
	{
		robot_in_out_of_range = true;
		Beep(500, 200);
		Beep(600, 200);
		Beep(700, 200);
		ResetAll();
	}
	else if ((fabs(task_goal_pos[0] - pos[0]) > 100.0f) || (fabs(task_goal_pos[1] - pos[1]) > 100.0f) && (robot_in_out_of_range == true))
		robot_in_out_of_range = false;

}

// Show current command.
void ShowCommand(char *str)
{
	gotoxy(1, 16);
	printf(str);
}

// Show arm status.
void ShowStatus(char *str)
{
	gotoxy(1, 17);
	printf(str);
}

// Show control loop frequency.
void ShowFrequency(int time)
{
	gotoxy(1, 18);
	char str[256];

	if (time <= 1)
		time = 1;

	if (time > 0)
		sprintf(str, "Freq: %f", 1000.0 / (float)time);
	else
		sprintf(str, "Freq: Normal");
	printf(str);
}

// When a package with message ID=350, the first two bytes contain the information for
// the status of the manus. This is clarified in page 23 of Transparent mode manual.
void PrintStatus(void)
{
	gotoxy(1, 26);
	actual_cbox = manus_status & 0x07;//get the cbox mode
	ofstream errors;
	errors.open("C:\\MANUS\\CommonSpace\\movement_files\\error.txt", ios::app);
	switch (actual_cbox)
	{
	case 0:
		printf("MODE: No move mode     ");
		break;
	case 1:
		printf("MODE: Cartesian mode    ");
		break;
	case 4:
		printf("MODE: Joint mode    ");
		break;
	case 5:
		printf("MODE: Folding out mode\n");
		break;
	case 6:
		printf("MODE: Folding in mode\n");
		break;
	}

	if ((manus_status & 0xC0) == 0xC0)
	{
		switch (manus_message)
		{
		case 0:
			printf("ERROR: %d\n", manus_message);
			break;
		case 1:
			printf("ERROR: I/O 80C552 error\n");
			break;
		case 2:
			printf("ERROR: %d\n", manus_message);
			break;
		case 3:
			printf("ERROR: %d\n", manus_message);
			break;
		case 4:
			printf("ERROR: absolute encoder error\n");
			break;
		case 5:
			printf("ERROR: not defined\n");
			break;
		case 6:
			printf("ERROR: %d\n", manus_message);
			break;
		case 7:
			printf("ERROR: %d\n", manus_message);
			break;
		case 8:
			printf("ERROR: %d\n", manus_message);
			break;
		case 9:
			printf("ERROR: not defined\n");
			break;
		case 10:
			printf("ERROR: not defined\n");
			break;
		case 11:
			printf("ERROR: %d\n", manus_message);
			break;
		case 12:
			printf("ERROR: %d\n", manus_message);
			break;
		case 13:
			printf("ERROR: %d\n", manus_message);
			break;
		case 14:
			printf("ERROR: not defined\n");
			break;
		case 15:
			printf("ERROR: MANUS moving without user input\n");
			break;
		}
		ResetAll();
		int time_stamp = TimeCheck();
		errors << "error '" << manus_message << "', " << time_stamp << "\n";
		SendCommand(CAN, GUI, ROBOT_ERROR, manus_message);
	}
	else if ((manus_status & 0xC0) == 0x40)
	{
		switch (manus_message)
		{
		case STUCK_GRIPPER:
			printf("WARNING: STUCK_GRIPPER\t\t");
			break;
		case NOT_ATTACHE:
			printf("WARNING: NOT_ATTACHE\t\t");
			break;
		case ARM_FOLDED_STRETCHED:
			printf("WARNING: ARM_FOLDED_STRETCHED\t\t");
			break;
		case BLOCKED_DOF:
			printf("WARNING: BLOCKED_DOF\t\t");
			break;
		case MAX_M1_ROTATION:
			printf("WARNING: MAX_M1_ROTATION\t\t");
			break;
		default:
			printf("WARNING: unknown warning          ");
			break;
		}

		ResetAll();
		float time_stamp = TimeCheck();
		errors << "warning '" << manus_message << "', " << time_stamp << "\n";
		/* WARNING */
		if (!block_all_motions)
		{
			block_all_motions = true;
			SendCommand(CAN, GUI, ROBOT_WARNING, manus_message);
		}

		//TTSSpeak( WARNING_MESSAGE );
		//cbox=0;
		//End of manus_message switch
	} //End of if-condition for Warnings
	else if ((manus_status & 0xC0) == 0x80)
	{
		switch (manus_message)
		{
		case 0:     printf("GENERAL_MESSAGE: MANUS folded\n"); break;
		case 1:     printf("GENERAL_MESSAGE: MANUS unfolded\n"); break;
		case 2:     printf("GENERAL_MESSAGE: gripper ready\n"); break;
		case 3:     printf("GENERAL_MESSAGE: absolute measuring is ready"); break;
		default:    printf("GENERAL_MESSAGE: unknown message"); break;
		}
	}
}

//=============================================================================
// Manus arm control.
//=============================================================================
// Reset all variables.
void ResetAll(void)
{
	for (int i = 0; i < 8; i++)
		speed[i] = 0;
	if ((cbox == FOLD_IN) || (cbox == FOLD_OUT))
		cbox = FREE;
	rotation_start1 = false;
	rotation_start2 = false;
	new_status = true;
	test_x_flag = false;
	job_done = true;
	job_complete = true;
	job_complete2 = true;
	approach_flag = false;
	retreat_flag = false;
	auto_mode_start = false;	// set it to the first status;; new auto-mode objective will be given
	home_pos_flag = false;		// reset home_pos_flag
	adjust_pos = false;
	grab_in_progress = false;
	init_fine_motion = false;

	if (set_point_vel || set_points_control)
	{
		set_point_vel = false;
		set_points_control = false;
		ClientSocketComm(CAN, VIS, WRITE_SET_POS_DONE, NULL);
	}
}

// Read first 0x0D4 and 0x4D4
bool Read_D4(void)
{
	int t1 = TimeCheck(), t2;
	bool found_0D4 = false;
	bool found_4D4 = false;
	while ((!found_0D4) || (!found_4D4))
	{
		stsResult = m_objPCANBasic.Read(m_Handle, &rcvMsg, &CANTimeStamp);
		if (stsResult == PCAN_ERROR_OK)
		{
			if (rcvMsg.ID == 0x0D4)
				found_0D4 = true;
			if (rcvMsg.ID == 0x4D4)
				found_4D4 = true;
		}
		else
		{
			t2 = TimeCheck();
			if (t2 - t1 > 10000)
				break;
			Sleep(5);
		}
	}
	return (found_0D4 && found_4D4);
}

// Read first 0x350 and 0x360.
bool Read_350_360(void)
{
	int t1 = TimeCheck(), t2;
	bool found_350 = false;
	bool found_360 = false;
	while ((!found_350) || (!found_360))
	{
		stsResult = m_objPCANBasic.Read(m_Handle, &rcvMsg, &CANTimeStamp);
		if (stsResult == PCAN_ERROR_OK)
		{
			if (rcvMsg.ID == 0x350)
			{
				pos[0] = ((char)rcvMsg.RDATA[2] << 8) + (unsigned char)rcvMsg.RDATA[3];	// X/axis1.
				pos[1] = ((char)rcvMsg.RDATA[4] << 8) + (unsigned char)rcvMsg.RDATA[5];	// Y/axis2.
				pos[2] = ((char)rcvMsg.RDATA[6] << 8) + (unsigned char)rcvMsg.RDATA[7];	// Z/axis3.

				if (cbox == CARTESIAN)
					for (int i = 0; i < 3; i++)
						pos[i] = 0.022 * pos[i];
				else
					cout << "[Error]: Undefined the position reading!" << endl;

				found_350 = true;
			}
			if (rcvMsg.ID == 0x360)
			{
				pos[3] = ((char)rcvMsg.RDATA[0] << 8) + (unsigned char)rcvMsg.RDATA[1];	// Yaw/axis4.
				pos[4] = ((char)rcvMsg.RDATA[2] << 8) + (unsigned char)rcvMsg.RDATA[3];	// Pitch/axis5.
				pos[5] = ((char)rcvMsg.RDATA[4] << 8) + (unsigned char)rcvMsg.RDATA[5];	// Roll/axis6.
				pos[6] = ((char)rcvMsg.RDATA[6] << 8) + (unsigned char)rcvMsg.RDATA[7];	// gripper.

				if (cbox == CARTESIAN)
				{
					// Bring back the values between -180 and 180.
					for (int i = 3; i < 6; i++)
					{
						while (pos[i] > 1800)
							pos[i] = (pos[i] - 3600);
						while (pos[i] < -1800)
							pos[i] = (pos[i] + 3600);
					}
					// Yaw, pitch, roll in degrees.
					for (int i = 3; i < 6; i++)
						pos[i] = 0.1 * pos[i];

					// Process the roll angle. 
					if (reverse_flag)
						pos[5] = (pos[5] > 0) ? pos[5] - 180.0f : pos[5] + 180.0f;
				}
				else
					cout << "[Error]: Undefined the position reading!" << endl;

				found_360 = true;
			}
		}
		else
		{
			t2 = TimeCheck();
			if (t2 - t1 > 10000)
				break;
			Sleep(5);
		}
	}
	return (found_350 && found_360);
}

// Read first 0x37F.
bool Read_37F(void)
{
	int t1 = TimeCheck(), t2;
	bool found_37F = false;
	while (!found_37F)
	{
		stsResult = m_objPCANBasic.Read(m_Handle, &rcvMsg, &CANTimeStamp);
		if (stsResult == PCAN_ERROR_OK && rcvMsg.ID == 0x37F)
			found_37F = true;
		else
		{
			t2 = TimeCheck();
			if (t2 - t1 > 10000)
				break;
			Sleep(5);
		}
	}
	return found_37F;
}

// Open the grabber.
bool Open_Grabber(void)
{
	bool open_grabber = false;
	while (!open_grabber)
	{
		stsResult = m_objPCANBasic.Read(m_Handle, &rcvMsg, &CANTimeStamp);
		if (stsResult == PCAN_ERROR_OK && rcvMsg.ID == 0x37F)
		{
			int temp1 = 500, temp2 = 10;

			// Open the grabber.
			ManualControl('u');
			SetTransmitMessage(xmitMsg);
			while (temp1)
			{
				stsResult = m_objPCANBasic.Write(m_Handle, &xmitMsg);
				if (stsResult != PCAN_ERROR_OK)
					return open_grabber;
				--temp1;
				Sleep(20);
			}

			// Stop the arm.
			ManualControl(' ');
			SetTransmitMessage(xmitMsg);
			while (temp2)
			{
				stsResult = m_objPCANBasic.Write(m_Handle, &xmitMsg);
				if (stsResult != PCAN_ERROR_OK)
					return open_grabber;
				--temp2;
				Sleep(20);
			}
			Sleep(100);
			open_grabber = true;
		}
		else
			Sleep(5);
	}
	return open_grabber;
}

// In manual control the movement data is set according to the key presssed. This function 
// takes that input and sets the Speed variable to the corresponding movement associated with 
// that particular input. Also the Mode or Cbox is set from the keyboard input.
void ManualControl(char ch)
{
	FILE* fid;
	ifstream infile;

	Matrix<3, 1> dw, dm, dp;
	Matrix<3, 1> ca, cr, wa, wr;
	Matrix<3, 1> wt, ypr;
	Matrix<3, 3> Rv;
	Matrix<3, 1> tv;

	ColumnVector<6> t;
	Matrix<3, 3> Ree2w, Rypr2w, Rc2w;
	ColumnVector<3> vw, ww, vc, wc, wypr, vt;


	switch (ch)
	{


	case '0':	// Free.
		cbox = FREE;
		new_status = true;
		break;
	case '1':	// Cartesian.
		cbox = CARTESIAN;
		new_status = true;
		break;
	case '4':   // Joint.
		cbox = JOINT;
		new_status = true;
		break;
	case '5':	// Fold-out.
		cbox = FOLD_OUT;
		new_status = true;
		break;
	case '6':	// Fold-in.
		cbox = FOLD_IN;
		new_status = true;
		break;
	case '+':	// Increase Speed.
		speed_mode++;
		speed_mode = (speed_mode > 4) ? 4 : speed_mode;
		UpdatePos(pos);
		SendCommand(CAN, GUI, UPDATE_SPEED, 0);
		break;
	case '-':	// Decrease Speed.
		speed_mode--;
		speed_mode = (speed_mode < 0) ? 0 : speed_mode;
		UpdatePos(pos);
		SendCommand(CAN, GUI, UPDATE_SPEED, 0);
		break;
	case '9':	// Reverse roll angle.
		reverse_flag = (reverse_flag == false) ? true : false;
		break;
	case 'i':	// Lift up.
		if (cbox == CARTESIAN) { speed[0] = 1; new_status = true; }
		else if (cbox == JOINT) { speed[0] = 1; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'k':	// Lift up.
		if (cbox == CARTESIAN) { speed[0] = -1; new_status = true; }
		else if (cbox == JOINT) { speed[0] = -1; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'q':	// X axis.
		if (cbox == CARTESIAN) { speed[1] = linear_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[1] = MAX_JOINT; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'a':	// X axis.
		if (cbox == CARTESIAN) { speed[1] = -linear_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[1] = -MAX_JOINT; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'w':	// Y axis.
		if (cbox == CARTESIAN) { speed[2] = linear_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[2] = MAX_JOINT; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 's':	// Y axis.
		if (cbox == CARTESIAN) { speed[2] = -linear_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[2] = -MAX_JOINT; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'e':	// Z axis.
		if (cbox == CARTESIAN) { speed[3] = linear_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[3] = MAX_JOINT; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'd':	// Z axis.
		if (cbox == CARTESIAN) { speed[3] = -linear_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[3] = -MAX_JOINT; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'r':	// Yaw.
		if (cbox == CARTESIAN) { speed[4] = angular_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[4] = MAX_JOINT_YPR; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'f':	// Yaw.
		if (cbox == CARTESIAN) { speed[4] = -angular_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[4] = -MAX_JOINT_YPR; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 't':	// Pitch.
		if (cbox == CARTESIAN) { speed[5] = angular_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[5] = MAX_JOINT_YPR; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'g':	// Pitch.
		if (cbox == CARTESIAN) { speed[5] = -angular_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[5] = -MAX_JOINT_YPR; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'y':	// Roll.
		if (cbox == CARTESIAN) { speed[6] = angular_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[6] = MAX_JOINT_YPR; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'h':	// Roll.
		if (cbox == CARTESIAN) { speed[6] = -angular_speed_limit[speed_mode]; new_status = true; }
		else if (cbox == JOINT) { speed[6] = -MAX_JOINT_YPR; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'u':	// Open gripper.
		if (cbox == CARTESIAN) { speed[7] = MAX_CART_GRIP; new_status = true; grasp_test = 2; grasp_inipos = 0; }
		else if (cbox == JOINT) { speed[7] = MAX_JOINT_GRIP; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		open_in_progress = true;
		init_force = 0;
		break;
	case 'j':	// Close gripper.
		if (cbox == CARTESIAN) { speed[7] = -MAX_CART_GRIP; new_status = true; grasp_test = 1; grasp_inipos = pos[6]; }
		else if (cbox == JOINT) { speed[7] = -MAX_JOINT_GRIP; new_status = true; }
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		if (!init_system)
		{
			grab_in_progress = true;
			SendCommand(CAN, FSR, GRAB, ' ');
		}
		break;
	case ' ': {
		grasp_test = 0;
		grasp_inipos = 0;
		ResetAll();
	}
			  break;
	case EXIT:
		gotoxy(1, 38);
		if (!UnloadAll())
			cout << "[Error!]: Fail the unload! Press any key to exit the program!" << endl;
		else
			cout << "[Info]: Unload success! Press any key to exit the program!" << endl;
		cin.get();
		exit(1);
	case '?':
		Help();
		break;
	case 'l':	// return to user
		mode = AUTO_MODE;
		home_pos_flag = false;
		job_complete = false;
		job_complete2 = true;
		ShowCommand("COM: Set-Point Control\n");
		break;
	case 'x':
		ShowCommand("COM: GO!!(R+T)\n");
		if (cbox == CARTESIAN)
		{
			if (!auto_mode_start)	// read auto_mode_info at the beginning
			{
				rotation_start1 = true;
				rotation_start2 = false;
				auto_mode_start = true;
			}

			if (auto_mode_start)	// rotation & translation
				new_status = true;
		}
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'n':
		ShowCommand("COM: Approach\n");
		ca = 0, 0, 1;
		wa = C2W_transform(pos) * ca;
		if (cbox == CARTESIAN)
		{
			for (int i = 0; i < 3; i++)
				speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
			new_status = true;
		}
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'm':
		ShowCommand("COM: Retreat\n");
		ca = 0, 0, -1;
		wa = C2W_transform(pos) * ca;
		if (cbox == CARTESIAN)
		{
			for (int i = 0; i < 3; i++)
				speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
			new_status = true;
		}
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case '2':
		ShowCommand("COM: Relative Right\n");
		ca = 1, 0, 0;
		wa = C2W_transform2(pos) * ca;
		if (cbox == CARTESIAN)
		{
			for (int i = 0; i < 3; i++)
				speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
			new_status = true;
		}
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case '3':
		ShowCommand("COM: Relative Left\n");
		ca = -1, 0, 0;
		wa = C2W_transform2(pos) * ca;
		if (cbox == CARTESIAN)
		{
			for (int i = 0; i < 3; i++)
				speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
			new_status = true;
		}
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'S':
		//ShowCommand("COM: Relative Left\n");
		//ReadSugspeed();
		//gotoxy(1, 45);
		//cout << suggspeed[0]<< suggspeed[1] << suggspeed[2] << suggspeed[3] << suggspeed[4] << suggspeed[5] << suggspeed[6] << endl;
		//cout << TimeCheck() << endl;
		//ca = -1, 0, 0;
		//wa = C2W_transform2(pos) * ca;
		if (cbox == CARTESIAN)
		{
			for (int i = 0; i < 6; i++)
				speed[i + 1] = suggspeed[i];
			//for (int i = 3; i < 6; i++)
			//	speed[i + 1] = 1 * angular_speed_limit[speed_mode];
			new_status = true;

		}
		else cout << "new button test." << endl;
		break;
	case '<':
		ShowCommand("COM: Relative Forward\n");
		ca = 0, 0, 1;
		wa = C2W_transform2(pos) * ca;
		if (cbox == CARTESIAN)
		{
			for (int i = 0; i < 3; i++)
				speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
			new_status = true;
		}
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case '>':
		ShowCommand("COM: Relative Backward\n");
		ca = 0, 0, -1;
		wa = C2W_transform2(pos) * ca;
		if (cbox == CARTESIAN)
		{
			for (int i = 0; i < 3; i++)
				speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
			new_status = true;
		}
		else cout << "[Warning!]: speed assignment without selecting a CBOX." << endl;
		break;
	case 'v': // ??? wz 07282011
		cout << endl << "[case 'v']vvvvvvvvvvvvvvvvvvvvvvvvvvv" << endl;
		infile.open("C:\\MANUS\\CommonSpace\\Setting\\myvel.txt");
		infile >> t;
		infile.close();
		vwg = t(1), t(2), t(3);	// translational velocity in CC
		wwg = t(4), t(5), t(6);	// rotational velocity in CC
		mode = AUTO_MODE;
		home_pos_flag = false;
		test_x_flag = true;
		job_complete2 = true; job_complete = false;
		pos_index = -1;

		for (int i = 0; i < 8; ++i)
			opos[i] = pos[i];

		potime = TimeCheck();

		for (int i = 0; i < 3; ++i)
			oypr[i] = 0.0f;

		new_status = true;
		break;
	case 'c':
		cout << endl << "[case 'c']ccccccccccccccccccccccccccc" << endl;
		infile.open("C:\\MANUS\\CommonSpace\\Setting\\myvel.txt");
		infile >> t;
		infile.close();

		vc = t(1), t(2), t(3);
		wc = t(4), t(5), t(6);

		Rc2w = C2W_transform(pos);
		Ree2w = EE2C_transform(pos);
		Rypr2w = 0, Ree2w(2, 1), Ree2w(1, 1),
			0, -Ree2w(1, 1), Ree2w(2, 1),
			1, 0, Ree2w(3, 1);

		wypr = inverse(Rypr2w) * Rc2w * wc;

		speed[1] = 0;
		speed[2] = 0;
		speed[3] = 0;
		speed[4] = wypr(1);
		speed[5] = wypr(2);
		speed[6] = wypr(3);

		new_status = true;
		break;
	case 'z': // pre set pos
		mode = AUTO_MODE;
		home_pos_flag = true;
		job_complete = false;
		job_complete2 = true;
		pos_index = -1;
		ShowCommand("COM: Init Position\n");
		break;
	case '!': // pre set pos 1 wz 07282011
		mode = AUTO_MODE;
		home_pos_flag = true;
		job_complete = false;
		job_complete2 = true;
		pos_index = 999;
		ShowCommand("COM: User Position#0 \n");
		break;
	case ',': // pre set pos 2 wz 07282011
		mode = AUTO_MODE;
		home_pos_flag = true;
		job_complete = false;
		job_complete2 = true;
		pos_index = 99;
		ShowCommand("COM: User Position#1 \n");
		break;
	case '.': // pre set pos 3 wz 07282011
		mode = AUTO_MODE;
		home_pos_flag = true;
		job_complete = false;
		job_complete2 = true;
		pos_index = 6;
		ShowCommand("COM: User Position#2 \n");
		break;
	case '/': // pre set pos 4 wz 07282011
		mode = AUTO_MODE;
		home_pos_flag = true;
		job_complete = false;
		job_complete2 = true;
		pos_index = 10;
		ShowCommand("COM: User Position#3 \n");
		break;
	case 'b':
		write_init_pos = true;
		fid = fopen("C:\\MANUS\\CommonSpace\\Run\\curr_points.txt", "w");
		for (int i = 0; i < 6; i++)
			fprintf(fid, "%f ", pos[i]);
		fclose(fid);
		break;
	case '\b': // ??
		mode = AUTO_MODE;
		home_pos_flag = true;
		job_complete = false;
		job_complete2 = true;
		pos_index = 911;
		break;
	case '&':
		gotoxy(1, 28);
		for (int j = 0; j < QUE_SIZE; j++)
		{
			if (j == pos_queue_index)
				printf("[%d] %06.2f ", j, good_pos[j][0]);
			else
				printf("(%d) %06.2f ", j, good_pos[j][0]);
		}
		break;
	case 'p':
		fu_flag = (fu_flag) ? false : true;
		cout << "[flag] reliable " << ((fu_flag) ? "on" : "off") << endl;
		break;
		//Brandon 12/15/16 SPACE MOUSE CONTROL
	case'#':
		if (spaceMouseEnabled)
		{
			//These > values are the deadbands for the mouse. Change for more/less sensitivity.	

			////// Gripper mode
			//if (spaceButtonsToggle[1] /*closed*/)
			//{
			//	for (int i = 1; i < 7; i++)
			//		speed[i] = 0;
			//	abs(spaceMouse[3]) > 250 ? speed[7] = MAX_CART_GRIP*sign(spaceMouse[3]) : speed[7] = 0;
			//}


			////Close Gripper
			if (!spaceButtonsToggle[1] && !spaceMouse_stop /*opened*/)
			{
				speed[7] = 0;
				if (spaceMouseMode == 0) {// arm mode
					//1,2,3 xyz
					for (int i = 1; i < 4; i++)
						abs(spaceMouse[i - 1]) > spacemouse_translation_sensitivity ? speed[i] = linear_speed_limit[speed_mode] * sign(spaceMouse[i - 1]) : speed[i] = 0;
					for (int i = 4; i < 7; i++)
						speed[i] = 0;
				}
				else if (spaceMouseMode == 1) {  // wrist mode
					//4,5,6 ypr
					for (int i = 4; i < 7; i++)
						abs(spaceMouse[i - 1]) > spacemouse_rotation_sensitivity ? speed[i] = angular_speed_limit[speed_mode] * sign(spaceMouse[i - 1]) : speed[i] = 0;
					for (int i = 1; i < 4; i++)
						speed[i] = 0;
				}
				else if (spaceMouseMode == 2)  // hybird mode
				{
					for (int i = 1; i < 4; i++)
						abs(spaceMouse[i - 1]) > spacemouse_translation_sensitivity ? speed[i] = linear_speed_limit[speed_mode] * sign(spaceMouse[i - 1]) : speed[i] = 0;
					for (int i = 4; i < 7; i++)
						abs(spaceMouse[i - 1]) > spacemouse_rotation_sensitivity ? speed[i] = angular_speed_limit[speed_mode] * sign(spaceMouse[i - 1]) : speed[i] = 0;
				}
				else if (spaceMouseMode == 3)  // hybird mode in gripper frame
				{
					int sp_command[3];
					int command_max = 0;
					int command_axis = -1;

					for (int i = 1; i < 4; i++)//   space mouse only move along the axis has the highest command 
					{
						abs(spaceMouse[i - 1]) > 1700 ? sp_command[i - 1] = spaceMouse[i - 1] : sp_command[i - 1] = 0;
						if (abs(sp_command[i - 1]) > 0 && abs(sp_command[i - 1]) > abs(command_max))
						{
							command_max = sp_command[i - 1];
							command_axis = i - 1;
						}
					}

					switch (command_axis)//-1 : no motion,all speed are 0; 0: along forward/ backward,  1: L\R   ,2: up/down 
					{
					case(0)://0 : along forward / backward
						if (command_max > 0)//approach 
						{
							ca = 0, 0, 1;
							wa = C2W_transform(pos) * ca;
							for (int i = 0; i < 3; i++)
							{
								speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
							}
						}
						else if (command_max < 0)//retreat
						{
							ca = 0, 0, -1;
							wa = C2W_transform(pos) * ca;
							for (int i = 0; i < 3; i++)
							{
								speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
							}
						}
						break;
					case(1):// 1: L\R   ,2: up/down 
						if (command_max > 0)//left
						{
							ca = -1, 0, 0;
							wa = C2W_transform2(pos) * ca;
							for (int i = 0; i < 3; i++)
							{
								speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
							}
						}
						else if (command_max < 0) // right
						{
							ca = 1, 0, 0;
							wa = C2W_transform2(pos) * ca;
							for (int i = 0; i < 3; i++)
							{
								speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];

							}
						}
						break;
					case(2)://2: up/down 
						speed[3] = linear_speed_limit[speed_mode] * sign(spaceMouse[2]);
						break;
					case(-1):
						for (int i = 0; i < 7; i++)
						{
							speed[i] = 0;
						}
						break;
					default:
						for (int i = 0; i < 7; i++)
						{
							speed[i] = 0;
						}
						break;
					}
					//abs(spaceMouse[i - 1])>1800 ? speed[i] = linear_speed_limit[speed_mode] * sign(spaceMouse[i - 1]) : speed[i] = 0;

					for (int i = 4; i < 7; i++)
						abs(spaceMouse[i - 1]) > 1800 ? speed[i] = angular_speed_limit[speed_mode] * sign(spaceMouse[i - 1]) : speed[i] = 0;
				}
				else if (spaceMouseMode == 5)  // hybird mode in gripper frame
				{
					int sp_command[3];
					int command_max = 0;
					int command_axis = -1;

					for (int i = 1; i < 4; i++)//   space mouse only move along the axis has the highest command 
					{
						abs(spaceMouse[i - 1]) > 1700 ? sp_command[i - 1] = spaceMouse[i - 1] : sp_command[i - 1] = 0;
						if (abs(sp_command[i - 1]) > 0 && abs(sp_command[i - 1]) > abs(command_max))
						{
							command_max = sp_command[i - 1];
							command_axis = i - 1;
						}
					}

					switch (command_axis)//-1 : no motion,all speed are 0; 0: along forward/ backward,  1: L\R   ,2: up/down 
					{
					case(0)://0 : along forward / backward
						if (command_max > 0)//one click mode 
						{
							for (int i = 0; i < 6; i++)
								speed[i + 1] = suggspeed[i];
							//for (int i = 3; i < 6; i++)
							//	speed[i + 1] = 1 * angular_speed_limit[speed_mode];
							new_status = true;
						}
						else if (command_max < 0)//retreat
						{
							ca = 0, 0, -1;
							wa = C2W_transform(pos) * ca;
							for (int i = 0; i < 3; i++)
							{
								speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
							}
						}
						break;
					case(1):// 1: L\R   ,2: up/down 
						if (command_max > 0)//left
						{
							ca = -1, 0, 0;
							wa = C2W_transform2(pos) * ca;
							for (int i = 0; i < 3; i++)
							{
								speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];
							}
						}
						else if (command_max < 0) // right
						{
							ca = 1, 0, 0;
							wa = C2W_transform2(pos) * ca;
							for (int i = 0; i < 3; i++)
							{
								speed[i + 1] = wa(i + 1, 1) * linear_speed_limit[speed_mode];

							}
						}
						break;
					case(2)://2: up/down 
						speed[3] = linear_speed_limit[speed_mode] * sign(spaceMouse[2]);
						break;
					case(-1):
						for (int i = 0; i < 7; i++)
						{
							speed[i] = 0;
						}
						break;
					default:
						for (int i = 0; i < 7; i++)
						{
							speed[i] = 0;
						}
						break;
					}
					//abs(spaceMouse[i - 1])>1800 ? speed[i] = linear_speed_limit[speed_mode] * sign(spaceMouse[i - 1]) : speed[i] = 0;

					if (command_axis == 0 && command_max > 0)
					{

					}
					else
					{
						for (int i = 4; i < 7; i++)
							abs(spaceMouse[i - 1]) > 1800 ? speed[i] = angular_speed_limit[speed_mode] * sign(spaceMouse[i - 1]) : speed[i] = 0;
					}
				}
				spm_operation = 0;
				for (int i = 1; i < 7; i++)
					spm_operation += abs(spaceMouse[i - 1]);
			}
			else if (!spaceButtonsToggle[1] && spaceMouse_stop)
			{
				spm_operation = 0;
				for (int i = 1; i < 7; i++)
					spm_operation += abs(spaceMouse[i - 1]);
				if (spm_operation < 10)
					spaceMouse_stop = false;
			}
			new_status = true;
		}
		break;
	default:

		for (int i = 0; i < 8; i++)
			speed[i] = 0;
		if ((cbox == FOLD_IN) || (cbox == FOLD_OUT))
			cbox = FREE;
		break;
	}
}

// This function will decode the information in the packages sent by the manus
// For different modes or Cbox and package ID(350,360,37f) the data is interpreted differently.
void Decode(TPCANMsg& rcvMsg, TPCANMsg& xmitMsg)
{
	switch (rcvMsg.ID)
	{
	case 0x350:
		//
		if (prevID != 0x350)
		{
			//if (debug_on) 
			//	timing << "35 " << TimeCheck() << "\n";
			prevID = 0x350;
		}

		// We recover first two bytes for status.
		manus_status = rcvMsg.RDATA[0];
		manus_message = rcvMsg.RDATA[1] & 00001111;

		// Decode and store the position information.

		raw_pos[0] = ((char)rcvMsg.RDATA[2] << 8) + (unsigned char)rcvMsg.RDATA[3];	/* X/axis1 */
		raw_pos[1] = ((char)rcvMsg.RDATA[4] << 8) + (unsigned char)rcvMsg.RDATA[5];	/* Y/axis2 */
		raw_pos[2] = ((char)rcvMsg.RDATA[6] << 8) + (unsigned char)rcvMsg.RDATA[7];	/* Z/axis3 */

		//
		//gotoxy(1, 53);
		//cout << suggspeed[6] << endl;
		if (new_status & (mode == MANUAL_MODE))
			SetTransmitMessage(xmitMsg);
		else if (mode == AUTO_MODE)
			new_status = false;
		//else if (suggspeed[6]==1)
		//	SetTransmitMessage(xmitMsg);
		else
		{
			xmitMsg.ID = rcvMsg.ID;
			xmitMsg.LEN = 0;
			xmitMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		}

		//cartesian mode, x,y,z positions are decoded into milimeters in decimal form
		if (cbox == CARTESIAN)
			for (int i = 0; i < 3; i++)
				pos[i] = 0.022 * raw_pos[i];

		//
		PrintStatus();
		UpdatePos(pos);
		break;

	case 0x360:
		//
		if (prevID != 0x360)
		{
			//if ( debug_on ) 
			//	timing << "36 " << TimeCheck() << "\n";
			prevID = 0x360;
		}

		//
		raw_pos[3] = ((char)rcvMsg.RDATA[0] << 8) + (unsigned char)rcvMsg.RDATA[1];/* Yaw/axis4 */
		raw_pos[4] = ((char)rcvMsg.RDATA[2] << 8) + (unsigned char)rcvMsg.RDATA[3];/* Pitch/axis5 */
		raw_pos[5] = ((char)rcvMsg.RDATA[4] << 8) + (unsigned char)rcvMsg.RDATA[5];/* Roll/axis6 */
		raw_pos[6] = ((char)rcvMsg.RDATA[6] << 8) + (unsigned char)rcvMsg.RDATA[7];//gripper


		//
		if (mode == AUTO_MODE)
			new_status = true;
		if (new_status)
			SetTransmitMessage(xmitMsg);
		else
		{
			xmitMsg.ID = rcvMsg.ID;
			xmitMsg.LEN = 0;
			xmitMsg.MSGTYPE = PCAN_MESSAGE_STANDARD;
		}

		// Cartesian mode.
		if (cbox == CARTESIAN)// read pos info from robot
		{
			// Bring back the values between -180 and 180.
			for (int i = 3; i < 6; i++)
			{
				while (raw_pos[i] > 1800)
					raw_pos[i] = (raw_pos[i] - 3600);
				while (raw_pos[i] < -1800)
					raw_pos[i] = (raw_pos[i] + 3600);
			}
			// Yaw, pitch, roll in degrees.
			for (int i = 3; i < 6; i++)
				pos[i] = 0.1* raw_pos[i];

			//

			pos[3] = pos[3] - 2;//-0*1.2
			pos[4] = pos[4];//-0*2.6
			pos[5] = pos[5];//zc -0*5.2
			if (pos[5] < -180.0)
				pos[5] = 360 - abs(pos[5]);
			if (reverse_flag)
				pos[5] = (pos[5] > 0) ? pos[5] - 180.0f : pos[5] + 180.0f;

			pos[6] = (raw_pos[6] > 0) ? (raw_pos[6] - 32768.0f) : (raw_pos[6] + 32768.0f);
		}

		//Convert raw data to angular mode
		if (cbox == JOINT)
		{
			for (int i = 0; i < 7; i++) {
				pos[i] = raw_pos[i];
			}
		}
		//UpdatePos(pos);
		//printPos(pos, 26, 26);
		break;

	case 0x37F:
		//
		if (prevID != 0x37F)
		{
			//if (debug_on) 
			//	timing << "37 " << TimeCheck() << "\n";
			prevID = 0x37F;
		}
		// Joint mode
		if (cbox == JOINT)
		{
			int joint3 = 0;
			int joint4 = 0;
			for (int i = 0; i < 7; i++) {
				pos[i] = raw_pos[i];
			}
			Apos[0] = pos[0];
			Apos[4] = pos[4];
			Apos[5] = pos[5];
			Apos[6] = pos[6];

			// Correction for dependancy between joint 2 and 3
			Apos[1] = (-1) * pos[1];
			joint3 = pos[2];
			joint3 = joint3 + 900 + Apos[1];

			if (joint3 >= 0)
				Apos[2] = 1800 - joint3;
			else
				Apos[2] = -1800 - joint3;

			joint4 = pos[3] + 900;
			Apos[3] = joint4;

			// Bring back the angles between -180 to 180
			for (int i = 0; i < 7; i++)
			{
				while (Apos[i] > 1800)
					Apos[i] = (Apos[i] - 3600);
				while (Apos[i] < -1800)
					Apos[i] = (Apos[i] + 3600);
			}

			for (int i = 0; i < 8; i++)
				Apos[i] = 0.1 * Apos[i];
			gotoxy(1, 21);
			/*printf("[q] %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);*/
			//printf("[q] %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \n",Apos[0], Apos[1], Apos[2], Apos[3], Apos[4], Apos[5]);
			DisplayPos(Apos);
			gotoxy(1, 35);
			cout << "[6R0] " << endl;
			cout << GWj_Transformation(Apos);
		}
		// cartesian mode
		else
		{
			DisplayPos(pos);

			gotoxy(1, 35);

			//cout << "[wRc] " << endl;
			//cout << C2W_transform(pos);

			Matrix<3, 3> Rc2ee;
			Rc2ee = 1, 0, 0, 0, 1, 0, 0, 0, 1;
			cout << "[ee2w] " << endl;
			cout << EE2w_transform3(pos);

			//float pd_c[6];
			//if ( write_init_pos )
			//{
			//	FILE* fid = fopen("curr_points.txt","r");
			//	for (int i=0; i<6; i++)
			//		fscanf_s(fid,"%f ",&pd_c[i]);
			//	fclose(fid);
			//}

			//Matrix<3,1> dtw, dtcn;	
			//Matrix<3,3> Rcn2w, Rcn2co, Rco2w, Rc2ee;

			//dtw = pos[0] - pd_c[0], pos[1] - pd_c[1], pos[2] - pd_c[2];
			////cout << endl << dtw << endl; // wz
			//Rcn2w = C2W_transform( pos );
			////cout << endl << Rcn2w << endl; // wz
			//Rco2w = C2W_transform( pd_c );
			////cout << endl << Rco2w << endl; // wz
			//Rcn2co = transpose(Rco2w)*Rcn2w;
			////cout << endl << Rcn2co << endl; // wz
			//Rc2ee = 0,0,1,1,0,0,0,1,0;
			//dtcn = transpose( Rcn2w ) * dtw;
			////cout << endl << dtcn << endl; // wz

			//Matrix<3,4> mco, mcn;
			//mco = 1,-1,-1,1,1,1,-1,-1,1,1,1,1;
			//for (int i=0; i<4; i++)
			//{
			//	mco(1,i+1) = mco(1,i+1)/mco(3,i+1);	// convert into projective coord.
			//	mco(2,i+1) = mco(2,i+1)/mco(3,i+1);
			//	mco(3,i+1) = 1;	
			//}
			//mcn = transpose(Rcn2co)*mco;
			//for ( int i = 0; i < 4; i++ )
			//	for (int j = 0; j < 3; j++ )
			//		mcn(j+1,i+1) = mcn(j+1,i+1) - dtcn(j+1,1);

			//for ( int i = 0; i < 4; i++ )
			//{
			//	mcn(1,i+1) = mcn(1,i+1)/mcn(3,i+1);	// convert into projective coord.
			//	mcn(2,i+1) = mcn(2,i+1)/mcn(3,i+1);
			//	mcn(3,i+1) = 1;	
			//}

			//if ( write_init_pos )
			//{
			//	printf("%.2f %.2f %.2f %.2f %.2f %.2f \n",dtw(1,1),dtw(2,1),dtw(3,1),dtcn(1,1),dtcn(2,1),dtcn(3,1));
			//	printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \n",mco(1,1),mco(2,1),mco(1,2),mco(2,2),mco(1,3),mco(2,3),mco(1,4),mco(2,4));
			//	printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \n",mcn(1,1),mcn(2,1),mcn(1,2),mcn(2,2),mcn(1,3),mcn(2,3),mcn(1,4),mcn(2,4));
			//	for ( int i = 0; i <3 ; i++ )
			//	{
			//		for ( int j = 0; j < 3; j++ )
			//			printf("%.2f ", Rcn2co( i + 1, j + 1 ) );
			//		printf(" ");
			//		printf(" ");
			//		printf("\n");
			//	}
			//}
		}

		if (mode == AUTO_MODE)
		{
			if (!auto_mode_start)	// start flag!
			{
				auto_mode_start = true;
				if (home_pos_flag == true)	// go to preset home position 1, 2, 3, 4.
				{
					char home_pos_str[256];
					if (pos_index != 911)
					{
						sprintf_s(home_pos_str, "C:\\MANUS\\CommonSpace\\Setting\\home_pos%d.txt", pos_index);
						FILE* fid2 = fopen(home_pos_str, "r");
						if (fid2 != NULL)
						{
							float pd_h[6] = {};
							for (int i = 0; i < 6; ++i)
							{
								fscanf_s(fid2, "%f ", &pd_h[i]);
								pd[i] = (pd_h[i] == -1) ? pos[i] : pd_h[i];
							}
							fclose(fid2);
						}
						else
						{
							cerr << "[Error]: Can not find home_pos file!" << endl;
							for (int i = 0; i < 6; ++i)
								pd[i] = pos[i];
						}
					}
					else // pos_index = 911
					{
						bool empty_flag = false;
						adjust_queue_index(-1);
						for (int i = 0; i < 6; ++i)
						{
							if (good_pos[pos_queue_index][i])
								pd[i] = good_pos[pos_queue_index][i];
							else
							{
								empty_flag = true;
								pd[i] = pos[i];
							}
						}
						if (empty_flag)
							adjust_queue_index(+1);
					}
				}
			}
			if (!job_complete)
			{
				if (test_x_flag)
				{
					cout << endl << "test_x_flag is true!" << endl;
					pd_controlx();
				}
				else if (cbox == CARTESIAN)
					pd_control2();	//	y*,p*,and r* are estimated from the homography for automatic control. scaled translation will be used...
				else if (cbox == JOINT)
					//checkOneSecond();
					pd_controlJoint();
			}
		}
		else
			SetTransmitMessage(xmitMsg);

		break;
	}
}

// This function will set the ID, lenght of package and 8 bits of data into the CAN buffer.
void SetTransmitMessage(TPCANMsg& xmitMsg)
{
	//LPScheck();
	//// pitch limit



	if (block_flag)
	{
		for (int j = 0; j < 6; j++)
		{
			if (block_dir[j] == 2)
				speed[j + 1] = 0;
			else if (block_dir[j] == -1 && speed[j + 1] < 0)
				speed[j + 1] = 0;
			else if (block_dir[j] == 1 && speed[j + 1] > 0)
				speed[j + 1] = 0;
		}
	}

	if ((pos[4] < -80) || (pos[4] > 80))
	{
		if ((speed[5] < 0) && (pos[4] < -80))
			speed[5] = 0;

		if ((speed[5] > 0) && (pos[4] > 80))
			speed[5] = 0;// block the movement in the direction before sending the velocity out
	}

	xmitMsg.MSGTYPE = PCAN_MESSAGE_STANDARD; //Standard message type for all cboxes
	switch (cbox)
	{
	case CARTESIAN:
		xmitMsg.ID = 0x371;
		xmitMsg.LEN = 8;
		for (int i = 0; i < xmitMsg.LEN; i++)
		{
			// Cartesian movement data.	
			xmitMsg.RDATA[i] = (BYTE)speed[i];
			//gotoxy(1, 52);
			//cout << TimeCheck() << endl;
		}
		break;
	case JOINT:
		xmitMsg.ID = 0x374;
		xmitMsg.LEN = 8;
		for (int i = 0; i < xmitMsg.LEN; i++)
			// Joint movement data.
			xmitMsg.RDATA[i] = speed[i];
		break;
	case FOLD_OUT:
		xmitMsg.ID = 0x375;
		xmitMsg.LEN = 0;
		break;
	case FOLD_IN:
		xmitMsg.ID = 0x376;
		xmitMsg.LEN = 0;
		break;
	default:
		xmitMsg.ID = 0x370;
		xmitMsg.LEN = 0;
		break;
	}
}

// only rotation tracking. --071220 KIM
int pd_control2(void)
{
	// INITIALIZATION
	float Kp[6] = { 2, 2, 2, 0.8, 0.7, 0.6 };

	float pprev1[6] = {};
	float eprev1[6] = {};

	Matrix<3, 1> error_YPR;

	/* Read position information sent by robot */
	for (int i = 0; i < 6; i++)
		pprev1[i] = pos[i];									    // p is read off the robot

	// 010308 control
	if (home_pos_flag == true)
	{
		// go to preset pos
		register float tmp1, tmp2;
		for (int i = 0; i < 6; ++i)
		{
			if (i == 5) // roll -180 ~ 180 : linear scailing
			{
				tmp1 = (180.0f - pd[i]) - (180.0f - pprev1[i]);
				if (tmp1 >= 0.0f)
					if (tmp1 <= 180.0f)
						tmp2 = tmp1;
					else
						tmp2 = tmp1 - 360.0f;
				else
					if (tmp1 > -180.0f)
						tmp2 = tmp1;
					else
						tmp2 = 360.0f + tmp1;

				eprev1[i] = tmp2;
			}
			else
				eprev1[i] = pd[i] - pprev1[i];

			float control_input = Kp[i] * eprev1[i];

			if (i < 3)
				speed[i + 1] = (fabs(control_input) > linear_speed_limit[speed_mode]) ? sign(control_input) * linear_speed_limit[speed_mode] : control_input;
			else
				speed[i + 1] = (fabs(control_input) > angular_speed_limit[speed_mode]) ? sign(control_input) * angular_speed_limit[speed_mode] : control_input;
		}
	}
	else if (adjust_pos == true)
	{
		// go to preset pos
		register float tmp1, tmp2;
		for (int i = 0; i < 6; ++i)
		{
			if (i == 5) // roll -180 ~ 180 : linear scailing
			{
				tmp1 = (180.0f - pd[i]) - (180.0f - pprev1[i]);
				if (tmp1 >= 0.0f)
					if (tmp1 <= 180.0f)
						tmp2 = tmp1;
					else
						tmp2 = tmp1 - 360.0f;
				else
					if (tmp1 > -180.0f)
						tmp2 = tmp1;
					else
						tmp2 = 360.0f + tmp1;

				eprev1[i] = tmp2;
			}
			else
				eprev1[i] = pd[i] - pprev1[i];

			float control_input = Kp[i] * eprev1[i];

			if (i < 3)
				speed[i + 1] = (fabs(control_input) > linear_speed_limit[speed_mode]) ? sign(control_input) * linear_speed_limit[speed_mode] : control_input;
			else
				speed[i + 1] = (fabs(control_input) > angular_speed_limit[speed_mode]) ? sign(control_input) * angular_speed_limit[speed_mode] : control_input;
		}
	}
	else if (set_point_vel)	/* Gross Motion */
	{
		// gross motion
		float control_input, c_err;
		float w_lim = 8.0f, v_lim = 100.0f;
		float Kv[6] = { 5.0f, 5.0f, 5.0f, 20.0f, 20.0f, 20.0f };
		float convert = (M_PI / 36.0f) / (50.0f * 0.022f);

		Matrix<3, 1> dee2c, Vc, Vw, Ww, Wc, Wypr, We, Rde, WeRde, YPR_star;
		Matrix<3, 3> Rc2w, Rxyz2ypr, nRxyz2ypr, Rc2ee, Rc2eep, nRc2eep;
		Matrix<3, 3> Ree2w, Rypr2xyz, nRypr2xyz;
		dee2c = d_c_ee;
		Rc2ee = 0, 0, 1, 1, 0, 0, 0, 1, 0;

		Matrix<3, 1> Red;
		Red = EE2C_transform(pos) * dee2c;

		for (int i = 0; i < 3; ++i)
		{
			eprev1[i] = pd[i] - (pprev1[i] + Red(i + 1, 1));
			Vc(i + 1, 1) = Kv[i] * eprev1[i];
			YPR_star(i + 1, 1) = pd[i + 3];
		}

		/* Get errors and Wypr  rotation error */
		for (int i = 0; i < 3; ++i)
		{
			if (i == 2)
			{
				c_err = (YPR_star(i + 1, 1) - pos[i + 3]);
				c_err = (fabs(c_err) < 180.0f) ? -c_err : -(c_err - sign(c_err) * 360.0f);
				error_YPR(i + 1, 1) = c_err * M_PI / 180.0f;
			}
			else
				error_YPR(i + 1, 1) = (YPR_star(i + 1, 1) - pos[i + 3]) * M_PI / 180.0f;

			Wypr(i + 1, 1) = Kv[i + 3] * error_YPR(i + 1, 1);
		}

		if (TimeCheck() - start_time_set_point_vel >= 60)
		{
			start_time_set_point_vel = TimeCheck();

			Ree2w = EE2C_transform(pos);
			Rypr2xyz = 0, Ree2w(2, 1), Ree2w(1, 1),
				0, -Ree2w(1, 1), Ree2w(2, 1),
				1, 0, Ree2w(3, 1);
			nRypr2xyz = NormalizeRotationMatrix(Rypr2xyz);

			/* Limit Angular Velocity */
			for (int i = 0; i < 3; ++i)
			{
				control_input = Wypr(i + 1, 1);
				Wypr(i + 1, 1) = (fabs(control_input) > w_lim) ? (float)(sign(control_input) * w_lim) : (float)control_input;
			}

			/* Get We */
			Rc2eep = transpose(Ree2w) * nRypr2xyz;
			nRc2eep = NormalizeRotationMatrix(Rc2eep);
			Matrix<3, 1> Wypr2;
			for (int i = 0; i < 3; ++i)
				Wypr2(i + 1, 1) = Wypr(i + 1, 1) * M_PI / 180.0f;
			We = nRc2eep * Wypr2;	/* in degrees */

			Rde = We(2, 1) * dee2c(3, 1) - We(3, 1) * dee2c(2, 1),
				We(3, 1) * dee2c(1, 1) - We(1, 1) * dee2c(3, 1),
				We(1, 1) * dee2c(2, 1) - We(2, 1) * dee2c(1, 1);

			for (int i = 0; i < 3; ++i)
				Rde(i + 1, 1) = Rde(i + 1, 1) * convert;

			WeRde = Ree2w * Rde;
			Vw = Vc - WeRde;	/* CAUTION:: Here, Vc is not a velocity in the camera coord. but in the world coord. */

			/* Limit Linear Velocity */
			for (int i = 0; i < 3; ++i)
			{
				control_input = Vw(i + 1, 1);
				Vw(i + 1, 1) = (fabs(control_input) > v_lim) ? (float)(sign(control_input) * v_lim) : (float)control_input;
			}

			/* Set Speed Variables of MANUS ARM */
			for (int i = 0; i < 3; ++i)
				speed[i + 1] = (int)Vw(i + 1, 1);

			for (int i = 0; i < 3; ++i)
				speed[i + 4] = (int)Wypr(i + 1, 1);

			if (debug_on)
			{
				sprintf_s(user_str, "%scontrol_gross.log", debug_path);
				FILE* fp = fopen(user_str, "a");
				fprintf(fp, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n",
					start_time_set_point_vel, eprev1[0], eprev1[1], eprev1[2],
					error_YPR(1, 1), error_YPR(2, 1), error_YPR(3, 1), pos[0], pos[1], pos[2], pos[3], pos[4], pos[5],
					Vw(1, 1), Vw(2, 1), Vw(3, 1), Vc(1, 1), Vc(2, 1), Vc(3, 1), WeRde(1, 1), WeRde(2, 1), WeRde(3, 1), Wypr(1, 1), Wypr(2, 1), Wypr(3, 1));
				fclose(fp);
			}
		}
	}
	else
	{
		// return to user, 
		register float tmp1, tmp2;
		// translation (set-point move),
		for (int i = 0; i < 3; ++i)
		{
			eprev1[i] = pd[i] - pprev1[i];
			float control_input = Kp[i] * eprev1[i];
			float ref_velocity = (float)linear_speed_limit[speed_mode] * speed_factor;
			speed[i + 1] = (fabs(control_input) > ref_velocity) ? (int)sign(control_input) * ref_velocity : (int)control_input;
		}

		// rotation
		for (int i = 3; i < 6; ++i)
		{
			if (i == 5) // roll -180 ~ 180 : linear scailing
			{
				tmp1 = (180.0f - pd[i]) - (180.0f - pprev1[i]);
				if (tmp1 >= 0.0f)
				{
					if (tmp1 <= 180.0f)
						tmp2 = tmp1;
					else
						tmp2 = tmp1 - 360.0f;
				}
				else
				{
					if (tmp1 > -180.0f)
						tmp2 = tmp1;
					else
						tmp2 = 360.0f + tmp1;
				}

				eprev1[i] = tmp2;
			}
			else
				eprev1[i] = pd[i] - (pprev1[i]);

			float control_input = Kp[i] * eprev1[i];
			speed[i + 1] = (fabs(control_input) > angular_speed_limit[speed_mode]) ? (int)sign(control_input) * angular_speed_limit[speed_mode] : (int)control_input;
		}
	}

	/* SHOW SPEED/TARGET/ERROR */
	gotoxy(1, 22);
	printf("[S] %.0f %.0f %.0f %.0f %.0f %.0f %.0f %.0f \n", speed[0], speed[1], speed[2], speed[3], speed[4], speed[5], speed[6], speed[7]);
	gotoxy(1, 23);
	printf("[T] %.2f %.2f %.2f %.2f %.2f %.2f \n", pd[0], pd[1], pd[2], pd[3], pd[4], pd[5]);
	gotoxy(1, 25);
	printf("[E] %.2f %.2f %.2f %.2f %.2f %.2f \n", eprev1[0], eprev1[1], eprev1[2], eprev1[3], eprev1[4], eprev1[5]);

	////  Mushtaq  enable camera collision check on auto motion
	float cam_dist;
	cam_dist = DistanceBetween_Camera_Link3(pos);
	gotoxy(1, 41);
	printf("The closest distance between the camera and link3 %.3f", cam_dist);
	if ((cam_dist < 38) && !cam_cls)
	{
		ResetAll();
		cam_cls = true;
		//UpdateSpaceMouse(spacemousearray);
	}
	else if (cam_dist > 38 && cam_cls)
	{
		cam_cls = false;
	}


	/* [S] CONTROL OBJECTIVE */
	if (home_pos_flag == true)	// set to home position
	{
		if ((fabs(eprev1[0]) < T_ERR_BOUND) & (fabs(eprev1[1]) < T_ERR_BOUND) & (fabs(eprev1[2]) < T_ERR_BOUND)) //xyz position control
		{
			if ((fabs(eprev1[3]) < R_ERR_BOUND) & (fabs(eprev1[4]) < R_ERR_BOUND))	// yaw & pitch rotation control
			{
				if (fabs(eprev1[5]) < R_ERR_BOUND)	// roll rotation control
				{
					job_done = true;
					auto_mode_start = false;
					home_pos_flag = false;
					block_all_motions = false;
				}
			}
			else
				speed[6] = 0;	// set roll motion to 0
		}
		else
		{
			for (int i = 3; i < 6; i++)	// set all the rotation motion to 0
				speed[i + 1] = 0;
		}
	}
	else if (adjust_pos == true)	// set to adjust position
	{
		if ((fabs(eprev1[0]) < T_ERR_BOUND) & (fabs(eprev1[1]) < T_ERR_BOUND) & (fabs(eprev1[2]) < T_ERR_BOUND)) //xyz position control
		{
			if ((fabs(eprev1[3]) < R_ERR_BOUND) & (fabs(eprev1[4]) < R_ERR_BOUND))	// yaw & pitch rotation control
			{
				if (fabs(eprev1[5]) < R_ERR_BOUND)	// roll rotation control
				{
					job_done = true;
					auto_mode_start = false;
					adjust_pos = false;
					block_all_motions = false;
					//adjust_pos_end = true;
				}
			}
			else
				speed[6] = 0;	// set roll motion to 0
		}
		else
		{
			for (int i = 3; i < 6; i++)	// set all the rotation motion to 0
				speed[i + 1] = 0;
		}

	}
	else if (set_point_vel)
	{
		if ((fabs(eprev1[0]) < T_ERR_BOUND)
			&& (fabs(eprev1[1]) < T_ERR_BOUND)
			&& (fabs(eprev1[2]) < T_ERR_BOUND)
			&& (fabs(error_YPR(1, 1)) < R_V_ERR_YAW)
			&& (fabs(error_YPR(2, 1)) < R_V_ERR_PITCH)
			&& (fabs(error_YPR(3, 1)) < R_V_ERR_ROLL))
		{
			set_point_vel = false;
			set_points_control = false;
			job_done = true;
			auto_mode_start = false;
			home_pos_flag = false;
			block_all_motions = false;

			/* SEND ACK TO VIS */
			ClientSocketComm(CAN, VIS, WRITE_SET_POS_DONE, NULL);
		}
	}
	else
	{
		// translation - set-point control
		if ((fabs(eprev1[0]) < T_ERR_BOUND) && (fabs(eprev1[1]) < T_ERR_BOUND) && (fabs(eprev1[2]) < T_ERR_BOUND)
			&& (fabs(eprev1[3]) < R_ERR_BOUND) && (fabs(eprev1[4]) < R_ERR_BOUND) && (fabs(eprev1[5]) < R_ERR_BOUND))
		{
			set_points_control = false;
			job_done = true;
			auto_mode_start = false;
			home_pos_flag = false;
			block_all_motions = false;
			ClientSocketComm(CAN, VIS, WRITE_SET_POS_DONE, NULL);
		}
	}
	/* [E] CONTROL OBJECTIVE */


	return 0;
}

int checkOneSecond(void) {
	if (oneSecondStart == 0) {
		oneSecondStart = TimeCheck();
		speed[1] = 1;
	}
	int CurrentTime = TimeCheck();
	ofstream position_data;
	position_data.open("C:\\MANUS\\CommonSpace\\Position_data\\oneSecondTest.txt", std::ios_base::app);
	position_data << TimeCheck() << ", ";
	for (int i = 0; i < 6; i++)
		position_data << pd[i] << ", " << pos[i] * .1f << ", " << speed[i + 1] << ", ";
	position_data << "\n";
	if (abs(CurrentTime - oneSecondStart) > 60) {
		speed[1] = 0;

	}
	if (abs(CurrentTime - oneSecondStart) > 2500) {
		speed[1] = 0;
		job_done = true;
		auto_mode_start = false;
		block_all_motions = false;
		speed[1] = speed[2] = speed[3] = speed[4] = speed[5] = speed[6] = 0;
		SendCommand(CAN, VIS, 'Q', job_done);
		oneSecondStart = 0;
	}



	return 0;
}
//int checkOneSecond(void){
//	if(oneSecondStart == 0){
//		oneSecondStart = TimeCheck();
//		speed[1] = 1;
//	}
//	int CurrentTime = TimeCheck();
//	ofstream position_data;
//	position_data.open("C:\\MANUS\\CommonSpace\\Position_data\\oneSecondTest.txt", std::ios_base::app);
//	position_data << TimeCheck() <<", ";
//    for(int i = 0;i<6;i++)
//		position_data << pd[i] << ", " << pos[i]*.1f << ", "<< speed[i+1] << ", ";
//	position_data << "\n";
//	if(abs(CurrentTime - oneSecondStart) > 1000){
//		speed[1] = 0;
//
//	}
//	if(abs(CurrentTime - oneSecondStart) > 2500){
//		speed[1] = 0;
//		job_done = true;
//		auto_mode_start = false;
//		block_all_motions = false;
//		speed[1] = speed[2] = speed[3] = speed[4] = speed[5] = speed[6]= 0;
//		SendCommand(CAN,VIS,'Q',job_done);
//		oneSecondStart = 0;
//	}
//
//
//
//	return 0;
//}

int pd_controlJoint(void)
{
	int CurrentTime[6];
	// INITIALIZATION
	bool useRawJoints = true;
	float pprev1[6] = {};
	float eprev1[6] = {};

	// Joint control.
	if (!useRawJoints) {
		/* Read position information sent by robot */
		for (int i = 0; i < 6; i++)
			pprev1[i] = pos[i];									    // p is read off the robot
		int joint3 = 0;
		int joint4 = 0;
		Apos[0] = pos[0];
		Apos[4] = pos[4];
		Apos[5] = pos[5];
		Apos[6] = pos[6];

		// Correction for dependancy between joint 2 and 3
		Apos[1] = (-1) * pos[1];
		joint3 = pos[2];
		joint3 = joint3 + 900 + Apos[1];

		if (joint3 >= 0)
			Apos[2] = 1800 - joint3;
		else
			Apos[2] = -1800 - joint3;

		joint4 = pos[3] + 900;
		Apos[3] = joint4;

		// Bring back the angles between -180 to 180
		for (int i = 0; i < 7; i++)
		{
			while (Apos[i] > 1800)
				Apos[i] = (Apos[i] - 3600);
			while (Apos[i] < -1800)
				Apos[i] = (Apos[i] + 3600);
		}

		for (int i = 0; i < 8; i++)
			Apos[i] = 0.1 * Apos[i];
		int flip;
		float Kp[6] = { 1.1, 1, 1.1, 1, 1, 1 };
		for (int i = 0; i < 6; i++)
		{
			eprev1[i] = pd[i] - Apos[i];
			//Some of the joints needed to have their directions corrected from error.
			switch (i) {
			case 0:
				flip = -1;
				break;
			case 1:
				flip = 1;
				break;
			case 2:
				flip = 1;
				break;
			case 3:
				flip = -1;
				break;
			case 4:
				flip = 1;
				break;
			case 5:
				flip = -1;
				break;
			default:
				flip = 1;
				break;
			}
			float control_input = Kp[i] * eprev1[i] * flip;

			float speed_limit = angular_speed_limit[0] * 1.0f;
			speed[i + 1] = (fabs(control_input) > speed_limit) ? sign(control_input) * speed_limit : control_input;
		}
		if ((fabs(eprev1[0]) < J_ERR_BOUND) & (fabs(eprev1[1]) < J_ERR_BOUND) & (fabs(eprev1[2]) < J_ERR_BOUND) & (fabs(eprev1[3]) < J_ERR_BOUND) & (fabs(eprev1[4]) < J_ERR_BOUND) & (fabs(eprev1[5]) < J_ERR_BOUND))
		{
			job_done = true;
			auto_mode_start = false;
			block_all_motions = false;
			speed[1] = speed[2] = speed[3] = speed[4] = speed[5] = speed[6] = 0;
			SendCommand(CAN, VIS, 'Q', job_done);
		}
	}
	else if (useRawJoints) {
		int flip;

		float Kp[6] = { 1, 1, 1, 1, 1, 1 };
		//float Ki[6]={0.05,0.0,0.0,0.0,0.0,0.0};
		float Kd[6] = { 0.3, 0.3, 0.3, 0.3, 0.3, 0.3 };
		for (int i = 0; i < 6; i++)
		{
			switch (i) {
			case 0:
				flip = -1;
				break;
			case 1:
				flip = -1;
				break;
			case 2:
				flip = -1;
				break;
			case 3:
				flip = -1;
				break;
			case 4:
				flip = 1;
				break;
			case 5:
				flip = -1;
				break;
			default:
				flip = 1;
				break;
			}

			//Iteration Time
			CurrentTime[i] = TimeCheck();

			int DeltaT = abs(CurrentTime[i] - PrevTime[i]);

			float dDiff;

			eprev1[i] = (pd[i] - (pos[i] * .1f))*flip;
			dDiff = (eprev1[i] - oldDiff[i]) / DeltaT;

			float control_input = Kp[i] * eprev1[i] + Kd[i] * dDiff;

			oldDiff[i] = eprev1[i];
			PrevTime[i] = CurrentTime[i];

			float speed_limit = /*angular_speed_limit[0]*/1.0f * 1.0f;
			speed[i + 1] = (fabs(control_input) > speed_limit) ? sign(control_input) * speed_limit : control_input;

		}
		if ((fabs(eprev1[0]) < J_ERR_BOUND) & (fabs(eprev1[1]) < J_ERR_BOUND) & (fabs(eprev1[2]) < J_ERR_BOUND) & (fabs(eprev1[3]) < J_ERR_BOUND) & (fabs(eprev1[4]) < J_ERR_BOUND) & (fabs(eprev1[5]) < J_ERR_BOUND))
		{
			//for(int i = 0;i<6;i++){
			//	oldDiff[i] = 0;
			//	PrevTime[i] = 0;
			//}
			job_done = true;
			auto_mode_start = false;
			block_all_motions = false;
			speed[1] = speed[2] = speed[3] = speed[4] = speed[5] = speed[6] = 0;
			SendCommand(CAN, VIS, 'Q', job_done);
		}
	}
	/* SHOW SPEED/TARGET/ERROR */
	gotoxy(1, 21);
	printf("[q] %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \n", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
	gotoxy(1, 22);
	printf("[S] %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \n", speed[0], speed[1], speed[2], speed[3], speed[4], speed[5], speed[6], speed[7]);
	gotoxy(1, 23);
	printf("[T] %.2f %.2f %.2f %.2f %.2f %.2f \n", pd[0], pd[1], pd[2], pd[3], pd[4], pd[5]);
	gotoxy(1, 25);
	printf("[E] %.2f %.2f %.2f %.2f %.2f %.2f \n", eprev1[0], eprev1[1], eprev1[2], eprev1[3], eprev1[4], eprev1[5]);

	//Prints requested position, current position, and speed to text file
	ofstream position_data;
	position_data.open("C:\\MANUS\\CommonSpace\\Position_data\\positions.txt", std::ios_base::app);
	position_data << TimeCheck() << ", ";
	for (int i = 0; i < 6; i++)
		position_data << pd[i] << ", " << pos[i] * .1f << ", " << speed[i + 1] << ", ";
	position_data << "\n";
	return 0;
}


//
int pd_controlx()
{
	cout << "[Warning!]: pd_controlx mode!" << endl;
	ColumnVector<3> Vw, Ww, Wc, We, Wypr, WeRde, Rde, Pc, Dypr;
	Matrix<3, 3> Rc2w, Ree2w, Rypr2w;
	float convert = (M_PI / 36.0f) / (50.0f * 0.022f);
	float dt, dy, dp, dr;

	Rc2w = C2W_transform(pos);
	Ree2w = EE2C_transform(pos);

	Rypr2w = 0, Ree2w(2, 1), Ree2w(1, 1),
		0, -Ree2w(1, 1), Ree2w(2, 1),
		1, 0, Ree2w(3, 1);

	Wc = wwg;
	Wypr = inverse(Rypr2w) * Rc2w * Wc;

	pctime = TimeCheck();
	dt = fabs((float)(pctime - potime));

	dy = (pos[3] - opos[3]);
	dp = (pos[4] - opos[4]);

	float tmp1 = (180.0f - pos[5]) - (180.0f - opos[5]);
	dr = (tmp1 >= 0.0f) ? ((tmp1 <= 180.0f) ? tmp1 : tmp1 - 360.0f) : ((tmp1 > -180.0f) ? tmp1 : 360.0f + tmp1);

	cypr[0] = 20.0f*dy / dt;	cypr[1] = 20.0f*dp / dt;	cypr[2] = 20.0f*dr / dt;

	for (int i = 0; i < 3; ++i) {
		dypr[i] = oypr[i] + (cypr[i] - oypr[i]) * (dt / 1000.0f) * 5.0f;
		Dypr(i + 1) = (fabs(dypr[i]) > 0) ? dypr[i] : 0.0f;
	}

	potime = pctime;
	for (int i = 0; i < 3; ++i) {
		opos[i + 3] = pos[i + 3];
		oypr[i] = dypr[i];
	}

	We = transpose(Ree2w) * Rypr2w * Dypr;	// [deg]
	Rde = We(2) * d_c_ee(3, 1) - We(3) * d_c_ee(2, 1),
		We(3) * d_c_ee(1, 1) - We(1) * d_c_ee(3, 1),
		We(1) * d_c_ee(2, 1) - We(2) * d_c_ee(1, 1);

	for (int i = 0; i < 3; ++i)	Rde(i + 1) = -Rde(i + 1)/* * convert*/;

	WeRde = Ree2w * Rde;
	Vw = WeRde;

	speed[1] = (int)Vw(1);
	speed[2] = (int)Vw(2);
	speed[3] = (int)Vw(3);
	speed[4] = (int)Wypr(1);
	speed[5] = (int)Wypr(2);
	speed[6] = (int)Wypr(3);

	Pc = pos[0], pos[1], pos[2];
	Pc = Pc + Ree2w * d_c_ee;

	FILE *fp = fopen("C:\\MANUS\\CommonSpace\\Run\\outc.txt", "a");
	fprintf(fp, "%d ", TimeCheck());
	for (int i = 0; i < 6; ++i) fprintf(fp, "%.3f ", pos[i]);
	for (int i = 0; i < 3; ++i) fprintf(fp, "%.3f ", Pc(i + 1));
	for (int i = 0; i < 3; ++i) fprintf(fp, "%.3f ", Vw(i + 1));
	for (int i = 0; i < 3; ++i) fprintf(fp, "%.3f ", Wypr(i + 1));
	for (int i = 0; i < 3; ++i) fprintf(fp, "%.3f ", We(i + 1));
	for (int i = 0; i < 3; ++i) fprintf(fp, "%.3f ", Rde(i + 1));
	for (int i = 0; i < 3; ++i) fprintf(fp, "%.3f ", WeRde(i + 1));
	fprintf(fp, "\n");
	fclose(fp);

	return 0;
}

//=============================================================================
// Miscellaneous.
//=============================================================================
// Adjust the order in the command queue.
void adjust_queue_index(int seg)
{
	pos_queue_index += seg;

	if (pos_queue_index > QUE_SIZE - 1)
		pos_queue_index = (pos_queue_index - QUE_SIZE);
	else if (pos_queue_index < 0)
		pos_queue_index = (QUE_SIZE + pos_queue_index);
}

// Time check functions.
int TimeCheck()
{
	QueryPerformanceFrequency(&tickspersecond);
	QueryPerformanceCounter(&tick);
	tick_time.QuadPart = 1000 * tick.QuadPart / tickspersecond.QuadPart;
	return ((int)tick_time.QuadPart);
}

// Load dll, connect to shared memory and TCP.
bool LoadAll(void)
{
	// Create a mutex. For Sensor program sync.
	mutex = CreateMutex(NULL, true, _T("Sensor"));

	// Shared memory connection.
	if (!robot_pos.OpenMappedMemory("ROBOTPOS"))
	{
		cout << "[Error!]: Shared memory robot_pos is not available!" << endl;
		return false;
	}
	else
		cout << "[Info]: Shared memory robot_pos is available!" << endl;
	if (!force.OpenMappedMemory("FORCE"))
	{
		cout << "[Error!]: Shared memory force is not available!" << endl;
		return false;
	}
	if (!LPS.OpenMappedMemory("LPS"))
	{
		cout << "[Error!]: Shared memory LPS is not available!" << endl;
		return false;
	}
	if (!slip_vel.OpenMappedMemory("SLIPVEL"))
	{
		cout << "[Error!]: Shared memory slip_vel is not available!" << endl;
		return false;
	}
	if (!pos_vel.OpenMappedMemory("POSIT"))
	{
		cout << "[Error!]: Shared memory pos_vel is not available!" << endl;
		return false;
	}
	if (!takktile.OpenMappedMemory("TAKKTILE"))
	{
		cout << "[Error!]: Shared memory takktile is not available!" << endl;
		return false;
	}
	if (!spaceMouseValues.OpenMappedMemory("SPACEMOUSE"))
	{
		cout << "[Error!]: Shared memory takktile is not available!" << endl;
		return false;
	}
	if (!sug_speed.OpenMappedMemory("sug speed"))
	{
		cout << "[Error!]: Shared memory takktile is not available!" << endl;
		return false;
	}

	if (!block_direction.OpenMappedMemory("block"))
	{
		cout << "[Error!]: Shared memory takktile is not available!" << endl;
		return false;
	}


	if (!Obj_in.OpenMappedMemory("OBJ"))
	{
		cout << "[Error!]: Shared memory OBJ is not available!" << endl;
		return false;
	}
	// Network connection.
	if (!thread_CAN.CreateThread(sthread_CAN, NULL))
	{
		cout << "[Error!]: Communication thread is not available!" << endl;
		return false;
	}
	else
		cout << "[Info]: Communication thread is available!" << endl;

	// Initialize the PCANBasicClass object.
	stsResult = m_objPCANBasic.Initialize(m_Handle, PCAN_BAUD_250K);
	if (stsResult != PCAN_ERROR_OK)
	{
		cout << "[Error!]: PCAN can not be initialized!" << endl;
		return false;
	}
	else
		cout << "[Info]: PCAN is initialized successfully!" << endl;

	// Check to see the initialization packet from CAN card 0x0D4 and 0x4D4.
	if (!Read_D4())
	{
		cout << "[Error!]: 0x0D4 and 0x4D4 can not be read!" << endl;
		return false;
	}
	else
		cout << "[Info]: 0x0D4 and 0x4D4 are read from CAN bus!" << endl;

	// Read the initial position of the MANUS arm.
	if (!Read_350_360())
	{
		cout << "[Error!]: 0x350 and 0x360 can not be read!" << endl;
		return false;
	}
	else
		cout << "[Info]: 0x350 and 0x360 are read from CAN bus!" << endl;

	// Check and wait until the first 37F message is received.
	if (!Read_37F())
	{
		cout << "[Error!]: 0x37F can not be read!" << endl;
		return false;
	}
	else
		cout << "[Info]: 0x37F is read from CAN bus!" << endl;

	// Opening the grabber
	if (!Open_Grabber())
	{
		cout << "[Error!]: Grabber can not be open!" << endl;
		return false;
	}
	else
		cout << "[Info]: Grabber is open!" << endl;

	// Release and destroy the mutex.
	ReleaseMutex(mutex);
	CloseHandle(mutex);
	return true;
}

// Unload dll of CAN card, disconnect to shared memory and TCP.
bool UnloadAll(void)
{
	// Uninitialize PCAN
	stsResult = m_objPCANBasic.Uninitialize(m_Handle);
	if (stsResult != PCAN_ERROR_OK)
	{
		cout << "[Error!]: Fail to Uninitialize PCAN!" << endl;
		return false;
	}

	// Close TCP
	if (!thread_CAN.StopThread())
	{
		cout << "[Error!]: Fail to Uninitialize TCP connection!" << endl;
		return false;
	}

	// Close shared memory
	if (!robot_pos.CloseMappedMemory())
	{
		cout << "[Error!]: Fail to Uninitialize shared memory!" << endl;
		return false;
	}

	// Close debug
	//if ( debug_on )	
	//	timing.close(); 

	return true;
}

char reliable(int num, char val)
{
	switch (num)
	{
	case 0:
		break;
	case 1:
		val = 'i';
		break;
	case 2:
		val = 'k';
		break;
	case 3:
		val = 'q';
		break;
	case 4:
		val = 'a';
		break;
	case 5:
		val = 'w';
		break;
	case 6:
		val = 's';
		break;
	case 7:
		val = 'e';
		break;
	case 8:
		val = 'd';
		break;
	case 9:
		val = 'r';
		break;
	case 10:
		val = 'f';
		break;
	case 11:
		val = 't';
		break;
	case 12:
		val = 'g';
		break;
	case 13:
		val = 'y';
		break;
	case 14:
		val = 'h';
		break;
	case 15:
		val = 'u';
		break;
	case 16:
		val = 'j';
		break;
	}

	return val;
}

void ReadForce(float cur_for)
{
	register char read_pos_str[256], read_str[256];

	force->SetReadPos(0);
	force.Lock();
	force->Read((unsigned char *)read_pos_str, 7 * sizeof(BYTE), 0);
	force.Unlock();

	memcpy(read_str, read_pos_str, 7 * sizeof(BYTE));
	cur_for = (float)atof((const char *)read_str);

	cur_force = cur_for;

	gotoxy(1, 30);
	printf("Force: %.3f", cur_force);
	gotoxy(30, 30);
	printf("Speed mode: %d", speed_mode);
}

void ReadOBJ(void)
{

	register char Spacemouse_str[256], read_str[256], pos_sign[3];
	register int i, j;

	Obj_in->SetReadPos(0);
	Obj_in.Lock();
	Obj_in->Read((unsigned char *)Spacemouse_str, 2 * sizeof(BYTE), 0);
	Obj_in.Unlock();

	for (i = 0; i < 2; i++)
	{
		memcpy(read_str, Spacemouse_str + i, sizeof(BYTE));
		obj_in[i] = (int)atof((const char *)read_str);
	}

	//memcpy( read_str, Spacemouse_str + 49, sizeof(BYTE) );
	//read_str[1] = '\0';
	//SpaceMouseMode[7] = (float)atof( (const char *)read_str );

	//cout /*<< spacemousemode[0] << spacemousemode[1] << spacemousemode[2] */<<spacemousemode[3]<<endl;


}

void ReadLPS(int *LPS_value)
{
	register char read_LPS_str[256], read_str[256];
	register int i;

	LPS->SetReadPos(0);
	LPS.Lock();
	LPS->Read((unsigned char *)read_LPS_str, 8 * sizeof(BYTE), 0);
	LPS.Unlock();

	for (i = 0; i < 2; i++)
	{
		memcpy(read_str, read_LPS_str + 4 * i, 4 * sizeof(BYTE));
		LPS_value[i] = (int)atof((const char *)read_str);
		//LPS_value[1] = 10.0;
	}


	gotoxy(1, 31);
	printf("LPS  Left: %4d ,Right: %4d", LPS_value[0], LPS_value[1]);
}

void LPScheck(void)
{
	Matrix<3, 1> ca, wa;

	if ((LPS_value[0] > 800) || (LPS_value[1] > 800)) //top obstacle
	{
		if (speed[3] > 0)//block up
			speed[3] = 0;
		if (speed[5] > 0)//block hand up
			speed[5] = 0;
	}

	if (((LPS_value[0] > 500) && (LPS_value[0] < 900)) ||
		((LPS_value[1] > 600) && (LPS_value[1] < 900)))//front touch,move forward
	{
		//approach
		ca = 0, 0, 1;
		wa = C2W_transform(pos) * ca;
		if (cbox == CARTESIAN)
		{
			for (int i = 0; i < 3; i++)
				if ((speed[i + 1] * (wa(i + 1, 1) * linear_speed_limit[speed_mode])) > 0)
					speed[i + 1] = 0;
			//new_status = true;
		}

	}


	//bottom obstacle 
	if (((LPS_value[0] < 550) && (LPS_value[0] > 100)) || ((LPS_value[1] < 680) && (LPS_value[1] > 100)))
	{
		if (speed[3] < 0)// block down command 
			speed[3] = 0;
		if (speed[5] < 0)// block hand down
			speed[5] = 0;
	}


	if (((LPS_value[0] > 500) || ((LPS_value[1] > 100) && (LPS_value[1] < 600))) && (speed[6] > 0))//left top or right bottom obstacle, bock clockwise roll
		speed[6] = 0;

	if (((LPS_value[1] > 600) || ((LPS_value[0] > 100) && (LPS_value[0] < 500))) && (speed[6] < 0))//right top or left bottom obstacle, block counter-clockwise roll
		speed[6] = 0;


}
//
//void ReadSlip(double cur_vel)
//{
//	register char read_pos_str[256], read_str[256];
//
//	slip_vel->SetReadPos(0);
//	slip_vel.Lock();
//	slip_vel->Read((unsigned char *)read_pos_str, 8*sizeof(BYTE), 0 );
//	slip_vel.Unlock();
//
//	memcpy( read_str, read_pos_str, 8*sizeof(BYTE) );
//	cur_vel = (double)atof((const char *)read_str);
//
//	
//	cur_velocity = cur_vel*1000;
//	cur_velocity2 = cur_vel*1000;
//	raw_velocity = cur_vel*1000;
//	//if (initial_vel){
//	//	old_vel = cur_velocity;
//	//	initial_vel = false;
//	//}
//	//if ((cur_velocity==old_vel && cur_velocity == old_vel2 && cur_velocity == old_vel3 )/* || (cur_velocity<0)*/ ){
//	//	cur_velocity = 0;
//	//}
//	//old_vel3 = old_vel2;
//	//old_vel2 = old_vel;
//	//old_vel = cur_velocity2;
//
//	gotoxy(1,32);
//	printf("Velocity: %.3f",cur_velocity);
//
//}

//void ReadPosit(double cur_pos)
//{
//	register char read_pos_str[256], read_str[256];
//	//SleepMs(3);
//	pos_vel->SetReadPos(0);
//	pos_vel.Lock();
//	pos_vel->Read((unsigned char *)read_pos_str, 8*sizeof(BYTE), 0 );
//	pos_vel.Unlock();
//
//	memcpy( read_str, read_pos_str, 8*sizeof(BYTE) );
//	cur_pos = (double)atof((const char *)read_str);
//
//	//cur_position = cur_pos*1000;
//	cur_position = cur_pos;
//	gotoxy(1,33);
//	printf("Position: %.3f",cur_position);
//}

void ReadPosit(double cur_pos)//with position filter
{
	register char read_pos_str[256], read_str[256];
	//SleepMs(3);
	pos_vel->SetReadPos(0);
	pos_vel.Lock();
	pos_vel->Read((unsigned char *)read_pos_str, 8 * sizeof(BYTE), 0);
	pos_vel.Unlock();

	memcpy(read_str, read_pos_str, 8 * sizeof(BYTE));
	cur_pos = (double)atof((const char *)read_str);
	cur_pos_nf = cur_pos;
	//exp smoothing

	cur_pos_f = al * cur_pos + (1 - al)*o_p;
	o_p = cur_pos_f;

	//cur_position = cur_pos*1000;
	cur_position = cur_pos_f;
	gotoxy(1, 33);
	printf("Position: %.3f", cur_position);
}

void ReadVel(void)
{
	cur_t = TimeCheck();
	dtt = cur_t - old_t;
	cur_velocity = (cur_position - old_position) / (cur_t - old_t);
	//cur_velocity = (cur_position-old_position)/(2);
	old_t = cur_t;
	old_position = cur_position;

	cur_velocity_f = b2[0] * cur_velocity + b2[1] * x2d[0] + b2[2] * x2d[1] - a2[0] * y2d[0] - a2[1] * y2d[1];
	x2d[1] = x2d[0];
	x2d[0] = cur_velocity;
	y2d[1] = y2d[0];
	y2d[0] = cur_velocity_f;

	gotoxy(1, 32);
	printf("Velocity: %.3f", cur_velocity_f);
}

//
void ReadTaKK(void)
{
	//register char read_pos_str[256], read_str[256];
	//register int i;
	//takktile->SetReadPos(0);
	//takktile.Lock();
	//takktile->Read((unsigned char *)read_pos_str, 100*sizeof(BYTE), 0 );
	//takktile.Unlock();

	//for (i=0; i<12; i++)
	//{
	//	memcpy( read_str, read_pos_str + i*7, 7*sizeof(BYTE) );
	//	takk_reading[i] = (float)atof((const char *)read_str);
	//}

	//for (i=0; i<12; i++)
	//	cur_takk[i] = takk_reading[i];

	//gotoxy(1,40);
	//printf("[%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d]\n", cur_takk[0],cur_takk[1],cur_takk[2],cur_takk[3],cur_takk[4],cur_takk[5],
	//	cur_takk[6],cur_takk[7],cur_takk[8],cur_takk[9],cur_takk[10],cur_takk[11]);

	register char Spacemouse_str[256], read_str[256], pos_sign[3];
	register int i, j;

	takktile->SetReadPos(0);
	takktile.Lock();
	takktile->Read((unsigned char *)Spacemouse_str, 30 * sizeof(BYTE), 0);
	takktile.Unlock();

	int old_touch_pos[10];
	int sum0 = 0;
	int sum1 = 0;
	for (i = 0; i < 10; i++)
	{
		old_touch_pos[i] = touch_pos[i];
		memcpy(read_str, Spacemouse_str + i, sizeof(BYTE));
		touch_pos[i] = (int)atof((const char *)read_str);
		sum0 += touch_pos[i];
		if (touch_pos[i] != old_touch_pos[i])
			touch_pos_change[i] = 1;
		else
			touch_pos_change[i] = 0;
		sum1 += touch_pos_change[i];
	}

	//takk_touch[0] = takk_touch[1];
	//if (sum0 == 0)
	//	takk_touch[1] = false;
	//else
	//	takk_touch[1] = true;

	if (sum1 == 0)
		touch_pos_change[10] = 0;
	else
		touch_pos_change[10] = 1;
}
////
void GraspController(void)
{
	ofstream err_data;
	err_data.open("C:\\MANUS\\CommonSpace\\Sensor_Data\\error_data.txt", ios::app);
	ofstream exp_data;
	float w_hat = 0.04;//0.098
	float u_hat = 0.45;//0.45
	float u_hat_dot = 0;
	float tol = 0.01;

	int Que_count = 0;
	bool Que_tick = false;

	int cur_time = 0;
	int old_time = TimeCheck();
	float dt;
	float cur_distance;
	float old_pos = cur_position;

	float Fa = (1 / u_hat)*(w_hat + k * cur_velocity_f);
	float err = abs(cur_force - Fa);

	//err_data << cur_time << ", " << err << "\n";



	while (abs(err) > 0.15) // 0.25
	{

		SbTestCvsId;
		SleepMs(3);
		ReadForce(cur_force);
		//ReadSlip(cur_velocity);
		ReadPosit(cur_position);
		ReadVel();

		//force_que[Que_count] = cur_force;
		vel_que[Que_count] = cur_velocity_f;

		//Que_count = ( Que_count == 6 ) ? 0 : Que_count + 1;
		//if ( ( Que_count == 6 ) && ( !Que_tick ) )
		//	Que_tick = true;

		cur_time = TimeCheck();
		cur_distance = abs(cur_position - old_pos) * 1000;
		dt = (cur_time - old_time) / 1000;
		w_hat = gamma1 * cur_distance + .625;
		u_hat_dot = -gamma2 / u_hat * w_hat*cur_velocity_f;
		u_hat += u_hat_dot * dt;

		Fa = (1 / u_hat)*(w_hat + k * cur_velocity_f);
		err = cur_force - Fa;
		old_time = cur_time;
		old_pos = cur_position;

		//Read from MANUS.
		rcvMsg.ID = 0x0;
		m_objPCANBasic.Read(m_Handle, &rcvMsg, &CANTimeStamp);
		Decode(rcvMsg, xmitMsg);

		//err_data << cur_time << ", " << err << "\n";
		err_data << cur_time << ", " << cur_velocity_f << ", " << cur_force << ", " << cur_position << ", " << cur_pos_nf << ", " << cur_velocity << ", " << err << ", " << 0 << ", " << speed[7] << ", " << pos[6] << ", " << Fa << "\n";
		//exp_data << end_time << ", " << cur_velocity_f <<", "  << cur_force <<", " << cur_position << ", "<< cur_pos_nf << ", " << cur_velocity <<", " << grasp_start <<", " << grasp_end << ", " << speed[7] << ", " << grasp_npos<< "\n";

		gotoxy(1, 34);
		printf("Error: %.3f", err);

		if (cur_force == 0) {
			grasp_end = 1;
			break;
		}
		//if ( ( fabs(force_que[3] - force_que[2]) <= tol )
		//	&& ( fabs(force_que[2] - force_que[1]) <= tol )
		//	&& ( fabs(force_que[1] - force_que[0]) <= tol ) 
		//	&& ( fabs(force_que[4] - force_que[3]) <= tol ) 
		//	&& ( fabs(force_que[5] - force_que[4]) <= tol ) 
		//	/*&& ( fabs(force_que[6] - force_que[5]) <= tol )*/ ){
		//	gotoxy(1,34);
		//	printf("force flat                ");
		//	grasp_end = 1;
		//	break;
		//}
		if ((fabs(vel_que[0]) <= 0.05)
			&& (fabs(vel_que[1]) <= 0.05)
			&& (fabs(vel_que[2]) <= 0.05)
			&& (fabs(vel_que[3]) <= 0.05)
			&& (fabs(vel_que[4]) <= 0.05)
			&& (fabs(vel_que[5]) <= 0.05)) {
			gotoxy(1, 34);
			printf("velocity zero             ");
			grasp_end = 2;
			break;
		}

		//gotoxy(1,34);
		//printf("finish                   ");
		//Que_count++;

	}
	F_d = Fa + 0.1;
	if (grasp_end == 0) {
		grasp_end = 4;
		gotoxy(1, 34);
		printf("end                 ");
	}
	ResetAll();
	return;

}

//
//void GraspController(void)
//{
//	ofstream err_data;
//	err_data.open("C:\\MANUS\\CommonSpace\\Sensor_Data\\error_data.txt",ios::out);
//
//	float w_hat = 0.098;
//	float u_hat = 0.45;
//	float u_hat_dot = 0;
//	float tol = 0.05;
//
//	int Que_count = 0;
//	bool Que_tick = false;
//
//	int cur_time = 0;
//	int old_time = TimeCheck();
//	float dt;
//	float cur_distance;
//	float old_position = cur_position;
//
//	float Fa = (1/u_hat)*(w_hat + k*cur_velocity);
//	float err = cur_velocity;
//	err_data << cur_time << ", " << err << "\n";
//
//	gotoxy(1,42);
//	printf("graspcontrol start %.3f",err);
//
//	while( abs(err) > 10  ) // 0.25
//	{
//		//ReadForce(cur_force);
//		ReadTaKK(cur_takk);
//		ReadSlip(cur_velocity);
//		ReadPosit(cur_position);
//		//force_que[Que_count] = cur_force;
//
//		//Que_count = ( Que_count == 4 ) ? 0 : Que_count + 1;
//		//if ( ( Que_count == 4 ) && ( !Que_tick ) )
//		//	Que_tick = true;
//
//		cur_time = TimeCheck();
//		cur_distance = abs(cur_position - old_position);
//		dt = (cur_time - old_time)/1000;
//		w_hat = gamma1*cur_distance + .625;
//		u_hat_dot = -gamma2/u_hat*w_hat*cur_velocity;
//		u_hat += u_hat_dot*dt;
//
//		Fa = (1/u_hat)*(w_hat + k*cur_velocity);
//		err = cur_velocity;
//		old_time = cur_time;
//		old_position = cur_position;
//
//		err_data << cur_time << ", " << err << "\n";
//
//		//takk_touch00 = (abs(init_takk[0]-cur_takk[0]) > 3 || abs(init_takk[1]-cur_takk[1]) > 3 || abs(init_takk[2]-cur_takk[2]) > 3 || 
//		//	abs(init_takk[3]-cur_takk[3]) > 3 || abs(init_takk[4]-cur_takk[4]) > 3 || abs(init_takk[5]-cur_takk[5]) > 3);
//		//takk_touch11 = (abs(init_takk[6]-cur_takk[6]) > 3 || abs(init_takk[7]-cur_takk[7]) > 3 || abs(init_takk[8]-cur_takk[8]) > 3 || 
//		//		abs(init_takk[9]-cur_takk[9]) > 3 || abs(init_takk[10]-cur_takk[10]) > 3 || abs(init_takk[11]-cur_takk[11]) > 3);
//
//		gotoxy(1,39);
//		printf("Error: %.3f",err);
//
//		//gotoxy(1,40);
//		//printf("[%3d,%3d]\n", !(abs(init_takk[0]-cur_takk[0]) > 2 || abs(init_takk[1]-cur_takk[1]) > 2 || abs(init_takk[2]-cur_takk[2]) > 2 || 
//		//   abs(init_takk[3]-cur_takk[3]) > 2 || abs(init_takk[4]-cur_takk[4]) > 2 || abs(init_takk[5]-cur_takk[5]) > 2),!(abs(init_takk[6]-cur_takk[6]) > 2 || abs(init_takk[7]-cur_takk[7]) > 2 || abs(init_takk[8]-cur_takk[8]) > 2 || 
//		//  abs(init_takk[9]-cur_takk[9]) > 2 || abs(init_takk[10]-cur_takk[10]) > 2 || abs(init_takk[11]-cur_takk[11]) > 2));
//
//		if(!(abs(init_takk[0]-cur_takk[0]) > 2 || abs(init_takk[1]-cur_takk[1]) > 2 || abs(init_takk[2]-cur_takk[2]) > 2 || 
//		   abs(init_takk[3]-cur_takk[3]) > 2 || abs(init_takk[4]-cur_takk[4]) > 2 || abs(init_takk[5]-cur_takk[5]) > 2) && 
//		   !(abs(init_takk[6]-cur_takk[6]) > 2 || abs(init_takk[7]-cur_takk[7]) > 2 || abs(init_takk[8]-cur_takk[8]) > 2 || 
//		  abs(init_takk[9]-cur_takk[9]) > 2 || abs(init_takk[10]-cur_takk[10]) > 2 || abs(init_takk[11]-cur_takk[11]) > 2))
//				{gotoxy(1,41);
//				printf("graspcontrol stop [%3d,%3d]\n",!(abs(init_takk[0]-cur_takk[0]) > 2 || abs(init_takk[1]-cur_takk[1]) > 2 || abs(init_takk[2]-cur_takk[2]) > 2 || 
//		   abs(init_takk[3]-cur_takk[3]) > 2 || abs(init_takk[4]-cur_takk[4]) > 2 || abs(init_takk[5]-cur_takk[5]) > 2),!(abs(init_takk[6]-cur_takk[6]) > 2 || abs(init_takk[7]-cur_takk[7]) > 2 || abs(init_takk[8]-cur_takk[8]) > 2 || 
//		  abs(init_takk[9]-cur_takk[9]) > 2 || abs(init_takk[10]-cur_takk[10]) > 2 || abs(init_takk[11]-cur_takk[11]) > 2));
//				break;
//		}
//		//		
///*
//		if( cur_force == 0 )
//			break;
//		if ( ( fabs(force_que[3] - force_que[2]) <= tol )
//			&& ( fabs(force_que[2] - force_que[1]) <= tol )
//			&& ( fabs(force_que[1] - force_que[0]) <= tol ) )
//			break;*/
//		//Que_count++;
//	}
//
//	ResetAll();
//	return;
//}


void ReadSugspeed(void)
{

	//register char speed_str[256], read_str[256];
	//register int i;

	//sug_speed->SetReadPos(0);
	//sug_speed.Lock();
	//sug_speed->Read((unsigned char *)speed_str, 30 * sizeof(BYTE), 0);
	//sug_speed.Unlock();

	//for (i = 0; i < 6; i++)
	//{
	//	memcpy(read_str, speed_str + 3 * i, 3*sizeof(BYTE));
	//	suggspeed[i] = (int)atof((const char *)read_str);
	//}
	//memcpy(read_str, speed_str + 18, sizeof(BYTE));
	//suggspeed[6] = (int)atof((const char *)read_str);
	//gotoxy(1, 40);
	//cout << int(suggspeed[6]) << endl;
}



void Readblock_dir(void)
{
	block_flag = false;
	register char block_str[256], read_str[256];
	register int i;

	block_direction->SetReadPos(0);
	block_direction.Lock();
	block_direction->Read((unsigned char *)block_str, 20 * sizeof(BYTE), 0);
	block_direction.Unlock();

	for (i = 0; i < 6; i++)
	{
		memcpy(read_str, block_str + 3 * i, 3 * sizeof(BYTE));
		block_dir[i] = (int)atof((const char *)read_str);
		if (block_dir[i] != 0)
			block_flag = true;
	}

	memcpy(read_str, block_str + 18, sizeof(BYTE));
	btn_pressed = (int)atof((const char *)read_str);

	user_oprt[0] = user_oprt[1];//check user is operation or not
	if (btn_pressed == 1 || spm_operation > 0)
		user_oprt[1] = 1;
	else if (btn_pressed == 0 && spm_operation == 0)
		user_oprt[1] = 0;


	//gotoxy(1, 50);
	//cout << "block direction [";
	//for (int jj = 0; jj < 6; jj++)
	//{
	//	cout << "  " << jj + 1 << ':' << block_dir[jj] << "  ";
	//}
	//cout << "]    "<< spm_operation<<"      \n";
}


void LowPassFilter(void)
{
	int lpf_time_passed;

	float lpf_meas = cur_velocity;
	lpf_time_passed = abs(TimeCheck() - lpf_time);
	if (lpf_time_passed > 500)
		lpf_cur = lpf_meas;
	else
		lpf_cur = lpf_old + (lpf_meas - lpf_old) * ((float)(lpf_time_passed) / 1000.0f) * 5.0f;
	cur_velocity = lpf_cur;
	lpf_old = lpf_cur;
	lpf_time = TimeCheck();

}

void LowPassFilter2(void)
{
	int lpf_time_passed;

	float lpf_meas = cur_velocity;
	lpf_time_passed = abs(TimeCheck() - lpf_time2);
	if (lpf_time_passed > 500)
		lpf_cur = lpf_meas;
	else
		lpf_cur = lpf_old2 + (lpf_meas - lpf_old2) * ((float)(lpf_time_passed) / 1000.0f) * 5.0f;
	cur_velocity = lpf_cur2;
	lpf_old2 = lpf_cur2;
	lpf_time2 = TimeCheck();

}

void init_grasp(void)
{
	if (grab_in_progress)
	{

		//if ( cur_force>0.5 &&( fabs(force_que[3] - force_que[2]) <= tol )
		//	&& ( fabs(force_que[2] - force_que[1]) <= tol )
		//	&& ( fabs(force_que[1] - force_que[0]) <= tol ) 
		//	&& ( fabs(force_que[4] - force_que[3]) <= tol ) 
		//	/*&& ( fabs(force_que[5] - force_que[4]) <= tol*/ ) {
		//	ResetAll();
		//	}
		if (cur_force > 0) {
			P_int = pos[6];
		}

		if (abs(cur_velocity_f) > 0.005 / 1000 && cur_force > 0.1 && grasp_flag == 0) // new algoritm
		{
			ResetAll();
			ReadForce(cur_force);
			init_force = cur_force;
			init_stop = 1;
			hold_init = TimeCheck();
			//SendCommand2(SPC, TTS_SPEAK, READY_LIFT);
			//ready_lift = true;
			/*P_int = pos[6];	*/
		}
		else if (grasp_flag == 0) // force flatness detection grasping
		{
			tol = 0.2;
			if (cur_force > 4.5 && (fabs(force_que[3] - force_que[2]) <= tol)
				&& (fabs(force_que[2] - force_que[1]) <= tol)
				&& (fabs(force_que[1] - force_que[0]) <= tol)
				&& (fabs(force_que[4] - force_que[3]) <= tol) && init_stop == 0) {
				ResetAll();
				init_stop = 1;
			}
		}


		if ((cbox == CARTESIAN) && (pos[6] < -25000.0f))
		{
			SendCommand(CAN, FSR, FSR_END, EMPTY_MESSAGE);
			ResetAll();
			//continue;
		}
	}
}

void regrasp(void)
{
	int adj_time_out = 5000;
	grasp_end = 0;
	grasp_start = 0;
	grasp_npos = pos[6];
	e_force = cur_force - F_d;
	e_pos = pos[6] - stoppos;

	if (grasp_flag == 10 && (TimeCheck() - ini_adt) <= adj_time_out)
	{
		//if (adjust >=3 ){
		//	if (adjust<=3)
		//		adjust+=1;
		//	else adjust = 0;
		//}

		if (adjust == 2) {
			//ResetAll();
			adjust = 0;
		}
		if (adjust == 1)
			adjust = 2;

		//if (abs(e_pos)> 1000 && adjust == 0){    //position
		//	//speed[7] = (6*e_force); //force
		//	speed[7] = -(2*e_pos/1000); //g_pos
		//	new_status = true;
		//	//ManualControl( 'u' );
		//	adjust = 1;
		//}
		if (abs(e_force) > 0.1 && adjust == 0) {    //force
			speed[7] = (k2*e_force); //force

									 //speed[7] = -(2*e_pos/1000); //g_pos
			new_status = true;
			//ManualControl( 'u' );
			adjust = 1;
		}

		//else if (e_force < -0.3 && adjust == 0){
		//	//ManualControl( 'j' );
		//	speed[7] = -(7.5*e_force); 
		//	new_status = true;
		//	adjust = 1;
		//}

		if (abs(e_force) <= 0.2 && adjust == 0)
		{ //force
			ResetAll();
			if ((TimeCheck() - ini_adt) <= adj_time_out)
				grasp_flag = 10;
			else grasp_flag = 0;
		}
		//if (abs(e_pos) <= 1000 && adjust ==0){    // g_position
		//	ResetAll();
		//	if ((TimeCheck()-ini_adt)<=10000)
		//		grasp_flag = 10;
		//	else grasp_flag = 0;
		//}
	}

	if (grasp_flag == 10 && (TimeCheck() - ini_adt) > adj_time_out)
		grasp_flag = 0;

	if (grasp_flag >= 6 && grasp_flag < 10)
	{
		if (grasp_flag <= 8)
			grasp_flag += 1;
		else grasp_flag = 10;

	}

	//if (grasp_flag==5 )
	//{
	//	grasp_flag = 2;
	//}

	if (grasp_flag == 4)
	{
		if ((TimeCheck() - ini_adt) < adj_time_out)
			grasp_flag = 2;
		else
		{
			grasp_flag = 6;
			//ini_adt = TimeCheck();
			ResetAll();
		}

	}

	if (grasp_flag == 3)
	{
		//GraspController();
		cur_time = TimeCheck();
		cur_distance = (cur_position - old_pos) / 1000;
		dt0 = (cur_time - old_time);

		////original approach

		last_w_hat = w_hat;
		last_u_hat = u_hat;
		w_hat += gamma1 * cur_distance;//gamma1*cur_distance + .625
		u_hat_dot = -gamma2 / u_hat * (w_hat + k * cur_velocity_f)*cur_velocity_f;//-gamma2/u_hat*w_hat*cur_velocity_f
		u_hat += u_hat_dot * dt0 / 1000;
		if (u_hat <= 0.2)
			u_hat = 0.2;
		old_pos = cur_position;
		F_d = (1 / u_hat)*(w_hat + k * cur_velocity_f);


		if (F_d > 6) { F_d = 6; }
		if (F_d < 0) { F_d = 3; }
		//P_d = P_int+(p1*pow(F_d,3)+p2*pow(F_d,2)+p3*F_d+p4);
		gotoxy(1, 34);
		printf("F_desire: %.3f", F_d);
		gotoxy(20, 34);
		printf("u_h: %.3f", u_hat);
		gotoxy(35, 34);
		printf("w_h: %.3f", w_hat);
		//gotoxy(55,34);
		//printf("w_h: %.3f",pow(2.0,2.0));
		//err = cur_force - Fa;
		old_time = cur_time;

		//stoppos = pos[6]+5500;
		grasp_flag = 4;
		//ini_adt = TimeCheck();
		e_force = cur_force - F_d;
		//speed[7] = (k2*e_force); //force
		speed[7] = (k2*e_force); //position
		if (abs(speed[7]) < 1) {
			if (speed[7] > 0)	speed[7] = 1;
			else if (speed[7] < 0) speed[7] = -1;
		}
		if (abs(e_force) / F_d < 0.03)speed[7] = 0;
		new_status = true;
	}

	if (grasp_flag == 2)
	{
		grasp_flag = 3;

	}


	if (cur_force < 0.2)
	{
		last_w_hat = 6;//2     
		last_u_hat = 2;//2   
	}


	if (grasp_flag == 1)
	{
		//ManualControl( 'j' );
		ini_adt = TimeCheck();

		//initial estimated value
		w_hat = last_w_hat;//2     
		u_hat = last_u_hat;//2   
		u_hat_dot = 0;


		//float tol = 0.01;

		//int Que_count = 0;
		//bool Que_tick = false;

		//int cur_time = 0;
		//int old_time = TimeCheck();
		//float dt;
		//float cur_distance;
		old_pos = cur_position;

		F_d = (1 / u_hat)*(w_hat + k * cur_velocity_f);//original approach

													 //P_d = P_int+(p1*pow(F_d,3)+p2*pow(F_d,2)+p3*F_d+p4);


		e_force = cur_force - F_d;
		//e_pos = pos[6]-P_d;
		speed[7] = (k2*e_force); //gripper velocity
		new_status = true;
		grasp_inipos = pos[6];
		grasp_start = 1;
		grasp_flag = 2;
		old_time = TimeCheck();
	}

	if (grasp_flag == 11)
	{
		grasp_flag = 1;

	}



	if (((cur_force > 0.2) && (abs(cur_velocity_f) > 0.25 / 1000) &&
		!grab_in_progress && !open_in_progress && (grasp_flag == 0)
		&& (TimeCheck() - hold_init) > 1000))
	{
		grasp_flag = 11;
		//SendCommand2(SPC, TTS_SPEAK, ADJ_F);
		ResetAll();
	}
}

void oneclick(void)
{
	//if (assistant_flag)  // Assistant mode
	//{
		//**** Suggested move calculations ****//
		/*oneclick_mode = 1;*/
	for (int i = 0; i < 5; i++)
	{
		currentPosition[i] = pos[i];//(xcam,zcam,z,y,p,r)
	}
	currentPosition[5] = sign(pos[5])*(180 - fabs(pos[5]));
	float rotationThreshold = 4;
	float positionThreshold = 10;

	// Matlab flag, when Matlab processing is over, will turn to "1" for 4 second and return to "0" again, 
	// if no pair was detected, turns to '2' for 1 second
	if (oneclick_mode == 2)
	{
		float err_dp = 0;
		for (int i = 0; i < 6; ++i)
		{
			err_dp += pow(pos[i] - set_pos[i], 2);
		}
		//if (sqrt(err_dp) < 5)
		//{

		//}
		//SleepMs(1000);
		//if (job_done && !auto_mode_start && !adjust_pos)
		//gotoxy(1, 51);
		//cout << err_dp << endl;
		if (sqrt(err_dp) < 7 && !adjust_pos&&requestframe)
		{
			//SleepMs(1000);

			requestframe = false;
			moveto = false;
			//SleepMs(1000);
			t_reqframe = TimeCheck();
		}
		else if (!requestframe && !moveto && (TimeCheck() - t_reqframe > 1000))
		{
			//SleepMs(1000);
			SendCommand(CAN, SVR, REQUESTFRAME, 0);
			for (int i = 0; i < 6; i++)
			{
				set_pos[i] = orig_pos[i];
			}
			movetopos();
			oneclick_mode = 1;

			ofstream assistantFLAG;
			assistantFLAG.open("C:\\MANUS\\CommonSpace\\Assistant\\assistantFLAG.txt", ios::out);
			assistantFLAG << 1 << "\n";
			assistantFLAG.close();
			ofstream mFLAG;
			mFLAG.open("C:\\MANUS\\CommonSpace\\Assistant\\Mflag2.txt", ios::out);
			mFLAG << 1 << "\n";
			mFLAG.close();
		}


	}
	else if (oneclick_mode == 1)  // after assistant was turned on,until Matlab fnished processing, keep checking flag file.
	{
		char Mflag_str[256];
		sprintf_s(Mflag_str, "C:\\MANUS\\CommonSpace\\Assistant\\Mflag.txt");
		FILE* M_flag = fopen(Mflag_str, "r");
		float M_flag_reading[1] = {};
		int R_t;
		if (M_flag != NULL)
		{
			fscanf_s(M_flag, "%f ", &M_flag_reading[0]);

			if (M_flag_reading[0] == 1)//when Matlab flag is 1,reading the desire position from txt file. Found object
			{
				oneclick_mode = 4;
				//sg_stage = 1;
				FILE* R_pos = fopen("C:\\MANUS\\CommonSpace\\Assistant\\requestedPosition.txt", "r");
				if (R_pos != NULL)
				{
					float repos[11] = {};
					for (int i = 0; i < 11; ++i)
					{
						fscanf_s(R_pos, "%f ", &repos[i]);
						requestedPosition[i] = repos[i];//0,1,2 : desire gripper position in front of the object
														//3,4,5 : desire yaw,pit,roll corresponding to the object orientation
														//6,7...for btn suggestion,center the object    8,9,10 center position of the detected surface 
						if (requestedPosition[4] > -10)
						{
							grasp_side = 0;//front grasp
						}
						else if (requestedPosition[4] < -30)
							grasp_side = 1;// top grasp
						init_sug = true;
					}
				}
				fclose(R_pos);
				//if (btn_flag == false)// voice feedfack "ready to go" after Matlab part was finished and desire position was read.
				//{
				//	// read object mask from MATLAB output
				//	FILE* ROI = fopen("C:\\MANUS\\CommonSpace\\Assistant\\ROI.txt", "r");
				//	if (ROI != NULL)
				//	{
				//		float roi[4] = {};
				//		for (int i = 0; i < 4; ++i)
				//		{
				//			fscanf_s(ROI, "%f ", &roi[i]);
				//		}
				//		this->object_x = roi[0] * this->scaleFactorX;
				//		this->object_y = roi[1] * this->scaleFactorX;
				//		this->object_width = roi[2] * this->scaleFactorX;
				//		this->object_height = roi[3] * this->scaleFactorX;
				//		cout << "ROI loaded" << endl;
				//		this->object_box = true;
				//	}
				//	fclose(ROI);

				//	svr->SendCommand(SPC, TTS_SPEAK, AS_READY);
				//	strcpy(text_buff, "Found the object, ready to assist.");
				//	manual_main_1click_btn->activate();
				//	SleepMs(2000);
				//}

				p_frame_w(1, 1) = requestedPosition[8];
				p_frame_w(2, 1) = requestedPosition[9];
				p_frame_w(3, 1) = requestedPosition[10];
				update_sug = 1;
				suggestedButtonSwitch = 'Z';
				btn_flag = true;
				fine_adjust = '0';

			}
			else if (M_flag_reading[0] == 3)// when matlab can't find any pairs, remind the user try it again
			{
				oneclick_mode = 3;
				//svr->SendCommand(SPC, TTS_SPEAK, OBJ_FAR);
				//strcpy(text_buff, "Object is too far away.");
				assistant_flag = false;

				//manual_assistant_btn->copy_label("Assistant : Off");
				ofstream assistantFLAG;
				assistantFLAG.open("C:\\MANUS\\CommonSpace\\Assistant\\assistantFLAG.txt", ios::out);
				assistantFLAG << 0 << "\n";
				assistantFLAG.close();
				btn_flag = false;
				//close_sg = false;
				//gripper_center = false;
			}
			else if (M_flag_reading[0] == 2)// when matlab can't find any pairs, remind the user try it again
			{
				if (move_arm == 0)
				{
					oneclick_mode = 2;
					//svr->SendCommand(SPC, TTS_SPEAK, AS_ERR);
					//strcpy(text_buff, "Can't find the object, please move the arm and try it again.");
					//assistant_flag = false;

					//manual_assistant_btn->copy_label("Assistant : Off");
					ofstream assistantFLAG;
					assistantFLAG.open("C:\\MANUS\\CommonSpace\\Assistant\\assistantFLAG.txt", ios::out);
					assistantFLAG << 0 << "\n";
					assistantFLAG.close();
					btn_flag = false;

					// set adjusted position
					for (int i = 0; i < 6; i++)
					{
						set_pos[i] = pos[i];
						orig_pos[i] = pos[i];
					}
					if (orig_pos[4] < -30)
					{
						set_pos[1] = set_pos[1] + 140.0f;
						set_pos[3] = set_pos[3] - 18.0f;
					}
					else if (orig_pos[4] > 25)
					{
						set_pos[2] = set_pos[2] + 100.0f;
						set_pos[4] = set_pos[4] - 18.0f;
					}
					moveto = false;

					movetopos();
					requestframe = true;
					t_adj = TimeCheck();
					move_arm = 1;
				}
				else if (move_arm == 1)
				{
					ofstream assistantFLAG;
					assistantFLAG.open("C:\\MANUS\\CommonSpace\\Assistant\\assistantFLAG.txt", ios::out);
					assistantFLAG << 0 << "\n";
					assistantFLAG.close();
					btn_flag = false;
					oneclick_mode = 9;
				}
				//close_sg = false;
				//gripper_center = false;
			}
			fclose(M_flag);
		}
	}
	else if (oneclick_mode > 3)  // once Found the object, start culculate the assist speed 
	{

		float deltaPosition[13] = { 0 };
		float ee_deltaPosition[13] = { 0 };
		//Find delta robot positions
		for (int i = 0; i < 6; i++) {
			deltaPosition[i] = requestedPosition[i] - currentPosition[i];// in base frame
			ee_deltaPosition[i] = requestedPosition[i] - currentPosition[i];
		}
		deltaPosition[6] = requestedPosition[6] - currentPosition[3];
		deltaPosition[7] = requestedPosition[7] - currentPosition[4];

		//ee_deltaPosition[6] = requestedPosition[6] - currentPosition[3];
		//ee_deltaPosition[7] = requestedPosition[7] - currentPosition[4];


		deltaPosition[8] = requestedPosition[8] - currentPosition[0];
		deltaPosition[9] = requestedPosition[9] - currentPosition[1];
		deltaPosition[10] = requestedPosition[10] - currentPosition[2];
		ee_deltaPosition[8] = requestedPosition[8] - currentPosition[0];
		ee_deltaPosition[9] = requestedPosition[9] - currentPosition[1];
		ee_deltaPosition[10] = requestedPosition[10] - currentPosition[2];


		ee_deltaPosition[6] = atan(ee_deltaPosition[9] / ee_deltaPosition[8]) / 3.1415 * 180 - currentPosition[3];
		ee_deltaPosition[7] = atan(ee_deltaPosition[10] / sqrt(pow(ee_deltaPosition[8], 2) + pow(ee_deltaPosition[9], 2))) / 3.1415 * 180 - currentPosition[4];
		deltaPosition[11] = ee_deltaPosition[6];
		deltaPosition[12] = ee_deltaPosition[7];


		D2obj = sqrt(pow(deltaPosition[8], 2) + pow(deltaPosition[9], 2) + pow(deltaPosition[10], 2));
		Matrix<2, 1> e_xy, e_handxy;
		Matrix<2, 2> W2ee_xy;
		e_xy = ee_deltaPosition[0], ee_deltaPosition[1];
		W2ee_xy = cos(-pos[3] * M_PI / 180), -sin(-pos[3] * M_PI / 180),
			sin(-pos[3] * M_PI / 180), cos(-pos[3] * M_PI / 180);//

		e_handxy = W2ee_xy * e_xy;
		for (int i = 0; i < 2; i++) {
			//cout<<endl<<"Transformed Delta "<< deltaPosition[i]<<" ";
			ee_deltaPosition[i] = e_handxy(i + 1, 1);//for suggestion btn
		}

		old_fine_adjust = fine_adjust;

		if (!move_as_suggested&&  user_oprt[1] == 1 && update_sug != 1)
		{
			update_sug = 1;
			suggestedButtonSwitch = 'Z';
		}
		//suggest_btn2(ee_deltaPosition);
		if (spaceMouseEnabled && (spaceMouseMode != 3))
		{
			if (spaceMouseEnabled != spaceMouseEnabled_old || spaceMouseMode != spaceMouseMode_old)
				//after mode switching , start over the suggestion 
			{
				update_sug = 1;
				suggestedButtonSwitch = 'Z';
			}
			suggest_btn2(deltaPosition, 0);
		}
		else
		{
			suggest_btn2(ee_deltaPosition, 1);
		}



		//gotoxy(1, 27);
		//cout << "[";
		//for (int jj = 0; jj < 10; jj++)
		//{
		//	cout << "  " << jj << ':' << ee_deltaPosition[jj] << "  ";
		//}
		//cout << "] \n"<< suggestedMotion<<endl;


		// if user move the robot away far from the desire position, change assist mode to phase 1: reaching the object
		if (
			(fabs(deltaPosition[1]) > 20) ||
			(fabs(deltaPosition[2]) > 20) ||
			(abs(deltaPosition[3]) > 7) ||
			(abs(deltaPosition[4]) > 7) ||
			(abs(deltaPosition[5]) > 7))
		{

			if (deltaPosition[0] > 70)
			{
				fine_adjust = '0';//if deltaposion larger than the threshold,  active the reaching phase
			}
		}


		if (fine_adjust == '0')// phase 1: reaching object
		{

			if (user_oprt[0] == 0 && user_oprt[1] == 1 && oneclick_mode == 4)
				oneclick_mode = 5;




			if (D2obj > 300) {
				deltaPosition[3] = atan(deltaPosition[9] / deltaPosition[8]) / 3.1415 * 180 - currentPosition[3];
				deltaPosition[4] = atan(deltaPosition[10] / sqrt(pow(deltaPosition[8], 2) + pow(deltaPosition[9], 2))) / 3.1415 * 180 - currentPosition[4];
			}//desire yaw pitch,make sure the gripper is keep looking the object

			for (int i = 0; i < 6; i++) // desire speed setting
			{
				if ((i < 3) && ((abs(deltaPosition[i]) > 10)))
				{

					suggspeed[i] = deltaPosition[i] / sqrt(pow(deltaPosition[0], 2) + pow(deltaPosition[1], 2) + pow(deltaPosition[2], 2)) * 40;

				}
				else if ((i < 3) && ((abs(deltaPosition[i]) < 10)))
				{
					suggspeed[i] = 0;
				}

				if ((i > 2) && (abs(deltaPosition[i]) > 2))
				{
					suggspeed[i] = deltaPosition[i] / 4;//deltaPosition0[i]/15
					if (suggspeed[i] == 0)
						suggspeed[i] = sign(deltaPosition[i]) * 2;
				}
				else if ((i > 2) && (abs(deltaPosition[i]) < 2))
				{
					suggspeed[i] = 0;

				}
			}

			//cv::Mat delta_temp;
			//cv::Mat EE2W_m(3, 3, CV_32FC1);
			Matrix<3, 1> ca, wa, adj0, adj_dire, delta_temp;
			Matrix<3, 3> EE2W_m;
			EE2W_m = EE2w_transform3(pos);
			//EE2C_transform(pos);

			adj_dire = 0, 0, 0;


			//float adj0[3] = { 0, 0, 0 };
			//cv::Mat adj_dire(3, 1, CV_32FC1, adj0);


			if (grasp_side == 0)//for front grasp collision detection
			{
				if ((touch_pos[2] == 1 || touch_pos[4] == 1 ||
					touch_pos[6] == 1 || touch_pos[8] == 1) && !btm_cls)// buttom collision
				{
					adj_dire(3, 1) = 1 * 30;//buttom collision, z_d need shift along +z
					if (!btm_cls)
						for (int j = 0; j < 6; j++)
						{
							cls_pos[j] = pos[j];
						}
					btm_cls = true;

					delta_temp = EE2W_m * adj_dire;
					for (int k = 0; k < 3; k++)
					{
						requestedPosition[k] += delta_temp(k + 1, 1);
					}
					cout << "buttom collision" << endl;
				}

				if (btm_cls && (pos[2] - cls_pos[2]) > 20)
				{
					btm_cls = false;
				}

				if (btm_cls) {
					suggspeed[2] += 20;
				}

			}
			else if (grasp_side == 1)
			{

			}
			//SendSugspeed();

			int sum_speed = 0;
			for (int i = 0; i < 6; i++)
			{
				sum_speed += abs(suggspeed[i]);
			}
			//SendSugspeed();

			if ((sum_speed == 0) && (fine_adjust == '0'))//when reach the desire position, sum of abs(speed) goes to zero, than start auto gripping process
			{
				//svr->SendCommand3(GUI, CAN, TP_END, ' ');//stop
				//SleepMs(100);
				//svr->SendCommand(SPC, TTS_SPEAK, AS_DOWN);//facing object
				SleepMs(100);
				//strcpy(text_buff, "Gripper is in front of the object. ");
				fine_adjust = '1';
				//svr->SendCommand(SPC, TTS_SPEAK, ONECLICK_1);
				//strcpy(text_buff, "Approach to the object.");
				//cls_n = 0;
				for (int k = 0; k < 6; k++)
				{
					temp_pos[k] = requestedPosition[k];
				}
			}
		}
		else
			//1. approaching to the object    
			//2. if collision detected, retreat and move the gripper, approach again until second optical gate was hit.
		{

			Matrix<3, 1> ca, wa, adj0, adj_dire, delta_temp;
			Matrix<3, 3> EE2W_m;
			ca = 1, 0, 0;// forward for C2W_transform
			EE2W_m = EE2w_transform3(pos);
			wa = EE2W_m * ca;
			adj_dire = 0, 0, 0;


			//cv::Mat EE2W_m(3, 3, CV_32FC1);
			//EE2W_transform(robot_pos, EE2W_m);

			//float ca[3] = { 1, 0, 0 };
			//float adj0[3] = { 0, 0, 0 };
			//cv::Mat ca_M(3, 1, CV_32FC1, ca);
			//cv::Mat adj_dire(3, 1, CV_32FC1, adj0);
			//cv::Mat wa;
			//cv::Mat delta_temp;

			//wa = EE2W_m * ca_M;
			//cout << wa << endl;
			if (obj_in[1] == 1)
			{
				if (fine_adjust != '4')
				{
					//svr->SendCommand3(GUI, CAN, TP_END, ' ');//stop motion
					for (int i = 0; i < 6; i++)
						suggspeed[i] = 0;


					//SendSugspeed();//write to speed[]


					//SleepMs(100);
					//svr->SendCommand(SPC, TTS_SPEAK, ONECLICK_3);
				}

				fine_adjust = '4';
			}
			float d_po[6] = { 0 };

			switch (fine_adjust)//switch between different status of auto grasping
			{
			case '1'://approaching to the object
				oneclick_mode = 6;
				for (int i = 0; i < 3; i++)
					suggspeed[i] = wa(i + 1, 1) * 15;

				if (grasp_side == 0)
				{
					if ((touch_pos[2] == 1 || touch_pos[4] == 1 ||
						touch_pos[6] == 1 || touch_pos[8] == 1) && !btm_cls)// buttom collision
					{
						adj_dire(3, 1) = 30;//buttom collision, z_d need shift along +z
						if (!btm_cls)
							cls_pos[2] = pos[2];

						btm_cls = true;
					}

					if (btm_cls && (pos[2] - cls_pos[2]) > 25)
					{
						btm_cls = false;
					}

					delta_temp = EE2W_m * adj_dire;
					for (int k = 0; k < 3; k++)
					{
						temp_pos[k] += delta_temp(k + 1, 1);
					}

					if (btm_cls) {
						suggspeed[2] += 10;
						cout << "btmm" << endl;
					}
				}
				//SendSugspeed();

				if (obj_in[0] == 0)
				{

					if (touch_pos[0] == 1 || touch_pos[2] == 1 || touch_pos[5] == 1 || touch_pos[6] == 1 || touch_pos[1] == 1 || touch_pos[7] == 1)
					{
						//cls_n += 1;
						fine_adjust = '2';
						//svr->SendCommand(SPC, TTS_SPEAK, ONECLICK_2);//adjusting the gripper
						//strcpy(text_buff, "adjusting the gripper.");
						for (int j = 0; j < 6; j++)
						{
							cls_pos[j] = pos[j];
						}
						if (grasp_side == 0)// front grasp, front collision 
						{
							if (touch_pos[0] == 1)// front collision,  y desire need shift
							{
								adj_dire(2, 1) = 1 * 30;//left collision, y_d need shift along +y
							}
							else if (touch_pos[5] == 1)
							{
								adj_dire(2, 1) = -1 * 30;//right collision, y_d need shift along -y
							}


						}
						else if (grasp_side == 1)//top grasp, buttom corner collision
						{
							if (touch_pos[0] == 1 || touch_pos[2] == 1)// front collision,  y desire need shift
							{
								adj_dire(2, 1) = 1 * 30;//left collision, y_d need shift along +y
							}
							else if (touch_pos[5] == 1 || touch_pos[6] == 1)
							{
								adj_dire(2, 1) = -1 * 30;//right collision, y_d need shift along -y
							}
							if (touch_pos[1] == 1 || touch_pos[7] == 1)
							{
								adj_dire(1, 1) = 1 * 20;//
								adj_dire(3, 1) = 1 * 20;//
							}
							cout << "hit corner" << endl;
						}

						delta_temp = EE2W_m * adj_dire;//adjust desire gripper position in ee frame
						for (int k = 0; k < 3; k++)
						{
							temp_pos[k] += delta_temp(k + 1, 1);
						}

					}
				}
				else if (obj_in[0] == 1)
				{

					if (touch_pos[2] == 1 || touch_pos[6] == 1 && !btm_cls)
					{
						for (int j = 0; j < 6; j++)
						{
							cls_pos[j] = pos[j];
						}
						btm_cls = true;
					}
					if (grasp_side == 1)// top grasp, front collision 
					{
						if (touch_pos[2] == 1 || touch_pos[6] == 1)// front collision,  y desire need shift
						{
							adj_dire(1, 1) = -1 * 30;//left collision, y_d need shift along +y
							delta_temp = EE2W_m * adj_dire;//adjust desire gripper position in ee frame
							for (int k = 0; k < 3; k++)
							{
								temp_pos[k] = cls_pos[k] + delta_temp(k + 1, 1);
							}
							fine_adjust = '2';
						}


					}
				}
				break;
			case '2'://retreat from object after collision
				oneclick_mode = 7;
				int dist_cls;
				dist_cls = sqrt(pow((pos[0] - cls_pos[0]), 2) +
					pow((pos[1] - cls_pos[1]), 2) +
					pow((pos[2] - cls_pos[2]), 2));
				int dist_d;
				dist_d = sqrt(pow((pos[0] - temp_pos[0]), 2) +
					pow((pos[1] - temp_pos[1]), 2) +
					pow((pos[2] - temp_pos[2]), 2));


				for (int i = 0; i < 6; i++)
					d_po[i] = temp_pos[i] - pos[i];
				if (dist_cls < 20)//retreat first, just in case move along the collision position to desire position might move the object
				{
					for (int i = 0; i < 3; i++)
						suggspeed[i] = -wa(i + 1, 1) * 30;
					//SendSugspeed();
					//cout << "1" << endl;
				}
				else if (dist_d > 10)
				{
					for (int i = 0; i < 3; i++)
					{
						suggspeed[i] = d_po[i] /
							sqrt(pow(d_po[0], 2) + pow(d_po[1], 2) + pow(d_po[2], 2)) * 15;
					}
					//SendSugspeed();
					//cout << "2" << endl;
				}
				else
				{
					if (obj_in[0] == 0)
					{
						fine_adjust = '1';
						//svr->SendCommand(SPC, TTS_SPEAK, ONECLICK_1);
						//strcpy(text_buff, "Approach to the object.");
						for (int i = 0; i < 6; i++)
							suggspeed[i] = 0;
						//SendSugspeed();
					}
					else
					{
						fine_adjust = '4';
						//svr->SendCommand(SPC, TTS_SPEAK, OGB_IN);
						for (int i = 0; i < 6; i++)
							suggspeed[i] = 0;
						//SendSugspeed();
						btm_cls = false;
					}
					//cout << "3" << endl;
				}
				break;
			case '3'://move after retreat
					 //cout << "move gripper" << endl;

				break;
			case '4'://suggest close gripper
					 //cout << "close gripper" << endl;
				oneclick_mode = 8;
				break;

			}
		}
	}
	//}//  assistant mode end
}


int viewcheck(float Position[6], int axis, float offset, int ee)
{
	Matrix<3, 3> EE2W_m2, EE2c, EE2W_m2_t;
	Matrix<3, 1> p_frame, ROB_pos, camera_offset, testt, wa, ca;
	int track_x, track_y;
	float temp_pos[6];
	float dist_cam;
	bool cam_c = false;
	for (int i = 0; i < 6; i++)
	{
		temp_pos[i] = Position[i];
	}
	if (ee&&axis < 2)
	{
		switch (axis)
		{
		case(0):
			ca = 0, 0, offset;
			break;
		case(1):
			ca = -offset, 0, 0;
			break;
		default:
			break;
		}
		wa = C2W_transform2(pos) * ca;
		for (int j = 0; j < 3; j++)
		{
			temp_pos[j] += wa(j + 1, 1);
		}
	}
	else
	{
		temp_pos[axis] += offset;
	}


	EE2W_m2 = EE2w_transform2(temp_pos);
	EE2c = 0, 0, 1, -1, 0, 0, 0, -1, 0;
	ROB_pos = temp_pos[0], temp_pos[1], temp_pos[2];

	camera_offset = 95, 20, 70;
	EE2W_m2_t = transpose(EE2W_m2*EE2c);//*EE2c
	testt = EE2W_m2 * camera_offset;
	p_frame = EE2W_m2_t * (p_frame_w - ROB_pos - EE2W_m2 * camera_offset);

	track_x = int(p_frame(1, 1) * 535 / p_frame(3, 1)) + 323;
	track_y = int(p_frame(2, 1) * 535 / p_frame(3, 1)) + 237;

	if (track_x >= 40 && track_x <= 600 && track_y >= 40 && track_y <= 440)
		return 0;
	else if (track_x <= 40)//out of view from left side
		return 1;
	else if (track_x >= 600)//out of view from right side
		return 2;
	else if (track_y <= 40)//out of view from top side
		return 3;
	else if (track_y >= 440)//out of view from buttom side
		return 4;

}

void Operation_check(void)
{
	int user_cmd = 10;

	if (!spaceMouseEnabled)
	{
		switch (btn_cmd)
		{
		case '<':
			user_cmd = 8;
			break;
		case '>':
			user_cmd = -8;
			break;
		case '3':
			user_cmd = 2;
			break;
		case '2':
			user_cmd = -2;
			break;
		case 'e':
			user_cmd = 3;
			break;
		case 'd':
			user_cmd = -3;
			break;
		case 'y':
			user_cmd = 6;
			break;
		case 'h':
			user_cmd = -6;
			break;
		case 'f':
			user_cmd = 4;
			break;
		case 'r':
			user_cmd = -4;
			break;
		case 't':
			user_cmd = 5;
			break;
		case 'g':
			user_cmd = -5;
			break;
		default:
			user_cmd = 10;// no operation in list
			break;
		}
	}
	else
	{
		if (abs(spaceMouse[0]) > spacemouse_translation_sensitivity)
		{
			spaceMouse[0] > 0 ? user_cmd = 8 : user_cmd = -8;
		}
		else if (abs(spaceMouse[1]) > spacemouse_translation_sensitivity)
		{
			spaceMouse[1] > 0 ? user_cmd = 2 : user_cmd = -2;
		}
		else if (abs(spaceMouse[2]) > spacemouse_translation_sensitivity)
		{
			spaceMouse[2] > 0 ? user_cmd = 3 : user_cmd = -3;
		}
		else if (abs(spaceMouse[3]) > spacemouse_rotation_sensitivity)
		{
			spaceMouse[3] > 0 ? user_cmd = -4 : user_cmd = 4;
		}
		else if (abs(spaceMouse[4]) > spacemouse_rotation_sensitivity)
		{
			spaceMouse[4] > 0 ? user_cmd = 5 : user_cmd = -5;
		}
		else if (abs(spaceMouse[5]) > spacemouse_rotation_sensitivity)
		{
			spaceMouse[5] > 0 ? user_cmd = 6 : user_cmd = -6;
		}
		else {
			user_cmd = 10;
		}
	}

	if (user_cmd != 10 && user_cmd != suggestedMotion)
	{
		move_as_suggested = false;
	}
	else {
		move_as_suggested = true;// if no operation in list, then turn the flag to true.
	}

}

int cam_cls_check(float Position[6], int axis, float offset, int ee)
{
	Matrix<3, 1> wa, ca;
	float temp_pos[6];
	float dist_cam;
	bool cam_c = false;

	for (int i = 0; i < 6; i++)
	{
		temp_pos[i] = Position[i];
	}

	if (ee&&axis < 2)
	{
		switch (axis)
		{
		case(0):
			ca = 0, 0, offset;
			break;
		case(1):
			ca = -offset, 0, 0;
			break;
		default:
			break;
		}
		wa = C2W_transform2(pos) * ca;
		for (int j = 0; j < 3; j++)
		{
			temp_pos[j] += wa(j + 1, 1);
		}
	}
	else
	{
		temp_pos[axis] += offset;
	}

	temp_pos[5] = sign(temp_pos[5])*(180 - fabs(temp_pos[5]));
	dist_cam = DistanceBetween_Camera_Link3(temp_pos);

	if (dist_cam < 42)
	{
		//ResetAll();
		cam_c = true;
	}

	return cam_c;

}


void block_camcls_move(void)
{
	//bool cam_cls_flag = cam_cls_check(currentPosition, axis, deltaPosition[axis], ee);
	int block_movement[6] = { 0 };
	bool cam_flag = 0;
	for (int i = 0; i < 6; i++)// check for each axis
	{
		for (int j = 0; j < 2; j++)// check for each direction
		{
			for (int off = 1; off < 4; off++)  //check for each step size to make sure there is no collision in this direction,
											//since the distance is not linear, 
			{
				if (i<3)
					cam_flag = cam_cls_check(pos, i, -1*j*5*off, 0);//5 for translation motion
				else
					cam_flag = cam_cls_check(pos, i, 1 * j * 2 * off, 0);// 3 for rotation motion

				if (cam_flag)
					block_movement[i] = (-1) *j;
				else
					block_movement[i] = 0;

			}
		}
	}
}




void suggest_btn2(float deltaPosition[13], int ee)
{

	//int suggestedButtonSwitch;
	//int	suggestedButton = 0;

	float rotationThreshold = 3;
	float positionThreshold = 10;
	int coe_e = 2;//global?
	int axis;
	float cam_dist;
	//int thres;
	//float deltaPosition_o[6] ;// should be global
	//for (int i = 0; i < 6; i++)
	//{
	//	deltaPosition_o[i] = deltaPosition[i];
	//}

	//if (sg_stage == 2)
	//{
		//if (init_sug&&suggestedButtonSwitch != 'X')
		//	init_sug = false;

	//if (user_oprt[0] == 1 && user_oprt[1] == 0 && update_sug != 1)// has problem when user moves as suggested, the suggestion will still refresh again
	//	//after user's operation, start over the suggestion 
	//{
	//	update_sug = 1;
	//	suggestedButtonSwitch = 'Z';
	//}


	if (update_sug)//global
	{

		if (suggestedButtonSwitch == 'x')
			axis = 0;
		else if (suggestedButtonSwitch == 'Y')
			axis = 1;
		else if (suggestedButtonSwitch == 'Z')
			axis = 2;
		else if (suggestedButtonSwitch == 'y' || suggestedButtonSwitch == 'a')
			axis = 3;
		else if (suggestedButtonSwitch == 'p' || suggestedButtonSwitch == 'b')
			axis = 4;
		else if (suggestedButtonSwitch == 'r')
			axis = 5;
		else if (suggestedButtonSwitch == 'X')
		{
			update_sug = 0;
		}

		if (D2obj > 400)
		{
			if (update_sug != 0)
			{
				bool cam_cls_flag = cam_cls_check(currentPosition, axis, deltaPosition[axis], ee);
				int view_check = viewcheck(currentPosition, 0 * axis, 0 * deltaPosition[axis], 0 * ee);

				if (view_check != 0)//if the object is not in the view, first move the objec in view
				{
					if (view_check == 1)
					{
						suggestedButtonSwitch = 'a';
						thres = rotationThreshold;
					}
					else if (view_check == 2)
					{
						suggestedButtonSwitch = 'a';
						thres = rotationThreshold;
					}
					else if (view_check == 3)
					{
						suggestedButtonSwitch = 'b';
						thres = rotationThreshold;
					}
					else if (view_check == 4)
					{
						suggestedButtonSwitch = 'b';
						thres = rotationThreshold;
					}

				}
				else
				{
					if (viewcheck(currentPosition, axis, deltaPosition[axis], ee) != 0 ||
						(cam_cls_flag != 0))
						//when the object in view get first motion length,only once at the beginning, if the motion will colide or lost the object in view, 
						// then break the movement in half or more,
					{

						while (viewcheck(currentPosition, axis, deltaPosition[axis] / coe_e, ee) != 0)
						{
							coe_e += 1;
							if (coe_e > 4)
								break;
						}
						bool cam_cls_flag2 = cam_cls_check(currentPosition, axis, deltaPosition[axis], ee);
						if (cam_cls_flag2 != 0)
							// if the planed movement will collide , change to next direction
						{
							if (axis != 5)
							{
								suggestedButtonSwitch = suggested_btn_order[axis + 1];
							}
							else
								suggestedButtonSwitch = suggested_btn_order[0];
							return;
						}
						moveL[axis] = deltaPosition[axis] * (1.0 - 1.0 / coe_e);
						thres = moveL[axis];
						if (axis < 3)
						{
							if (abs(thres) < positionThreshold)
								thres = positionThreshold;
						}
						else if (axis > 2)
						{
							if (abs(thres) < rotationThreshold)
								thres = rotationThreshold;
						}
					}
					else
						// if the full length of deltaposition doesn't cause collision or lost the obejct, set the thres to the full length
					{
						//moveL[axis] = deltaPosition[axis];
						if (axis < 3)
						{
							thres = positionThreshold;
						}
						else if (axis > 2)
						{
							thres = rotationThreshold;
						}
					}
				}
			}
		}
		else// when gripper is close to the object, the track point may lost in the view since the camera has an offset respect to the gripper
			// so, when the gripper is very close to the object, just suggested the motion directly to the desire posiion. 
		{
			if (axis < 3)
			{
				thres = positionThreshold;
			}
			else if (axis > 2)
			{
				thres = rotationThreshold;
			}
		}
		update_sug = 0;
	}

	switch (suggestedButtonSwitch)
	{
	case 'Z':

		//if (deltaPosition[2] == moveL[2])
		//	thres = positionThreshold;
		//else
		//	thres = moveL[2];
		if (abs(deltaPosition[2]) > abs(thres))// Z 
		{
			if (deltaPosition[2] > 0)
			{
				//suggestedButton = "-y";//r,t,y
				suggestedMotion = 3;//r,t,y
			}
			else if (deltaPosition[2] < 0)
			{
				//suggestedButton = "+y";
				suggestedMotion = -3;
			}

		}
		else
		{
			//o_suggestedButtonSwitch = suggestedButtonSwitch;
			suggestedButtonSwitch = 'p';
			update_sug = 1;
		}
		break;
	case 'p'://left right
		//int thres;
		//if (deltaPosition[4] == moveL[4])
		//	thres = rotationThreshold;
		//else
		//	thres = (moveL[4]);

		if (fabs(deltaPosition[4]) > abs(thres))
		{
			//suggestedButton = deltaPosition[1] < 0 ? "R" : "L"; //2,3
			suggestedMotion = deltaPosition[4] < 0 ? -5 : 5; //2,3
		}
		else
		{
			//o_suggestedButtonSwitch = suggestedButtonSwitch;
			if (abs(deltaPosition[2]) < positionThreshold)
			{
				suggestedButtonSwitch = 'Y';
				update_sug = 1;
			}
			else
			{
				suggestedButtonSwitch = 'Z';
				update_sug = 1;
			}
		}
		break;
	case 'Y':
		//int thres;
		//if (deltaPosition[1] == moveL[1])
		//	thres = positionThreshold;
		//else
		//	thres =(moveL[1]);
		if (abs(deltaPosition[1]) > abs(thres))// Z 
		{
			if (deltaPosition[1] > 0)
			{
				//suggestedButton = "-y";//r,t,y
				suggestedMotion = 2;//r,t,y
			}
			else if (deltaPosition[1] < 0)
			{
				//suggestedButton = "+y";
				suggestedMotion = -2;
			}

		}
		else
		{
			//o_suggestedButtonSwitch = suggestedButtonSwitch;
			suggestedButtonSwitch = 'y';
			update_sug = 1;
		}
		break;
	case 'y'://
		//int thres;
		//if (deltaPosition[3] == moveL[3])
		//	thres = rotationThreshold;
		//else
		//	thres = (moveL[3]);

		if (fabs(deltaPosition[3]) > abs(thres))
		{
			suggestedMotion = deltaPosition[3] < 0 ? 4 : -4; //2,3
		}
		else
		{
			if (abs(deltaPosition[1]) < positionThreshold)
			{
				suggestedButtonSwitch = 'x';
				update_sug = 1;
			}
			else
			{
				suggestedButtonSwitch = 'Y';
				update_sug = 1;
			}
		}
		break;
	case 'x'://
		//int thres;
		//if (deltaPosition[0] == moveL[0])
		//	thres = positionThreshold;
		//else
		//	thres = (moveL[0]);
		if (abs(deltaPosition[0]) > abs(thres))// Z 
		{
			if (deltaPosition[0] > 0)
			{
				//suggestedButton = "-y";//r,t,y
				suggestedMotion = 8;//r,t,y
			}
			else if (deltaPosition[0] < 0)
			{
				//suggestedButton = "+y";
				suggestedMotion = -8;
			}

		}
		else
		{
			//o_suggestedButtonSwitch = suggestedButtonSwitch;
			suggestedButtonSwitch = 'r';
			update_sug = 1;
		}
		break;
	case 'r'://
		//int thres;
		//if (deltaPosition[5] == moveL[5])
		//	thres = rotationThreshold;
		//else
		//	thres = (moveL[5]);

		if (fabs(deltaPosition[5]) > abs(thres))
		{
			suggestedMotion = deltaPosition[5] < 0 ? -6 : 6; //2,3
		}
		else
		{
			if (abs(deltaPosition[0]) < positionThreshold)
			{
				suggestedButtonSwitch = 'X';
				update_sug = 1;
			}
			else if ((abs(deltaPosition[0]) < positionThreshold) &&
				(fabs(deltaPosition[1]) < positionThreshold) &&
				(fabs(deltaPosition[2]) < positionThreshold) &&
				(abs(deltaPosition[3]) < rotationThreshold) &&
				(abs(deltaPosition[4]) < rotationThreshold) &&
				(abs(deltaPosition[5]) < rotationThreshold))
			{
				suggestedButtonSwitch = 'Z';
				init_sug = true;
			}
			else
			{
				suggestedButtonSwitch = 'x';
				update_sug = 1;
			}
		}
		break;
	case 'X'://
		suggestedMotion = 9; //2,3
		if (!((abs(deltaPosition[0]) < 4 * positionThreshold) &&
			(fabs(deltaPosition[1]) < 4 * positionThreshold) &&
			(fabs(deltaPosition[2]) < 4 * positionThreshold) &&
			(abs(deltaPosition[3]) < 4 * rotationThreshold) &&
			(abs(deltaPosition[4]) < 4 * rotationThreshold) &&
			(abs(deltaPosition[5]) < 4 * rotationThreshold)))
		{
			suggestedButtonSwitch = 'Z';
		}
		break;
	case 'a':
		if (fabs(deltaPosition[11]) > rotationThreshold)
		{
			suggestedMotion = deltaPosition[6] < 0 ? 4 : -4;
		}
		else
		{
			suggestedButtonSwitch = '2';
			update_sug = 1;
		}
		break;
	case 'b':
		if (fabs(deltaPosition[12]) > rotationThreshold)
		{
			suggestedMotion = deltaPosition[7] < 0 ? -5 : 5;
		}
		else
		{
			suggestedButtonSwitch = 'Z';
			update_sug = 1;
		}
		break;
	default:
		break;
	}
	//}
}

void suggest_btn(float *deltaPosition)
{

	//int suggestedButtonSwitch;
	//int	suggestedButton = 0;
	float rotationThreshold = 3;
	float positionThreshold = 10;


	if (D2obj > 400 && sg_stage == 2)
	{
		sg_stage = 1;
	}
	else if (D2obj > 350 && sg_stage != 2) {
		sg_stage = 1;
	}
	else if (D2obj <= 350) {
		sg_stage = 2;
	}


	if (sg_stage == 1)
	{
		if (abs(deltaPosition[6]) > rotationThreshold&& init_sug)
		{
			if (deltaPosition[6] > 0) // yaw
			{
				suggestedMotion = -4;//r,t,y
			}
			else if (deltaPosition[6] < 0)
			{
				suggestedMotion = 4;
			}
		}
		else if (abs(deltaPosition[7]) > 2 * rotationThreshold&& init_sug)// pitch
		{
			if (deltaPosition[7] > 0)
			{
				//suggestedButton = "p";//r,t,y
				suggestedMotion = 5;//r,t,y
			}
			else if (deltaPosition[7] < 0)
			{
				//suggestedButton = "-p";
				suggestedMotion = -5;
			}
		}
		else if (init_sug&&btn_cmd == 'n')
		{
			init_sug = false;// once gripper are pointing to the object, start to achieve
			//suggestedButton = "A";//approach

		}
		else {
			suggestedMotion = 7;
			suggestedButtonSwitch = 'Y';

		}
	}
	else if (sg_stage == 2)
	{
		//if (init_sug&&suggestedButtonSwitch != 'X')
		//	init_sug = false;
		switch (suggestedButtonSwitch)
		{
		case 'Y':
			//if (xout)//center[1]  mark x position
			//{
			//	suggestedButtonSwitch = 'l';
			//	break;
			//}
			//else 
			if (abs(deltaPosition[3]) > 20)// yaw
			{
				if (deltaPosition[3] > 0)
				{
					//suggestedButton = "-y";//r,t,y
					suggestedMotion = -4;//r,t,y
				}
				else if (deltaPosition[3] < 0)
				{
					//suggestedButton = "+y";
					suggestedMotion = 4;
				}

			}
			else
			{
				//o_suggestedButtonSwitch = suggestedButtonSwitch;
				suggestedButtonSwitch = 'L';
			}
			break;
		case 'L'://left right
				 //if (xout)//center[1]  mark x position
				 //{
				 //	suggestedButtonSwitch = 'y';
				 //	break;
				 //}
				 //else 
			if (fabs(deltaPosition[1]) > 10)
			{
				//suggestedButton = deltaPosition[1] < 0 ? "R" : "L"; //2,3
				suggestedMotion = deltaPosition[1] < 0 ? -2 : 2; //2,3
			}
			else
			{
				//o_suggestedButtonSwitch = suggestedButtonSwitch;
				suggestedButtonSwitch = 'y';
			}
			break;
		case 'y':
			if (abs(deltaPosition[3]) > 3)//yaw again
			{
				if (deltaPosition[3] > 0)
				{
					//suggestedButton = "-y";//r,t,y
					suggestedMotion = -4;//r,t,y
				}
				else if (deltaPosition[3] < 0)
				{
					//suggestedButton = "+y";
					suggestedMotion = 4;
				}

			}
			else if (fabs(deltaPosition[1]) > 10)
			{
				//o_suggestedButtonSwitch = suggestedButtonSwitch;
				suggestedButtonSwitch = 'L';
			}
			else
			{
				suggestedButtonSwitch = 'P';
			}
			break;
		case 'U'://1st up down
				 //if (yout)
				 //{
				 //	suggestedButtonSwitch = 'p';
				 //	break;
				 //}
				 //else 
			if (fabs(deltaPosition[2]) > 50)// up/down
			{
				//Suggested Move is the z direction
				//suggestedButton = deltaPosition[2] > 0 ? "U" : "D"; //e,d
				//suggestedMotion = deltaPosition[2] > 0 ? 'e' : 'd'; //e,d
				if (deltaPosition[2] > 50)
				{
					//suggestedButton = "U";
					suggestedMotion = 3;
				}
				else if (deltaPosition[2] < -50)
				{
					//suggestedButton = "D";
					suggestedMotion = -3;
				}
			}
			else
			{
				//o_suggestedButtonSwitch = suggestedButtonSwitch;
				suggestedButtonSwitch = 'O';
			}
			break;
		case 'P'://1st pitch
				 //if (yout)
				 //{
				 //	suggestedButtonSwitch = 'u';
				 //	break;
				 //}
				 //else 
			if (abs(deltaPosition[4]) > 15)
			{
				if (deltaPosition[4] > 0)
				{
					//suggestedButton = "p";//r,t,y
					suggestedMotion = 5;//r,t,y
				}
				else if (deltaPosition[4] < 0)
				{
					//suggestedButton = "-p";
					suggestedMotion = -5;
				}
			}
			else
			{
				suggestedButtonSwitch = 'U';
			}
			break;
		case 'D'://2st up down
				 //if (yout)
				 //{
				 //	suggestedButtonSwitch = 'p';
				 //	break;
				 //}
				 //else 
			if (fabs(deltaPosition[2]) > 15)
			{
				//Suggested Move is the z direction
				//suggestedButton = deltaPosition[2] > 0 ? "U" : "D"; //e,d
				//suggestedMotion = deltaPosition[2] > 0 ? 'e' : 'd'; //e,d
				if (deltaPosition[2] > 10)
				{
					//suggestedButton = "U";
					suggestedMotion = 3;
				}
				else if (deltaPosition[2] < -10)
				{
					//suggestedButton = "D";
					suggestedMotion = -3;
				}
			}
			else
			{
				//o_suggestedButtonSwitch = suggestedButtonSwitch;
				suggestedButtonSwitch = 'F';
			}
			break;
		case 'O'://2st pitch
				 //if (yout)
				 //{
				 //	suggestedButtonSwitch = 'u';
				 //	break;
				 //}
				 //else 
			if (abs(deltaPosition[4]) > 3)
			{
				if (deltaPosition[4] > 0)
				{
					//suggestedButton = "p";//r,t,y
					suggestedMotion = 5;//r,t,y
				}
				else if (deltaPosition[4] < 0)
				{
					//suggestedButton = "-p";
					suggestedMotion = -5;
				}
			}
			else
			{
				suggestedButtonSwitch = 'D';
			}
			break;
		case 'F':
			if (deltaPosition[0] > positionThreshold)
			{
				//suggestedButton = deltaPosition[0] > 0 ? "F" : "B"; //n,m
				suggestedMotion = deltaPosition[0] > 0 ? 8 : -8; //n,m
			}
			else
			{
				suggestedButtonSwitch = 'R';
			}
			break;
		case 'R':
			if (abs(deltaPosition[5]) > rotationThreshold)
			{
				if (deltaPosition[5] > 0)
				{
					//suggestedButton = "+r";//r,t,y
					suggestedMotion = 6;//r,t,y
				}
				else if (deltaPosition[5] < 0)
				{
					//suggestedButton = "-r";
					suggestedMotion = -6;
				}
			}
			else
			{
				if ((deltaPosition[0] < positionThreshold) &&
					(fabs(deltaPosition[1]) < positionThreshold) &&
					(fabs(deltaPosition[2]) < 15) &&
					(abs(deltaPosition[3]) < rotationThreshold) &&
					(abs(deltaPosition[4]) < rotationThreshold) &&
					(abs(deltaPosition[5]) < rotationThreshold))
				{
					suggestedButtonSwitch = 'X';
					init_sug = true;
				}
				else
				{
					suggestedButtonSwitch = 'Y';
					break;
				}
				suggestedMotion = 0;
				if (fine_adjust == false)
				{
					//R_t = TimeCheck();
					//cout << "RT"<<R_t << endl;

					//fine_adjust = true;
					btn_flag = false;
					//fine_count = 0;
					//svr->SendCommand(SPC, TTS_SPEAK, AS_DOWN);
					//gripper_center = true;
				}
			}
			break;
		default:;
		}
	}
	//else
	//	suggestedMotion = 'x';
		//}
		//else
		//	suggestedMotion = 'x';

		//switch (suggestedMotion) {
		//case 'n':
		//	//manual_forward_btn->color(BTN_CLR);
		//	//manual_forward_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 'a':
		//	manual_approach_btn->color(BTN_CLR);
		//	manual_approach_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 'm':
		//	manual_bckward_btn->color(BTN_CLR);
		//	manual_bckward_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case '3':
		//	manual_left_btn->color(BTN_CLR);
		//	manual_left_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case '2':
		//	manual_right_btn->color(BTN_CLR);
		//	manual_right_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 'e':
		//	manual_up_btn->color(BTN_CLR);
		//	manual_up_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 'd':
		//	manual_dwn_btn->color(BTN_CLR);
		//	manual_dwn_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 'h':
		//	manual_roll_ccw_btn->color(BTN_CLR);
		//	manual_roll_ccw_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 'y':
		//	manual_roll_clw_btn->color(BTN_CLR);
		//	manual_roll_clw_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 'r':
		//	manual_yaw_left_btn->color(BTN_CLR);
		//	manual_yaw_left_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 'f':
		//	manual_yaw_rght_btn->color(BTN_CLR);
		//	manual_yaw_rght_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 'g':
		//	manual_pitch_dwn_btn->color(BTN_CLR);
		//	manual_pitch_dwn_btn->labelcolor(BTN_L_CLR);
		//	break;
		//case 't':
		//	manual_pitch_up_btn->color(BTN_CLR);
		//	manual_pitch_up_btn->labelcolor(BTN_L_CLR);
		//	break;
		//default:;
		//}
		//manual_mode_menu->redraw();

		//if (previousSuggestedMotion != suggestedMotion)
		//{
		//	svr->SendCommand3(GUI, CAN, TP_END, ' ');
		//	switch (previousSuggestedMotion)
		//	{
		//	case 'n':
		//		manual_forward_btn->color(FL_WHITE);
		//		manual_forward_btn->labelcolor(FL_BLACK);
		//		redraw();
		//		break;
		//	case 'a':
		//		manual_approach_btn->color(FL_WHITE);
		//		manual_approach_btn->labelcolor(FL_BLACK);
		//		redraw();
		//		break;
		//	case 'm':
		//		manual_bckward_btn->color(FL_WHITE);
		//		manual_bckward_btn->labelcolor(FL_BLACK);
		//		break;
		//	case '3':
		//		manual_left_btn->color(FL_WHITE);
		//		manual_left_btn->labelcolor(FL_BLACK);
		//		break;
		//	case '2':
		//		manual_right_btn->color(FL_WHITE);
		//		manual_right_btn->labelcolor(FL_BLACK);
		//		break;
		//	case 'e':
		//		manual_up_btn->color(FL_WHITE);
		//		manual_up_btn->labelcolor(FL_BLACK);
		//		break;
		//	case 'd':
		//		manual_dwn_btn->color(FL_WHITE);
		//		manual_dwn_btn->labelcolor(FL_BLACK);
		//		break;
		//	case 'h':
		//		manual_roll_ccw_btn->color(FL_WHITE);
		//		manual_roll_ccw_btn->labelcolor(FL_BLACK);
		//		break;
		//	case 'y':
		//		manual_roll_clw_btn->color(FL_WHITE);
		//		manual_roll_clw_btn->labelcolor(FL_BLACK);
		//		break;
		//	case 'r':
		//		manual_yaw_left_btn->color(FL_WHITE);
		//		manual_yaw_left_btn->labelcolor(FL_BLACK);
		//		break;
		//	case 'f':
		//		manual_yaw_rght_btn->color(FL_WHITE);
		//		manual_yaw_rght_btn->labelcolor(FL_BLACK);
		//		break;
		//	case 'g':
		//		manual_pitch_dwn_btn->color(FL_WHITE);
		//		manual_pitch_dwn_btn->labelcolor(FL_BLACK);
		//		break;
		//	case 't':
		//		manual_pitch_up_btn->color(FL_WHITE);
		//		manual_pitch_up_btn->labelcolor(FL_BLACK);
		//		break;
		//	default:;
		//	}
//}
}

void movetopos(void)// move to certain position
{
	//int packet_pos = 3;
	//float tmp;

	if (!moveto)
	{
		for (int i = 0; i < 6; ++i)
		{
			//packet_pos += packet_CAN.Read(tmp, packet_pos);
			pd[i] = set_pos[i];
		}
		start_time_set_point_vel = TimeCheck();
		//set_point_vel = true;		// look inside pdcontrol2()

		adjust_pos = true;
		job_complete = false;
		job_complete2 = true;

		if (myRcv.command == NULL)
		{
			myRcv.key.writeLock();
			myRcv.command = 'l';
			myRcv.key.unlock();
		}
		moveto = true;
	}
}



////--------------------------------------------------------
////SPACEMOUSE FUNCTIONS BELOW HERE
////--------------------------------------------------------
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_CONTROL));
	wcex.hCursor = LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = MAKEINTRESOURCE(IDC_CONTROL);
	wcex.lpszClassName = szWindowClass;
	wcex.hIconSm = NULL;

	return RegisterClassEx(&wcex);
}
LRESULT CALLBACK WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT ps;           /* used to paint the client area of a window */
	LONG addr;                /* address of our window */

	addr = GetClassLong(hWnd, 0);  /* get address */

	switch (msg)
	{
	case WM_ACTIVATEAPP:
		hdc = GetDC(hWnd);
		/* print buffers */
		TextOut(hdc, 0, 20, devicename, (int)_tcslen(devicename));
		TextOut(hdc, 15, 100, _T("TX: 0"), 5);
		TextOut(hdc, 15, 120, _T("TY: 0"), 5);
		TextOut(hdc, 15, 140, _T("TZ: 0"), 5);
		TextOut(hdc, 15, 160, _T("RX: 0"), 5);
		TextOut(hdc, 15, 180, _T("RY: 0"), 5);
		TextOut(hdc, 15, 200, _T("RZ: 0"), 5);
		TextOut(hdc, 15, 220, _T(" P: 0"), 5);

		/*release our window handle */
		ReleaseDC(hWnd, hdc);
	case WM_KEYDOWN:
	case WM_KEYUP:
		/* user hit a key to close program */
		if (wParam == VK_ESCAPE)
		{
			SendMessage(hWndMain, WM_CLOSE, 0, 0l);
		}
		break;

	case WM_PAINT:
		/* time to paint the window */
		if (addr)
		{
			hdc = BeginPaint(hWndMain, &ps);
			EndPaint(hWndMain, &ps);
		}

		break;

	case WM_CLOSE:
		/* cleanup the object info created */

		break;

	case WM_DESTROY:
		PostQuitMessage(0);
		return (0);
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}
// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}
/*--------------------------------------------------------------------------
* Function: SbInit()
*
* Description:
*    This function initializes the 3D mouse and opens ball for use.
*
*
* Args: None
*
*
* Return Value:
*    int  res         result of SiOpen, =0 if Fail =1 if it Works
*
*--------------------------------------------------------------------------*/
int SbInit()
{
	int res;                             /* result of SiOpen, to be returned  */
	SiOpenData oData;                    /* OS Independent data to open ball  */

	/*init the SpaceWare input library */
	if (SiInitialize() == SPW_DLL_LOAD_ERROR)
	{
		MessageBox(hWndMain, _T("Error: Could not load SiAppDll dll files"),
			NULL, MB_ICONEXCLAMATION);
	}

	SiOpenWinInit(&oData, hWndMain);    /* init Win. platform specific data  */

	/* open data, which will check for device type and return the device handle
	to be used by this function */
	if ((devHdl = SiOpen("spaceMouse", SI_ANY_DEVICE, SI_NO_MASK, SI_EVENT, &oData)) == NULL)
	{
		SiTerminate();  /* called to shut down the SpaceWare input library */
		res = 0;        /* could not open device */
		return res;
	}
	else
	{
		SiDeviceName devName;
		SiGetDeviceName(devHdl, &devName);
		SiSetUiMode(devHdl, SI_UI_NO_CONTROLS); /* Config SoftButton Win Display */
		SiGrabDevice(devHdl, SPW_TRUE); /* PREVENTS OTHER APPLICATIONS FROM RECEIVING 3D CONNEXION DATA !!! */
		_stprintf_s(devicename, SPW_NUM_ELEMENTS_IN(devicename), _T("%S"), devName.name);
		res = 1;        /* opened device succesfully */
		return res;
	}

}
/*--------------------------------------------------------------------------
* Function: DispatchLoopNT()
*
* Description:
*    This function contains the main message loop which constantly checks for
*    3D mouse Events and handles them apropriately.
*
* Args: None
*
*
* Return Value:
*    int  msg.wparam                  // event passed to window
*
*--------------------------------------------------------------------------*/
//int DispatchLoopNT()
//{
//	MSG            msg;      /* incoming message to be evaluated */
//	BOOL           handled;  /* is message handled yet */ 
//	SiSpwEvent     Event;    /* SpaceWare Event */ 
//	SiGetEventData EData;    /* SpaceWare Event Data */
//
//	handled = SPW_FALSE;     /* init handled */
//
//	/* start message loop */ 
//	while ( GetMessage( &msg, NULL, 0, 0 ) )
//	{
//		handled = SPW_FALSE;
//
//		/* init Window platform specific data for a call to SiGetEvent */
//		SiGetEventWinInit(&EData, msg.message, msg.wParam, msg.lParam);
//
//		/* check whether msg was a 3D mouse event and process it */
//		if (SiGetEvent (devHdl, SI_AVERAGE_EVENTS, &EData, &Event) == SI_IS_EVENT)
//		{
//			if (Event.type == SI_MOTION_EVENT)
//			{
//				SbMotionEvent(&Event);        /* process 3D mouse motion event */     
//			}
//			else if (Event.type == SI_ZERO_EVENT)
//			{
//				SbZeroEvent();                /* process 3D mouse zero event */     
//			}
//			else if (Event.type == SI_BUTTON_PRESS_EVENT)
//			{
//				SbButtonPressEvent(Event.u.hwButtonEvent.buttonNumber);  /* process button press event */
//			}
//			else if (Event.type == SI_BUTTON_RELEASE_EVENT)
//			{
//				SbButtonReleaseEvent(Event.u.hwButtonEvent.buttonNumber); /* process button release event */
//			}
//			else if (Event.type == SI_DEVICE_CHANGE_EVENT)
//			{
//				HandleDeviceChangeEvent(&Event); /* process 3D mouse device change event */
//			}
//			else if (Event.type == SI_CMD_EVENT)
//			{
//				HandleV3DCMDEvent(&Event); /* V3DCMD_* events */
//			}
//			else if (Event.type == SI_APP_EVENT)
//			{
//				HandleAppEvent(&Event); /* V3DCMD_* events */
//			}
//
//			handled = SPW_TRUE;              /* 3D mouse event handled */ 
//		}
//
//		/* not a 3D mouse event, let windows handle it */
//		if (handled == SPW_FALSE)
//		{
//			TranslateMessage( &msg );
//			DispatchMessage( &msg );
//		}
//	}
//
//	return( (int) msg.wParam );
//} 
/*--------------------------------------------------------------------------
* Function: HandleNTEvent
*
* Description:  This is a std. Win32 function to handle various window events
*
*
*
* Args: HWND hWnd                    // handle to window
*       unsigned msg                 // message to process
*       WPARAM wParam                // 32 bit msg parameter
*       LPARAM lParam                // 32 bit msg parameter
*
* Return Value:
*    int  msg.wparam                 // program done
*
*--------------------------------------------------------------------------*/
LRESULT WINAPI HandleNTEvent(HWND hWnd, unsigned msg, WPARAM wParam,
	LPARAM lParam)
{
	PAINTSTRUCT ps;           /* used to paint the client area of a window */
	LONG addr;                /* address of our window */

	addr = GetClassLong(hWnd, 0);  /* get address */

	switch (msg)
	{
	case WM_ACTIVATEAPP:
		hdc = GetDC(hWnd);
		/* print buffers */
		TextOut(hdc, 0, 20, devicename, (int)_tcslen(devicename));
		TextOut(hdc, 15, 100, _T("TX: 0"), 5);
		TextOut(hdc, 15, 120, _T("TY: 0"), 5);
		TextOut(hdc, 15, 140, _T("TZ: 0"), 5);
		TextOut(hdc, 15, 160, _T("RX: 0"), 5);
		TextOut(hdc, 15, 180, _T("RY: 0"), 5);
		TextOut(hdc, 15, 200, _T("RZ: 0"), 5);
		TextOut(hdc, 15, 220, _T(" P: 0"), 5);

		/*release our window handle */
		ReleaseDC(hWnd, hdc);
	case WM_KEYDOWN:
	case WM_KEYUP:
		/* user hit a key to close program */
		if (wParam == VK_ESCAPE)
		{
			SendMessage(hWndMain, WM_CLOSE, 0, 0l);
		}
		break;

	case WM_PAINT:
		/* time to paint the window */
		if (addr)
		{
			hdc = BeginPaint(hWndMain, &ps);
			EndPaint(hWndMain, &ps);
		}

		break;

	case WM_CLOSE:
		/* cleanup the object info created */

		break;

	case WM_DESTROY:
		PostQuitMessage(0);
		return (0);
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);

}
/*--------------------------------------------------------------------------
* Function: CreateSPWindow
*
* Description:  This creates the window for our app
*
*
*
* Args:  int  atx        // horiz. start point to put window
*        int  aty        // vert.  start point to put window
*        int  hi         // hight of window
*        int  wid        // width of window
*        char *string    // window caption
*
* Return Value:
*    NONE
*
*--------------------------------------------------------------------------*/
void CreateSPWindow(int atx, int aty, int hi, int wid, TCHAR *string)
{
	WNDCLASS  wndclass;     /* our own instance of the window class */
	//HINSTANCE hInst;        /* handle to our instance */

	hInst = NULL;             /* init handle */

	/* Register display window class */
	wndclass.style = CS_HREDRAW | CS_VREDRAW;
	wndclass.lpfnWndProc = (WNDPROC)HandleNTEvent;
	wndclass.cbClsExtra = 8;
	wndclass.cbWndExtra = 0;
	wndclass.hInstance = hInst;
	wndclass.hIcon = NULL;
	wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndclass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wndclass.lpszMenuName = NULL;
	wndclass.lpszClassName = _T("spaceMouse");

	RegisterClass(&wndclass);

	/* create the window */
	hWndMain = CreateWindow(_T("spaceMouse"),           /*Window class name*/
		string,              /*Window caption*/
		WS_OVERLAPPEDWINDOW, /*Window Style*///WS_OVERLAPPEDWINDOW
		atx, aty, wid, hi,
		NULL,	               /*parent window handle*/
		NULL,                /*window menu handle*/
		hInst,		         /*program instance handle*/
		NULL);               /*creation parameters*/

	/* display the window */
	ShowWindow(hWndMain, SW_SHOW);

	/* get handle of our window to draw on */
	hdc = GetDC(hWndMain);

	/* print buffers */
	TextOut(hdc, 0, 20, devicename, (int)_tcslen(devicename));
	TextOut(hdc, 15, 100, _T("TX: 0"), 5);
	TextOut(hdc, 15, 120, _T("TY: 0"), 5);
	TextOut(hdc, 15, 140, _T("TZ: 0"), 5);
	TextOut(hdc, 15, 160, _T("RX: 0"), 5);
	TextOut(hdc, 15, 180, _T("RY: 0"), 5);
	TextOut(hdc, 15, 200, _T("RZ: 0"), 5);
	TextOut(hdc, 15, 220, _T(" P: 0"), 5);

	/*release our window handle */
	ReleaseDC(hWndMain, hdc);

	UpdateWindow(hWndMain);

} /* end of CreateWindow */
/*----------------------------------------------------------------------
* Function: SbMotionEvent()
*
* Description:
*    This function receives motion information and prints out the info
*    on screen.
*
*
* Args:
*    SiSpwEvent *pEvent   Containts Data from a 3D mouse Event
*
* Return Value:
*    NONE
*
*----------------------------------------------------------------------*/
void SbMotionEvent(SiSpwEvent *pEvent)
{
	TCHAR buff0[30];                            /* text buffer for TX */
	TCHAR buff1[30];                            /* text buffer for TY */
	TCHAR buff2[30];                            /* text buffer for TZ */
	TCHAR buff3[30];                            /* text buffer for RX */
	TCHAR buff4[30];                            /* text buffer for RY */
	TCHAR buff5[30];                            /* text buffer for RZ */
	TCHAR buff6[30];                            /* text buffer for Period */

	int len0, len1, len2, len3, len4, len5, len6;	   /* length of each buffer */

	/* put the actual ball data into the buffers */
	len0 = _stprintf_s(buff0, 30, _T("TX: %d         "), pEvent->u.spwData.mData[SI_TX]);
	len1 = _stprintf_s(buff1, 30, _T("TY: %d         "), pEvent->u.spwData.mData[SI_TY]);
	len2 = _stprintf_s(buff2, 30, _T("TZ: %d         "), pEvent->u.spwData.mData[SI_TZ]);
	len3 = _stprintf_s(buff3, 30, _T("RX: %d         "), pEvent->u.spwData.mData[SI_RX]);
	len4 = _stprintf_s(buff4, 30, _T("RY: %d         "), pEvent->u.spwData.mData[SI_RY]);
	len5 = _stprintf_s(buff5, 30, _T("RZ: %d         "), pEvent->u.spwData.mData[SI_RZ]);
	len6 = _stprintf_s(buff6, 30, _T(" P: %d         "), pEvent->u.spwData.period);

	spaceMouse[0] = pEvent->u.spwData.mData[SI_TZ];
	spaceMouse[1] = -(pEvent->u.spwData.mData[SI_TX]);
	spaceMouse[2] = pEvent->u.spwData.mData[SI_TY];

	spaceMouse[3] = pEvent->u.spwData.mData[SI_RY];
	spaceMouse[4] = pEvent->u.spwData.mData[SI_RX];
	spaceMouse[5] = pEvent->u.spwData.mData[SI_RZ];


	//gotoxy( 1, 22);
	//printf("[sm] %5d %5d %5d %5d %5d %5d %5d \n",spaceMouse[0],spaceMouse[1],spaceMouse[2],spaceMouse[3],spaceMouse[4],spaceMouse[5],spaceMouse[6]);

	/* get handle of our window to draw on */
	hdc = GetDC(hWndMain);

	/* print buffers */
	TCHAR *buf = _T("Motion Event              ");
	TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
	TextOut(hdc, 0, 20, devicename, (int)_tcslen(devicename));
	TextOut(hdc, 15, 100, buff0, len0);
	TextOut(hdc, 15, 120, buff1, len1);
	TextOut(hdc, 15, 140, buff2, len2);
	TextOut(hdc, 15, 160, buff3, len3);
	TextOut(hdc, 15, 180, buff4, len4);
	TextOut(hdc, 15, 200, buff5, len5);
	TextOut(hdc, 15, 220, buff6, len6);

	/* Dump to debugger output buffer to get a running log */
	//_RPT3(_CRT_WARN,"%S %S %S",  buff0, buff1, buff2);
	//_RPT3(_CRT_WARN," %S %S %S", buff3, buff4, buff5);
	//_RPT1(_CRT_WARN," %S\n", buff6);

	// Also dump to stdout for a searchable log
	/*printf("%d %d %d %d %d %d\n",
	pEvent->u.spwData.mData[SI_TX],
	pEvent->u.spwData.mData[SI_TY],
	pEvent->u.spwData.mData[SI_TZ],
	pEvent->u.spwData.mData[SI_RX],
	pEvent->u.spwData.mData[SI_RY],
	pEvent->u.spwData.mData[SI_RZ]);
	*/
	/*release our window handle */
	ReleaseDC(hWndMain, hdc);
}
/*----------------------------------------------------------------------
* Function: SbZeroEvent()
*
* Description:
*    This function clears the previous data, no motion data was received
*
*
*
* Args:
*    NONE
*
* Return Value:
*    NONE
*
*----------------------------------------------------------------------*/
void SbZeroEvent()
{
	/* get handle of our window to draw on */
	hdc = GetDC(hWndMain);

	/* print null data */
	TextOut(hdc, 0, 0, _T("Zero Event                  "), 28);
	TextOut(hdc, 0, 20, devicename, (int)_tcslen(devicename));
	TextOut(hdc, 15, 100, _T("TX: 0          "), 15);
	TextOut(hdc, 15, 120, _T("TY: 0          "), 15);
	TextOut(hdc, 15, 140, _T("TZ: 0          "), 15);
	TextOut(hdc, 15, 160, _T("RX: 0          "), 15);
	TextOut(hdc, 15, 180, _T("RY: 0          "), 15);
	TextOut(hdc, 15, 200, _T("RZ: 0          "), 15);
	TextOut(hdc, 15, 220, _T(" P: 0          "), 15);
	for (int i = 0; i < 6; i++) {
		spaceMouse[i] = 0;
	}
	//gotoxy( 1, 22);
	//printf("[sm] %5d %5d %5d %5d %5d %5d %5d [enabled]%d [mode]%d \n",spaceMouse[0],spaceMouse[1],spaceMouse[2],spaceMouse[3],spaceMouse[4],spaceMouse[5],spaceMouse[6],spaceMouseEnabled,spaceMouseMode);


	/*release our window handle */
	ReleaseDC(hWndMain, hdc);
}
TCHAR *V3DKeyToName(TCHAR *buf, int buflen, V3DKey v3dk)
{
	switch (v3dk)
	{
	case s3dm::V3DK_MENU: _tcscpy_s(buf, buflen, _T("V3DK_MENU")); break;
	case s3dm::V3DK_FIT: _tcscpy_s(buf, buflen, _T("V3DK_FIT")); break;
	case s3dm::V3DK_TOP: _tcscpy_s(buf, buflen, _T("V3DK_TOP")); break;
	case s3dm::V3DK_LEFT: _tcscpy_s(buf, buflen, _T("V3DK_LEFT")); break;
	case s3dm::V3DK_RIGHT: _tcscpy_s(buf, buflen, _T("V3DK_RIGHT")); break;
	case s3dm::V3DK_FRONT: _tcscpy_s(buf, buflen, _T("V3DK_FRONT")); break;
	case s3dm::V3DK_BOTTOM: _tcscpy_s(buf, buflen, _T("V3DK_BOTTOM")); break;
	case s3dm::V3DK_BACK: _tcscpy_s(buf, buflen, _T("V3DK_BACK")); break;
	case s3dm::V3DK_ROLL_CW: _tcscpy_s(buf, buflen, _T("V3DK_ROLL_CW")); break;
	case s3dm::V3DK_ROLL_CCW: _tcscpy_s(buf, buflen, _T("V3DK_ROLL_CCW")); break;
	case s3dm::V3DK_ISO1: _tcscpy_s(buf, buflen, _T("V3DK_ISO1")); break;
	case s3dm::V3DK_ISO2: _tcscpy_s(buf, buflen, _T("V3DK_ISO2")); break;
	case s3dm::V3DK_1: _tcscpy_s(buf, buflen, _T("V3DK_1")); break;
	case s3dm::V3DK_2: _tcscpy_s(buf, buflen, _T("V3DK_2")); break;
	case s3dm::V3DK_3: _tcscpy_s(buf, buflen, _T("V3DK_3")); break;
	case s3dm::V3DK_4: _tcscpy_s(buf, buflen, _T("V3DK_4")); break;
	case s3dm::V3DK_5: _tcscpy_s(buf, buflen, _T("V3DK_5")); break;
	case s3dm::V3DK_6: _tcscpy_s(buf, buflen, _T("V3DK_6")); break;
	case s3dm::V3DK_7: _tcscpy_s(buf, buflen, _T("V3DK_7")); break;
	case s3dm::V3DK_8: _tcscpy_s(buf, buflen, _T("V3DK_8")); break;
	case s3dm::V3DK_9: _tcscpy_s(buf, buflen, _T("V3DK_9")); break;
	case s3dm::V3DK_10: _tcscpy_s(buf, buflen, _T("V3DK_10")); break;
	case s3dm::V3DK_ESC: _tcscpy_s(buf, buflen, _T("V3DK_ESC")); break;
	case s3dm::V3DK_ALT: _tcscpy_s(buf, buflen, _T("V3DK_ALT")); break;
	case s3dm::V3DK_SHIFT: _tcscpy_s(buf, buflen, _T("V3DK_SHIFT")); break;
	case s3dm::V3DK_CTRL: _tcscpy_s(buf, buflen, _T("V3DK_CTRL")); break;
	case s3dm::V3DK_ROTATE: _tcscpy_s(buf, buflen, _T("V3DK_ROTATE")); break;
	case s3dm::V3DK_PANZOOM: _tcscpy_s(buf, buflen, _T("V3DK_PANZOOM")); break;
	case s3dm::V3DK_DOMINANT: _tcscpy_s(buf, buflen, _T("V3DK_DOMINANT")); break;
	case s3dm::V3DK_PLUS: _tcscpy_s(buf, buflen, _T("V3DK_PLUS")); break;
	case s3dm::V3DK_MINUS: _tcscpy_s(buf, buflen, _T("V3DK_MINUS")); break;
	case s3dm::V3DK_SPIN_CW: _tcscpy_s(buf, buflen, _T("V3DK_SPIN_CW")); break;
	case s3dm::V3DK_SPIN_CCW: _tcscpy_s(buf, buflen, _T("V3DK_SPIN_CCW")); break;
	case s3dm::V3DK_TILT_CW: _tcscpy_s(buf, buflen, _T("V3DK_TILT_CW")); break;
	case s3dm::V3DK_TILT_CCW: _tcscpy_s(buf, buflen, _T("V3DK_TILT_CCW")); break;
	case V3DK_ENTER: _tcscpy_s(buf, buflen, _T("V3DK_ENTER")); break;
	case V3DK_DELETE: _tcscpy_s(buf, buflen, _T("V3DK_DELETE")); break;
	case V3DK_RESERVED0: _tcscpy_s(buf, buflen, _T("V3DK_RESERVED0")); break;
	case V3DK_RESERVED1: _tcscpy_s(buf, buflen, _T("V3DK_RESERVED1")); break;
	case V3DK_RESERVED2: _tcscpy_s(buf, buflen, _T("V3DK_RESERVED2")); break;
	case V3DK_F1: _tcscpy_s(buf, buflen, _T("V3DK_F1")); break;
	case V3DK_F2: _tcscpy_s(buf, buflen, _T("V3DK_F2")); break;
	case V3DK_F3: _tcscpy_s(buf, buflen, _T("V3DK_F3")); break;
	case V3DK_F4: _tcscpy_s(buf, buflen, _T("V3DK_F4")); break;
	case V3DK_F5: _tcscpy_s(buf, buflen, _T("V3DK_F5")); break;
	case V3DK_F6: _tcscpy_s(buf, buflen, _T("V3DK_F6")); break;
	case V3DK_F7: _tcscpy_s(buf, buflen, _T("V3DK_F7")); break;
	case V3DK_F8: _tcscpy_s(buf, buflen, _T("V3DK_F8")); break;
	case V3DK_F9: _tcscpy_s(buf, buflen, _T("V3DK_F9")); break;
	case V3DK_F10: _tcscpy_s(buf, buflen, _T("V3DK_F10")); break;
	case V3DK_F11: _tcscpy_s(buf, buflen, _T("V3DK_F11")); break;
	case V3DK_F12: _tcscpy_s(buf, buflen, _T("V3DK_F12")); break;
	case V3DK_F13: _tcscpy_s(buf, buflen, _T("V3DK_F13")); break;
	case V3DK_F14: _tcscpy_s(buf, buflen, _T("V3DK_F14")); break;
	case V3DK_F15: _tcscpy_s(buf, buflen, _T("V3DK_F15")); break;
	case V3DK_F16: _tcscpy_s(buf, buflen, _T("V3DK_F16")); break;
	case V3DK_F17: _tcscpy_s(buf, buflen, _T("V3DK_F17")); break;
	case V3DK_F18: _tcscpy_s(buf, buflen, _T("V3DK_F18")); break;
	case V3DK_F19: _tcscpy_s(buf, buflen, _T("V3DK_F19")); break;
	case V3DK_F20: _tcscpy_s(buf, buflen, _T("V3DK_F20")); break;
	case V3DK_F21: _tcscpy_s(buf, buflen, _T("V3DK_F21")); break;
	case V3DK_F22: _tcscpy_s(buf, buflen, _T("V3DK_F22")); break;
	case V3DK_F23: _tcscpy_s(buf, buflen, _T("V3DK_F23")); break;
	case V3DK_F24: _tcscpy_s(buf, buflen, _T("V3DK_F24")); break;
	case V3DK_F25: _tcscpy_s(buf, buflen, _T("V3DK_F25")); break;
	case V3DK_F26: _tcscpy_s(buf, buflen, _T("V3DK_F26")); break;
	case V3DK_F27: _tcscpy_s(buf, buflen, _T("V3DK_F27")); break;
	case V3DK_F28: _tcscpy_s(buf, buflen, _T("V3DK_F28")); break;
	case V3DK_F29: _tcscpy_s(buf, buflen, _T("V3DK_F29")); break;
	case V3DK_F30: _tcscpy_s(buf, buflen, _T("V3DK_F30")); break;
	case V3DK_F31: _tcscpy_s(buf, buflen, _T("V3DK_F31")); break;
	case V3DK_F32: _tcscpy_s(buf, buflen, _T("V3DK_F32")); break;
	case V3DK_F33: _tcscpy_s(buf, buflen, _T("V3DK_F33")); break;
	case V3DK_F34: _tcscpy_s(buf, buflen, _T("V3DK_F34")); break;
	case V3DK_F35: _tcscpy_s(buf, buflen, _T("V3DK_F35")); break;
	case V3DK_F36: _tcscpy_s(buf, buflen, _T("V3DK_F36")); break;
	case V3DK_11: _tcscpy_s(buf, buflen, _T("V3DK_11")); break;
	case V3DK_12: _tcscpy_s(buf, buflen, _T("V3DK_12")); break;
	case V3DK_13: _tcscpy_s(buf, buflen, _T("V3DK_13")); break;
	case V3DK_14: _tcscpy_s(buf, buflen, _T("V3DK_14")); break;
	case V3DK_15: _tcscpy_s(buf, buflen, _T("V3DK_15")); break;
	case V3DK_16: _tcscpy_s(buf, buflen, _T("V3DK_16")); break;
	case V3DK_17: _tcscpy_s(buf, buflen, _T("V3DK_17")); break;
	case V3DK_18: _tcscpy_s(buf, buflen, _T("V3DK_18")); break;
	case V3DK_19: _tcscpy_s(buf, buflen, _T("V3DK_19")); break;
	case V3DK_20: _tcscpy_s(buf, buflen, _T("V3DK_20")); break;
	case V3DK_21: _tcscpy_s(buf, buflen, _T("V3DK_21")); break;
	case V3DK_22: _tcscpy_s(buf, buflen, _T("V3DK_22")); break;
	case V3DK_23: _tcscpy_s(buf, buflen, _T("V3DK_23")); break;
	case V3DK_24: _tcscpy_s(buf, buflen, _T("V3DK_24")); break;
	case V3DK_25: _tcscpy_s(buf, buflen, _T("V3DK_25")); break;
	case V3DK_26: _tcscpy_s(buf, buflen, _T("V3DK_26")); break;
	case V3DK_27: _tcscpy_s(buf, buflen, _T("V3DK_27")); break;
	case V3DK_28: _tcscpy_s(buf, buflen, _T("V3DK_28")); break;
	case V3DK_29: _tcscpy_s(buf, buflen, _T("V3DK_29")); break;
	case V3DK_30: _tcscpy_s(buf, buflen, _T("V3DK_30")); break;
	case V3DK_31: _tcscpy_s(buf, buflen, _T("V3DK_31")); break;
	case V3DK_32: _tcscpy_s(buf, buflen, _T("V3DK_32")); break;
	case V3DK_33: _tcscpy_s(buf, buflen, _T("V3DK_33")); break;
	case V3DK_34: _tcscpy_s(buf, buflen, _T("V3DK_34")); break;
	case V3DK_35: _tcscpy_s(buf, buflen, _T("V3DK_35")); break;
	case V3DK_36: _tcscpy_s(buf, buflen, _T("V3DK_36")); break;
	case V3DK_VIEW_1: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_1")); break;
	case V3DK_VIEW_2: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_2")); break;
	case V3DK_VIEW_3: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_3")); break;
	case V3DK_VIEW_4: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_4")); break;
	case V3DK_VIEW_5: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_5")); break;
	case V3DK_VIEW_6: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_6")); break;
	case V3DK_VIEW_7: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_7")); break;
	case V3DK_VIEW_8: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_8")); break;
	case V3DK_VIEW_9: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_9")); break;
	case V3DK_VIEW_10: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_10")); break;
	case V3DK_VIEW_11: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_11")); break;
	case V3DK_VIEW_12: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_12")); break;
	case V3DK_VIEW_13: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_13")); break;
	case V3DK_VIEW_14: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_14")); break;
	case V3DK_VIEW_15: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_15")); break;
	case V3DK_VIEW_16: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_16")); break;
	case V3DK_VIEW_17: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_17")); break;
	case V3DK_VIEW_18: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_18")); break;
	case V3DK_VIEW_19: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_19")); break;
	case V3DK_VIEW_20: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_20")); break;
	case V3DK_VIEW_21: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_21")); break;
	case V3DK_VIEW_22: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_22")); break;
	case V3DK_VIEW_23: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_23")); break;
	case V3DK_VIEW_24: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_24")); break;
	case V3DK_VIEW_25: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_25")); break;
	case V3DK_VIEW_26: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_26")); break;
	case V3DK_VIEW_27: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_27")); break;
	case V3DK_VIEW_28: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_28")); break;
	case V3DK_VIEW_29: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_29")); break;
	case V3DK_VIEW_30: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_30")); break;
	case V3DK_VIEW_31: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_31")); break;
	case V3DK_VIEW_32: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_32")); break;
	case V3DK_VIEW_33: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_33")); break;
	case V3DK_VIEW_34: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_34")); break;
	case V3DK_VIEW_35: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_35")); break;
	case V3DK_VIEW_36: _tcscpy_s(buf, buflen, _T("V3DK_VIEW_36")); break;
	case V3DK_SAVE_VIEW_1: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_1")); break;
	case V3DK_SAVE_VIEW_2: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_2")); break;
	case V3DK_SAVE_VIEW_3: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_3")); break;
	case V3DK_SAVE_VIEW_4: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_4")); break;
	case V3DK_SAVE_VIEW_5: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_5")); break;
	case V3DK_SAVE_VIEW_6: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_6")); break;
	case V3DK_SAVE_VIEW_7: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_7")); break;
	case V3DK_SAVE_VIEW_8: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_8")); break;
	case V3DK_SAVE_VIEW_9: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_9")); break;
	case V3DK_SAVE_VIEW_10: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_10")); break;
	case V3DK_SAVE_VIEW_11: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_11")); break;
	case V3DK_SAVE_VIEW_12: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_12")); break;
	case V3DK_SAVE_VIEW_13: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_13")); break;
	case V3DK_SAVE_VIEW_14: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_14")); break;
	case V3DK_SAVE_VIEW_15: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_15")); break;
	case V3DK_SAVE_VIEW_16: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_16")); break;
	case V3DK_SAVE_VIEW_17: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_17")); break;
	case V3DK_SAVE_VIEW_18: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_18")); break;
	case V3DK_SAVE_VIEW_19: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_19")); break;
	case V3DK_SAVE_VIEW_20: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_20")); break;
	case V3DK_SAVE_VIEW_21: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_21")); break;
	case V3DK_SAVE_VIEW_22: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_22")); break;
	case V3DK_SAVE_VIEW_23: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_23")); break;
	case V3DK_SAVE_VIEW_24: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_24")); break;
	case V3DK_SAVE_VIEW_25: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_25")); break;
	case V3DK_SAVE_VIEW_26: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_26")); break;
	case V3DK_SAVE_VIEW_27: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_27")); break;
	case V3DK_SAVE_VIEW_28: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_28")); break;
	case V3DK_SAVE_VIEW_29: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_29")); break;
	case V3DK_SAVE_VIEW_30: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_30")); break;
	case V3DK_SAVE_VIEW_31: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_31")); break;
	case V3DK_SAVE_VIEW_32: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_32")); break;
	case V3DK_SAVE_VIEW_33: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_33")); break;
	case V3DK_SAVE_VIEW_34: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_34")); break;
	case V3DK_SAVE_VIEW_35: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_35")); break;
	case V3DK_SAVE_VIEW_36: _tcscpy_s(buf, buflen, _T("V3DK_SAVE_VIEW_36")); break;
	case V3DK_TAB: _tcscpy_s(buf, buflen, _T("V3DK_TAB	")); break;
	case V3DK_SPACE: _tcscpy_s(buf, buflen, _T("V3DK_SPACE")); break;
	case V3DK_MENU_1: _tcscpy_s(buf, buflen, _T("V3DK_MENU_1")); break;
	case V3DK_MENU_2: _tcscpy_s(buf, buflen, _T("V3DK_MENU_2")); break;
	case V3DK_MENU_3: _tcscpy_s(buf, buflen, _T("V3DK_MENU_3")); break;
	case V3DK_MENU_4: _tcscpy_s(buf, buflen, _T("V3DK_MENU_4")); break;
	case V3DK_MENU_5: _tcscpy_s(buf, buflen, _T("V3DK_MENU_5")); break;
	case V3DK_MENU_6: _tcscpy_s(buf, buflen, _T("V3DK_MENU_6")); break;
	case V3DK_MENU_7: _tcscpy_s(buf, buflen, _T("V3DK_MENU_7")); break;
	case V3DK_MENU_8: _tcscpy_s(buf, buflen, _T("V3DK_MENU_8")); break;
	case V3DK_MENU_9: _tcscpy_s(buf, buflen, _T("V3DK_MENU_9")); break;
	case V3DK_MENU_10: _tcscpy_s(buf, buflen, _T("V3DK_MENU_10")); break;
	case V3DK_MENU_11: _tcscpy_s(buf, buflen, _T("V3DK_MENU_11")); break;
	case V3DK_MENU_12: _tcscpy_s(buf, buflen, _T("V3DK_MENU_12")); break;
	case V3DK_MENU_13: _tcscpy_s(buf, buflen, _T("V3DK_MENU_13")); break;
	case V3DK_MENU_14: _tcscpy_s(buf, buflen, _T("V3DK_MENU_14")); break;
	case V3DK_MENU_15: _tcscpy_s(buf, buflen, _T("V3DK_MENU_15")); break;
	case V3DK_MENU_16: _tcscpy_s(buf, buflen, _T("V3DK_MENU_16")); break;
	case V3DK_USER: _tcscpy_s(buf, buflen, _T("V3DK_USER")); break;

	default:
		// Put unrecognized buttons in parens
		_stprintf_s(buf, buflen, _T("(unrecognized V3DKey %d)"), v3dk);
		break;
	}

	return buf;
}
/*----------------------------------------------------------------------
* Function: SbButtonPressEvent()
*
* Description:
*    This function receives 3D mouse button information and prints out the
*    info on screen.
*
*
* Args:
*    int     buttonnumber   //Containts number of button pressed
*
* Return Value:
*    NONE
*
*----------------------------------------------------------------------*/
void SbButtonPressEvent(int buttonnumber)
{

	TCHAR bn[100];
	int mask = 0x0;
	/* get handle of our window to draw on */
	hdc = GetDC(hWndMain);

	SiButtonName name;
	SiGetButtonName(devHdl, buttonnumber, &name);

	if (buttonnumber == 6) {
		mask = 0x1;
		spaceButtons[0] = true;
		tempButton = true;

	}
	if (buttonnumber == 8) {
		mask = 0x2;
		spaceButtons[1] = true;
		tempButton2 = true;

	}
	spaceMouse[6] ^= mask;
	spaceButtonsToggle[2] = spaceButtons[2];
	if (spaceButtons[0] && spaceButtons[1])
	{
		spaceButtons[2] = !spaceButtons[2];
		spaceButtonsToggle[0] = false;
		spaceButtonsToggle[1] = false;
		//spaceMouseMode = 0;

	}


	V3DKeyToName(bn, SPW_NUM_ELEMENTS_IN(bn), (V3DKey)buttonnumber);

	// (provide a switch statement for easy cut & paste)
	switch (buttonnumber)
	{
	case s3dm::V3DK_MENU: break;
	case s3dm::V3DK_FIT: break;
	case s3dm::V3DK_TOP: break;
	case s3dm::V3DK_LEFT: break;
	case s3dm::V3DK_RIGHT: break;
	case s3dm::V3DK_FRONT: break;
	case s3dm::V3DK_BOTTOM: break;
	case s3dm::V3DK_BACK: break;
	case s3dm::V3DK_ROLL_CW: break;
	case s3dm::V3DK_ROLL_CCW: break;
	case s3dm::V3DK_ISO1: break;
	case s3dm::V3DK_ISO2: break;
	case s3dm::V3DK_1: break;
	case s3dm::V3DK_2: break;
	case s3dm::V3DK_3: break;
	case s3dm::V3DK_4: break;
	case s3dm::V3DK_5: break;
	case s3dm::V3DK_6: break;
	case s3dm::V3DK_7: break;
	case s3dm::V3DK_8: break;
	case s3dm::V3DK_9: break;
	case s3dm::V3DK_10: break;
	case s3dm::V3DK_ESC: break;
	case s3dm::V3DK_ALT: break;
	case s3dm::V3DK_SHIFT: break;
	case s3dm::V3DK_CTRL: break;
	case s3dm::V3DK_ROTATE: break;
	case s3dm::V3DK_PANZOOM: break;
	case s3dm::V3DK_DOMINANT: break;
	case s3dm::V3DK_PLUS: break;
	case s3dm::V3DK_MINUS: break;
	case s3dm::V3DK_SPIN_CW: break;
	case s3dm::V3DK_SPIN_CCW: break;
	case s3dm::V3DK_TILT_CW: break;
	case s3dm::V3DK_TILT_CCW: break;
	case V3DK_ENTER: break;
	case V3DK_DELETE: break;
	case V3DK_RESERVED0: break;
	case V3DK_RESERVED1: break;
	case V3DK_RESERVED2: break;
	case V3DK_F1: break;
	case V3DK_F2: break;
	case V3DK_F3: break;
	case V3DK_F4: break;
	case V3DK_F5: break;
	case V3DK_F6: break;
	case V3DK_F7: break;
	case V3DK_F8: break;
	case V3DK_F9: break;
	case V3DK_F10: break;
	case V3DK_F11: break;
	case V3DK_F12: break;
	case V3DK_F13: break;
	case V3DK_F14: break;
	case V3DK_F15: break;
	case V3DK_F16: break;
	case V3DK_F17: break;
	case V3DK_F18: break;
	case V3DK_F19: break;
	case V3DK_F20: break;
	case V3DK_F21: break;
	case V3DK_F22: break;
	case V3DK_F23: break;
	case V3DK_F24: break;
	case V3DK_F25: break;
	case V3DK_F26: break;
	case V3DK_F27: break;
	case V3DK_F28: break;
	case V3DK_F29: break;
	case V3DK_F30: break;
	case V3DK_F31: break;
	case V3DK_F32: break;
	case V3DK_F33: break;
	case V3DK_F34: break;
	case V3DK_F35: break;
	case V3DK_F36: break;
	case V3DK_11: break;
	case V3DK_12: break;
	case V3DK_13: break;
	case V3DK_14: break;
	case V3DK_15: break;
	case V3DK_16: break;
	case V3DK_17: break;
	case V3DK_18: break;
	case V3DK_19: break;
	case V3DK_20: break;
	case V3DK_21: break;
	case V3DK_22: break;
	case V3DK_23: break;
	case V3DK_24: break;
	case V3DK_25: break;
	case V3DK_26: break;
	case V3DK_27: break;
	case V3DK_28: break;
	case V3DK_29: break;
	case V3DK_30: break;
	case V3DK_31: break;
	case V3DK_32: break;
	case V3DK_33: break;
	case V3DK_34: break;
	case V3DK_35: break;
	case V3DK_36: break;
	case V3DK_VIEW_1: break;
	case V3DK_VIEW_2: break;
	case V3DK_VIEW_3: break;
	case V3DK_VIEW_4: break;
	case V3DK_VIEW_5: break;
	case V3DK_VIEW_6: break;
	case V3DK_VIEW_7: break;
	case V3DK_VIEW_8: break;
	case V3DK_VIEW_9: break;
	case V3DK_VIEW_10: break;
	case V3DK_VIEW_11: break;
	case V3DK_VIEW_12: break;
	case V3DK_VIEW_13: break;
	case V3DK_VIEW_14: break;
	case V3DK_VIEW_15: break;
	case V3DK_VIEW_16: break;
	case V3DK_VIEW_17: break;
	case V3DK_VIEW_18: break;
	case V3DK_VIEW_19: break;
	case V3DK_VIEW_20: break;
	case V3DK_VIEW_21: break;
	case V3DK_VIEW_22: break;
	case V3DK_VIEW_23: break;
	case V3DK_VIEW_24: break;
	case V3DK_VIEW_25: break;
	case V3DK_VIEW_26: break;
	case V3DK_VIEW_27: break;
	case V3DK_VIEW_28: break;
	case V3DK_VIEW_29: break;
	case V3DK_VIEW_30: break;
	case V3DK_VIEW_31: break;
	case V3DK_VIEW_32: break;
	case V3DK_VIEW_33: break;
	case V3DK_VIEW_34: break;
	case V3DK_VIEW_35: break;
	case V3DK_VIEW_36: break;
	case V3DK_SAVE_VIEW_1: break;
	case V3DK_SAVE_VIEW_2: break;
	case V3DK_SAVE_VIEW_3: break;
	case V3DK_SAVE_VIEW_4: break;
	case V3DK_SAVE_VIEW_5: break;
	case V3DK_SAVE_VIEW_6: break;
	case V3DK_SAVE_VIEW_7: break;
	case V3DK_SAVE_VIEW_8: break;
	case V3DK_SAVE_VIEW_9: break;
	case V3DK_SAVE_VIEW_10: break;
	case V3DK_SAVE_VIEW_11: break;
	case V3DK_SAVE_VIEW_12: break;
	case V3DK_SAVE_VIEW_13: break;
	case V3DK_SAVE_VIEW_14: break;
	case V3DK_SAVE_VIEW_15: break;
	case V3DK_SAVE_VIEW_16: break;
	case V3DK_SAVE_VIEW_17: break;
	case V3DK_SAVE_VIEW_18: break;
	case V3DK_SAVE_VIEW_19: break;
	case V3DK_SAVE_VIEW_20: break;
	case V3DK_SAVE_VIEW_21: break;
	case V3DK_SAVE_VIEW_22: break;
	case V3DK_SAVE_VIEW_23: break;
	case V3DK_SAVE_VIEW_24: break;
	case V3DK_SAVE_VIEW_25: break;
	case V3DK_SAVE_VIEW_26: break;
	case V3DK_SAVE_VIEW_27: break;
	case V3DK_SAVE_VIEW_28: break;
	case V3DK_SAVE_VIEW_29: break;
	case V3DK_SAVE_VIEW_30: break;
	case V3DK_SAVE_VIEW_31: break;
	case V3DK_SAVE_VIEW_32: break;
	case V3DK_SAVE_VIEW_33: break;
	case V3DK_SAVE_VIEW_34: break;
	case V3DK_SAVE_VIEW_35: break;
	case V3DK_SAVE_VIEW_36: break;
	case V3DK_TAB: break;
	case V3DK_SPACE: break;
	case V3DK_MENU_1: break;
	case V3DK_MENU_2: break;
	case V3DK_MENU_3: break;
	case V3DK_MENU_4: break;
	case V3DK_MENU_5: break;
	case V3DK_MENU_6: break;
	case V3DK_MENU_7: break;
	case V3DK_MENU_8: break;
	case V3DK_MENU_9: break;
	case V3DK_MENU_10: break;
	case V3DK_MENU_11: break;
	case V3DK_MENU_12: break;
	case V3DK_MENU_13: break;
	case V3DK_MENU_14: break;
	case V3DK_MENU_15: break;
	case V3DK_MENU_16: break;
	case V3DK_USER: break;

	default: break;
	}

	TCHAR fullStr[100];
	//_tcscpy_s(fullStr, SPW_NUM_ELEMENTS_IN(fullStr), _T("-------------------------------------------------------"));
	TextOut(hdc, 0, 0, fullStr, (int)_tcslen(fullStr));

	_tprintf_s(fullStr, SPW_NUM_ELEMENTS_IN(fullStr), "%s Pressed", bn);
	TextOut(hdc, 0, 0, fullStr, (int)_tcslen(fullStr));

	// Output to stdout for tracing
	printf("%S\n", fullStr);
	OutputDebugString(fullStr);
	//gotoxy( 1, 22);
	//printf("[sm] %5d %5d %5d %5d %5d %5d %5d [enabled]%d [mode]%d \n",spaceMouse[0],spaceMouse[1],spaceMouse[2],spaceMouse[3],spaceMouse[4],spaceMouse[5],spaceMouse[6],spaceMouseEnabled,spaceMouseMode);
	/*release our window handle */
	ReleaseDC(hWndMain, hdc);
}
/*----------------------------------------------------------------------
* Function: SbButtonReleaseEvent()
*
* Description:
*    This function receives 3D mouse button information and prints out the
*    info on screen.
*
*
* Args:
*    int     buttonnumber   //Containts number of button pressed
*
* Return Value:
*    NONE
*
*----------------------------------------------------------------------*/
void SbButtonReleaseEvent(int buttonnumber)
{
	TCHAR bn[100];
	int mask = 0x0;
	if (buttonnumber == 6) {

		spaceButtons[0] = false;
		if (spaceButtons[1] == false) tempButton = false;
		if (tempButton == false && tempButton2 == false)
		{
			spaceButtonsToggle[0] = !spaceButtonsToggle[0];
			spaceButtonsToggle[2] = true;
		}

		mask = 0x1;
		tempButton2 = false;
	}
	if (buttonnumber == 8) {

		spaceButtons[1] = false;
		if (spaceButtons[0] == false) tempButton2 = false;
		if (tempButton == false && tempButton2 == false)spaceButtonsToggle[1] = !spaceButtonsToggle[1];
		mask = 0x2;

		tempButton = false;
	}
	/*spaceMouse[6] ^=mask;*/
	/* get handle of our window to draw on */
	hdc = GetDC(hWndMain);

	//if(buttonnumber == 6 && spaceMouse[6] == 1)
	//	spaceMouse[6] = 0;
	//else if(spaceMouse[6]==3)
	//	spaceMouse[6] = 2;
	//if(buttonnumber == 8 && spaceMouse[6] == 2)
	//	spaceMouse[6] = 0;
	//else if(spaceMouse[6] == 3)
	//	spaceMouse[6] = 1;

	//gotoxy( 1, 22);
	//printf("[sm] %5d %5d %5d %5d %5d %5d %5d \n",spaceMouse[0],spaceMouse[1],spaceMouse[2],spaceMouse[3],spaceMouse[4],spaceMouse[5],spaceMouse[6]);
	V3DKeyToName(bn, SPW_NUM_ELEMENTS_IN(bn), (V3DKey)buttonnumber);

	// (provide a switch statement for easy cut & paste)
	switch (buttonnumber)
	{
	case s3dm::V3DK_MENU: break;
	case s3dm::V3DK_FIT: break;
	case s3dm::V3DK_TOP: break;
	case s3dm::V3DK_LEFT: break;
	case s3dm::V3DK_RIGHT: break;
	case s3dm::V3DK_FRONT: break;
	case s3dm::V3DK_BOTTOM: break;
	case s3dm::V3DK_BACK: break;
	case s3dm::V3DK_ROLL_CW: break;
	case s3dm::V3DK_ROLL_CCW: break;
	case s3dm::V3DK_ISO1: break;
	case s3dm::V3DK_ISO2: break;
	case s3dm::V3DK_1: break;
	case s3dm::V3DK_2: break;
	case s3dm::V3DK_3: break;
	case s3dm::V3DK_4: break;
	case s3dm::V3DK_5: break;
	case s3dm::V3DK_6: break;
	case s3dm::V3DK_7: break;
	case s3dm::V3DK_8: break;
	case s3dm::V3DK_9: break;
	case s3dm::V3DK_10: break;
	case s3dm::V3DK_ESC: break;
	case s3dm::V3DK_ALT: break;
	case s3dm::V3DK_SHIFT: break;
	case s3dm::V3DK_CTRL: break;
	case s3dm::V3DK_ROTATE: break;
	case s3dm::V3DK_PANZOOM: break;
	case s3dm::V3DK_DOMINANT: break;
	case s3dm::V3DK_PLUS: break;
	case s3dm::V3DK_MINUS: break;
	case s3dm::V3DK_SPIN_CW: break;
	case s3dm::V3DK_SPIN_CCW: break;
	case s3dm::V3DK_TILT_CW: break;
	case s3dm::V3DK_TILT_CCW: break;
	case V3DK_ENTER: break;
	case V3DK_DELETE: break;
	case V3DK_RESERVED0: break;
	case V3DK_RESERVED1: break;
	case V3DK_RESERVED2: break;
	case V3DK_F1: break;
	case V3DK_F2: break;
	case V3DK_F3: break;
	case V3DK_F4: break;
	case V3DK_F5: break;
	case V3DK_F6: break;
	case V3DK_F7: break;
	case V3DK_F8: break;
	case V3DK_F9: break;
	case V3DK_F10: break;
	case V3DK_F11: break;
	case V3DK_F12: break;
	case V3DK_F13: break;
	case V3DK_F14: break;
	case V3DK_F15: break;
	case V3DK_F16: break;
	case V3DK_F17: break;
	case V3DK_F18: break;
	case V3DK_F19: break;
	case V3DK_F20: break;
	case V3DK_F21: break;
	case V3DK_F22: break;
	case V3DK_F23: break;
	case V3DK_F24: break;
	case V3DK_F25: break;
	case V3DK_F26: break;
	case V3DK_F27: break;
	case V3DK_F28: break;
	case V3DK_F29: break;
	case V3DK_F30: break;
	case V3DK_F31: break;
	case V3DK_F32: break;
	case V3DK_F33: break;
	case V3DK_F34: break;
	case V3DK_F35: break;
	case V3DK_F36: break;
	case V3DK_11: break;
	case V3DK_12: break;
	case V3DK_13: break;
	case V3DK_14: break;
	case V3DK_15: break;
	case V3DK_16: break;
	case V3DK_17: break;
	case V3DK_18: break;
	case V3DK_19: break;
	case V3DK_20: break;
	case V3DK_21: break;
	case V3DK_22: break;
	case V3DK_23: break;
	case V3DK_24: break;
	case V3DK_25: break;
	case V3DK_26: break;
	case V3DK_27: break;
	case V3DK_28: break;
	case V3DK_29: break;
	case V3DK_30: break;
	case V3DK_31: break;
	case V3DK_32: break;
	case V3DK_33: break;
	case V3DK_34: break;
	case V3DK_35: break;
	case V3DK_36: break;
	case V3DK_VIEW_1: break;
	case V3DK_VIEW_2: break;
	case V3DK_VIEW_3: break;
	case V3DK_VIEW_4: break;
	case V3DK_VIEW_5: break;
	case V3DK_VIEW_6: break;
	case V3DK_VIEW_7: break;
	case V3DK_VIEW_8: break;
	case V3DK_VIEW_9: break;
	case V3DK_VIEW_10: break;
	case V3DK_VIEW_11: break;
	case V3DK_VIEW_12: break;
	case V3DK_VIEW_13: break;
	case V3DK_VIEW_14: break;
	case V3DK_VIEW_15: break;
	case V3DK_VIEW_16: break;
	case V3DK_VIEW_17: break;
	case V3DK_VIEW_18: break;
	case V3DK_VIEW_19: break;
	case V3DK_VIEW_20: break;
	case V3DK_VIEW_21: break;
	case V3DK_VIEW_22: break;
	case V3DK_VIEW_23: break;
	case V3DK_VIEW_24: break;
	case V3DK_VIEW_25: break;
	case V3DK_VIEW_26: break;
	case V3DK_VIEW_27: break;
	case V3DK_VIEW_28: break;
	case V3DK_VIEW_29: break;
	case V3DK_VIEW_30: break;
	case V3DK_VIEW_31: break;
	case V3DK_VIEW_32: break;
	case V3DK_VIEW_33: break;
	case V3DK_VIEW_34: break;
	case V3DK_VIEW_35: break;
	case V3DK_VIEW_36: break;
	case V3DK_SAVE_VIEW_1: break;
	case V3DK_SAVE_VIEW_2: break;
	case V3DK_SAVE_VIEW_3: break;
	case V3DK_SAVE_VIEW_4: break;
	case V3DK_SAVE_VIEW_5: break;
	case V3DK_SAVE_VIEW_6: break;
	case V3DK_SAVE_VIEW_7: break;
	case V3DK_SAVE_VIEW_8: break;
	case V3DK_SAVE_VIEW_9: break;
	case V3DK_SAVE_VIEW_10: break;
	case V3DK_SAVE_VIEW_11: break;
	case V3DK_SAVE_VIEW_12: break;
	case V3DK_SAVE_VIEW_13: break;
	case V3DK_SAVE_VIEW_14: break;
	case V3DK_SAVE_VIEW_15: break;
	case V3DK_SAVE_VIEW_16: break;
	case V3DK_SAVE_VIEW_17: break;
	case V3DK_SAVE_VIEW_18: break;
	case V3DK_SAVE_VIEW_19: break;
	case V3DK_SAVE_VIEW_20: break;
	case V3DK_SAVE_VIEW_21: break;
	case V3DK_SAVE_VIEW_22: break;
	case V3DK_SAVE_VIEW_23: break;
	case V3DK_SAVE_VIEW_24: break;
	case V3DK_SAVE_VIEW_25: break;
	case V3DK_SAVE_VIEW_26: break;
	case V3DK_SAVE_VIEW_27: break;
	case V3DK_SAVE_VIEW_28: break;
	case V3DK_SAVE_VIEW_29: break;
	case V3DK_SAVE_VIEW_30: break;
	case V3DK_SAVE_VIEW_31: break;
	case V3DK_SAVE_VIEW_32: break;
	case V3DK_SAVE_VIEW_33: break;
	case V3DK_SAVE_VIEW_34: break;
	case V3DK_SAVE_VIEW_35: break;
	case V3DK_SAVE_VIEW_36: break;
	case V3DK_TAB: break;
	case V3DK_SPACE: break;
	case V3DK_MENU_1: break;
	case V3DK_MENU_2: break;
	case V3DK_MENU_3: break;
	case V3DK_MENU_4: break;
	case V3DK_MENU_5: break;
	case V3DK_MENU_6: break;
	case V3DK_MENU_7: break;
	case V3DK_MENU_8: break;
	case V3DK_MENU_9: break;
	case V3DK_MENU_10: break;
	case V3DK_MENU_11: break;
	case V3DK_MENU_12: break;
	case V3DK_MENU_13: break;
	case V3DK_MENU_14: break;
	case V3DK_MENU_15: break;
	case V3DK_MENU_16: break;
	case V3DK_USER: break;

	default: break;
	}

	TCHAR fullStr[100];
	////_tcscpy_s(fullStr, SPW_NUM_ELEMENTS_IN(fullStr), _T("--------------------------------------------------------"));
	TextOut(hdc, 0, 0, fullStr, (int)_tcslen(fullStr));

	_tprintf_s(fullStr, SPW_NUM_ELEMENTS_IN(fullStr), "%s Released", bn);
	TextOut(hdc, 0, 0, fullStr, (int)_tcslen(fullStr));

	// Output to stdout for tracing
	printf("%S\n", fullStr);
	OutputDebugString(fullStr);

	/*release our window handle */
	ReleaseDC(hWndMain, hdc);
}
void HandleDeviceChangeEvent(SiSpwEvent *pEvent)
{
	TCHAR buf[100];
	hdc = GetDC(hWndMain);

	switch (pEvent->u.deviceChangeEventData.type)
	{
	case SI_DEVICE_CHANGE_CONNECT:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("Device ID %d connected"), pEvent->u.deviceChangeEventData.devID);
		TextOut(hdc, 0, 20, buf, (int)_tcslen(buf));
		break;
	case SI_DEVICE_CHANGE_DISCONNECT:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("Device ID %d disconnected"), pEvent->u.deviceChangeEventData.devID);
		TextOut(hdc, 0, 20, buf, (int)_tcslen(buf));
		break;
	default:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("Unknown deviceChangeEvent type: %d"), pEvent->u.deviceChangeEventData.type);
		TextOut(hdc, 0, 20, buf, (int)_tcslen(buf));
		break;
	}
	ReleaseDC(hWndMain, hdc);
}
void HandleV3DCMDEvent(SiSpwEvent *pEvent)
{
	TCHAR buf[100];
	hdc = GetDC(hWndMain);
	switch (pEvent->u.cmdEventData.functionNumber)
	{
	case V3DCMD_MENU_OPTIONS:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_MENU_OPTIONS(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_FIT:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_FIT(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F1:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F1(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F2:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F2(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F3:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F3(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F4:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F4(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F5:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F5(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F6:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F6(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F7:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F7(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F8:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F8(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F9:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F9(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F10:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F10(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F11:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F11(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_KEY_F12:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_KEY_F12(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_TOP:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_TOP(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_LEFT:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_LEFT(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_RIGHT:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_RIGHT(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_FRONT:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_FRONT(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_BOTTOM:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_BOTTOM(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_BACK:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_BACK(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_ROLLCW:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_ROLLCW(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_ROLLCCW:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_ROLLCCW(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_ISO1:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_ISO1(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_ISO2:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_ISO2(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_1:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_1(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_2:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_2(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_3:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_3(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_SAVE_VIEW_1:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_SAVE_VIEW_1(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_SAVE_VIEW_2:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_SAVE_VIEW_2(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_SAVE_VIEW_3:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_SAVE_VIEW_3(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_QZ_IN:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_QZ_IN(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_VIEW_QZ_OUT:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_VIEW_QZ_OUT(%s)"), pEvent->u.cmdEventData.pressed ? L"Pressed" : L"Released");
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_MOTIONMACRO_ZOOM:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_MOTIONMACRO_ZOOM(ZoomFrom=%d, ZoomTo=%d, Scale=%f)"), pEvent->u.cmdEventData.iArgs[0], pEvent->u.cmdEventData.iArgs[1], pEvent->u.cmdEventData.fArgs[0]);
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_MOTIONMACRO_ZOOMIN_CENTERTOCENTER:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_MOTIONMACRO_ZOOMIN_CENTERTOCENTER(Scale=%f)"), pEvent->u.cmdEventData.fArgs[0]);
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_MOTIONMACRO_ZOOMIN_CURSORTOCENTER:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_MOTIONMACRO_ZOOMIN_CURSORTOCENTER(Scale=%f)"), pEvent->u.cmdEventData.fArgs[0]);
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_MOTIONMACRO_ZOOMIN_CURSORTOCURSOR:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_MOTIONMACRO_ZOOMIN_CURSORTOCURSOR(Scale=%f)"), pEvent->u.cmdEventData.fArgs[0]);
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_MOTIONMACRO_ZOOMOUT_CENTERTOCENTER:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_MOTIONMACRO_ZOOMOUT_CENTERTOCENTER(Scale=%f)"), pEvent->u.cmdEventData.fArgs[0]);
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_MOTIONMACRO_ZOOMOUT_CURSORTOCENTER:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_MOTIONMACRO_ZOOMOUT_CURSORTOCENTER(Scale=%f)"), pEvent->u.cmdEventData.fArgs[0]);
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	case V3DCMD_MOTIONMACRO_ZOOMOUT_CURSORTOCURSOR:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("V3DCMD_MOTIONMACRO_ZOOMOUT_CURSORTOCURSOR(Scale=%f)"), pEvent->u.cmdEventData.fArgs[0]);
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	default:
		_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("Unhandled V3DCMD: number = %d"), pEvent->u.cmdEventData.functionNumber);
		TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
		break;
	}
	ReleaseDC(hWndMain, hdc);
}
void HandleAppEvent(SiSpwEvent *pEvent)
{
	TCHAR buf[100];
	hdc = GetDC(hWndMain);
	_stprintf_s(buf, SPW_NUM_ELEMENTS_IN(buf), _T("AppCmd: %S %s"), pEvent->u.appCommandData.id.appCmdID, pEvent->u.appCommandData.pressed ? _T(" pressed") : _T(" released"));
	TextOut(hdc, 0, 0, buf, (int)_tcslen(buf));
}

void printPos(float* pos, int locx, int locy)
{
	gotoxy(locx, locy);
	for (int i = 0; i < 7; i++) {
		printf("%0.2f   ", pos[i]);
	}
}