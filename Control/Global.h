#ifndef _GLOBAL_WZ_
#define _GLOBAL_WZ_

//=============================================================================
// System Includes
//=============================================================================
#include <iostream>
#include <fstream>

//=============================================================================
// Project Includes
//=============================================================================
#include "PCANBasicClass.h"							// PCAN functions.
#include "RWLock.hpp"								// Interlock object.
#include "Matrix.hpp"								// Matrix definition.
#include "Transform.hpp"							// ColumnVector definition.
#include "cxutils/networking/tcpclient.h"			// TCP client connections.
#include "cxutils/ipc/messageclient.h"				// Shared/mapped memory message client.
#include "Macro.h"



#define NOMINMAX
#include <tchar.h>
#include <windows.h>
#include <cassert>
#include <tchar.h>
#include "spwmacro.h"  /* Common macros used by SpaceWare functions. */
#include "si.h"        /* Required for any SpaceWare support within an app.*/
#include "siapp.h"     /* Required for siapp.lib symbols */
#include "virtualkeys.hpp"
#include "resource.h"
#include <float.h>
#include <stdlib.h>
#include <crtdbg.h>
#include <stdio.h>



// MANUS states.
extern float pos[8];
extern float raw_pos[8];
extern float Apos[7];
extern float speed[8];
extern char cbox;
extern unsigned char manus_status, manus_message;
extern unsigned char actual_cbox;
extern int start_time;
extern int end_time;
extern int spaceMouse[7];
extern bool spaceMouseEnabled;
extern bool spaceMouse_stop ;
extern int spaceMouseMode;
extern bool spaceButtons[3];
extern bool spaceButtonsToggle[3];
extern bool tempButton;
extern bool tempButton2;
extern int block_dir[6];
extern bool block_flag;
extern int spm_operation;
extern bool ready_lift;
extern int spm_gripper;// for active or disable the slip sensor
extern int spacemouse_translation_sensitivity;
extern int spacemouse_rotation_sensitivity;

// Interlock object.
typedef struct 
{
	char command;
	char send;
	RWLock key;
}received;
extern received myRcv;

// Mutex
extern HANDLE mutex;

// TCP communication.
extern CxUtils::Thread thread_CAN;
extern CxUtils::Packet packet_CAN;
extern CxUtils::TcpClient client_CAN;
extern unsigned char source, command, destination;

// Shared memory.
extern CxUtils::MappedMemory	robot_pos;
extern CxUtils::MappedMemory	force;
extern CxUtils::MappedMemory	LPS;
extern CxUtils::MappedMemory	slip_vel;
extern CxUtils::MappedMemory	pos_vel;
extern CxUtils::MappedMemory	takktile;
extern CxUtils::MappedMemory    spaceMouseValues;
extern CxUtils::MappedMemory    sug_speed;
extern CxUtils::MappedMemory	block_direction;
extern CxUtils::MappedMemory	Obj_in;
// Home position of the arm.
extern int pos_index;

// Command queue.
extern int pos_queue_index;

// Arm position and states.
extern ColumnVector<3> vwg, wwg;
extern float task_goal_pos[3];
extern float good_pos[QUE_SIZE][8];
extern float opos[8];
extern float rpos[8];

// Speed control.
extern int linear_speed_limit[5];
extern int angular_speed_limit[5];
extern int suggspeed[7];
extern int oneSecondStart;
//extern int joint_speed_limit[5];
extern int speed_mode;
extern int start_time_set_point_vel;
extern int mode;
extern int gripper_closedness;
extern float speed_factor;

extern float iSum[6];
extern float oldDiff[6];
extern float PrevTime[6];

//Grasping Control
extern float cur_force;
extern float raw_velocity;
extern float init_force;
//extern bool initial_vel;//zc
extern double cur_velocity;
extern double cur_velocity_f;
//extern double cur_vel;// zc
//extern double old_vel;// zc
//extern double old_vel2;// zc
//extern double old_vel3;// zc
extern double cur_position;
extern double old_position;//zc
extern double old_pos;//zc
extern int old_t ;//zc
extern int cur_t ;//zc
extern int dtt ;
extern float a2[2] ;
extern float b2[3] ;
extern float x2d[2] ;
extern float y2d[2] ;
extern float al ;
extern double o_p ;
extern double cur_pos_f;
extern double cur_pos_nf;
extern int hold_init ;
extern int stt;

extern int grasp_flag ;
extern int grasp_end ;
extern int grasp_start ;
extern int grasp_inipos ;//zc
extern int grasp_npos ;//zc
extern bool grasp_move ;//zc
extern int grasp_test ;
extern int init_stop ;
extern float tol ;
extern float new_force ;
extern float old_force ;
extern int force_count ;
extern int force_t;
extern float F_d;
extern float e_force;
extern int adjust;
extern int ini_adt;
extern bool testt;
extern float stoppos;
extern float e_pos;
extern float P_d;
extern float p1;
extern float  p2;
extern float p3;
extern float p4;
extern float P_int;


extern float w_hat;//0.098
extern float u_hat;//0.45
extern float last_w_hat;//
extern float last_u_hat;//



extern float u_hat_dot;
extern int cur_time ;
extern int dt0;
extern int old_time;
extern float cur_distance;
extern float k;
extern float k2;
extern int Que_count;
extern bool Que_tick ;
extern float gamma1;
extern float gamma2;
extern float force_que[5];
extern float vel_que[6];//zc

//extern int cur_takk[12];
//extern int init_takk[12];
extern int touch_pos[10];
extern int touch_pos_change[11];

//one click
extern int oneclick_mode ;
extern bool assistant_flag;
extern float requestedPosition[11];
extern float temp_pos[6];
extern int grasp_side ;
extern bool btn_flag;
extern char fine_adjust;
extern char old_fine_adjust;
extern float D2obj;
extern bool btm_cls ;
extern float cls_pos[6];
extern int btn_pressed;
extern int user_oprt[2];
extern bool moveto;
extern float set_pos[6];
extern float orig_pos[6];
extern bool adjust_pos;
extern bool requestframe;
extern int t_reqframe;
extern int t_adj;
extern int move_arm;

//btn suggestion
extern int suggestedMotion;
extern int previousSuggestedMotion;
extern unsigned char		suggestedButtonSwitch;
extern bool init_sug;
extern char btn_cmd;
extern int sg_stage;
extern bool update_sug;
extern float moveL[6];
extern float currentPosition[6];
extern Matrix<3, 1> p_frame_w;
extern float thres;
// optical gate
extern int obj_in[2];

//LPS sensor
extern int LPS_value[2];

extern bool cam_cls ;

//Lowpass Filter
extern int lpf_time;
extern float lpf_old; 
extern float lpf_cur;

extern int lpf_time2;
extern float lpf_old2; 
extern float lpf_cur2;

// Other globals.
extern DWORD prevID;
extern float pd[6];			//desired home position for template1 on 12/24/07 - KIM	12/31/07
extern float Kp[6];
extern float eprev1[6];
extern float oypr[3], cypr[3], dypr[3];
extern Matrix<3,1> d_c_ee;

// Debug information storage and time variables.
//extern std::ofstream timing;
extern char *debug_path;
extern char user_str[256];
extern int pctime, potime;
extern LARGE_INTEGER tickspersecond, tick, tick_time;

// PCAN variables.
extern TPCANHandle m_Handle;
extern TPCANStatus stsResult;
extern TPCANMsg rcvMsg, xmitMsg;
extern TPCANTimestamp CANTimeStamp;
extern PCANBasicClass m_objPCANBasic;

// Bool states.
extern bool init_fine_motion;
extern bool block_all_motions;
extern bool debug_on;
extern bool robot_in_out_of_range;
extern bool grab_in_progress;
extern bool open_in_progress;
extern bool init_system;
extern bool reverse_flag;
extern bool tts_in_progress;
extern bool set_point_vel;
extern bool store_time_info;
extern bool firstAccess;
extern bool auto_mode_start;
extern bool home_pos_flag;
extern bool test_x_flag;
extern bool job_complete2;
extern bool job_done;
extern bool approach_flag;
extern bool retreat_flag;
extern bool new_status; 
extern bool job_complete; 
extern bool do_roll;
extern bool write_init_pos;
extern bool set_points_control;
extern bool rotation_start1;
extern bool rotation_start2;
extern bool grasp_control;

#define MAX_LOADSTRING 100
//Space mouse
extern HDC          hdc;         /* Handle to Device Context used to draw on screen */
extern HWND         hWndMain;    /* Handle to Main Window */
extern SiHdl        devHdl;      /* Handle to 3D Mouse Device */
extern TCHAR devicename[100];

// Global Variables (are (usually) evil; this code comes from the Visual C++ Win32 Application project template):
extern HINSTANCE hInst;								// current instance
extern TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
extern TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
extern ATOM				MyRegisterClass(HINSTANCE hInstance);
extern BOOL				InitInstance(HINSTANCE, int);
extern LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);

#endif