#include "Global.h"

using namespace std;
using namespace CxUtils;

// Mutex
HANDLE mutex;

// MANUS states.
float pos[8] = {0};
float raw_pos[8] = { 0 };
float Apos[7];
float speed[8] = {0};
int spaceMouse[7] = {0};
bool spaceMouseEnabled = false;
bool spaceMouseEnabled_old = false;
bool spaceMouse_stop = false;
int spaceMouseMode = 0;
int spaceMouseMode_count = 5;
int spaceMouseMode_old = 0;
bool spaceButtons[3] = {false};
bool spaceButtonsToggle[3] = {false};
char cbox = CARTESIAN;
unsigned char manus_status = 0, manus_message = 0;
unsigned char actual_cbox = 0;
int start_time = 0;
int end_time = 0;
bool tempButton = false;
bool tempButton2 = false;

// Interlock object.
received myRcv;

// TCP communication.
Thread thread_CAN;
Packet packet_CAN;
TcpClient client_CAN;
unsigned char source, command, destination;

// Shared memory.
MappedMemory robot_pos;
MappedMemory LPS;
MappedMemory force;
MappedMemory slip_vel;
MappedMemory pos_vel;
MappedMemory takktile;
MappedMemory spaceMouseValues;
MappedMemory sug_speed;
MappedMemory block_direction;
MappedMemory block_direction2;
MappedMemory Obj_in;

// Home position of the arm.
int pos_index = -1;

// Command queue.
int pos_queue_index = -1;

// Arm position and states.
ColumnVector<3> vwg, wwg;
float task_goal_pos[3] = {380, 410, 180};
float good_pos[QUE_SIZE][8];
float opos[8];
float rpos[8];

// Speed control.
int linear_speed_limit[5] = {10,30,50,20,127};
int angular_speed_limit[5] = {1,3,5,7,10};
int suggspeed[7];
//int joint_speed_limit[5] = {.5,1.5,.5,.7,1};

int oneSecondStart = 0;

int speed_mode = 2;
int start_time_set_point_vel;
int mode = MANUAL_MODE;
int gripper_closedness = 0;
float speed_factor = 1.0f;

float iSum[6] = {0};
float oldDiff[6] = {0};

float PrevTime[6] = {0};

//Grasping Control
float cur_force = 0;
float e_force = 0;
float raw_velocity;
float init_force = 0;
double cur_velocity = 0;
double cur_velocity_f = 0;//zc
//bool initial_vel = true;//zc
//double old_vel = 0;//zc
//double old_vel2 = 0;//zc
//double old_vel3 = 0;//zc
int old_t = 0;//zc
int cur_t = 0;//zc
int dtt= 0 ;
float a2[2] = {  -1.6041  ,  0.6705};
float b2[3] = {0.0166   , 0.0332  ,  0.0166};//fc = 25
float x2d[2] = { 0,0 };
float y2d[2] = { 0,0 };
float al = 0.03;
double o_p  = 0;
double cur_pos_f = 0;
double cur_pos_nf = 0;
float tol = 0.18;
float new_force = 0;
float old_force = 0;
float stoppos = 0;
float e_pos = 0;
int force_count = 0;
int force_t = 0;

//one click
int oneclick_mode = 0;//used to define the stages of the autonomous movement(suggest motion, one click)
bool assistant_flag = false;
float requestedPosition[11];
float temp_pos[6];
int grasp_side = 0;// 0: front grasp, 1: top grasp
bool btn_flag = false;
char fine_adjust = '0';//flag for defining 1 click status
char old_fine_adjust = '0';
float D2obj ;
bool btm_cls = false;
float cls_pos[6];//collision position for auto grasp
int btn_pressed;
int user_oprt[2] = {0};
bool oprt_start = false;
bool moveto = false;
float set_pos[6] = { 0 };
float orig_pos[6] = { 0 };
bool adjust_pos = false;

bool requestframe = false;
int t_reqframe = 0;
int t_adj = 0;
int move_arm = -1;
int block_movement[14] = { 0 };
//btn suggestion
int suggestedMotion;
int previousSuggestedMotion;
unsigned char suggestedButtonSwitch = 'Z';
unsigned char suggested_btn_order[6] = { 'Z' ,'p','Y','y' ,'x' ,'r' };// suggested motion order, 1.up/down  2.pitch 3.left/right  4.yaw  5. forward/backward  6. roll
bool init_sug;
char btn_cmd;
int sg_stage;
bool update_sug;
float moveL[6];
float currentPosition[6];
Matrix<3, 1> p_frame_w;
bool move_as_suggested[2] = { false };

//optical gate 
int obj_in[2];


// LPS sensor
int LPS_value[2];
bool cam_cls = false;


bool testt = false;
int adjust = 0;
int ini_adt = 0;
int grasp_flag = 0;
int grasp_end = 0;
int grasp_start = 0;
int grasp_inipos = 0 ;//zc
int grasp_npos  = 0;//zc
bool grasp_move  = false;//zc
int grasp_test = 0 ;
int init_stop = 0;
int hold_init = 0;
int stt = 0; 

double old_position = 0;//zc
double old_pos = 0;//zc
double cur_position = 0;
float k =1500 ;//5  zc
float k2 = 6;//5  zc
float gamma1 = 2000; // 0.004, 0.001, 0.005, 0.00085
float gamma2 = 50;//1.5
float w_hat = 2;//0.098
float u_hat = 2;//0.45

float last_w_hat ;//
float last_u_hat ;//

float u_hat_dot = 0;
int cur_time = 0;
int dt0;
int old_time;
float cur_distance;
int Que_count = 0;
bool Que_tick = false;



float force_que[5] = {0};
float vel_que[6] = {0};//zc
float F_d = 0;
float P_d = 0;
float p1 =      -185.9;
float p2 =        1544;
float p3 =       -5421;
float p4 =      -61.91;
float P_int = 0;

//int cur_takk[12] = {0};
//int init_takk[12] = {0};
int touch_pos[10];
int touch_pos_change[11];


//Lowpass Filter
int lpf_time = 0; // 0.002 
float lpf_old = 0; 
float lpf_cur = 0;

int lpf_time2 = 0; // 0.002 
float lpf_old2 = 0; 
float lpf_cur2 = 0;

// Other globals.
DWORD prevID  = 0x0;
float pd[6] = {0,0,0,0,0,0};			//desired home position for template1 on 12/24/07 - KIM	12/31/07
float Kp[6] = {1,1,1,0.8,0.7,0.6};
float eprev1[6];
float oypr[3], cypr[3], dypr[3];
Matrix<3,1> d_c_ee;

// Debug information storage and time variables.
//ofstream timing;
char *debug_path = "C:\\MANUS\\CommonSpace\\run\\";
char user_str[256];
int pctime, potime;
LARGE_INTEGER tickspersecond, tick, tick_time;

// PCAN variables.
TPCANHandle m_Handle = PCAN_PCIBUS1;
TPCANStatus stsResult;
TPCANMsg rcvMsg, xmitMsg;
TPCANTimestamp CANTimeStamp;
PCANBasicClass m_objPCANBasic;

// Bool states.
bool init_fine_motion = false;
bool block_all_motions = false;
bool debug_on = true;
bool robot_in_out_of_range = false;
bool grab_in_progress = false;
bool open_in_progress = false;
bool init_system = true;
bool reverse_flag = false;
bool tts_in_progress = false;
bool set_point_vel = false;
bool store_time_info = false;
bool firstAccess=true;
bool auto_mode_start = false;
bool home_pos_flag = false;
bool test_x_flag = false;
bool job_complete2 = false;
bool job_done = false;
bool approach_flag = false;
bool retreat_flag = false;
bool new_status = false; 
bool job_complete = false;
bool do_roll=false;
bool write_init_pos = false;
bool set_points_control = false;
bool rotation_start1 = false;
bool rotation_start2 = false;
bool grasp_control = false;
bool ready_lift = false;




//Space Mouse
int block_dir[6];
bool block_flag;
int spm_operation;
int spm_gripper;// for active or disable the slip sensor
float thres;
int spacemouse_translation_sensitivity=1500;
int spacemouse_rotation_sensitivity=1500;
int spacemouse_operation[6];
//Space mouse
HDC          hdc;         /* Handle to Device Context used to draw on screen */
HWND         hWndMain;    /* Handle to Main Window */
SiHdl        devHdl;      /* Handle to 3D Mouse Device */
TCHAR devicename[100] = _T("");

#define MAX_LOADSTRING 100
// Global Variables (are (usually) evil; this code comes from the Visual C++ Win32 Application project template):
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);

