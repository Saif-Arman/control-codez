// Control.cpp : Defines the entry point for the console application.
//
#if defined(UNICODE) || defined(_UNICODE)
#define tcout std::wcout
#define tcin std::wcin
#else
#define tcout std::cout
#define tcin std::cin
#endif
static char CvsId[]="(C) 1997-2015 3Dconnexion: $Id: 3DxTest32.cpp 13022 2016-05-25 15:43:58Z jwick $";


///////
//=============================================================================
// System includes.
//=============================================================================
#include "stdafx.h"
#include <conio.h>
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include <process.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <cctype>
#include <string.h>
#include <process.h>
#include <math.h>
#include <cmath>
using namespace std;

//=============================================================================
// Project includes.
//=============================================================================
#include "PCANBasicClass.h"							// PCAN functions.
#include "RWLock.hpp"								// Interlock object.
#include "Matrix.hpp"								// Matrix definition.
#include "Transform.hpp"							// ColumnVector definition.
#include "cxutils/networking/tcpclient.h"			// TCP client connections.
#include "cxutils/ipc/messageclient.h"				// Shared/mapped memory message client.
#include "MANUS_define.h"
#include "CSetConsole.h"
#include "ForceTorqueManager.h"							// Class to calibrate MANUS FT Sensor


//=============================================================================
// Namespace.
//=============================================================================
using namespace CxUtils;

//=============================================================================
// Macro Definitions, Global Variables and Sub function.
//=============================================================================
#include "Macro.h"
#include "Global.h"
#include "Functions.h"

//=============================================================================
// Main function
//=============================================================================


using namespace WinMainParameters; //Used for calling windows api from Main()

int main(int argc, char* argv[])
{
	// Set console properties.
	CSetConsole CSetConsole_obj;
	//if ( !CSetConsole_obj.SetConsole( _T("2.CONTROL_WZ_07_22_2011"), 1366, 240, 640, 650, 80, 101 ) )
	if ( !CSetConsole_obj.SetConsole( _T("2.CONTROL_WZ_06_FEB_2024"), -1280, 240, 640, 650, 80, 101 ) )
	{
		tcout << "Fail to set console!" << endl;
		tcin.get();
		return 1;
	}
	// Initial time.
	int t0, tc, t0d;
	t0 = TimeCheck();
	t0d = TimeCheck();

	// General variables.
	Matrix<3,3> translation, rotation;
	Matrix<3,1> wt;
	float control_input, tmp1, tmp2;
	char ch;
	d_c_ee = 115.0f,-15.5f,-25.0f;				

	//takk flag
	bool takk_touch0 = false;
	bool takk_touch1 = false;
	//int takk_t = 3;

	//save velocity data from slip sensor
	ofstream exp_data;
	exp_data.open("C:\\MANUS\\CommonSpace\\Sensor_Data\\vel_data.txt",ios::out);

	ofstream test_data;
	test_data.open("C:\\MANUS\\CommonSpace\\Sensor_Data\\t_data.txt",ios::out);

	/*err_data.open("C:\\MANUS\\CommonSpace\\Sensor_Data\\error_data.txt",ios::out);*/
	//double y_v = 0,y_p = 0,dt = 0,y_p_old = 0;
	//Load dll, connect to shared memory and TCP.
	if ( !LoadAll() )
	{
		cout << "[Error!]: Fail the load! Press any key to exit the program!"<< endl;
		cin.get();
		return 1;
	}
	else
		cout << "[Info]: Load successful! Entering the main loop!" << endl; 

	int init_action = 3;


	//space mouse initialize
	//int  res;
	HINSTANCE hInstance = GetHInstance();
	HINSTANCE hPrevInstance = GetHPrevInstance();
	LPWSTR lpCmdLine = GetLPCmdLine();
	int nCmdShow = GetNCmdShow();

	// Assert that the values returned are expected.
	assert(hInstance != nullptr);
	assert(hPrevInstance == nullptr);
	assert(lpCmdLine != nullptr);

	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);
	int  hsize, vsize;
	hsize = 280;
	vsize = 215;
	CreateSPWindow(0, 0, hsize, vsize, _T("spaceMouse"));
	InvalidateRect(hWndMain, NULL, FALSE);
	/* Initialize 3D mouse */
	//res =SbInit();


	///* if 3D mouse was not detected then print error, close win., exit prog. */
	//if (res < 1)
	//{
	//	MessageBox(hWndMain, 
	//		_T("Sorry - No supported 3Dconnexion device available.\n"),
	//		NULL, MB_OK);
	//	if (hWndMain != NULL)
	//	{
	//		DestroyWindow(hWndMain);    /* destroy window */  
	//	}

	//	ExitProcess(1);                /* exit program */
	//}
	MSG            msg;      /* incoming message to be evaluated */
	BOOL           handled;  /* is message handled yet */ 
	SiSpwEvent     Event;    /* SpaceWare Event */ 
	SiGetEventData EData;    /* SpaceWare Event Data */
	//end space mouse initialize
	// Main Loop.
	handled = SPW_FALSE;     /* init handled */

	while( 1 )
	{
		start_time = TimeCheck();
		
		///* start message loop */ This is for the Space Mouse!
		if(GetMessage( &msg, NULL, 0, 0 )) 
		{

			handled = SPW_FALSE;

			//	/* init Window platform specific data for a call to SiGetEvent */
			SiGetEventWinInit(&EData, msg.message, msg.wParam, msg.lParam);

			//	/* check whether msg was a 3D mouse event and p rocess it */
			if (SiGetEvent (devHdl, SI_AVERAGE_EVENTS, &EData, &Event) == SI_IS_EVENT)
			{
				switch (Event.type)
				{
					case SI_MOTION_EVENT:
						SbMotionEvent(&Event);        /* process 3D mouse motion event */
						break;

					case SI_ZERO_EVENT:
						SbZeroEvent();                /* process 3D mouse zero event */
						break;

					case SI_BUTTON_PRESS_EVENT:
						//printf("ENTERING BUTTON PRESS");
						SbButtonPressEvent(Event.u.hwButtonEvent.buttonNumber);  /* process button press event */
						break;

					case SI_BUTTON_RELEASE_EVENT:
						SbButtonReleaseEvent(Event.u.hwButtonEvent.buttonNumber); /* process button release event */
						break;

					case SI_DEVICE_CHANGE_EVENT:
						HandleDeviceChangeEvent(&Event); /* process 3D mouse device change event */
						break;

					case SI_CMD_EVENT:
						HandleV3DCMDEvent(&Event); /* V3DCMD_* events */
						break;

					case SI_APP_EVENT:
						HandleAppEvent(&Event); /* V3DCMD_* events */
						break;
				}

				handled = SPW_TRUE;              /* 3D mouse event handled */ 
			}

			//	/* not a 3D mouse event, let windows handle it */ Commenting this prevents moving and closing of space mouse window.
			//Uncommenting causes input errors!
			//if (handled == SPW_FALSE)
			//{
			//	TranslateMessage( &msg );
			//	DispatchMessage( &msg );
			//}
		}

		int spacemousearray[4] = {spaceMouseEnabled, spaceMouseMode, spaceButtonsToggle[1], int(grasp_flag!=0)};
		
		//ReadSugspeed();
		switch (mode)
		{
			case MANUAL_MODE:
			{
				//Brandon/Mat 12/15/16
				spaceMouseEnabled_old = spaceMouseEnabled;
				spaceMouseEnabled = spaceButtons[2];//Sets the enable state to the toggle state of pressing both buttons
				//Increments the mode per one button press and release for swapping between hand,arm, and both axis.
				spaceMouseMode_old = spaceMouseMode;

				if ((spaceButtonsToggle[0] || spaceButtonsToggle[1])
					&& spaceMouseEnabled
					&& !btn_gripper_ctl_flag) //once space mouse activated, spaceButtonsToggle[0,1] can only change mode, then set to 0.
				{

					if (spaceButtonsToggle[0])
					{
						spaceMouseMode <= 0 ? spaceMouseMode = spaceMouseMode_count : spaceMouseMode--;
						spaceButtonsToggle[0] = false;
					}

					if (spaceButtonsToggle[1])
					{
						//spaceMouseMode_old = spaceMouseMode;
						spaceMouseMode >= spaceMouseMode_count ? spaceMouseMode = 0 : spaceMouseMode++;
						spaceButtonsToggle[1] = false;
					}
					//spaceMouseMode >=3  ? spaceMouseMode=0:spaceMouseMode++;	// spaceMouseMode  0 arm mode, 1 wrist mode, 2 hybird mode.  3 gripper frame
					//spaceButtonsToggle[0] = false;								//spaceButtonsToggle[1]  1 for gripper mode
				}

				gotoxy(1, 19);
				printf("spaceButtons[0]  %d,   [1]  %d,  [2]  %d,ctl 0  %d ,1  %d ,toogle %d ", spaceButtons[0], spaceButtons[1], spaceButtons[2], btn_gripper_ctl[0], btn_gripper_ctl[1], btn_gripper_ctl_flag);
				gotoxy(1, 20);
				//spaceMouseEnabled?
				//	printf("Space Mouse Enabled [Mode]: %d [GRIP]: %d  [stop flag]: %d  move as suggest %d ", spaceMouseMode,spaceButtonsToggle[1], spaceMouse_stop, move_as_suggested[1])
				//	:printf("Space Mouse Disabled                         move as suggest %d  ", move_as_suggested[1]);
				if (spaceMouseEnabled)
				{
					printf("Space Mouse Enabled [Mode]: %d [GRIP]: %d  [stop flag]: %d  move as suggest %d ", spaceMouseMode, spaceButtonsToggle[1], spaceMouse_stop, move_as_suggested[1]);
				}
				else
				{
					printf("Space Mouse Disabled                         move as suggest %d  ", move_as_suggested[1]);
				}

				if ((init_system) && (rcvMsg.ID == 0x37f))
				{
					switch (init_action)
					{
					case 3:
						// Cartesian.
						ManualControl('1');
						init_action--;
						break;

					case 2:
						// Cartesian.
						ManualControl('1');
						init_action--;
						break;

					case 1:
						init_action--;
						break;

					case 0:
						if (fabs(pos[5]) < 30.0f)
						{
							// Reverse roll angle.
							ManualControl('9');
						}

						init_system = false;
						break;
					}
				}

				if ((!init_system) && (fabs(pos[5]) < 45.0f))
				{
					// Reverse roll angle.
					ManualControl('9');
				}

				// Does this first if need to go after the next 2?
				if (spaceMouseEnabled/*&&spaceMouseMode!=4*/)
				{
					ManualControl('#');// arm, wrist, hybrid, hybrid in gripper frame modes
				}
				else if (spaceMouseEnabled && spaceMouseMode == 4)  // gripper mode 
				{
					if (spaceMouse[2] < -500)
					{
						ManualControl('j');//close
						spm_gripper = 1;
						spacemouse_operation[2] = -1;
					}
					else if (spaceMouse[2] > 500)
					{
						ManualControl('u');//open
						spm_gripper = 2;
						spacemouse_operation[2] = 1;
					}
					else if (spaceMouse[2] < 300 && spaceMouse[2]>-300)
					{
						ManualControl(' ');
						spm_gripper = 0;
						spacemouse_operation[2] = 0;
					}
				}
				else if (spaceMouseEnabled && spaceMouseMode == 5)// one click mode
				{
					if (spaceMouse[0] > 1500)
					{
						ManualControl('S');//one click mode
					}
					else
					{
						ManualControl('#');
					}
				}
				else
				{
					spm_gripper = 0;
				}

				if (!spaceMouseEnabled && spm_operation != 0)
				{
					spm_operation = 0;
				}

				// True if a key has been pressed.
				if (_kbhit())
				{
					ch = _getch();
					ManualControl(ch);
					//gotoxy(1, 48);
					//cout << TimeCheck() << endl;
				}

				// Rafael - 01/07/08
				if (myRcv.key.tryReadLock() == -1)
				{
					cout << "\n  Received data: Busy updating " << "        " << std::endl << std::flush;
				}
				else
				{
					if (myRcv.command == NULL)
					{
						myRcv.key.unlock();
						//btn_cmd = '~';
					}
					else
					{
						ch = myRcv.command;
						btn_cmd = ch;

						ManualControl(ch);  //GUI button command
						myRcv.key.unlock();
						myRcv.key.writeLock();
						myRcv.command = NULL;
						myRcv.key.unlock();
					}
					gotoxy(1, 42);
					cout << btn_cmd << endl;
				}

				// Translate to eliminate the offset.
				if ((auto_mode_start) & (rotation_start1))
				{
					rotation_start1 = false;
					rotation_start2 = true;
				}

				// Angular motion.
				if ((auto_mode_start) & (rotation_start2))	// rotation	
				{
					for (int i = 3; i < 6; i++)
					{
						if (i == 5) // roll -180 ~ 180 : linear scailing
						{
							/*tmp1 = (180.0f - pd[i]) - (180.0f - pos[i]);*/ // why is this 180 here? Nick 2024
							tmp1 = pos[i] - pd[i];
							if (tmp1 >= 0.0f)
							{
								if (tmp1 <= 180.0f)
									tmp2 = tmp1;
								else
									tmp2 = std::fmodf(tmp1, 360.0f); // Nick 2024
									/*tmp2 = tmp1 - 360.0f;*/ 
							}
							else
							{
								if (tmp1 > -180.0f)
									tmp2 = tmp1;
								else
									tmp2 = std::fmodf(tmp1, 360.0f); // Nick 2024
									//tmp2 = 360.0f + tmp1;
							}
							eprev1[i] = tmp2;
						}
						else
							eprev1[i] = pd[i] - pos[i];

						control_input = Kp[i] * eprev1[i];
						float speed_limit = angular_speed_limit[speed_mode] * 1.0f;
						speed[i + 1] = (fabs(control_input) > speed_limit) ? sign(control_input) * speed_limit : control_input;
					}

					if ((fabs(eprev1[3]) < R_ERR_BOUND) & (fabs(eprev1[4]) < R_ERR_BOUND))	// yaw & pitch rotation control
					{
						speed[4] = 0; speed[5] = 0;

						if (fabs(eprev1[5]) < R_ERR_BOUND)	// roll rotation control
						{
							rotation_start2 = false;
							speed[4] = 0;
							speed[5] = 0;
							speed[6] = 0;
						}
					}
					else
						speed[6] = 0;
					DisplaySpeed();
				}

				// Linear motion.
				if ((auto_mode_start) & (!rotation_start2) & (!rotation_start1) & (cbox == CARTESIAN)) // translation
				{
					// position control.
					for (int i = 0; i < 3; i++)
					{
						eprev1[i] = pd[i] - pos[i];

						control_input = Kp[i] * eprev1[i];
						float speed_limit = linear_speed_limit[speed_mode] * 1.0f;
						speed[i + 1] = (fabs(control_input) > speed_limit) ? sign(control_input) * speed_limit : control_input;
					}
					if ((fabs(eprev1[0]) < T_ERR_BOUND) & (fabs(eprev1[1]) < T_ERR_BOUND) & (fabs(eprev1[2]) < T_ERR_BOUND))
					{
						auto_mode_start = false;
						speed[1] = speed[2] = speed[3] = 0;
					}
					DisplaySpeed();
				}
				//Joint P control Brandon 10/28/16
				if ((auto_mode_start) & (!rotation_start2) & (!rotation_start1) & (cbox == JOINT)) // Joint control
				{
					int joint3 = 0;
					int joint4 = 0;
					Apos[0] = pos[0];
					Apos[4] = pos[4];
					Apos[5] = pos[5];
					Apos[6] = pos[6];

					// Correction for dependancy between joint 2 and 3
					Apos[1] = (-1) * pos[1];
					joint3 = static_cast<int>(pos[2] + 0.5); // +0.5 to round int
					joint3 = joint3 + 900 + static_cast<int>(Apos[1] + 0.5); // +0.5 to round int

					if (joint3 >= 0)
						Apos[2] = static_cast<float>(1800 - joint3);
					else
						Apos[2] = static_cast<float>(-1800 - joint3);

					joint4 = static_cast<int>(pos[3] + 900 + 0.5); // +0.5 to round int
					Apos[3] = static_cast<float>(joint4);

					// Bring back the angles between -180 to 180
					for (int i = 0; i < 7; i++)
					{
						while (Apos[i] > 1800.0f)
							Apos[i] = (Apos[i] - 3600.0f);
						while (Apos[i] < -1800.0f)
							Apos[i] = (Apos[i] + 3600.0f);
					}

					for (int i = 0; i < 8; i++)
						Apos[i] = 0.1f * Apos[i];
					float Kp[6] = { .5f, .5f, .5f, .5f, .5f, .5f };
					// Joint control.
					for (int i = 0; i < 6; i++)
					{
						eprev1[i] = pd[i] - Apos[i + 1];

						control_input = Kp[i] * eprev1[i];
						float speed_limit = linear_speed_limit[speed_mode] * 1.0f;
						speed[i + 1] = (fabs(control_input) > speed_limit) ? sign(control_input) * speed_limit : control_input;
					}
					if ((fabs(eprev1[0]) < T_ERR_BOUND) & (fabs(eprev1[1]) < T_ERR_BOUND) & (fabs(eprev1[2]) < T_ERR_BOUND))
					{
						auto_mode_start = false;
						speed[1] = speed[2] = speed[3] = 0;
					}
					DisplaySpeed();
				}

				//init_grasp2(); // sets pos_before_lifting and ready_to_lift
				//regrasping_algorithm2();
				init_grasp(); // sets pos_before_lifting and ready_to_lift //mushtaq
				regrasping_algorithm();
				//Robson Expt for lifting up objects after they have been grasped
				//if (ready_to_lift && grasp_flag == INITIAL && abs(pos[2] - pos_before_lifting) <= 100) {
				//	// pos[2] lift arm pos
				//	speed[3] =linear_speed_limit[1];// lift up motion
				//    new_status = true; 
				//}
				//else if (ready_to_lift && grasp_flag == INITIAL && abs(pos[2] - pos_before_lifting) > 100) {
				//	speed[3] = 0; // stop lifting since the arm has reached to the desired position 
				//	new_status = true;
				//}

				//grasping_with_desired_force(3.81);

				// Open the grabber.
				if (open_in_progress)
				{
					init_stop = 0; // regrasping flag set to 0 because gripper is opened by the user Robson 
					if ((cbox == CARTESIAN) && (pos[6] > 14000.0f))
					{
						//SleepMs( 500 );
						SendCommand(CAN, FSR, FSR_RESET, EMPTY_MESSAGE);
						open_in_progress = false;
						ready_to_lift = false;
						//Robson 
						//Reset Grasping Algo parameters
						F_d = 0;
						F_d1 = 0;
						F_d2 = 0;
						last_b_hat = init_b_hat;
						last_a_hat = init_a_hat;
						a_hat = 0;
						b_hat = 0;
						angl_dis = 0;
						lin_vel = 0;
						ang_vel = 0;
					}
				}

				// Read from MANUS.
				rcvMsg.ID = 0x0;
				m_objPCANBasic.Read(m_Handle, &rcvMsg, &CANTimeStamp);
				Decode(rcvMsg, xmitMsg);

				tc = TimeCheck();
				if ((new_status == true) && (rcvMsg.ID == 0x37f))
				{
					if (((rotation_start1) | (rotation_start2)))
					{

						stsResult = m_objPCANBasic.Write(m_Handle, &xmitMsg);
						if (stsResult != PCAN_ERROR_OK)
							cout << "[Error!]: Fail to write to PCAN!" << endl;
					}
					else if (((!rotation_start2) & (!rotation_start1)))
					{

						stsResult = m_objPCANBasic.Write(m_Handle, &xmitMsg);
						if (stsResult != PCAN_ERROR_OK)
							cout << "[Error!]: Fail to write to PCAN!" << endl;
					}
					if (auto_mode_start == false)
					{
						new_status = false;

					}
					SleepMs(5);//zc was 30
				}

				fflush(stdin);
				job_done = false;
				break;
			}

			case AUTO_MODE:
			{
				for (int i = 0; i < 8; i++)
					speed[i] = 0;
				//cbox = CARTESIAN;
				// Translation, approach, retreat.
				if ((job_complete == true) & (job_complete2 == false))
					job_done = job_complete2;
				// Rotation.	
				else if ((job_complete == false) & (job_complete2 == true))
					job_done = job_complete;
				//
				else if (home_pos_flag == true)
					job_done = false;

				SendCommand(CAN, GUI, UPDATE_IN_MOTION, 1);

				// KIM - open-loop & closed-loop
				while (!job_done)
				{
					//gotoxy(1, 50);
					//cout << TimeCheck() << endl;
					// Check if its time to read a new message.          
					m_objPCANBasic.Read(m_Handle, &rcvMsg, &CANTimeStamp);
					Decode(rcvMsg, xmitMsg);

					tc = TimeCheck();

					// Write a message.
					if (rcvMsg.ID == 0x37f)
					{
						stsResult = m_objPCANBasic.Write(m_Handle, &xmitMsg);
						if (stsResult != PCAN_ERROR_OK)
							cout << "[Error!]: Fail to write to PCAN!" << endl;
						SleepMs(30);
					}

					if (_kbhit())
					{
						if (_getch() == ' ')
							ResetAll();
					}

					fflush(stdin);
				}

				SendCommand(CAN, GUI, UPDATE_IN_MOTION, 0);
				mode = MANUAL_MODE;
				for (int i = 0; i < 8; ++i)
					speed[i] = 0;

				ShowStatus("STAT: Job Complete\n");
				btn_cmd = '*';
				break;
			}
		}

		end_time = TimeCheck();
		ShowFrequency( end_time - start_time );
		ReadForce(cur_force);
		//FTMgr.ReadForce(cur_force);
		ReadLPS(LPS_value);
		new_force = cur_force;
		
		if (end_time-force_t>40)
		{
			force_que[force_count] =  new_force;
			force_count = ( force_count == 4 ) ? 0 : force_count + 1;
			old_force = new_force;
			cout << "Force Que: "<< old_force<< endl;
			force_t = TimeCheck();
		}

		FTMgr.interact_perceive();
		FTMgr.ReadForceTorque();
		FTMgr.check_force(); // Only checks if interact_perceive_state is true
		ReadPosit();
		ReadPosit2();
		ReadVel();
		ReadVel2();
		ReadOBJ();
		SleepMs(2);
		UpdateSpaceMouse(spacemousearray);
		Readblock_dir();
		ReadTaKK();

		if (assistant_flag)
		{
			oneclick();
		}
		
		exp_data << end_time << ", " << cur_velocity_f << ", " << cur_velocity << ", " << cur_position << ", " << cur_pos_nf << ", " << cur_force << ", " << lift_in_progress << ", " << grasp_end << ", " << speed[7] << ", " << pos[6] << ", " << pos_before_lifting << ", " << grasp_flag << ", " << e_force << ", " << lin_vel << ", " << F_d << ", " << F_d1 << ", " << F_d2 << ", " << pos[2] << ", " << speed[3] << ", " << dt0 << ", " << u_hat_dot << ", " << P_d << ", " << P_int << "," << cur_velocity_f2 << "," << cur_velocity2 << "," << cur_position2 << "," << cur_pos_nf << "," << a_hat << "," << b_hat << "," << ang_vel << "," << angl_dis << "," <<speed_mode<< "," << k1 << "," << k2 << "," << k3 << "," << gamma_1 << "," << gamma_2 <<","<< lin_dist1<< "," << lin_dist2 <<"," << F_ee[0]<<"," << F_ee[1] <<"," << F_ee[2]<<","<< T_ee[0]<<"," <<T_ee[1]<<","<< T_ee[2]<<","<<elapsed_time1<<","<<Vx_ee<<","<<Vy_ee <<","<< Vz_ee<<","<< counter << "\n"; //u_hat_dot
		
		float cam_dist;
		cam_dist = DistanceBetween_Camera_Link3(pos);
		gotoxy(1, 40);
		//printf("The closest distance between the camera and link3 %.3f", cam_dist);
		if ((cam_dist < 38) && !cam_cls)
		{
			ResetAll();
			cam_cls = true;
			block_camcls_move();
		}
		else if (cam_dist > 38 && cam_cls) 
		{
			cam_cls = false;
			//block_camcls_move();
			for (int i = 1; i < 13; i++)
			{
				block_movement[i] = 0; 
			}
		}
	}

	
	// Unload dll of CAN card, disconnect to shared memory and TCP.
	if ( !UnloadAll() )
		cout << "[Error!]: Fail the unload! Press any key to exit the program!" << endl;
	else
		cout << "[Info]: Unload successful! Press any key to exit the program!" << endl;
	cin.get();
	return 0;
}
