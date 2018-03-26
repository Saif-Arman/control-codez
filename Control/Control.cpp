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
	if ( !CSetConsole_obj.SetConsole( _T("2.CONTROL_WZ_07_22_2011"), -1280, 240, 640, 430, 80, 101 ) )
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
	int  res; 
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
	res = SbInit();


	/* if 3D mouse was not detected then print error, close win., exit prog. */
	if (res < 1)
	{
		MessageBox(hWndMain, 
			_T("Sorry - No supported 3Dconnexion device available.\n"),
			NULL, MB_OK);
		if (hWndMain != NULL)
		{
			DestroyWindow(hWndMain);    /* destroy window */  
		}

		ExitProcess(1);                /* exit program */
	}
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
				if (Event.type == SI_MOTION_EVENT)
				{
					SbMotionEvent(&Event);        /* process 3D mouse motion event */     
				}
				else if (Event.type == SI_ZERO_EVENT)
				{
					SbZeroEvent();                /* process 3D mouse zero event */     
				}
				else if (Event.type == SI_BUTTON_PRESS_EVENT)
				{
					//printf("ENTERING BUTTON PRESS");
					SbButtonPressEvent(Event.u.hwButtonEvent.buttonNumber);  /* process button press event */
				}
				else if (Event.type == SI_BUTTON_RELEASE_EVENT)
				{
					SbButtonReleaseEvent(Event.u.hwButtonEvent.buttonNumber); /* process button release event */
				}
				else if (Event.type == SI_DEVICE_CHANGE_EVENT)
				{
					HandleDeviceChangeEvent(&Event); /* process 3D mouse device change event */
				}
				else if (Event.type == SI_CMD_EVENT)
				{
					HandleV3DCMDEvent(&Event); /* V3DCMD_* events */
				}
				else if (Event.type == SI_APP_EVENT)
				{
					HandleAppEvent(&Event); /* V3DCMD_* events */
				}

				handled = SPW_TRUE;              /* 3D mouse event handled */ 
			}

			//	/* not a 3D mouse event, let windows handle it */ Commenting this prevents moving and closing of space mouse window.
			//Uncommenting causes input errors!
			//if (handled == SPW_FALSE)
			//{
				//TranslateMessage( &msg );
				//DispatchMessage( &msg );
			//}
		}
		int spacemousearray[4] = {spaceMouseEnabled,spaceMouseMode,spaceButtonsToggle[1],int(grasp_flag!=0)};
		gotoxy(20,30);
		cout << int(grasp_flag == 0) << endl;
		//ReadSugspeed();
		switch ( mode )
		{ 
		case MANUAL_MODE:

			//Brandon/Mat 12/15/16
			spaceMouseEnabled_old = spaceMouseEnabled;
			spaceMouseEnabled = spaceButtons[2];//Sets the enable state to the toggle state of pressing both buttons
			//Increments the mode per one button press and release for swapping between hand,arm, and both axis.
			spaceMouseMode_old = spaceMouseMode;
			if((spaceButtonsToggle[0]|| spaceButtonsToggle[1])&&spaceMouseEnabled)
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
			gotoxy( 1, 20);
			spaceMouseEnabled?
				printf("Space Mouse Enabled [Mode]: %d [GRIP]: %d  [stop flag]: %d  move as suggest %d ", spaceMouseMode,spaceButtonsToggle[1], spaceMouse_stop, move_as_suggested)
				:printf("Space Mouse Disabled                         move as suggest %d  ", move_as_suggested);
			if ( ( init_system ) && ( rcvMsg.ID == 0x37f ) )
			{
				switch (init_action)
				{
				case 3:
					// Cartesian.
					ManualControl( '1' );
					init_action--;
					break;
				case 2:
					// Cartesian.
					ManualControl( '1' );
					init_action--;
					break;
				case 1:
					init_action--;
					break;
				case 0:
					if ( fabs( pos[5] ) < 30.0f )
					{
						// Reverse roll angle.
						ManualControl( '9' );
					}

					init_system = false;
					break;
				}
			}

			if ( ( !init_system ) && ( fabs(pos[5]) < 45.0f ) )
			{
				// Reverse roll angle.
				ManualControl('9');
			}
			if(spaceMouseEnabled&& spaceMouseMode!=4)
				ManualControl('#');// arm, wrist, hybrid, hybrid in gripper frame modes
			else if (spaceMouseEnabled&& spaceMouseMode == 4)  // gripper mode 
			{
				if (spaceMouse[2] < -500)
				{
					ManualControl('j');//close
					spm_gripper = 1;
				}
				else if (spaceMouse[2] > 500)
				{
					ManualControl('u');//open
					spm_gripper = 2;
				}
				else if (spaceMouse[2] < 300 && spaceMouse[2]>-300)
				{
					ManualControl(' ');
					spm_gripper = 0;
				}
			}
			//else if (spaceMouseEnabled&& spaceMouseMode == 5)// one click mode
			//{
			//	if (spaceMouse[0] >1500)
			//	{
			//		ManualControl('S');//one click mode
			//	}
			//	else
			//	{
			//		ManualControl('#');
			//	}
			//}
			else
				spm_gripper = 0;

			if (!spaceMouseEnabled&&spm_operation != 0)
				spm_operation = 0;



			// True if a key has been pressed.
			if ( _kbhit() ) 
			{
				ch =_getch();
				ManualControl( ch );
				//gotoxy(1, 48);
				//cout << TimeCheck() << endl;
			}
			
			// Rafael - 01/07/08
			if ( myRcv.key.tryReadLock() == -1 )
			{
				cout << "\n  Received data: Busy updating " << "        "<< std::endl << std::flush;
			}    
			else
			{
				if ( myRcv.command != NULL )
				{
					ch = myRcv.command;
					btn_cmd = ch;
					gotoxy(1,42);
					cout << (btn_cmd=='n') << endl;
					ManualControl( ch );  //GUI button command
					myRcv.key.unlock();
					myRcv.key.writeLock();
					myRcv.command = NULL;
					myRcv.key.unlock();
				}
				else
				{
					myRcv.key.unlock();
					btn_cmd = ' ';
				}
			}			

			// Translate to eliminate the offset.
			if ( ( auto_mode_start ) & ( rotation_start1 ) )
			{				
				rotation_start1 = false;
				rotation_start2 = true;
			}

			// Angular motion.
			if ( ( auto_mode_start ) & ( rotation_start2 ) )	// rotation	
			{
				for ( int i = 3; i < 6; i++ )
				{
					if ( i == 5 ) // roll -180 ~ 180 : linear scailing
					{
						tmp1 = ( 180.0f - pd[i] ) - ( 180.0f - pos[i] );
						if ( tmp1 >= 0.0f )
							if ( tmp1 <= 180.0f )
								tmp2 = tmp1;
							else
								tmp2 = tmp1 - 360.0f;
						else
							if ( tmp1 > -180.0f )
								tmp2 = tmp1;
							else
								tmp2 = 360.0f + tmp1;
						eprev1[i] = tmp2;
					}
					else
						eprev1[i] = pd[i] - pos[i];

					control_input = Kp[i] * eprev1[i];
					float speed_limit = angular_speed_limit[speed_mode] * 1.0f;
					speed[i+1] = ( fabs( control_input ) > speed_limit ) ? sign( control_input ) * speed_limit : control_input;
				}
				if ( ( fabs( eprev1[3] ) < R_ERR_BOUND ) & ( fabs( eprev1[4] )< R_ERR_BOUND ) )	// yaw & pitch rotation control
				{
					speed[4] = 0; speed[5] = 0;

					if ( fabs( eprev1[5] ) < R_ERR_BOUND )	// roll rotation control
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
			if ( ( auto_mode_start ) & ( !rotation_start2 ) & ( !rotation_start1 ) & (cbox == CARTESIAN)) // translation
			{
				// position control.
				for ( int i = 0; i < 3; i++ )
				{					
					eprev1[i] = pd[i] - pos[i];

					control_input = Kp[i] * eprev1[i];
					float speed_limit = linear_speed_limit[speed_mode] * 1.0f;
					speed[i+1] = ( fabs( control_input ) > speed_limit ) ? sign( control_input ) * speed_limit : control_input;
				}
				if ( ( fabs( eprev1[0] ) < T_ERR_BOUND ) & ( fabs( eprev1[1] ) < T_ERR_BOUND ) & ( fabs( eprev1[2] ) < T_ERR_BOUND ) )
				{
					auto_mode_start = false;
					speed[1] = speed[2] = speed[3] = 0;
				}
				DisplaySpeed();
			}
			//Joint P control Brandon 10/28/16
			if ( ( auto_mode_start ) & ( !rotation_start2 ) & ( !rotation_start1 ) & (cbox == JOINT )) // Joint control
			{
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

				if( joint3 >= 0 )
					Apos[2] = 1800 - joint3;
				else
					Apos[2] = -1800 - joint3;

				joint4 = pos[3] + 900;
				Apos[3] = joint4;

				// Bring back the angles between -180 to 180
				for( int i = 0; i < 7; i++ )
				{
					while( Apos[i] > 1800 )
						Apos[i] = ( Apos[i] - 3600 );
					while( Apos[i] < -1800 )
						Apos[i] = ( Apos[i] + 3600 );
				}

				for( int i = 0; i < 8; i++ )
					Apos[i] = 0.1 * Apos[i];
				float Kp[6] = {.5,.5,.5,.5,.5,.5};
				// Joint control.
				for ( int i = 0; i < 6; i++ )
				{					
					eprev1[i] = pd[i] - Apos[i+1];

					control_input = Kp[i] * eprev1[i];
					float speed_limit = linear_speed_limit[speed_mode] * 1.0f;
					speed[i+1] = ( fabs( control_input ) > speed_limit ) ? sign( control_input ) * speed_limit : control_input;
				}
				if ( ( fabs( eprev1[0] ) < T_ERR_BOUND ) & ( fabs( eprev1[1] ) < T_ERR_BOUND ) & ( fabs( eprev1[2] ) < T_ERR_BOUND ) )
				{
					auto_mode_start = false;
					speed[1] = speed[2] = speed[3] = 0;
				}
				DisplaySpeed();
			}
			/*if(cur_force < init_force-0.2 && cur_force != 0)
			{
			float start_force = cur_force;
			ResetAll();

			myRcv.key.writeLock();
			myRcv.command = 'j';
			myRcv.key.unlock();

			grab_in_progress = true;

			while(cur_force < init_force+.1)
			{
			ReadForce(cur_force);
			if(cur_force == 0)
			break;
			}
			ResetAll();
			continue;
			}*/
			//ReadSlip(cur_velocity);
			////Slip Detection and control -- Comment out if testing other gripping mechanisms


			regrasp();
			init_grasp();

			//if (grasp_test == 1)
			//	grasp_npos = pos[6];
			//	if (abs(grasp_npos- grasp_inipos)>= 500)
			//		grasp_move = true;





			// Open the grabber.
			if ( open_in_progress )
			{
				init_stop = 0;
				if ( ( cbox == CARTESIAN ) && ( pos[6] > 14000.0f ) )
				{
					SleepMs( 500 );
					SendCommand( CAN, FSR, FSR_RESET, EMPTY_MESSAGE );
					open_in_progress = false;
					ready_lift = false;
					//continue; //  nessary?
				}
			}

			// Read from MANUS.
			rcvMsg.ID = 0x0;
			m_objPCANBasic.Read( m_Handle, &rcvMsg, &CANTimeStamp );
			Decode( rcvMsg, xmitMsg );				

			tc = TimeCheck();
			if ( ( new_status == true ) && ( rcvMsg.ID == 0x37f ) )
			{
				if ( ( ( rotation_start1 ) | ( rotation_start2 ) ) )
				{
					//if ( debug_on )
					//{
					//	timing << "0 " << TimeCheck()					 
					//		<< " " << pos[0] << " " << pos[1] << " " << pos[2] 
					//		<< " " << pos[3] << " " << pos[4] << " " << pos[5] 
					//		<< "\n";
					//}
					stsResult = m_objPCANBasic.Write( m_Handle, &xmitMsg );
					if ( stsResult != PCAN_ERROR_OK )
						cout << "[Error!]: Fail to write to PCAN!" << endl; 
				}
				else if ( ( (!rotation_start2) & (!rotation_start1) ) )
				{
					//if ( debug_on )
					//{
					//	timing << "1 " << TimeCheck()					 
					//		<< " " << pos[0] << " " << pos[1] << " " << pos[2] 
					//		<< " " << pos[3] << " " << pos[4] << " " << pos[5] 
					//		<< "\n";
					//}
					stsResult = m_objPCANBasic.Write( m_Handle, &xmitMsg );
					if ( stsResult != PCAN_ERROR_OK )
						cout << "[Error!]: Fail to write to PCAN!" << endl; 
				}
				if (auto_mode_start == false)
				{
					new_status = false;
					//gotoxy(1, 43);
					//cout << (TimeCheck() ) << endl;
					//if ((suggspeed[6] != 0) )
					//{
					//	new_status = true;
					//	//gotoxy(1, 43);
					//	//cout << prevID << endl;
					//}
				}
				SleepMs( 5 );//zc was 30
				//if ( debug_on )
				//{
				//	timing << "2 " << TimeCheck()					 
				//			<< " " << pos[0] << " " << pos[1] << " " << pos[2] 
				//			<< " " << pos[3] << " " << pos[4] << " " << pos[5] 
				//			<< "\n";
				//}
			}

			fflush( stdin );
			job_done = false;
			break;

		case AUTO_MODE:

			for ( int i = 0 ; i < 8 ; i++ )                
				speed[i] = 0;       
			//cbox = CARTESIAN;
			// Translation, approach, retreat.
			if ( ( job_complete == true ) & ( job_complete2 == false ) )		
				job_done = job_complete2;
			// Rotation.	
			else if ( ( job_complete == false ) & ( job_complete2 == true ) )		
				job_done = job_complete;
			//
			else if ( home_pos_flag == true )
				job_done = false;

			SendCommand( CAN, GUI, UPDATE_IN_MOTION, 1 );

			// KIM - open-loop & closed-loop
			while ( !job_done )	
			{                
				//gotoxy(1, 50);
				//cout << TimeCheck() << endl;
				// Check if its time to read a new message.          
				m_objPCANBasic.Read( m_Handle, &rcvMsg, &CANTimeStamp );
				Decode( rcvMsg, xmitMsg );                

				tc = TimeCheck();

				// Write a message.
				if ( rcvMsg.ID == 0x37f )
				{
					stsResult = m_objPCANBasic.Write( m_Handle, &xmitMsg );
					if ( stsResult != PCAN_ERROR_OK )
						cout << "[Error!]: Fail to write to PCAN!" << endl; 
					SleepMs( 30 );
				}

				if ( _kbhit() )
				{
					if ( _getch() == ' ' )
						ResetAll();
				}

				fflush( stdin );
			}

			SendCommand( CAN, GUI, UPDATE_IN_MOTION, 0 );
			mode = MANUAL_MODE;			
			for ( int i = 0 ; i < 8 ; ++i )        
				speed[i] = 0;			
			ShowStatus( "STAT: Job Complete\n" );
			break;
		}


		end_time = TimeCheck();
		ShowFrequency( end_time - start_time );
		ReadForce(cur_force);
		ReadLPS(LPS_value);
		new_force = cur_force;
		
		if (end_time-force_t>40){
			force_que[force_count] =  new_force;
			force_count = ( force_count == 4 ) ? 0 : force_count + 1;
			old_force = new_force;
			force_t = TimeCheck();
		}

		//ReadSlip(cur_velocity);

		ReadPosit(cur_position);
		ReadVel();
		ReadOBJ();
		SleepMs(2);
		if (assistant_flag)
		{
			oneclick();
		}


		UpdateSpaceMouse(spacemousearray);
		//ReadSugspeed();

		//cout << "x  " << suggspeed[0] << "   Y  " << suggspeed[1] << "   Z  " << suggspeed[2] 
		//	<< "   yal  " << suggspeed[3] << "   pitch  " << suggspeed[4] << "   rool  " << suggspeed[5]  << endl;
		//cout << TimeCheck() << endl;
		ReadTaKK();
		//if(!grab_in_progress )
		//	for(int i = 0;i < 12; i++)
		//		init_takk[i] = cur_takk[i];

		//LowPassFilter();
		//exp_data << end_time << ", " << force_que[0] <<", "  << force_que[1] <<", "<< force_que[2] <<  ", "<< force_que[3] <<  ", " << cur_force<< ", "<< new_force<< ", " << old_force<<", " << force_count<<"\n";//force_count
		exp_data << end_time << ", " << cur_velocity_f <<", "  << cur_force <<", " << cur_position << ", "<< cur_pos_nf << ", " << cur_velocity <<", " << grasp_start <<", " << grasp_end << ", " << speed[7] << ", " << pos[6]<<", " << F_d<<", " << grasp_flag<< ", " << e_force<<", " << adjust<<", " << speed[7]<<", " << stoppos <<", " << e_pos<<", " << w_hat <<", " << u_hat<<", " << dt0<<", " << u_hat_dot<<", " << P_d<<", " << P_int<<"\n";//u_hat_dot
		//exp_data << end_time << ", " << speed[0] << ", " << speed[1] << ", " << speed[2] << ", " << speed[3] << ", " << 
		//	speed[4] << ", " << speed[5] << ", " << speed[6] << ", " << speed[7] << ", " << pos[0] << ", " << pos[1] << ", " << 
		//	pos[2] << ", " << pos[3] << ", " << pos[4] << ", " << pos[5] << ", " << pos[6] << ", " << pos[7] <<  "\n";//u_hat_dot

		
		//exp_data << end_time << ", " << grasp_test <<", "  << cur_force <<", "<< grasp_inipos<<  ", " << grasp_npos<< ", " << init_stop<< ", " << speed[7]<<"\n";//grasp_move
		//exp_data << end_time << ", " << grasp_test <<", "  << cur_force <<", "<< cur_velocity <<  ", "<< cur_velocity_f <<  ", " << grasp_npos<< ", " << init_stop<< ", " << speed[7]<<"\n";//grasp_move
		//old_vel = cur_velocity;
		//LPScheck();
		//gotoxy(1, 50);
		//cout <<"status"<< new_status<< auto_mode_start <<"pressed"<<suggspeed[6]<< endl;

		Readblock_dir();
		Operation_check();
		////  Mushtaq
		float cam_dist;
		cam_dist = DistanceBetween_Camera_Link3(pos);
		gotoxy(1, 39);
		printf("The closest distance between the camera and link3 %.3f", cam_dist);
		if ((cam_dist < 38) && !cam_cls)
		{
			ResetAll();
			cam_cls = true;
			block_camcls_move();
			gotoxy(1,59);
			printf("block direction 1. %d   2. %d  3.  %d  4. %d  5. %d   . %d  ", block_movement[0], block_movement[1], block_movement[2], block_movement[3], block_movement[4], block_movement[5]);

		}
		else if (cam_dist > 38 && cam_cls) 
		{
			cam_cls = false;
		}


		///
	}




	///

	
	// Unload dll of CAN card, disconnect to shared memory and TCP.
	if ( !UnloadAll() )
		cout << "[Error!]: Fail the unload! Press any key to exit the program!" << endl;
	else
		cout << "[Info]: Unload successful! Press any key to exit the program!" << endl;
	cin.get();
	return 0;
}
