//=============================================================================
// System Includes
//=============================================================================
#include <conio.h>
#include <stdio.h>
#include <windows.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <cctype>
#include <string.h>
#include <process.h>
#include <math.h>
using namespace std;

//=============================================================================
// Project Includes
//=============================================================================
#include "PCANBasicClass.h"							// PCAN functions.
#include "RWLock.hpp"								// Interlock object.
#include "Matrix.hpp"								// Matrix definition.
#include "Transform.hpp"							// ColumnVector definition.
#include "cxutils/networking/tcpclient.h"			// TCP client connections.
#include "cxutils/ipc/messageclient.h"				// Shared/mapped memory message client.
#include "MANUS_define.h"
using namespace CxUtils;

//=============================================================================
// Macro Definitions, Global Variables and Sub function
//=============================================================================
#include "Macro.h"
#include "Global.h"
#include "Functions.h"


//=============================================================================
// Main function
//=============================================================================
int main(int argc, char* argv[])
{
	SetConsoleTitle("2.ROBOT_WZ_07_21_2011");
	// Initial time.
	int t0, tc, t0d;
	t0 = TimeCheck();
	t0d = TimeCheck();
	//if ( debug_on ) 
	//{
	//	sprintf_s( user_str, "%scontrol_timing.txt", debug_path );
	//	timing.open( user_str );
	//}

    // General variables.
    Matrix<3,3> translation, rotation;
	Matrix<3,1> wt;
	float control_input, tmp1, tmp2;
	char ch;
	d_c_ee = 115.0f,-15.5f,-25.0f;				

	// Load dll, connect to shared memory and TCP.
	if ( !LoadAll() )
	{
		cout << "[Error!]: Fail the load! Press any key to exit the program!"<< endl;
		cin.get();
		return 1;
	}
	else
		cout << "[Info]: Load successful! Entering the main loop!" << endl; 

	int init_action = 3;
    // Main Loop.
    while(1)
    {    
        switch ( mode )
        { 
		case MANUAL_MODE:
			if ( ( init_system ) && ( rcvMsg.ID == 0x37f ) )
			{
				switch (init_action)
				{
					case 3:
						ManualControl( '1' );
						init_action--;
						break;
					case 2:
						ManualControl( '1' );
						init_action--;
						break;
					case 1:
						init_action--;
						break;
					case 0:
						if ( fabs( pos[5] ) < 30.0f )
						{
							ManualControl( '9' );
						}

						init_system = false;
						break;
				}
			}

			if ( ( !init_system ) && ( fabs(pos[5]) < 45.0f ) )
			{
				ManualControl('9');
			}

			// True if a key has been pressed.
			if ( _kbhit() ) 
			{
				ch =_getch();
				ManualControl( ch );
			}

			// Rafael - 01/07/08
			if( myRcv.key.tryReadLock() == -1 )
			{
				cout << "\n  Received data: Busy updating " << "        "<< std::endl << std::flush;
			}    
			else
			{
				if( myRcv.command != NULL )
				{
					ch = myRcv.command;
					ManualControl( ch );
					myRcv.key.unlock();
					myRcv.key.writeLock();
					myRcv.command=NULL;
					myRcv.key.unlock();
				}
				else
				{
					myRcv.key.unlock();
				}
			}			

			// Translate to eliminate the offset.
			if ( ( auto_mode_start) & ( rotation_start1 ) )
			{				
				rotation_start1 = false;
				rotation_start2 = true;
			}

			// Angular motion.
			if ( ( auto_mode_start) & ( rotation_start2 ) )	// rotation	
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
			if ( ( auto_mode_start ) & ( !rotation_start2 ) & ( !rotation_start1 ) ) // translation
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

			// Grab the object.
			if ( grab_in_progress )
			{
				if ( ( cbox == CARTESIAN ) && ( pos[6] < -25000.0f ) )
				{
					SendCommand( CAN, FSR, FSR_END, ' ' );
					ResetAll();
					continue;
				}
			}

			// Open the grabber.
			if ( open_in_progress )
			{
				if ( ( cbox == CARTESIAN ) && ( pos[6] > 14000.0f ) )
				{
					SleepMs( 500 );
					SendCommand( CAN, FSR, FSR_RESET, ' ' );
					open_in_progress = false;
					continue;
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
				if ( auto_mode_start == false )
					new_status = false;
				SleepMs( 30 );
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
			cbox = CARTESIAN;

			if ( ( job_complete == true ) & ( job_complete2 == false ) )		// translation, approach, retreat
				job_done = job_complete2;
			else if ( ( job_complete == false ) & ( job_complete2 == true ) )	// rotation			
				job_done = job_complete;
			else if ( home_pos_flag == true )
				job_done = false;

			SendCommand( CAN, GUI, UPDATE_IN_MOTION, 1 );

			while ( !job_done )	// KIM - open-loop & closed-loop
            {                
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
			for ( int i=0 ; i < 8 ; i++ )        
				speed[i] = 0;			
			ShowStatus("STAT: Job Complete\n");
			break;
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