#ifndef _FUNCTIONS_WZ_
#define _FUNCTIONS_WZ_

//=============================================================================
// System Includes
//=============================================================================
#include <conio.h>
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
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
#include "MANUS_define.h"
//#include "SpaceMouse.h"
//=============================================================================
// Macro Definitions, Global Variables and Sub function
//=============================================================================
#include "Macro.h"
#include "Global.h"

//=============================================================================
// Maths functions.
//=============================================================================
inline int sign( int n ) 
{ 
	return ( ( n >= 0 ) ? 1 : -1 ); 
}
inline float sign(float n) 
{ 
	return ( ( n >= 0 ) ? 1.0f : -1.0f ); 
}
Matrix<4,4> GWj_Transformation( float *joint );
Matrix<3,3> C2W_transform( float *position );
Matrix<3,3> C2W_transform2( float *position );
Matrix<3, 3> EE2w_transform3(float *position);//zc test
Matrix<3, 3> EE2w_transform2(float *position);//zc test
Matrix<3,3> EE2C_transform( float *position );
Matrix<3,3> EE2C_transform2( float *position );
Matrix<3,3> NormalizeRotationMatrix( Matrix<3,3> );

///////////
ColumnVector<3> inverse_kinematics_MANUS_q1q2q3(float x, float y, float z);
float dist3D_Segment_to_Segment(ColumnVector<3> P1, ColumnVector<3> P2, ColumnVector<3> P3, ColumnVector<3> P4);
ColumnVector<3> link3_1st_end_postion(float q1, float q2, float q3);
float DistanceBetween_Camera_Link3(float *position);

//=============================================================================
// Communication functions.
//=============================================================================
void sthread_CAN( void *arg );						// Communication thread.
void ClientSocketComm( unsigned char src, unsigned char dst, unsigned char comm, float* param );
													// Send status info through socket communication.
void SendCommand( const unsigned char , const unsigned char , const unsigned char , const unsigned char  );
													// Send commands to certain program.
void TTSSpeak( unsigned char );						// Send voice command to SPC.

//=============================================================================
// Information display.
//=============================================================================
void gotoxy( int x, int y );						// Go to certain position on the screen.
void Help( void );									// Display control keys.
void UpdatePos( const float *pos );					// Update the pos information in shared memory.
void UpdateSpaceMouse(const int *spacemouse);
void DisplayPos( float* );							// Display current position.
void DisplaySpeed( void );							// Display speed information.
void ShowCommand( char *str );						// Show current command.
void ShowStatus( char *str );						// Show arm status.
void ShowFrequency( int time );                     // Show control loop frequency.
void PrintStatus( void );							// Show box mode and check is error on the robot has occurred.

//=============================================================================
// Manus arm control.
//=============================================================================
void ResetAll();									// Reset all variables.
bool Read_D4( void );								// Read first 0x0D4 and 0x4D4
bool Read_350_360( void );							// Read first 0x350 and 0x360.
bool Read_37F( void );								// Read first 0x37F.
bool Open_Grabber( void );							// Open the grabber.
void ManualControl( char ch );						// Set the variable speed that later will be used to set message data.
void Decode( TPCANMsg& rcvMsg, TPCANMsg& xmitMsg ); // Decode the information in the packages sent by the Manus.
void SetTransmitMessage( TPCANMsg& xmitMsg );		// Set the message to be transmitted into the buffer.
int pd_control2( void );
int pd_controlJoint( void );// A simple p control given the desired rotation of end effector.
int checkOneSecond(void);
int pd_controlx( void );							//

//=============================================================================
// Miscellaneous.
//=============================================================================
void adjust_queue_index( int seg );					// Adjust the order in the command queue.
int TimeCheck( void );								// Time check functions.
bool LoadAll( void );								// Load dll, connect to shared memory and TCP.
bool UnloadAll( void );								// Unload dll of CAN card, disconnect to shared memory and TCP.
char reliable(int num,char val);					//function meant to make the robot less reliable
void ReadForce(float cur_for);						//function to read force data from shared memory
void ReadLPS(int *LPS_value);						//function to read LPS data from shared memory
//void ReadSlip(double cur_vel);						//function to read slip sensor velocity data from shared memory
void ReadPosit();						//function to read slip sensor position data from shared memory
void ReadTaKK(void);					//function to read takktile sensor data
void GraspController(void);							//Controller for slip detection 
void LowPassFilter( void );							//Lowpass filter for velocity data
void LowPassFilter2( void );						//Lowpass filter for position data
void ReadVel( void );                               //calculate the velocity from position data
void LPScheck(void);
void SendCommand2(unsigned char target, unsigned char command, unsigned char snd_data);
void ReadSugspeed(void);
void Readblock_dir(void);
void oneclick(void);
void regrasp(void);
void init_grasp(void);
void ReadOBJ(void);
void suggest_btn(float *ee_deltaPosition);
void suggest_btn2(float ee_deltaPosition[13], int ee);
void movetopos(void);
int viewcheck(float Position[6], int axis, float offset, int ee, bool small_bound, float offset0[6]);
bool cam_cls_check(float Position[6], int axis, float offset, int ee, bool roll_correction);
float cam_cls_check2(float Position[6], int axis, float offset, int ee, bool roll_correction);
void Operation_check(void);
void block_camcls_move(void);
#endif




//-------------------------------------------------------------------------
//SpaceMouse
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------
//SpaceMouse
//-------------------------------------------------------------------------
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
#ifndef WINMAINPARAMETERS_H
#define WINMAINPARAMETERS_H

// Exclude the min and max macros from Windows.h
//#define NOMINMAX
//#include <Windows.h>
//#include <cassert>

// This namespace contains functions designed to return the parameters that would be
// passed in to the WinMain function in a UNICODE program.
namespace WinMainParameters {

	// Returns the value that would be passed to the first wWinMain HINSTANCE parameter or
	// null if an error was encountered.
	// If this function returns null, call GetLastError to get more information.
	inline HINSTANCE GetHInstance() {
		// Passing null to GetModuleHandle returns the HMODULE of
		// the file used to create the calling process. This is the same value
		// as the hInstance passed in to WinMain. The main use of this value
		// is by RegisterClassEx, which uses it to get the full address of the
		// user's WndProc.
		return static_cast<HINSTANCE>(::GetModuleHandleW(nullptr));
	}

	// Returns the value that would be passed to the second wWinMain HINSTANCE parameter.
	// This function always returns null as per the WinMain documentation.
	inline HINSTANCE GetHPrevInstance() {
		return static_cast<HINSTANCE>(nullptr);
	}

	// Returns the value that would be passed to the wWinMain LPWSTR parameter. If there
	// are no command line parameters, this returns a valid pointer to a null terminator
	// character (i.e. an empty string).
	// Note: The caller must not free the returned value. Attempting to free it will cause undefined
	// behavior.
	inline LPWSTR GetLPCmdLine() {
		// The first argument is the program name. To allow it to have spaces, it can be surrounded by
		// quotes. We must track if the first argument is quoted since a space is also used to separate
		// each parameter.
		bool isQuoted = false;
		const wchar_t space = L' ';
		const wchar_t quote = L'\"';
		const wchar_t nullTerminator = L'\0';

		LPWSTR lpCmdLine = ::GetCommandLineW();
		assert(lpCmdLine != nullptr);

		// The lpCmdLine in a WinMain is the command line as a string excluding the program name.
		// Program names can be quoted to allow for space characters so we need to deal with that.
		while (*lpCmdLine <= space || isQuoted) {
			if (*lpCmdLine == quote) {
				isQuoted = !isQuoted;
			}
			lpCmdLine++;
		}

		// Get past any additional whitespace between the end of the program name and the beginning
		// of the first parameter (if any). If we reach a null terminator we are done (i.e. there are
		// no arguments and the pointer itself is still properly valid).
		while (*lpCmdLine <= space && *lpCmdLine != nullTerminator) {
			lpCmdLine++;
		}

		// This will now be a valid pointer to either a null terminator or to the first character of
		// the first command line parameter after the program name.
		return lpCmdLine;
	}

	// Returns the value that would be passed to the wWinMain int parameter.
	inline int GetNCmdShow() {
		// It's possible that the process was started with STARTUPINFOW that could have a value for
		// show window other than SW_SHOWDEFAULT. If so we retrieve and return that value. Otherwise
		// we return SW_SHOWDEFAULT.
		::STARTUPINFOW startupInfo;
		::GetStartupInfoW(&startupInfo);
		if ((startupInfo.dwFlags & STARTF_USESHOWWINDOW) != 0) {
			return startupInfo.wShowWindow;
		}
		return SW_SHOWDEFAULT;
	}
}

#endif

static char SbTestCvsId[]="(C) 1998-2015 3Dconnexion: $Id: 3DxTEST32.H 13020 2016-05-25 09:51:06Z jwick $";
/* Global Variables */

//#ifndef SBTEST32_H
//#define SBTEST32_H
//
///* Functions --See cpp file for additional details */
//
LRESULT  WINAPI HandleNTEvent (HWND hWnd, unsigned msg, WPARAM wParam, LPARAM lParam);
int   DispatchLoopNT(); 
void  CreateSPWindow(int, int, int, int, TCHAR *);
int   SbInit();
void  SbMotionEvent(SiSpwEvent *pEvent);
void  SbZeroEvent();
void  SbButtonPressEvent(int buttonnumber);
void  SbButtonReleaseEvent(int buttonnumber);
void  HandleDeviceChangeEvent(SiSpwEvent *pEvent);
void  HandleV3DCMDEvent(SiSpwEvent *pEvent);
void  HandleAppEvent(SiSpwEvent *pEvent);
//#endif


//=============================================================================
// Space Mouse
//=============================================================================

