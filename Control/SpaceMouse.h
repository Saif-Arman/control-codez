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

HDC          hdc;         /* Handle to Device Context used to draw on screen */
HWND         hWndMain;    /* Handle to Main Window */
SiHdl        devHdl;      /* Handle to 3D Mouse Device */
TCHAR devicename[100] = _T("");
using namespace WinMainParameters; //Used for calling windows api from Main()
#define MAX_LOADSTRING 100
// Global Variables (are (usually) evil; this code comes from the Visual C++ Win32 Application project template):
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);