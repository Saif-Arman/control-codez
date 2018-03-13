#ifndef virtualkeys_HPP_INCLUDED_
#define virtualkeys_HPP_INCLUDED_
// virtualkeys.hpp
/*
 * Copyright notice:
 * Copyright (c) 2010-2015 3Dconnexion. All rights reserved. 
 * 
 * This file and source code are an integral part of the "3Dconnexion
 * Software Developer Kit", including all accompanying documentation,
 * and is protected by intellectual property laws. All use of the
 * 3Dconnexion Software Developer Kit is subject to the License
 * Agreement found in the "LicenseAgreementSDK.txt" file.
 * All rights not expressly granted by 3Dconnexion are reserved.
 */
///////////////////////////////////////////////////////////////////////////////////
// History
//
// $Id: virtualkeys.hpp 13046 2016-06-02 08:37:50Z jwick $
//
// 22.10.12 MSB Fix: Number of keys returned for SpacePilot Pro is incorrect (27)
// 19.06.12 MSB Added SM Touch and Generic 2 Button SM
// 09.03.12 MSB Fix VirtualKeyToHid not correctly converting SpaceMousePro buttons >V3DK_3
// 03.02.12 MSB Changed the labels of the "programmable buttons" back to "1" etc
// 22.09.11 MSB Added V3DK_USER above which the users may add their own virtual keys
// 02.08.11 MSB Added pid for Viking
//              Added virtualkey / hid definition for Viking
//              Added member to the tag_VirtualKeys struct for the number of
//              buttons on the device
//              Added methods to retrieve the number of buttons on a device
//              Added methods to map the hid buttons numbers to a consecutive
//              sequence (and back again)
// 11.03.11 MSB Fix incorrect label for V3DK_ROLL_CCW
// 09.03.11 MSB Added methods to return the labels of the keys on the device
// 19.10.10 MSB Moved the standard 3dmouse virtual buttons to the s3dm namespace
// 28.09.10 MSB Added spin and tilt buttons
//              Added structure to convert virtual key number to string identifier
// 04.12.09 MSB Fix spelling mistake 'panzoon'
//

#define _TRACE_VIRTUAL_KEYS 0


#if !defined(numberof)
#define numberof(_x) (sizeof(_x)/sizeof(_x[0]))
#endif

namespace s3dm {

   // This enum comes from trunk/inc/V3DKey.h. They must remain synced.
   enum e3dmouse_virtual_key 
   {
   V3DK_INVALID     = 0,
   V3DK_MENU		= 1,
   V3DK_FIT			= 2,
   V3DK_TOP			= 3,
   V3DK_LEFT		= 4,
   V3DK_RIGHT		= 5,
   V3DK_FRONT		= 6,
   V3DK_BOTTOM		= 7,
   V3DK_BACK		= 8,
   V3DK_ROLL_CW		= 9,
   V3DK_ROLL_CCW	= 10,
   V3DK_ISO1		= 11,
   V3DK_ISO2		= 12,
   V3DK_1			= 13,
   V3DK_2			= 14,
   V3DK_3			= 15,
   V3DK_4			= 16,
   V3DK_5			= 17,
   V3DK_6			= 18,
   V3DK_7			= 19,
   V3DK_8			= 20,
   V3DK_9			= 21,
   V3DK_10			= 22,
   V3DK_ESC			= 23,
   V3DK_ALT			= 24,
   V3DK_SHIFT		= 25,
   V3DK_CTRL		= 26,
   V3DK_ROTATE		= 27,
   V3DK_PANZOOM		= 28,
   V3DK_DOMINANT	= 29,
   V3DK_PLUS		= 30,
   V3DK_MINUS		= 31,
   V3DK_SPIN_CW		= 32,
   V3DK_SPIN_CCW	= 33,
   V3DK_TILT_CW		= 34,
   V3DK_TILT_CCW	= 35,
   V3DK_ENTER		= 36,
   V3DK_DELETE		= 37,
   V3DK_RESERVED0	= 38,
   V3DK_RESERVED1	= 39,
   V3DK_RESERVED2	= 40,
   V3DK_F1			= 41,
   V3DK_F2			= 42,
   V3DK_F3			= 43,
   V3DK_F4			= 44,
   V3DK_F5			= 45,
   V3DK_F6			= 46,
   V3DK_F7			= 47,
   V3DK_F8			= 48,
   V3DK_F9			= 49,
   V3DK_F10			= 50,
   V3DK_F11			= 51,
   V3DK_F12			= 52,
   V3DK_F13			= 53,
   V3DK_F14			= 54,
   V3DK_F15			= 55,
   V3DK_F16			= 56,
   V3DK_F17			= 57,
   V3DK_F18			= 58,
   V3DK_F19			= 59,
   V3DK_F20			= 60,
   V3DK_F21			= 61,
   V3DK_F22			= 62,
   V3DK_F23			= 63,
   V3DK_F24			= 64,
   V3DK_F25			= 65,
   V3DK_F26			= 66,
   V3DK_F27			= 67,
   V3DK_F28			= 68,
   V3DK_F29			= 69,
   V3DK_F30			= 70,
   V3DK_F31			= 71,
   V3DK_F32			= 72,
   V3DK_F33			= 73,
   V3DK_F34			= 74,
   V3DK_F35			= 75,
   V3DK_F36			= 76,
   V3DK_11			= 77,
   V3DK_12			= 78,
   V3DK_13			= 79,
   V3DK_14			= 80,
   V3DK_15			= 81,
   V3DK_16			= 82,
   V3DK_17			= 83,
   V3DK_18			= 84,
   V3DK_19			= 85,
   V3DK_20			= 86,
   V3DK_21			= 87,
   V3DK_22			= 88,
   V3DK_23			= 89,
   V3DK_24			= 90,
   V3DK_25			= 91,
   V3DK_26			= 92,
   V3DK_27			= 93,
   V3DK_28			= 94,
   V3DK_29			= 95,
   V3DK_30			= 96,
   V3DK_31			= 97,
   V3DK_32			= 98,
   V3DK_33			= 99,
   V3DK_34			= 100,
   V3DK_35			= 101,
   V3DK_36			= 102,
   V3DK_VIEW_1		= 103,
   V3DK_VIEW_2		= 104,
   V3DK_VIEW_3		= 105,
   V3DK_VIEW_4		= 106,
   V3DK_VIEW_5		= 107,
   V3DK_VIEW_6		= 108,
   V3DK_VIEW_7		= 109,
   V3DK_VIEW_8		= 110,
   V3DK_VIEW_9		= 111,
   V3DK_VIEW_10		= 112,
   V3DK_VIEW_11		= 113,
   V3DK_VIEW_12		= 114,
   V3DK_VIEW_13		= 115,
   V3DK_VIEW_14		= 116,
   V3DK_VIEW_15		= 117,
   V3DK_VIEW_16		= 118,
   V3DK_VIEW_17		= 119,
   V3DK_VIEW_18		= 120,
   V3DK_VIEW_19		= 121,
   V3DK_VIEW_20		= 122,
   V3DK_VIEW_21		= 123,
   V3DK_VIEW_22		= 124,
   V3DK_VIEW_23		= 125,
   V3DK_VIEW_24		= 126,
   V3DK_VIEW_25		= 127,
   V3DK_VIEW_26		= 128,
   V3DK_VIEW_27		= 129,
   V3DK_VIEW_28		= 130,
   V3DK_VIEW_29		= 131,
   V3DK_VIEW_30		= 132,
   V3DK_VIEW_31		= 133,
   V3DK_VIEW_32		= 134,
   V3DK_VIEW_33		= 135,
   V3DK_VIEW_34		= 136,
   V3DK_VIEW_35		= 137,
   V3DK_VIEW_36		= 138,
   V3DK_SAVE_VIEW_1	= 139,
   V3DK_SAVE_VIEW_2	= 140,
   V3DK_SAVE_VIEW_3	= 141,
   V3DK_SAVE_VIEW_4	= 142,
   V3DK_SAVE_VIEW_5	= 143,
   V3DK_SAVE_VIEW_6	= 144,
   V3DK_SAVE_VIEW_7	= 145,
   V3DK_SAVE_VIEW_8	= 146,
   V3DK_SAVE_VIEW_9	= 147,
   V3DK_SAVE_VIEW_10= 148,
   V3DK_SAVE_VIEW_11= 149,
   V3DK_SAVE_VIEW_12= 150,
   V3DK_SAVE_VIEW_13= 151,
   V3DK_SAVE_VIEW_14= 152,
   V3DK_SAVE_VIEW_15= 153,
   V3DK_SAVE_VIEW_16= 154,
   V3DK_SAVE_VIEW_17= 155,
   V3DK_SAVE_VIEW_18= 156,
   V3DK_SAVE_VIEW_19= 157,
   V3DK_SAVE_VIEW_20= 158,
   V3DK_SAVE_VIEW_21= 159,
   V3DK_SAVE_VIEW_22= 160,
   V3DK_SAVE_VIEW_23= 161,
   V3DK_SAVE_VIEW_24= 162,
   V3DK_SAVE_VIEW_25= 163,
   V3DK_SAVE_VIEW_26= 164,
   V3DK_SAVE_VIEW_27= 165,
   V3DK_SAVE_VIEW_28= 166,
   V3DK_SAVE_VIEW_29= 167,
   V3DK_SAVE_VIEW_30= 168,
   V3DK_SAVE_VIEW_31= 169,
   V3DK_SAVE_VIEW_32= 170,
   V3DK_SAVE_VIEW_33= 171,
   V3DK_SAVE_VIEW_34= 172,
   V3DK_SAVE_VIEW_35= 173,
   V3DK_SAVE_VIEW_36= 174,
   V3DK_TAB			= 175,
   V3DK_SPACE		= 176,
   V3DK_MENU_1      = 177,
   V3DK_MENU_2      = 178,
   V3DK_MENU_3      = 179,
   V3DK_MENU_4      = 180,
   V3DK_MENU_5      = 181,
   V3DK_MENU_6      = 182,
   V3DK_MENU_7      = 183,
   V3DK_MENU_8      = 184,
   V3DK_MENU_9      = 185,
   V3DK_MENU_10     = 186,
   V3DK_MENU_11     = 187,
   V3DK_MENU_12     = 188,
   V3DK_MENU_13     = 189,
   V3DK_MENU_14     = 190,
   V3DK_MENU_15     = 191,
   V3DK_MENU_16     = 192,
   /* add more here as needed - don't change value of anything that may already be used */
   V3DK_USER        = 0x10000
   };

   static const e3dmouse_virtual_key VirtualKeys[]=
   {
      V3DK_MENU, V3DK_FIT
      , V3DK_TOP, V3DK_LEFT, V3DK_RIGHT, V3DK_FRONT, V3DK_BOTTOM, V3DK_BACK
      , V3DK_ROLL_CW, V3DK_ROLL_CCW
      , V3DK_ISO1, V3DK_ISO2
      , V3DK_1, V3DK_2, V3DK_3, V3DK_4, V3DK_5, V3DK_6, V3DK_7, V3DK_8, V3DK_9, V3DK_10
      , V3DK_ESC, V3DK_ALT, V3DK_SHIFT, V3DK_CTRL
      , V3DK_ROTATE, V3DK_PANZOOM, V3DK_DOMINANT
      , V3DK_PLUS, V3DK_MINUS
      , V3DK_SPIN_CW, V3DK_SPIN_CCW
      , V3DK_TILT_CW, V3DK_TILT_CCW
   };

   static const size_t VirtualKeyCount = numberof(VirtualKeys);
   static const size_t MaxKeyCount = 256; // Arbitary number for sanity checks

   struct tag_VirtualKeyLabel
   {
     e3dmouse_virtual_key vkey;
     const char* szLabel;
   };

   static const struct tag_VirtualKeyLabel VirtualKeyLabel[] =
   {
     {V3DK_MENU, "MENU"} , {V3DK_FIT, "FIT"}
     , {V3DK_TOP, "T"}, {V3DK_LEFT, "L"}, {V3DK_RIGHT, "R"}, {V3DK_FRONT, "F"}, {V3DK_BOTTOM, "B"}, {V3DK_BACK, "BK"}
     , {V3DK_ROLL_CW, "Roll +"}, {V3DK_ROLL_CCW, "Roll -"}
     , {V3DK_ISO1, "ISO1"}, {V3DK_ISO2, "ISO2"}
     , {V3DK_1, "1"}, {V3DK_2, "2"}, {V3DK_3, "3"}, {V3DK_4, "4"}, {V3DK_5, "5"}
     , {V3DK_6, "6"}, {V3DK_7, "7"}, {V3DK_8, "8"}, {V3DK_9, "9"}, {V3DK_10, "10"}
     , {V3DK_ESC, "ESC"}, {V3DK_ALT, "ALT"}, {V3DK_SHIFT, "SHIFT"}, {V3DK_CTRL, "CTRL"}
     , {V3DK_ROTATE, "Rotate"}, {V3DK_PANZOOM, "Pan Zoom"}, {V3DK_DOMINANT, "Dom"}
     , {V3DK_PLUS, "+"}, {V3DK_MINUS, "-"}
     , {V3DK_SPIN_CW, "Spin +"}, {V3DK_SPIN_CCW, "Spin -"}
     , {V3DK_TILT_CW, "Tilt +"}, {V3DK_TILT_CCW, "Tilt -"}
     , {V3DK_ENTER, "Enter"}, {V3DK_DELETE, "Delete"}
     , {V3DK_RESERVED0, "Reserved0"}, {V3DK_RESERVED1, "Reserved1"}, {V3DK_RESERVED2, "Reserved2"}
     , {V3DK_F1, "F1"}, {V3DK_F2, "F2"}, {V3DK_F3, "F3"}, {V3DK_F4, "F4"}, {V3DK_F5, "F5"}
     , {V3DK_F6, "F6"}, {V3DK_F7, "F7"}, {V3DK_F8, "F8"}, {V3DK_F9, "F9"}, {V3DK_F10, "F10"}
     , {V3DK_F11, "F11"}, {V3DK_F12, "F12"}, {V3DK_F13, "F13"}, {V3DK_F14, "F14"}, {V3DK_F15, "F15"}
     , {V3DK_F16, "F16"}, {V3DK_F17, "F17"}, {V3DK_F18, "F18"}, {V3DK_F19, "F19"}, {V3DK_F20, "F20"}
     , {V3DK_F21, "F21"}, {V3DK_F22, "F22"}, {V3DK_F23, "F23"}, {V3DK_F24, "F24"}, {V3DK_F25, "F25"}
     , {V3DK_F26, "F26"}, {V3DK_F27, "F27"}, {V3DK_F28, "F28"}, {V3DK_F29, "F29"}, {V3DK_F30, "F30"}
     , {V3DK_F31, "F31"}, {V3DK_F32, "F32"}, {V3DK_F33, "F33"}, {V3DK_F34, "F34"}, {V3DK_F35, "F35"}
     , {V3DK_F36, "F36"}
     , {V3DK_11, "11"}, {V3DK_12, "12"}, {V3DK_13, "13"}, {V3DK_14, "14"}, {V3DK_15, "15"}
     , {V3DK_16, "16"}, {V3DK_17, "17"}, {V3DK_18, "18"}, {V3DK_19, "19"}, {V3DK_20, "20"}
     , {V3DK_21, "21"}, {V3DK_22, "22"}, {V3DK_23, "23"}, {V3DK_24, "24"}, {V3DK_25, "25"}
     , {V3DK_26, "26"}, {V3DK_27, "27"}, {V3DK_28, "28"}, {V3DK_29, "29"}, {V3DK_30, "30"}
     , {V3DK_31, "31"}, {V3DK_32, "32"}, {V3DK_33, "33"}, {V3DK_34, "34"}, {V3DK_35, "35"}
     , {V3DK_36, "36"}
     , {V3DK_VIEW_1, "VIEW 1"}, {V3DK_VIEW_2, "VIEW 2"}, {V3DK_VIEW_3, "VIEW 3"}, {V3DK_VIEW_4, "VIEW 4"}, {V3DK_VIEW_5, "VIEW 5"}
     , {V3DK_VIEW_6, "VIEW 6"}, {V3DK_VIEW_7, "VIEW 7"}, {V3DK_VIEW_8, "VIEW 8"}, {V3DK_VIEW_9, "VIEW 9"}, {V3DK_VIEW_10, "VIEW 10"}
     , {V3DK_VIEW_11, "VIEW 11"}, {V3DK_VIEW_12, "VIEW 12"}, {V3DK_VIEW_13, "VIEW 13"}, {V3DK_VIEW_14, "VIEW 14"}, {V3DK_VIEW_15, "VIEW 15"}
     , {V3DK_VIEW_16, "VIEW 16"}, {V3DK_VIEW_17, "VIEW 17"}, {V3DK_VIEW_18, "VIEW 18"}, {V3DK_VIEW_19, "VIEW 19"}, {V3DK_VIEW_20, "VIEW 20"}
     , {V3DK_VIEW_21, "VIEW 21"}, {V3DK_VIEW_22, "VIEW 22"}, {V3DK_VIEW_23, "VIEW 23"}, {V3DK_VIEW_24, "VIEW 24"}, {V3DK_VIEW_25, "VIEW 25"}
     , {V3DK_VIEW_26, "VIEW 26"}, {V3DK_VIEW_27, "VIEW 27"}, {V3DK_VIEW_28, "VIEW 28"}, {V3DK_VIEW_29, "VIEW 29"}, {V3DK_VIEW_30, "VIEW 30"}
     , {V3DK_VIEW_31, "VIEW 31"}, {V3DK_VIEW_32, "VIEW 32"}, {V3DK_VIEW_33, "VIEW 33"}, {V3DK_VIEW_34, "VIEW 34"}, {V3DK_VIEW_35, "VIEW 35"}
     , {V3DK_VIEW_36, "VIEW 36"}
     , {V3DK_SAVE_VIEW_1, "SAVE VIEW 1"}, {V3DK_SAVE_VIEW_2, "SAVE VIEW 2"}, {V3DK_SAVE_VIEW_3, "SAVE VIEW 3"}, {V3DK_SAVE_VIEW_4, "SAVE VIEW 4"}, {V3DK_SAVE_VIEW_5, "SAVE VIEW 5"}
     , {V3DK_SAVE_VIEW_6, "SAVE VIEW 6"}, {V3DK_SAVE_VIEW_7, "SAVE VIEW 7"}, {V3DK_SAVE_VIEW_8, "SAVE VIEW 8"}, {V3DK_SAVE_VIEW_9, "SAVE VIEW 9"}, {V3DK_SAVE_VIEW_10, "SAVE VIEW 10"}
     , {V3DK_SAVE_VIEW_11, "SAVE VIEW 11"}, {V3DK_SAVE_VIEW_12, "SAVE VIEW 12"}, {V3DK_SAVE_VIEW_13, "SAVE VIEW 13"}, {V3DK_SAVE_VIEW_14, "SAVE VIEW 14"}, {V3DK_SAVE_VIEW_15, "SAVE VIEW 15"}
     , {V3DK_SAVE_VIEW_16, "VIEW 16"}, {V3DK_SAVE_VIEW_17, "VIEW 17"}, {V3DK_SAVE_VIEW_18, "VIEW 18"}, {V3DK_SAVE_VIEW_19, "VIEW 19"}, {V3DK_SAVE_VIEW_20, "VIEW 20"}
     , {V3DK_SAVE_VIEW_21, "VIEW 21"}, {V3DK_SAVE_VIEW_22, "VIEW 22"}, {V3DK_SAVE_VIEW_23, "VIEW 23"}, {V3DK_SAVE_VIEW_24, "VIEW 24"}, {V3DK_SAVE_VIEW_25, "VIEW 25"}
     , {V3DK_SAVE_VIEW_26, "VIEW 26"}, {V3DK_SAVE_VIEW_27, "VIEW 27"}, {V3DK_SAVE_VIEW_28, "VIEW 28"}, {V3DK_SAVE_VIEW_29, "VIEW 29"}, {V3DK_SAVE_VIEW_30, "VIEW 30"}
     , {V3DK_SAVE_VIEW_31, "VIEW 31"}, {V3DK_SAVE_VIEW_32, "VIEW 32"}, {V3DK_SAVE_VIEW_33, "VIEW 33"}, {V3DK_SAVE_VIEW_34, "VIEW 34"}, {V3DK_SAVE_VIEW_35, "VIEW 35"}
     , {V3DK_SAVE_VIEW_36, "VIEW 36"}
     , {V3DK_TAB, "Tab"}, {V3DK_SPACE, "Space"}
     , {V3DK_MENU_1, "MENU 1"}, {V3DK_MENU_2, "MENU 2"}, {V3DK_MENU_3, "MENU 3"}, {V3DK_MENU_4, "MENU 4"}
	 , {V3DK_MENU_5, "MENU 5"}, {V3DK_MENU_6, "MENU 6"}, {V3DK_MENU_7, "MENU 7"}, {V3DK_MENU_8, "MENU 8"}
     , {V3DK_MENU_9, "MENU 9"}, {V3DK_MENU_10, "MENU 10"}, {V3DK_MENU_11, "MENU 11"}, {V3DK_MENU_12, "MENU 12"}
	 , {V3DK_MENU_13, "MENU 13"}, {V3DK_MENU_14, "MENU 14"}, {V3DK_MENU_15, "MENU 15"}, {V3DK_MENU_16, "MENU 16"}
     , {V3DK_USER, "USER"}
   };

   struct tag_VirtualKeyId
   {
     e3dmouse_virtual_key vkey;
     const char* szId;
   };

   static const struct tag_VirtualKeyId VirtualKeyId[] =
   {
     {V3DK_INVALID, "V3DK_INVALID"}
     , {V3DK_MENU, "V3DK_MENU"} , {V3DK_FIT, "V3DK_FIT"}
     , {V3DK_TOP, "V3DK_TOP"}, {V3DK_LEFT, "V3DK_LEFT"}, {V3DK_RIGHT, "V3DK_RIGHT"}, {V3DK_FRONT, "V3DK_FRONT"}, {V3DK_BOTTOM, "V3DK_BOTTOM"}, {V3DK_BACK, "V3DK_BACK"}
     , {V3DK_ROLL_CW, "V3DK_ROLL_CW"}, {V3DK_ROLL_CCW, "V3DK_ROLL_CCW"}
     , {V3DK_ISO1, "V3DK_ISO1"}, {V3DK_ISO2, "V3DK_ISO2"}
     , {V3DK_1, "V3DK_1"}, {V3DK_2, "V3DK_2"}, {V3DK_3, "V3DK_3"}, {V3DK_4, "V3DK_4"}, {V3DK_5, "V3DK_5"}
     , {V3DK_6, "V3DK_6"}, {V3DK_7, "V3DK_7"}, {V3DK_8, "V3DK_8"}, {V3DK_9, "V3DK_9"}, {V3DK_10, "V3DK_10"}
     , {V3DK_ESC, "V3DK_ESC"}, {V3DK_ALT, "V3DK_ALT"}, {V3DK_SHIFT, "V3DK_SHIFT"}, {V3DK_CTRL, "V3DK_CTRL"}
     , {V3DK_ROTATE, "V3DK_ROTATE"}, {V3DK_PANZOOM, "V3DK_PANZOOM"}, {V3DK_DOMINANT, "V3DK_DOMINANT"}
     , {V3DK_PLUS, "V3DK_PLUS"}, {V3DK_MINUS, "V3DK_MINUS"}
     , {V3DK_SPIN_CW, "V3DK_SPIN_CW"}, {V3DK_SPIN_CCW, "V3DK_SPIN_CCW"}
     , {V3DK_TILT_CW, "V3DK_TILT_CW"}, {V3DK_TILT_CCW, "V3DK_TILT_CCW"}
     , {V3DK_ENTER, "V3DK_ENTER"}, {V3DK_DELETE, "V3DK_DELETE"}
     , {V3DK_RESERVED0, "V3DK_RESERVED0"}, {V3DK_RESERVED1, "V3DK_RESERVED1"}, {V3DK_RESERVED2, "V3DK_RESERVED2"}
     , {V3DK_F1, "V3DK_F1"}, {V3DK_F2, "V3DK_F2"}, {V3DK_F3, "V3DK_F3"}, {V3DK_F4, "V3DK_F4"}, {V3DK_F5, "V3DK_F5"}
     , {V3DK_F6, "V3DK_F6"}, {V3DK_F7, "V3DK_F7"}, {V3DK_F8, "V3DK_F8"}, {V3DK_F9, "V3DK_F9"}, {V3DK_F10, "V3DK_F10"}
     , {V3DK_F11, "V3DK_F11"}, {V3DK_F12, "V3DK_F12"}, {V3DK_F13, "V3DK_F13"}, {V3DK_F14, "V3DK_F14"}, {V3DK_F15, "V3DK_F15"}
     , {V3DK_F16, "V3DK_F16"}, {V3DK_F17, "V3DK_F17"}, {V3DK_F18, "V3DK_F18"}, {V3DK_F19, "V3DK_F19"}, {V3DK_F20, "V3DK_F20"}
     , {V3DK_F21, "V3DK_F21"}, {V3DK_F22, "V3DK_F22"}, {V3DK_F23, "V3DK_F23"}, {V3DK_F24, "V3DK_F24"}, {V3DK_F25, "V3DK_F25"}
     , {V3DK_F26, "V3DK_F26"}, {V3DK_F27, "V3DK_F27"}, {V3DK_F28, "V3DK_F28"}, {V3DK_F29, "V3DK_F29"}, {V3DK_F30, "V3DK_F30"}
     , {V3DK_F31, "V3DK_F31"}, {V3DK_F32, "V3DK_F32"}, {V3DK_F33, "V3DK_F33"}, {V3DK_F34, "V3DK_F34"}, {V3DK_F35, "V3DK_F35"}
     , {V3DK_F36, "V3DK_F36"}
     , {V3DK_11, "V3DK_11"}, {V3DK_12, "V3DK_12"}, {V3DK_13, "V3DK_13"}, {V3DK_14, "V3DK_14"}, {V3DK_15, "V3DK_15"}
     , {V3DK_16, "V3DK_16"}, {V3DK_17, "V3DK_17"}, {V3DK_18, "V3DK_18"}, {V3DK_19, "V3DK_19"}, {V3DK_20, "V3DK_20"}
     , {V3DK_21, "V3DK_21"}, {V3DK_22, "V3DK_22"}, {V3DK_23, "V3DK_23"}, {V3DK_24, "V3DK_24"}, {V3DK_25, "V3DK_25"}
     , {V3DK_26, "V3DK_26"}, {V3DK_27, "V3DK_27"}, {V3DK_28, "V3DK_28"}, {V3DK_29, "V3DK_29"}, {V3DK_30, "V3DK_30"}
     , {V3DK_31, "V3DK_31"}, {V3DK_32, "V3DK_32"}, {V3DK_33, "V3DK_33"}, {V3DK_34, "V3DK_34"}, {V3DK_35, "V3DK_35"}
     , {V3DK_36, "V3DK_36"}
     , {V3DK_VIEW_1, "V3DK_VIEW_1"}, {V3DK_VIEW_2, "V3DK_VIEW_2"}, {V3DK_VIEW_3, "V3DK_VIEW_3"}, {V3DK_VIEW_4, "V3DK_VIEW_4"}, {V3DK_VIEW_5, "V3DK_VIEW_5"}
     , {V3DK_VIEW_6, "V3DK_VIEW_6"}, {V3DK_VIEW_7, "V3DK_VIEW_7"}, {V3DK_VIEW_8, "V3DK_VIEW_8"}, {V3DK_VIEW_9, "V3DK_VIEW_9"}, {V3DK_VIEW_10, "V3DK_VIEW_10"}
     , {V3DK_VIEW_11, "V3DK_VIEW_11"}, {V3DK_VIEW_12, "V3DK_VIEW_12"}, {V3DK_VIEW_13, "V3DK_VIEW_13"}, {V3DK_VIEW_14, "V3DK_VIEW_14"}, {V3DK_VIEW_15, "V3DK_VIEW_15"}
     , {V3DK_VIEW_16, "V3DK_VIEW_16"}, {V3DK_VIEW_17, "V3DK_VIEW_17"}, {V3DK_VIEW_18, "V3DK_VIEW_18"}, {V3DK_VIEW_19, "V3DK_VIEW_19"}, {V3DK_VIEW_20, "V3DK_VIEW_20"}
     , {V3DK_VIEW_21, "V3DK_VIEW_21"}, {V3DK_VIEW_22, "V3DK_VIEW_22"}, {V3DK_VIEW_23, "V3DK_VIEW_23"}, {V3DK_VIEW_24, "V3DK_VIEW_24"}, {V3DK_VIEW_25, "V3DK_VIEW_25"}
     , {V3DK_VIEW_26, "V3DK_VIEW_26"}, {V3DK_VIEW_27, "V3DK_VIEW_27"}, {V3DK_VIEW_28, "V3DK_VIEW_28"}, {V3DK_VIEW_29, "V3DK_VIEW_29"}, {V3DK_VIEW_30, "V3DK_VIEW_30"}
     , {V3DK_VIEW_31, "V3DK_VIEW_31"}, {V3DK_VIEW_32, "V3DK_VIEW_32"}, {V3DK_VIEW_33, "V3DK_VIEW_33"}, {V3DK_VIEW_34, "V3DK_VIEW_34"}, {V3DK_VIEW_35, "V3DK_VIEW_35"}
     , {V3DK_VIEW_36, "V3DK_VIEW_36"}
     , {V3DK_SAVE_VIEW_1, "V3DK_SAVE_VIEW_1"}, {V3DK_SAVE_VIEW_2, "V3DK_SAVE_VIEW_2"}, {V3DK_SAVE_VIEW_3, "V3DK_SAVE_VIEW_3"}, {V3DK_SAVE_VIEW_4, "V3DK_SAVE_VIEW_4"}, {V3DK_SAVE_VIEW_5, "V3DK_SAVE_VIEW_5"}
     , {V3DK_SAVE_VIEW_6, "V3DK_SAVE_VIEW_6"}, {V3DK_SAVE_VIEW_7, "V3DK_SAVE_VIEW_7"}, {V3DK_SAVE_VIEW_8, "V3DK_SAVE_VIEW_8"}, {V3DK_SAVE_VIEW_9, "V3DK_SAVE_VIEW_9"}, {V3DK_SAVE_VIEW_10, "V3DK_SAVE_VIEW_10"}
     , {V3DK_SAVE_VIEW_11, "V3DK_SAVE_VIEW_11"}, {V3DK_SAVE_VIEW_12, "V3DK_SAVE_VIEW_12"}, {V3DK_SAVE_VIEW_13, "V3DK_SAVE_VIEW_13"}, {V3DK_SAVE_VIEW_14, "V3DK_SAVE_VIEW_14"}, {V3DK_SAVE_VIEW_15, "V3DK_SAVE_VIEW_15"}
     , {V3DK_SAVE_VIEW_16, "V3DK_SAVE_VIEW_16"}, {V3DK_SAVE_VIEW_17, "V3DK_SAVE_VIEW_17"}, {V3DK_SAVE_VIEW_18, "V3DK_SAVE_VIEW_18"}, {V3DK_SAVE_VIEW_19, "V3DK_SAVE_VIEW_19"}, {V3DK_SAVE_VIEW_20, "V3DK_SAVE_VIEW_20"}
     , {V3DK_SAVE_VIEW_21, "V3DK_SAVE_VIEW_21"}, {V3DK_SAVE_VIEW_22, "V3DK_SAVE_VIEW_22"}, {V3DK_SAVE_VIEW_23, "V3DK_SAVE_VIEW_23"}, {V3DK_SAVE_VIEW_24, "V3DK_SAVE_VIEW_24"}, {V3DK_SAVE_VIEW_25, "V3DK_SAVE_VIEW_25"}
     , {V3DK_SAVE_VIEW_26, "V3DK_SAVE_VIEW_26"}, {V3DK_SAVE_VIEW_27, "V3DK_SAVE_VIEW_27"}, {V3DK_SAVE_VIEW_28, "V3DK_SAVE_VIEW_28"}, {V3DK_SAVE_VIEW_29, "V3DK_SAVE_VIEW_29"}, {V3DK_SAVE_VIEW_30, "V3DK_SAVE_VIEW_30"}
     , {V3DK_SAVE_VIEW_31, "V3DK_SAVE_VIEW_31"}, {V3DK_SAVE_VIEW_32, "V3DK_SAVE_VIEW_32"}, {V3DK_SAVE_VIEW_33, "V3DK_SAVE_VIEW_33"}, {V3DK_SAVE_VIEW_34, "V3DK_SAVE_VIEW_34"}, {V3DK_SAVE_VIEW_35, "V3DK_SAVE_VIEW_35"}
     , {V3DK_SAVE_VIEW_36, "V3DK_SAVE_VIEW_36"}
     , {V3DK_TAB, "V3DK_TAB"}, {V3DK_SPACE, "V3DK_SPACE"}
     , {V3DK_MENU_1, "V3DK_MENU_1"}, {V3DK_MENU_2, "V3DK_MENU_2"}, {V3DK_MENU_3, "V3DK_MENU_3"}, {V3DK_MENU_4, "V3DK_MENU_4"}
	 , {V3DK_MENU_5, "V3DK_MENU_5"}, {V3DK_MENU_6, "V3DK_MENU_6"}, {V3DK_MENU_7, "V3DK_MENU_7"}, {V3DK_MENU_8, "V3DK_MENU_8"}
     , {V3DK_MENU_9, "V3DK_MENU_9"}, {V3DK_MENU_10, "V3DK_MENU_10"}, {V3DK_MENU_11, "V3DK_MENU_11"}, {V3DK_MENU_12, "V3DK_MENU_12"}
	 , {V3DK_MENU_13, "V3DK_MENU_13"}, {V3DK_MENU_14, "V3DK_MENU_14"}, {V3DK_MENU_15, "V3DK_MENU_15"}, {V3DK_MENU_16, "V3DK_MENU_16"}
     , {V3DK_USER, "V3DK_USER"}
   };

   /*-----------------------------------------------------------------------------
   *
   * const  wchar_t* VirtualKeyToId(e3dmouse_virtual_key virtualkey)
   *
   * Args:
   *    virtualkey  the 3dmouse virtual key 
   *
   * Return Value:
   *    Returns a string representation of the standard 3dmouse virtual key, or
   *    an empty string 
   *
   * Description:
   *    Converts a 3dmouse virtual key number to its string identifier
   *
   *---------------------------------------------------------------------------*/
   __inline const char* VirtualKeyToId(e3dmouse_virtual_key virtualkey)
   {
      if (0 < virtualkey && virtualkey <= numberof(VirtualKeyId) 
        && virtualkey == VirtualKeyId[virtualkey-1].vkey)
        return VirtualKeyId[virtualkey-1].szId;

      for (size_t i=0; i<numberof(VirtualKeyId); ++i)
      {
        if (VirtualKeyId[i].vkey == virtualkey)
          return VirtualKeyId[i].szId;
      }
      return "";
   }


   /*-----------------------------------------------------------------------------
   *
   * const e3dmouse_virtual_key IdToVirtualKey ( TCHAR *id )
   *
   * Args:
   *    id  - the 3dmouse virtual key ID (a string)
   *
   * Return Value:
   *    The virtual_key number for the id, 
   *    or V3DK_INVALID if it is not a valid tag_VirtualKeyId
   *
   * Description:
   *    Converts a 3dmouse virtual key id (a string) to the V3DK number
   *
   *---------------------------------------------------------------------------*/
   __inline e3dmouse_virtual_key IdToVirtualKey(const char* id)
   {
      for (size_t i=0; i<sizeof(VirtualKeyId)/sizeof(VirtualKeyId[0]); ++i)
      {
        if (_tcsicmp (VirtualKeyId[i].szId, id) == 0)
          return VirtualKeyId[i].vkey;
      }
      return V3DK_INVALID;
   }


   /*-----------------------------------------------------------------------------
   *
   * const wchar_t* GetKeyLabel(e3dmouse_virtual_key virtualkey)
   *
   * Args:
   *    virtualkey  the 3dmouse virtual key 
   *
   * Return Value:
   *    Returns a string of thye label used on the standard 3dmouse virtual key, or
   *    an empty string 
   *
   * Description:
   *    Converts a 3dmouse virtual key number to its label
   *
   *---------------------------------------------------------------------------*/
   __inline const char* GetKeyLabel(e3dmouse_virtual_key virtualkey)
   {
      for (size_t i=0; i<numberof(VirtualKeyLabel); ++i)
      {
        if (VirtualKeyLabel[i].vkey == virtualkey)
          return VirtualKeyLabel[i].szLabel;
      }
      return "";
   }

} // namespace s3dm

namespace tdx {
   enum e3dconnexion_pid {
      eSpacePilot = 0xc625
      , eSpaceNavigator = 0xc626
      , eSpaceExplorer = 0xc627
      , eSpaceNavigatorForNotebooks = 0xc628
      , eSpacePilotPRO = 0xc629
      , eSpaceMousePRO = 0xc62b
      , eSpaceMouseTouch = 0xc62c
      , eSpaceMouse = 0xc62d
      , eSpaceMouseEnterprise = 0xc633
   };


   struct tag_VirtualKeys
   {
      e3dconnexion_pid pid;
      size_t nLength;
      s3dm::e3dmouse_virtual_key *vkeys;
      size_t nKeys;
   };

   static const s3dm::e3dmouse_virtual_key SpaceExplorerKeys [] = 
   {
      s3dm::V3DK_INVALID     // there is no button 0
      , s3dm::V3DK_1, s3dm::V3DK_2
      , s3dm::V3DK_TOP, s3dm::V3DK_LEFT, s3dm::V3DK_RIGHT, s3dm::V3DK_FRONT
      , s3dm::V3DK_ESC, s3dm::V3DK_ALT, s3dm::V3DK_SHIFT, s3dm::V3DK_CTRL
      , s3dm::V3DK_FIT, s3dm::V3DK_MENU
      , s3dm::V3DK_PLUS, s3dm::V3DK_MINUS
      , s3dm::V3DK_ROTATE
   };

   static const s3dm::e3dmouse_virtual_key SpacePilotKeys [] = 
   {
      s3dm::V3DK_INVALID 
      , s3dm::V3DK_1, s3dm::V3DK_2, s3dm::V3DK_3, s3dm::V3DK_4, s3dm::V3DK_5, s3dm::V3DK_6
      , s3dm::V3DK_TOP, s3dm::V3DK_LEFT, s3dm::V3DK_RIGHT, s3dm::V3DK_FRONT
      , s3dm::V3DK_ESC, s3dm::V3DK_ALT, s3dm::V3DK_SHIFT, s3dm::V3DK_CTRL
      , s3dm::V3DK_FIT, s3dm::V3DK_MENU
      , s3dm::V3DK_PLUS, s3dm::V3DK_MINUS
      , s3dm::V3DK_DOMINANT, s3dm::V3DK_ROTATE
      , static_cast<s3dm::e3dmouse_virtual_key>(s3dm::V3DK_USER+0x01)
   };

   static const s3dm::e3dmouse_virtual_key SpaceMouseKeys [] = 
   {
      s3dm::V3DK_INVALID 
      , s3dm::V3DK_MENU, s3dm::V3DK_FIT
   };

   static const s3dm::e3dmouse_virtual_key SpacePilotProKeys [] = 
   {
      s3dm::V3DK_INVALID 
      , s3dm::V3DK_MENU, s3dm::V3DK_FIT
      , s3dm::V3DK_TOP, s3dm::V3DK_LEFT, s3dm::V3DK_RIGHT, s3dm::V3DK_FRONT, s3dm::V3DK_BOTTOM, s3dm::V3DK_BACK
      , s3dm::V3DK_ROLL_CW, s3dm::V3DK_ROLL_CCW
      , s3dm::V3DK_ISO1, s3dm::V3DK_ISO2
      , s3dm::V3DK_1, s3dm::V3DK_2, s3dm::V3DK_3, s3dm::V3DK_4, s3dm::V3DK_5
      , s3dm::V3DK_6, s3dm::V3DK_7, s3dm::V3DK_8, s3dm::V3DK_9, s3dm::V3DK_10
      , s3dm::V3DK_ESC, s3dm::V3DK_ALT, s3dm::V3DK_SHIFT, s3dm::V3DK_CTRL
      , s3dm::V3DK_ROTATE, s3dm::V3DK_PANZOOM, s3dm::V3DK_DOMINANT
      , s3dm::V3DK_PLUS, s3dm::V3DK_MINUS
   };

   static const s3dm::e3dmouse_virtual_key SpaceMouseProKeys [] = 
   {
      s3dm::V3DK_INVALID 
      , s3dm::V3DK_MENU, s3dm::V3DK_FIT
      , s3dm::V3DK_TOP, s3dm::V3DK_INVALID, s3dm::V3DK_RIGHT, s3dm::V3DK_FRONT, s3dm::V3DK_INVALID, s3dm::V3DK_INVALID
      , s3dm::V3DK_ROLL_CW, s3dm::V3DK_INVALID
      , s3dm::V3DK_INVALID, s3dm::V3DK_INVALID
      , s3dm::V3DK_1, s3dm::V3DK_2, s3dm::V3DK_3, s3dm::V3DK_4, s3dm::V3DK_INVALID
      , s3dm::V3DK_INVALID, s3dm::V3DK_INVALID, s3dm::V3DK_INVALID, s3dm::V3DK_INVALID, s3dm::V3DK_INVALID
      , s3dm::V3DK_ESC, s3dm::V3DK_ALT, s3dm::V3DK_SHIFT, s3dm::V3DK_CTRL
      , s3dm::V3DK_ROTATE
   };

   static const s3dm::e3dmouse_virtual_key SpaceMouseTouchKeys [] = 
   {
      s3dm::V3DK_INVALID 
      , s3dm::V3DK_MENU, s3dm::V3DK_FIT
      , s3dm::V3DK_TOP, s3dm::V3DK_LEFT, s3dm::V3DK_RIGHT, s3dm::V3DK_FRONT, s3dm::V3DK_BOTTOM, s3dm::V3DK_BACK
      , s3dm::V3DK_ROLL_CW, s3dm::V3DK_ROLL_CCW
      , s3dm::V3DK_ISO1, s3dm::V3DK_ISO2
      , s3dm::V3DK_1, s3dm::V3DK_2, s3dm::V3DK_3, s3dm::V3DK_4, s3dm::V3DK_5
      , s3dm::V3DK_6, s3dm::V3DK_7, s3dm::V3DK_8, s3dm::V3DK_9, s3dm::V3DK_10
   };

   static const s3dm::e3dmouse_virtual_key SpaceMouseEnterpriseKeys [] = 
   {
      s3dm::V3DK_INVALID 
      , s3dm::V3DK_MENU, s3dm::V3DK_FIT
      , s3dm::V3DK_TOP, s3dm::V3DK_LEFT, s3dm::V3DK_RIGHT, s3dm::V3DK_FRONT, s3dm::V3DK_BOTTOM, s3dm::V3DK_BACK
      , s3dm::V3DK_ROLL_CW, s3dm::V3DK_ROLL_CCW
      , s3dm::V3DK_ISO1, s3dm::V3DK_ISO2
      , s3dm::V3DK_1, s3dm::V3DK_2, s3dm::V3DK_3, s3dm::V3DK_4, s3dm::V3DK_5
      , s3dm::V3DK_6, s3dm::V3DK_7, s3dm::V3DK_8, s3dm::V3DK_9, s3dm::V3DK_10
      , s3dm::V3DK_ESC, s3dm::V3DK_ALT, s3dm::V3DK_SHIFT, s3dm::V3DK_CTRL
      , s3dm::V3DK_ROTATE
      , /* 28 */ s3dm::V3DK_INVALID, s3dm::V3DK_INVALID
      , /* 30 */ s3dm::V3DK_INVALID, s3dm::V3DK_INVALID, s3dm::V3DK_INVALID, s3dm::V3DK_INVALID, s3dm::V3DK_INVALID
      , /* 35 */ s3dm::V3DK_INVALID
      , s3dm::V3DK_ENTER
      , s3dm::V3DK_DELETE
      , s3dm::V3DK_INVALID
      , s3dm::V3DK_INVALID
      , /* 40 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 45 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 50 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 55 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 60 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 65 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 70 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 75 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , s3dm::V3DK_11, s3dm::V3DK_12
      , s3dm::V3DK_INVALID
      , /* 80 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 85 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 90 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 95 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 100 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID, s3dm::V3DK_INVALID
      , s3dm::V3DK_VIEW_1,s3dm::V3DK_VIEW_2,s3dm::V3DK_VIEW_3
      , /* 106 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 110 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 115 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 120 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 125 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 130 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 135 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , s3dm::V3DK_SAVE_VIEW_1,s3dm::V3DK_SAVE_VIEW_2,s3dm::V3DK_SAVE_VIEW_3
      , /* 142 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 145 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 150 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 155 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 160 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 165 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , /* 170 */ s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID,s3dm::V3DK_INVALID
      , s3dm::V3DK_TAB,s3dm::V3DK_SPACE
   };

   static const struct tag_VirtualKeys _3dmouseHID2VirtualKeys[]= 
   {
      eSpacePilot
      , numberof(SpacePilotKeys)
      , const_cast<s3dm::e3dmouse_virtual_key *>(SpacePilotKeys)
      , numberof(SpacePilotKeys)-1

      , eSpaceExplorer
      , numberof(SpaceExplorerKeys)
      , const_cast<s3dm::e3dmouse_virtual_key *>(SpaceExplorerKeys)
      , numberof(SpaceExplorerKeys)-1

      , eSpaceNavigator
      , numberof(SpaceMouseKeys)
      , const_cast<s3dm::e3dmouse_virtual_key *>(SpaceMouseKeys)
      , numberof(SpaceMouseKeys)-1
      
      , eSpaceNavigatorForNotebooks
      , numberof(SpaceMouseKeys)
      , const_cast<s3dm::e3dmouse_virtual_key *>(SpaceMouseKeys)
      , numberof(SpaceMouseKeys)-1
      
      , eSpacePilotPRO
      , numberof(SpacePilotProKeys)
      , const_cast<s3dm::e3dmouse_virtual_key *>(SpacePilotProKeys)
      , numberof(SpacePilotProKeys)-1

      , eSpaceMousePRO
      , numberof(SpaceMouseProKeys)
      , const_cast<s3dm::e3dmouse_virtual_key *>(SpaceMouseProKeys)
      , 15

      , eSpaceMouse
      , numberof(SpaceMouseKeys)
      , const_cast<s3dm::e3dmouse_virtual_key *>(SpaceMouseKeys)
      , numberof(SpaceMouseKeys)-1

      , eSpaceMouseTouch
      , numberof(SpaceMouseTouchKeys)
      , const_cast<s3dm::e3dmouse_virtual_key *>(SpaceMouseTouchKeys)
      , numberof(SpaceMouseTouchKeys)-1

      , eSpaceMouseEnterprise
      , numberof(SpaceMouseEnterpriseKeys)
      , const_cast<s3dm::e3dmouse_virtual_key *>(SpaceMouseEnterpriseKeys)
      , numberof(SpaceMouseEnterpriseKeys)-1

	  // Where is SMW?  Maybe it doesn't matter because 1&2 are 1&2
   };



   /*-----------------------------------------------------------------------------
   *
   * unsigned short HidToVirtualKey(unsigned short pid, unsigned short hidKeyCode)
   *
   * Args:
   *    pid - USB Product ID (PID) of 3D mouse device 
   *    hidKeyCode - Hid keycode as retrieved from a Raw Input packet
   *
   * Return Value:
   *    Returns the standard 3d mouse virtual key (button identifier) or zero if an error occurs.
   *
   * Description:
   *    Converts a hid device keycode (button identifier) of a pre-2009 3Dconnexion USB device
   *    to the standard 3d mouse virtual key definition.
   *
   *---------------------------------------------------------------------------*/
   __inline unsigned short HidToVirtualKey(unsigned long pid, unsigned short hidKeyCode)
   {
      unsigned short virtualkey=hidKeyCode;
      for (size_t i=0; i<numberof(_3dmouseHID2VirtualKeys); ++i)
      {
         if (pid == _3dmouseHID2VirtualKeys[i].pid)
         {
            if (hidKeyCode < _3dmouseHID2VirtualKeys[i].nLength)
               virtualkey = _3dmouseHID2VirtualKeys[i].vkeys[hidKeyCode];

//          Change 10/24/2012: if the key doesn't need translating then pass it through
//            else
//              virtualkey = s3dm::V3DK_INVALID;
            break;
         }
      }
      // Remaining devices are unchanged
#if _TRACE_VIRTUAL_KEYS
     TRACE(L"Converted %d to %s(=%d) for pid 0x%x\n", hidKeyCode, VirtualKeyToId(virtualkey), virtualkey, pid);
#endif
      return virtualkey;
   }

   /*-----------------------------------------------------------------------------
   *
   * unsigned short VirtualKeyToHid(unsigned short pid, unsigned short virtualkey)
   *
   * Args:
   *    pid - USB Product ID (PID) of 3D mouse device 
   *    virtualkey - standard 3d mouse virtual key
   *
   * Return Value:
   *    Returns the Hid keycode as retrieved from a Raw Input packet
   *
   * Description:
   *    Converts a standard 3d mouse virtual key definition 
   *    to the hid device keycode (button identifier).
   *
   *---------------------------------------------------------------------------*/
   __inline unsigned short VirtualKeyToHid(unsigned long pid, unsigned short virtualkey)
   {
      unsigned short hidKeyCode = virtualkey;
      for (size_t i=0; i<numberof(_3dmouseHID2VirtualKeys); ++i)
      {
         if (pid == _3dmouseHID2VirtualKeys[i].pid)
         {
            for (unsigned short hidCode=0; hidCode<_3dmouseHID2VirtualKeys[i].nLength; ++hidCode)
            {
              if (virtualkey==_3dmouseHID2VirtualKeys[i].vkeys[hidCode])
                return hidCode;
            }
//          Change 10/24/2012: if the key doesn't need translating then pass it through
            return hidKeyCode;
         }
      }
      // Remaining devices are unchanged
#if _TRACE_VIRTUAL_KEYS
     TRACE(L"Converted %d to %s(=%d) for pid 0x%x\n", virtualkey, VirtualKeyToId(virtualkey), hidKeyCode, pid);
#endif
      return hidKeyCode;
   }

   /*-----------------------------------------------------------------------------
   *
   * unsigned int NumberOfButtons(unsigned short pid)
   *
   * Args:
   *    pid - USB Product ID (PID) of 3D mouse device 
   *
   * Return Value:
   *    Returns the number of buttons of the device.
   *
   * Description:
   *   Returns the number of buttons of the device.
   *
   *---------------------------------------------------------------------------*/
   __inline size_t NumberOfButtons(unsigned long pid)
   {
      for (size_t i=0; i<numberof(_3dmouseHID2VirtualKeys); ++i)
      {
         if (pid == _3dmouseHID2VirtualKeys[i].pid)
            return _3dmouseHID2VirtualKeys[i].nKeys;
      }
      return 0;
   }

   /*-----------------------------------------------------------------------------
   *
   * int HidToIndex(unsigned short pid, unsigned short hidKeyCode)
   *
   * Args:
   *    pid - USB Product ID (PID) of 3D mouse device 
   *    hidKeyCode - Hid keycode as retrieved from a Raw Input packet
   *
   * Return Value:
   *    Returns the index of the hid button or -1 if an error occurs.
   *
   * Description:
   *    Converts a hid device keycode (button identifier) to a zero based 
   *    sequential index.
   *
   *---------------------------------------------------------------------------*/
   __inline int HidToIndex(unsigned long pid, unsigned short hidKeyCode)
   {
      for (size_t i=0; i<numberof(_3dmouseHID2VirtualKeys); ++i)
      {
         if (pid == _3dmouseHID2VirtualKeys[i].pid)
         {
            int index=-1;
            if (hidKeyCode < _3dmouseHID2VirtualKeys[i].nLength)
            {
              unsigned short virtualkey = _3dmouseHID2VirtualKeys[i].vkeys[hidKeyCode];
              if (virtualkey != s3dm::V3DK_INVALID)
              {
                for (int key=1; key<=hidKeyCode; ++key)
                {
                  if (_3dmouseHID2VirtualKeys[i].vkeys[key] != s3dm::V3DK_INVALID)
                    ++index;
                }
              }
            }
            return index;
         }
      }
      return hidKeyCode-1;
   }

   /*-----------------------------------------------------------------------------
   *
   * unsigned short IndexToHid(unsigned short pid, int index)
   *
   * Args:
   *    pid - USB Product ID (PID) of 3D mouse device 
   *    index - index of button
   *
   * Return Value:
   *    Returns the Hid keycode of the nth button or 0 if an error occurs.
   *
   * Description:
   *    Returns the hid device keycode of the nth button
   *
   *---------------------------------------------------------------------------*/
   __inline unsigned short IndexToHid(unsigned long pid, int index)
   {
      if (index < 0)
        return 0;
      for (size_t i=0; i<numberof(_3dmouseHID2VirtualKeys); ++i)
      {
         if (pid == _3dmouseHID2VirtualKeys[i].pid)
         {
            if (index < static_cast<int>(_3dmouseHID2VirtualKeys[i].nLength))
            {
              for (size_t key=1; key<_3dmouseHID2VirtualKeys[i].nLength; ++key)
              {
                if (_3dmouseHID2VirtualKeys[i].vkeys[key] != s3dm::V3DK_INVALID)
                {
                  --index;
                  if (index == -1)
                    return static_cast<unsigned short>(key);
                }
              }
            }
            return 0;
         }
      }
      return static_cast<unsigned short>(index+1);
   }

}; //namespace tdx
#endif // virtualkeys_HPP_INCLUDED_
