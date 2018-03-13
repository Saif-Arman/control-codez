#ifndef _MARCO_WZ_
#define _MARCO_WZ_

#define STUCK_GRIPPER			0
#define NOT_ATTACHE				1
#define ARM_FOLDED_STRECHED		2
#define BLOCKED_DOF				3
#define MAX_M1_ROTATION			4
#define MOVEMENT_ERROR			5
#define MEMORY186_FULL			6

#define MAX_CART				70
#define MAX_JOINT				1
#define MAX_JOINT_GRIP			10
#define MAX_CART_GRIP			13
#define MAX_JOINT_YPR			3
#define MAX_CART_YPR			6

#define	T_ERR_BOUND				(1.0f)
#define J_ERR_BOUND             (0.25f)
#define	R_ERR_BOUND				(2.0f)
#define MAX_CART_T				(5.0f)
#define R_V_ERR_YAW				(0.05f)
#define R_V_ERR_PITCH			(0.05f)
#define R_V_ERR_ROLL			(0.1f)
#define QUE_SIZE				(10)
#define FREE					0
#define MANUAL_MODE				1
#define AUTO_MODE				2
#define CARTESIAN				1
#define SET_ZERO   				2
#define JOINT					4
#define FOLD_OUT				5
#define FOLD_IN					6
#define EXIT					'o'
#define STUCK_GRIPPER			0
#define NOT_ATTACHE				1
#define ARM_FOLDED_STRETCHED	2
#define BLOCKED_DOF				3
#define MAX_M1_ROTATION			4
#define MAX_CART_GRIP			11//original15    zc
#define MAX_JOINT_GRIP			10
#define M_PI 3.1415926535897

#endif