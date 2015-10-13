/*! \file epos.h
    
  header file for libEPOS functions

  zhangfy, 22 May 2014

*/

#ifndef _EPOS_H
#define _EPOS_H

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <stdint.h>  /* int types with given size */
#include <math.h>
#include "std_msgs/String.h"

/* added oct06 for openTCPEPOS() */
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* all EPOS data exchange is based on 16bit words, but other types are
   also used...*/
typedef unsigned long DWORD ; ///< \brief 32bit type for EPOS data exchange
typedef unsigned short WORD ; ///< \brief 16bit type for EPOS data exchange
#ifndef CPP
typedef char BYTE ; ///< \brief 8bit type for EPOS data exchange
#endif

/* EPOS will reset communication after 500ms of inactivity */

/*! \brief try NTRY times to read one byte from EPOS, the give up */
#define NTRY      5
/*! \brief sleep TRYSLEEP usec between read() from EPOS, if no data available */
//#define TRYSLEEP  (unsigned int)1e5 
#define TRYSLEEP  (unsigned int)12500  ///this is a suitable sleep time, it is quite close to the minimum sleep time considering the baudrate 115200

/* EPOS codes */

#define E_OK      0x4f  ///< EPOS answer code for <em>all fine</em>
#define E_FAIL    0x46  ///< EPOS answer code to indicate a <em>failure</em>
#define E_ANS     0x00  ///< EPOS code to indicate an answer <em>frame</em>


/* EPOS error codes (Communication Guide, 6.4)  */

/* CANopen defined error codes */
#define E_NOERR         0x00000000   ///< Error code: no error
#define E_ONOTEX        0x06020000   ///< Error code: object does not exist
#define E_SUBINEX       0x06090011   ///< Error code: subindex does not exist
#define E_OUTMEM        0x05040005   ///< Error code: out of memory
#define E_NOACCES       0x06010000   ///< Error code: Unsupported access to an object
#define E_WRITEONLY     0x06010001   ///< Error code: Attempt to read a write-only object
#define E_READONLY      0x06010002   ///< Error code: Attempt to write a read-only object
#define E_PARAMINCOMP   0x06040043   ///< Error code: general parameter incompatibility 
#define E_INTINCOMP     0x06040047   ///< Error code: general internal incompatibility in the device 
#define E_HWERR         0x06060000   ///< Error code: access failed due to an hardware error
#define E_PRAGNEX       0x06090030   ///< Error code: value range of parameter exeeded
#define E_PARHIGH       0x06090031   ///< Error code: value of parameter written is too high
#define E_PARLOW        0x06090032   ///< Error code: value of parameter written is too low
#define E_PARREL        0x06090036   ///< Error code: maximum value is less than minimum value


/* maxon specific error codes */
#define E_NMTSTATE      0x0f00ffc0   ///< Error code: wrong NMT state
#define E_RS232         0x0f00ffbf   ///< Error code: rs232 command illegeal
#define E_PASSWD        0x0f00ffbe   ///< Error code: password incorrect
#define E_NSERV         0x0f00ffbc   ///< Error code: device not in service mode
#define E_NODEID        0x0f00fb9    ///< Error code: error in Node-ID



/* EPOS Statusword -- singe bits, see firmware spec 14.1.58 */
#define E_BIT15        0x8000      ///< bit code: position referenced to home position
#define E_BIT14        0x4000      ///< bit code: refresh cycle of power stage
#define E_BIT13        0x2000      ///< bit code: OpMode specific, some error
#define E_BIT12        0x1000      ///< bit code: OpMode specific
#define E_BIT11        0x0800      ///< bit code: NOT USED
#define E_BIT10        0x0400      ///< bit code: Target reached
#define E_BIT09        0x0200      ///< bit code: Remote (?)
#define E_BIT08        0x0100      ///< bit code: offset current measured (?)
#define E_BIT07        0x0080      ///< bit code: WARNING
#define E_BIT06        0x0040      ///< bit code: switch on disable
#define E_BIT05        0x0020      ///< bit code: quick stop
#define E_BIT04        0x0010      ///< bit code: voltage enabled
#define E_BIT03        0x0008      ///< bit code: FAULT
#define E_BIT02        0x0004      ///< bit code: operation enable
#define E_BIT01        0x0002      ///< bit code: switched on
#define E_BIT00        0x0001      ///< bit code: ready to switch on             


/* EPOS modes of operation, firmware spec 14.1.59 (p.133, tbl. 72) */
#define E_HOMING      6 ///< EPOS operation mode: homing
#define E_PROFVEL     3 ///< EPOS operation mode: profile velocity mode
#define E_PROFPOS     1 ///< EPOS operation mode: profile position mode
// the modes below should not be used by user, defined here only for
// completeness
#define E_POSMOD     -1 ///< EPOS operation mode: position mode
#define E_VELMOD     -2 ///< EPOS operation mode: velocity mode
#define E_CURRMOD    -3 ///< EPOS operation mode: current mode
#define E_DIAGMOD    -4 ///< EPOS operation mode: diagnostics mode
#define E_MASTERENCMOD -5 ///< EPOS operation mode:internal
#define E_STEPDIRECMOD -6 ///< EPOS operation mode:internal


/* globals, defined in epos.cpp */

/*!serial port file descriptor*/
extern std::string port; 
/*! EPOS global error status */
extern DWORD E_error; 

/*!motor control parameters define */
extern int MaxProfVelocity; 	// permitted maximum profile velocity
extern int ProfVelocity;     	// profile velocity
extern int ProfAcceleration; 	// profile acceleration
extern int ProfDeceleration; 	// profile deceleration


/* all high-level functions return <0 in case of error */

/*! open the connection to EPOS */
int openEPOS(const char *device);
/*! open the connection to EPOS via RS232-over-TCP/IP (LSW special) */
int openTCPEPOS(char *ip, short unsigned port);
/*! close the connection to EPOS */
int closeEPOS();
/*! check if the connection to EPOS is alive */
int checkEPOS();


/*! \brief check global variable E_error for EPOS error code */
int checkEPOSerror();

/*! \brief check EPOS status, return state according to firmware spec 8.1.1 */
int checkEPOSstate();
/*! \brief pretty-print EPOS state */
int printEPOSstate();
/*! \brief change EPOS state   ==> firmware spec 8.1.3 */
int changeEPOSstate(int state);



/*! \brief example from EPOS com. guide: ask EPOS for software version 

firmware spec 14.1.33
** returns software version as HEX **

*/
int readSWversion();


/*! \brief ask for device name,  
   device name is placed in 'name' (string must be big enough, NO CHECKING!!)
*/
int readDeviceName(char *name);


/*! \brief ask for RS232 timeout; firmware spec 14.1.35 */
int readRS232timeout();

/*! \brief read digital input polarity mask */
int readDInputPolarity(WORD* w);

/*! \brief set home switch polarity -- firmware spec 14.1.47 */
int setHomePolarity(int pol);



/*! \brief read Statusword; 14.1.58 */
int readStatusword(WORD *eposStatus);
/*! \brief pretty-print Statusword */
int printEPOSstatusword(WORD statusword);



/*! \brief read EPOS control word (firmware spec 14.1.57) */
int readControlword(WORD *w);
/*! \brief pretty-print Controlword */
int printEPOScontrolword(WORD controlword);


/*! \brief set EPOS mode of operation -- 14.1.59 */
int setOpMode(int OpMode);
int setProfile_Position_Mode(int max_velocity, int velocity, int acceleration, int deceleration);

/*! \brief read and returns  EPOS mode of operation -- 14.1.60 
here, RETURN(0) MEANS ERROR! 
'-1' is a valid OpMode, but 0 is not!
*/
int readOpMode();


/*! \brief read actual position; 14.1.61 */
int readDemandPosition(long *val);
/*! \brief read actual position; 14.1.62 */
int readActualPosition(long *val);

/*! \brief read position window; 14.1.64 */
int readPositionWindow(unsigned long int *value);
/*! \brief write position window; 14.1.64 */
int writePositionWindow(unsigned long int value);

/*! \brief read actual position; 14.1.67 */
int readDemandVelocity(long *val);
/*! \brief read actual position; 14.1.68 */
int readActualVelocity(long *val);

/*! \brief read actual current; 14.1.69 */
int readActualCurrent(short *val);

/*! \brief read target position; 14.1.70 */
int readTargetPosition(long *val);



/*! \brief does a homing move. Give homing mode (see firmware 9.3) and start
   position */
int doHoming(int method, long int start);


/*! \brief set OpMode to ProfilePosition and make relative movement */
int moveRelative(long int steps);
/*! \brief set OpMode to ProfilePosition and make absolute movement */
int moveAbsolute(long int steps);

/*! \brief reads position, velocity and current and displays them in an
   endless loop. Returns after target position has been reached */
int monitorStatus();
/*! \brief as monitorStatus(), but also waits for Homing Attained' signal */
int monitorHomingStatus();

/*! \brief waits for positoning to finish, argument is timeout in
   seconds. give timeout==0 to disable timeout */
int waitForTarget(unsigned int t);


#endif
