/*======================*/
/*       Includes       */
/*======================*/
//system headers
#include <stdio.h>
#include <errno.h>
#ifndef _WIN32
#include <inttypes.h>
#include <pthread.h>
#include <syslog.h>
#include <unistd.h>
#endif
#include <windows.h>
#include <malloc.h>
#include <assert.h>
//project headers
extern "C" {
#include "IXXAT/vcinpl.h"
}
#include "select.hpp"
#include "canDef.h"
#include "canAPI.h"

CANAPI_BEGIN


#define CH_COUNT			(int)2 // number of CAN channels

unsigned char CAN_ID = 0;

//////////////////////////////////////////////////////////////////////////
// global variables
//////////////////////////////////////////////////////////////////////////
static HANDLE hDevice[CH_COUNT] = {(HANDLE) 0, (HANDLE) 0};  // device handle
static LONG   lCtrlNo[CH_COUNT] = {         0,          0};  // controller number
static HANDLE hCanCtl[CH_COUNT] = {(HANDLE)-1, (HANDLE)-1};  // controller handle 
static HANDLE hCanChn[CH_COUNT] = {(HANDLE)-1, (HANDLE)-1};  // channel handle

//////////////////////////////////////////////////////////////////////////
// static function prototypes
//////////////////////////////////////////////////////////////////////////
HRESULT SelectDevice ( UINT32 dwCanChNo, BOOL fUserSelect );
HRESULT InitSocket   ( UINT32 dwCanChNo, UINT32 dwCanNo );
void    FinalizeApp  ( UINT32 dwCanChNo );
void    DisplayError ( /*UINT32 dwCanChNo,*/ HRESULT hResult );


#define	STD		(bool)0
#define	EXT		(bool)1

/**
  This function transmit a CAN data frame.
*/
int canWrite(HANDLE handle,
			 unsigned long id, 
			 void * msg,
			 unsigned int dlc,
			 int mode)
{
	if (handle < 0)
		return -1;

	HRESULT hResult;
	CANMSG  sCanMsg;
	UINT8   i;

	sCanMsg.dwTime   = 0;
	sCanMsg.dwMsgId  = (id << 2) | CAN_ID;    // CAN message identifier

	sCanMsg.uMsgInfo.Bytes.bType  = CAN_MSGTYPE_DATA;
	sCanMsg.uMsgInfo.Bytes.bFlags = CAN_MAKE_MSGFLAGS(dlc,0,0,0,mode);
	sCanMsg.uMsgInfo.Bits.srr     = 0;

	for (i = 0; i < sCanMsg.uMsgInfo.Bits.dlc; i++)
	{
		sCanMsg.abData[i] = ((unsigned char*)msg)[i];
	}

	// write the CAN message into the transmit FIFO
	hResult = canChannelSendMessage(handle, INFINITE, &sCanMsg);

	if (hResult != VCI_OK)
	{
		DisplayError(hResult);
	}

	return hResult;
}


/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
int command_can_open(int ch)
{
	assert(ch >= 1 && ch <= CH_COUNT);

	HRESULT hResult;

	hResult = SelectDevice( ch-1, TRUE );
	if ( VCI_OK != hResult ) {
		DisplayError(hResult);
		return hResult;
	}

	hResult = InitSocket( ch-1, lCtrlNo[ch-1] );
	DisplayError(hResult);
	return hResult;
}

int command_can_open_ex(int ch, int type, int index)
{
	return command_can_open(ch);
}

int command_can_reset(int ch)
{
	return -1;
}

int command_can_close(int ch)
{
	FinalizeApp(ch);
	return 0;
}

int command_can_set_id(int ch, unsigned char can_id)
{
	CAN_ID = can_id;
	return 0;
}

int command_servo_on(int ch)
{
	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_ON;
	ret = canWrite(hCanChn[ch - 1], Txid, data, 0, STD);

	return ret;
}

int command_servo_off(int ch)
{
	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_OFF;
	ret = canWrite(hCanChn[ch - 1], Txid, data, 0, STD);

	return ret;
}

int command_set_torque(int ch, int findex, short* pwm)
{
	assert(findex >= 0 && findex < NUM_OF_FINGERS);

	long Txid;
	short duty[4];
	int ret;

	if (findex >= 0 && findex < NUM_OF_FINGERS)
	{
		duty[0] = pwm[0];
		duty[1] = pwm[1];
		duty[2] = pwm[2];
		duty[3] = pwm[3];

		Txid = ID_CMD_SET_TORQUE_1 + findex;

		ret = canWrite(hCanChn[ch - 1], Txid, (unsigned char *)duty, 8, STD);
	}
	else
		return -1;

	return ret;
}

int command_set_pose(int ch, int findex, short* jposition)
{
	assert(findex >= 0 && findex < NUM_OF_FINGERS);

	long Txid;
	short pose[4];
	int ret;

	if (findex >= 0 && findex < NUM_OF_FINGERS)
	{
		pose[0] = jposition[0];
		pose[1] = jposition[1];
		pose[2] = jposition[2];
		pose[3] = jposition[3];

		Txid = ID_CMD_SET_POSE_1 + findex;

		ret = canWrite(hCanChn[ch - 1], Txid, (unsigned char *)pose, 8, STD);
	}
	else
		return -1;

	return ret;
}

int command_set_period(int ch, short* period)
{
	long Txid;
	can_period_msg_t msg;
	int ret;

	Txid = ID_CMD_SET_PERIOD;
	if (period != 0)
	{
		msg.position = period[0];
		msg.imu = period[1];
		msg.temp = period[2];
	}
	else
	{
		msg.position = 0;
		msg.imu = 0;
		msg.temp = 0;
	}
	ret = canWrite(hCanChn[ch - 1], Txid, (unsigned char *)&msg, 6, STD);

	return ret;
}

int command_set_device_id(int ch, unsigned char did)
{
	long Txid;
	int ret;
	can_config_msg_t msg;

	Txid = ID_CMD_CONFIG;
	msg.set = 0x01;
	msg.did = did;
	msg.baudrate = 0;
	ret = canWrite(hCanChn[ch - 1], Txid, (unsigned char *)&msg, 6, STD);

	return ret;
}

int command_set_rs485_baudrate(int ch, unsigned int baudrate)
{
	long Txid;
	int ret;
	can_config_msg_t msg;

	Txid = ID_CMD_CONFIG;

	msg.set = 0x02;
	msg.did = 0;
	msg.baudrate = baudrate;
	ret = canWrite(hCanChn[ch - 1], Txid, (unsigned char *)&msg, 6, STD);

	return ret;
}

int request_hand_information(int ch)
{
	long Txid = ID_RTR_HAND_INFO;
	int ret = canWrite(hCanChn[ch - 1], Txid, NULL, 0, STD);

	return ret;
}

int request_hand_serial(int ch)
{
	long Txid = ID_RTR_SERIAL;
	int ret = canWrite(hCanChn[ch - 1], Txid, NULL, 0, STD);

	return ret;
}

int request_finger_pose(int ch, int findex)
{
	assert(findex >= 0 && findex < NUM_OF_FINGERS);

	long Txid = ID_RTR_FINGER_POSE + findex;
	int ret = canWrite(hCanChn[ch - 1], Txid, NULL, 0, STD);

	return ret;
}

int request_imu_data(int ch)
{
	long Txid = ID_RTR_IMU_DATA;
	int ret = canWrite(hCanChn[ch - 1], Txid, NULL, 0, STD);

	return ret;
}

int request_temperature(int ch, int sindex)
{
	assert(sindex >= 0 && sindex < NUM_OF_TEMP_SENSORS);

	long Txid = ID_RTR_TEMPERATURE + sindex;
	int ret = canWrite(hCanChn[ch - 1], Txid, NULL, 0, STD);

	return ret;
}

int get_message(int ch, int* id, int* len, unsigned char* data, int blocking)
{
	HRESULT hResult;
	CANMSG  sCanMsg;

	//hResult = canChannelReadMessage(hCanChn[ch-1], (blocking ? INFINITE : 0), &sCanMsg);
	if (blocking)
		hResult = canChannelReadMessage(hCanChn[ch-1], INFINITE, &sCanMsg);
	else
		hResult = canChannelPeekMessage(hCanChn[ch-1], &sCanMsg);
	
	if (hResult == VCI_OK)
	{
		if (sCanMsg.uMsgInfo.Bytes.bType == CAN_MSGTYPE_DATA)
		{
			if (sCanMsg.uMsgInfo.Bits.rtr == 0)
			{
				*id = (sCanMsg.dwMsgId & 0xfffffffc) >> 2;
				*len = (int)( sCanMsg.uMsgInfo.Bits.dlc );
				for(int nd=0; nd<(*len); nd++) data[nd] = sCanMsg.abData[nd];

#ifdef _DEBUG
				/*UINT8 j;
				printf("\nTime: %10u  ID: %3X  DLC: %1u  Data:",
					sCanMsg.dwTime,
					sCanMsg.dwMsgId,
					sCanMsg.uMsgInfo.Bits.dlc);
				for (j = 0; j < sCanMsg.uMsgInfo.Bits.dlc; j++)
					printf(" %.2X", sCanMsg.abData[j]);
				printf("\n");*/
#endif
			}
			else
			{
				printf("\nTime: %10u ID: %3X  DLC: %1u  Remote Frame",
						sCanMsg.dwTime,
						sCanMsg.dwMsgId,
						sCanMsg.uMsgInfo.Bits.dlc);
			}
		}
		else if (sCanMsg.uMsgInfo.Bytes.bType == CAN_MSGTYPE_INFO)
		{
			//
			// show informational frames
			//
			switch (sCanMsg.abData[0])
			{
				case CAN_INFO_START: printf("\nCAN started..."); break;
				case CAN_INFO_STOP : printf("\nCAN stoped...");  break;
				case CAN_INFO_RESET: printf("\nCAN reseted..."); break;
			}
		}
		else if (sCanMsg.uMsgInfo.Bytes.bType == CAN_MSGTYPE_ERROR)
		{
			//
			// show error frames
			//
			switch (sCanMsg.abData[0])
			{
				case CAN_ERROR_STUFF: printf("\nstuff error...");          break; 
				case CAN_ERROR_FORM : printf("\nform error...");           break; 
				case CAN_ERROR_ACK  : printf("\nacknowledgment error..."); break;
				case CAN_ERROR_BIT  : printf("\nbit error...");            break; 
				case CAN_ERROR_CRC  : printf("\nCRC error...");            break; 
				case CAN_ERROR_OTHER:
				default             : printf("\nother error...");          break;
			}
		}
    }
	else
	{
		 if (VCI_E_RXQUEUE_EMPTY != hResult &&
			 VCI_E_TIMEOUT != hResult)
			 DisplayError(hResult);
	}
	
	return hResult;
}

/**
  Selects the first CAN adapter.

  @param fUserSelect
    If this parameter is set to TRUE the functions display a dialog box which 
    allows the user to select the device.

  @return
    VCI_OK on success, otherwise an Error code
*/
HRESULT SelectDevice( UINT32 dwCanChNo, BOOL fUserSelect )
{
  HRESULT hResult; // error code

  if (fUserSelect == FALSE)
  {
    HANDLE        hEnum;   // enumerator handle
    VCIDEVICEINFO sInfo;   // device info

    //
    // open the device list
    //
    hResult = vciEnumDeviceOpen(&hEnum);

    //
    // retrieve information about the first
    // device within the device list
    //
    if (hResult == VCI_OK)
    {
      hResult = vciEnumDeviceNext(hEnum, &sInfo);
    }

    //
    // close the device list (no longer needed)
    //
    vciEnumDeviceClose(hEnum);

    //
    // open the device
    //
    if (hResult == VCI_OK)
    {
      hResult = vciDeviceOpen(sInfo.VciObjectId, &(hDevice[dwCanChNo]));
    }

    //
    // always select controller 0
    //
    lCtrlNo[dwCanChNo] = 0;
  }
  else
  {
    //
    // open a device selected by the user
    //
    hResult = SocketSelectDlg(NULL, VCI_BUS_CAN, &(hDevice[dwCanChNo]), &(lCtrlNo[dwCanChNo]));
  }

  DisplayError(hResult);
  return hResult;
}

/**
  Opens the specified socket, creates a message channel, initializes
  and starts the CAN controller.

  @param dwCanNo
    Number of the CAN controller to open.

  @return
    VCI_OK on success, otherwise an Error code

  @note
    If <dwCanNo> is set to 0xFFFFFFFF, the function shows a dialog box
    which allows the user to select the VCI device and CAN controller.
*/
HRESULT InitSocket( UINT32 dwCanChNo, UINT32 dwCanNo )
{
  HRESULT hResult;

  //
  // create a message channel
  //
  if (hDevice[dwCanChNo] != NULL)
  {
    //
    // create and initialize a message channel
    //
    hResult = canChannelOpen(hDevice[dwCanChNo], dwCanNo, FALSE, &(hCanChn[dwCanChNo]));

    //
    // initialize the message channel
    //
    if (hResult == VCI_OK)
    {
      UINT16 wRxFifoSize  = 1024;
      UINT16 wRxThreshold = 1;
      UINT16 wTxFifoSize  = 128;
      UINT16 wTxThreshold = 1;

      hResult = canChannelInitialize( hCanChn[dwCanChNo],
                                      wRxFifoSize, wRxThreshold,
                                      wTxFifoSize, wTxThreshold);
    }

    //
    // activate the CAN channel
    //
    if (hResult == VCI_OK)
    {
      hResult = canChannelActivate(hCanChn[dwCanChNo], TRUE);
    }

    //
    // open the CAN controller
    //
    if (hResult == VCI_OK)
    {
      hResult = canControlOpen(hDevice[dwCanChNo], dwCanNo, &(hCanCtl[dwCanChNo]));
      // this function fails if the controller is in use
      // by another application.
    }

    //
    // initialize the CAN controller
    //
    if (hResult == VCI_OK)
    { 
      hResult = canControlInitialize(hCanCtl[dwCanChNo], CAN_OPMODE_STANDARD | CAN_OPMODE_ERRFRAME,
		                             CAN_BT0_1000KB, CAN_BT1_1000KB);
    }

    //
    // set the acceptance filter
    //
    if (hResult == VCI_OK)
    { 
       hResult = canControlSetAccFilter( hCanCtl[dwCanChNo], FALSE,
                                         CAN_ACC_CODE_ALL, CAN_ACC_MASK_ALL);
    }

    //
    // start the CAN controller
    //
    if (hResult == VCI_OK)
    {
      hResult = canControlStart(hCanCtl[dwCanChNo], TRUE);
    }
  }
  else
  {
    hResult = VCI_E_INVHANDLE;
  }

  DisplayError(hResult);
  return hResult;
}

/**
  Finalizes the application
*/
void FinalizeApp( UINT32 dwCanChNo )
{
  //
  // close all open handles

  canControlReset(hCanCtl[dwCanChNo]);
  canChannelClose(hCanChn[dwCanChNo]);
  canControlClose(hCanCtl[dwCanChNo]);

  vciDeviceClose(hDevice[dwCanChNo]);

  hCanCtl[dwCanChNo] = (HANDLE) 0;
  hCanChn[dwCanChNo] = (HANDLE)-1;
  hDevice[dwCanChNo] = (HANDLE)-1;
}

/**
  This function displays a message box for the specified error code.

  @param hResult
    Error code or -1 to display the error code returned by GetLastError().
*/
void DisplayError( /*UINT32 dwCanChNo,*/ HRESULT hResult )
{
  char szError[VCI_MAX_ERRSTRLEN];

  if (hResult != NO_ERROR)
  {
    if (hResult == -1)
      hResult = GetLastError();

    szError[0] = 0;
    vciFormatError(hResult, szError, sizeof(szError));
    //MessageBoxA(NULL, szError, "rDeviceAllegroHandIXXATCAN", MB_OK | MB_ICONSTOP);
    printf("%s\n", szError);
  }
}


CANAPI_END
