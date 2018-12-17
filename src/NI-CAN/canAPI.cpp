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
#include "NI-CAN/nican.h"
}
#include "canDef.h"
#include "canAPI.h"

CANAPI_BEGIN


#define CH_COUNT			(int)1 // number of CAN channels
int hd[CH_COUNT];


#define canMSG_MASK             0x00ff      // Used to mask the non-info bits
#define canMSG_RTR              0x0001      // Message is a remote request
#define canMSG_STD              0x0002      // Message has a standard ID
#define canMSG_EXT              0x0004      // Message has an extended ID
#define canMSG_WAKEUP           0x0008      // Message to be sent / was received in wakeup mode
#define canMSG_NERR             0x0010      // NERR was active during the message
#define canMSG_ERROR_FRAME      0x0020      // Message is an error frame
#define canMSG_TXACK            0x0040      // Message is a TX ACK (msg is really sent)
#define canMSG_TXRQ             0x0080      // Message is a TX REQUEST (msg is transfered to the chip)



//#define _DUMP_RXFRAME (1)

/* NI-CAN Status    */
NCTYPE_STATUS Status=0; 

/* NI-CAN handles */
NCTYPE_OBJH TxHandle=0;

/* NI-CAN Frame Type for Write Object */ 
NCTYPE_CAN_DATA	Transmit;

/* NI-CAN Frame Type for Write Object */ 
NCTYPE_CAN_FRAME TxFrame;

/* NI-CAN Frame Type for Read Object */ 
NCTYPE_CAN_STRUCT RxFrame;

/* This function converts the absolute time obtained from ncReadMult into a
   string. */
void AbsTimeToString(NCTYPE_ABS_TIME *time, char *TimeString)
{

   SYSTEMTIME	stime;
   FILETIME		localftime;

   FileTimeToLocalFileTime((FILETIME *)(time), &localftime);
   FileTimeToSystemTime(&localftime, &stime);
	  sprintf(TimeString, "%02d:%02d:%02d.%03d",
            stime.wHour, stime.wMinute, stime.wSecond, 
            stime.wMilliseconds);
}

/* Print a description of an NI-CAN error/warning. */
void PrintStat(NCTYPE_STATUS Status, char *source) 
{
	char StatusString[1024];
     
	if (Status != 0) 
	{
		ncStatusToString(Status, sizeof(StatusString), StatusString);
		printf("\n%s\nSource = %s\n", StatusString, source);

		// On error, close object handle.
		printf("<< CAN: Close\n");
		ncCloseObject(TxHandle);
		TxHandle = 0;
		hd[0] = TxHandle;
		//hd[1] = TxHandle;
		//exit(1);
	}
}

/* Print read frame */
void PrintRxFrame()
{
	char output[15];
	char CharBuff[50];
	AbsTimeToString(&RxFrame.Timestamp, &output[0]);
	printf("%s     ", output);
	sprintf (&CharBuff[0], "%8.8X", RxFrame.ArbitrationId);
	printf("%s     ", CharBuff);
	sprintf (&CharBuff[0], "%s","CAN Data Frame");
	printf("%s     ", CharBuff); 
	sprintf (&CharBuff[0], "%1d", RxFrame.DataLength);
	printf("%s     ", CharBuff); 
	for (int j=0; j<RxFrame.DataLength; j++)
	{
		sprintf (CharBuff, " %02X", RxFrame.Data[j]);
		printf("%s", CharBuff); 
	}
	printf("\n");
}

int canWriteRTR(int handle,	long id)
{
	if (!handle)
		return -1;

	TxFrame.IsRemote = NC_FRMTYPE_REMOTE;
	TxFrame.ArbitrationId = id;
	TxFrame.DataLength = 0;
	Status = ncWrite(handle, sizeof(NCTYPE_CAN_FRAME), &TxFrame);
	if (Status < 0)
	{
		PrintStat(Status, "ncWrite");
		return Status;
	}
	return Status;
}

int canWrite(int handle,
			 long id, 
			 void * msg,
			 unsigned int dlc,
			 unsigned int flag)
{
	if (!handle)
		return -1;

	TxFrame.IsRemote = NC_FRMTYPE_DATA;
	TxFrame.ArbitrationId = id;
	TxFrame.DataLength = dlc;
	for (int i=0; i<dlc; i++)
		TxFrame.Data[i] = ((NCTYPE_UINT8_P)msg)[i];
	Status = ncWrite(handle, sizeof(NCTYPE_CAN_FRAME), &TxFrame);
	if (Status < 0)
	{
		PrintStat(Status, "ncWrite");
		return Status;
	}
	return Status;
}

int canRead(int handle,
			long * id,
			void * msg,
			unsigned int * dlc,
			unsigned int * /*flag*/,
			unsigned long * time)
{
	if (!handle)
		return -1;

	memset(&RxFrame, 0, sizeof(NCTYPE_CAN_FRAME));
	//RxFrame.FrameType = NC_FRMTYPE_DATA;
	RxFrame.DataLength = *dlc;
	Status = ncRead(handle, sizeof(NCTYPE_CAN_STRUCT), &RxFrame);
	if (Status < 0 || NC_FRMTYPE_DATA != RxFrame.FrameType)
	{
		PrintStat(Status, "ncRead");
		return Status;
	}
	else if (Status == CanWarnOldData)
	{
		(*dlc) = 0;
		return Status;
	}

	(*id) = RxFrame.ArbitrationId;
	//(*time) = (FILETIME)(RxFrame.Timestamp);
	(*dlc) = RxFrame.DataLength;
	for (int i=0; i<RxFrame.DataLength; i++)
		((NCTYPE_UINT8_P)msg)[i] = RxFrame.Data[i];
#ifdef _DUMP_RXFRAME
	PrintRxFrame();
#endif
	return Status;
}

int canReadWait(int handle,
				long * id,
				void * msg,
				unsigned int * dlc,
				unsigned int * /*flag*/,
				unsigned long * time,
				unsigned long timeout)
{
	if (!handle)
		return -1;

	NCTYPE_STATE currentState;
	Status = ncWaitForState(handle, NC_ST_READ_AVAIL, timeout, &currentState);
	if (Status < 0)
	{
		PrintStat(Status, "ncWaitForState");
		return Status;
	}

	memset(&RxFrame, 0, sizeof(NCTYPE_CAN_FRAME));
	RxFrame.FrameType = NC_FRMTYPE_DATA;
	RxFrame.DataLength = *dlc;
	Status = ncRead(handle, sizeof(NCTYPE_CAN_STRUCT), &RxFrame);
	if (Status < 0)
	{
		PrintStat(Status, "ncRead");
		return Status;
	}

	(*id) = RxFrame.ArbitrationId;
	//(*time) = (FILETIME)(RxFrame.Timestamp);
	(*dlc) = RxFrame.DataLength;
	for (int i=0; i<RxFrame.DataLength; i++)
		((NCTYPE_UINT8_P)msg)[i] = RxFrame.Data[i];
#ifdef _DUMP_RXFRAME
	PrintRxFrame();
#endif
	return Status;
}

/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
int command_can_open(int ch)
{
	NCTYPE_ATTRID		AttrIdList[8];
	NCTYPE_UINT32		AttrValueList[8];
	//NCTYPE_UINT32		Baudrate = 125000;  // BAUD_125K
	NCTYPE_UINT32		Baudrate = NC_BAUD_1000K;
	char				Interface[15];
	
	sprintf_s(Interface, "CAN%d", ch);
	
	// Configure the CAN Network Interface Object	
	AttrIdList[0] =     NC_ATTR_BAUD_RATE;   
	AttrValueList[0] =  Baudrate;
	AttrIdList[1] =     NC_ATTR_START_ON_OPEN;
	AttrValueList[1] =  NC_TRUE;
	AttrIdList[2] =     NC_ATTR_READ_Q_LEN;
	AttrValueList[2] =  100;
	AttrIdList[3] =     NC_ATTR_WRITE_Q_LEN;
	AttrValueList[3] =  10;	
	AttrIdList[4] =     NC_ATTR_CAN_COMP_STD;
	AttrValueList[4] =  0;//0xCFFFFFFF;
	AttrIdList[5] =     NC_ATTR_CAN_MASK_STD;
	AttrValueList[5] =  NC_CAN_MASK_STD_DONTCARE;
	AttrIdList[6] =     NC_ATTR_CAN_COMP_XTD;
	AttrValueList[6] =  0;//0xCFFFFFFF;
	AttrIdList[7] =     NC_ATTR_CAN_MASK_XTD;
	AttrValueList[7] =  NC_CAN_MASK_XTD_DONTCARE;

	
	printf("<< CAN: Config\n");
	Status = ncConfig(Interface, 2, AttrIdList, AttrValueList);
	if (Status < 0) 
	{
		PrintStat(Status, "ncConfig");
		return Status;
	}
	printf("   - Done\n");
    
	// open the CAN Network Interface Object
	printf("<< CAN: Open Channel\n");
	Status = ncOpenObject (Interface, &TxHandle);
	if (Status < 0) 
	{
		PrintStat(Status, "ncOpenObject");
		return Status;
	}
	hd[0] = TxHandle;
	//hd[1] = TxHandle;
	printf("   - Done\n");
	return 1;
}

int command_can_open_ex(int ch, int type, int index)
{
	return command_can_open(ch);
}

int command_can_reset(int ch)
{
	printf("<< CAN: Reset Bus\n");

	// stop the CAN Network
	Status = ncAction(TxHandle, NC_OP_STOP, 0);
	if (Status < 0) 
	{
		PrintStat(Status, "ncAction(NC_OP_STOP)");
		return Status;
	}
	// start the CAN Network
	Status = ncAction(TxHandle, NC_OP_START, 0);
	if (Status < 0) 
	{
		PrintStat(Status, "ncAction(NC_OP_START)");
		return Status;
	}

	/*while (true)
	{
		memset(rdata, NULL, sizeof(rdata));
		Status = canRead(TxHandle, &id, rdata, &dlc, &flags, &timestamp);
		if (Status != 0) break;
		printf("    %ld+%ld (%d)", id-id%128, id%128, dlc);
		for(int nd=0; nd<(int)dlc; nd++) printf(" %3d ", rdata[nd]);
		printf("\n");
	}*/

	/*int nc;

	printf("<< CAN: Reset Bus\n");
	int ret_c;
	for(nc=0; nc<CH_COUNT; nc++) {
		ret_c = canResetBus(hd[nc]);
		if (ret_c < 0) return ret_c;
		printf("    Ch.%2d (OK)\n", nc, ret_c);
	}*/
	printf("   - Done\n");
	Sleep(200);
	return 1;
}

int command_can_close(int ch)
{
	if (!TxHandle)
		return 0;

	printf("<< CAN: Close\n");
	Status = ncCloseObject(TxHandle);    
	if (Status < 0)
	{
		PrintStat(Status, "ncCloseObject");
		return Status;
	}
	TxHandle = 0;
	hd[0] = TxHandle;
	//hd[1] = TxHandle;
	printf("   - Done\n");
	return 1;
}

int command_can_set_id(int ch, unsigned char can_id)
{
	return 0;
}

int command_servo_on(int ch)
{
	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_ON;
	ret = canWrite(TxHandle, Txid, data, 0, canMSG_STD);

	return ret;
}

int command_servo_off(int ch)
{
	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_OFF;
	ret = canWrite(TxHandle, Txid, data, 0, canMSG_STD);

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

		ret = canWrite(TxHandle, Txid, (unsigned char*)duty, 8, canMSG_STD);
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

		canWrite(TxHandle, Txid, (unsigned char *)pose, 8, canMSG_STD);
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
	ret = canWrite(TxHandle, Txid, (unsigned char *)&msg, 8, canMSG_STD);

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
	ret = canWrite(TxHandle, Txid, (unsigned char *)&msg, 6, canMSG_STD);

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
	ret = canWrite(TxHandle, Txid, (unsigned char *)&msg, 6, canMSG_STD);

	return ret;
}

int request_hand_information(int ch)
{
	long Txid = ID_RTR_HAND_INFO;
	int ret = canWriteRTR(ch, Txid);

	return ret;
}

int request_hand_serial(int ch)
{
	long Txid = ID_RTR_SERIAL;
	int ret = canWriteRTR(ch, Txid);

	return ret;
}

int request_finger_pose(int ch, int findex)
{
	assert(findex >= 0 && findex < NUM_OF_FINGERS);

	long Txid = ID_RTR_FINGER_POSE + findex;
	int ret = canWriteRTR(ch, Txid);

	return ret;
}

int request_imu_data(int ch)
{
	long Txid = ID_RTR_IMU_DATA;
	int ret = canWriteRTR(ch, Txid);

	return ret;
}

int request_temperature(int ch, int sindex)
{
	assert(sindex >= 0 && sindex < NUM_OF_TEMP_SENSORS);

	long Txid = ID_RTR_TEMPERATURE + sindex;
	int ret = canWriteRTR(ch, Txid);

	return ret;
}


int get_message(int ch, int* id, int* len, unsigned char* data, int blocking)
{
	long id_;
	unsigned char rdata[8];
	unsigned int dlc, flags;
	unsigned long timestamp;


	memset(rdata, NULL, sizeof(rdata));
	Status = canRead(TxHandle, &id_, rdata, &dlc, &flags, &timestamp);
	if (Status != 0) return Status;
	//printf("    %ld+%ld (%d)", id-id%128, id%128, dlc);
	//for(int nd=0; nd<(int)dlc; nd++) printf(" %3d ", rdata[nd]);
	//printf("\n");

	*id = id_;
	*len = (int)dlc;
	for(int nd=0; nd<(int)dlc; nd++) data[nd] = rdata[nd];

	return 0;
}



CANAPI_END
