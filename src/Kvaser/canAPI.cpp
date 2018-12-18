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

#include "canDef.h"
#include "canAPI.h"
#include "Kvaser/canlib.h"

CANAPI_BEGIN


#define CH_COUNT			(int)2 // number of CAN channels
unsigned char CAN_ID = 0;
static int hCAN[CH_COUNT] = {-1, -1}; // CAN channel handles

#define	STD		(bool)0
#define	EXT		(bool)1


/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
int command_can_open(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	canStatus ret;
	
	printf("<< CAN: Initialize Library...\n");
	canInitializeLibrary();
	printf("\t- Done\n");
	Sleep(200);

	printf("<< CAN: Open Channel...\n");
	hCAN[ch] = canOpenChannel(ch, canOPEN_EXCLUSIVE);
	if (hCAN[ch] < 0) return -1;
	printf("\t- Ch.%2d (OK)\n", ch);
	printf("\t- Done\n");
	Sleep(200);

	printf("<< CAN: Set Bus Parameter...\n");
	ret = canSetBusParams(hCAN[ch], BAUD_1M, 0, 0, 0, 0, 0);
	if (ret < 0) return -2;
	printf("\t- Done\n");
	Sleep(200);

	printf("<< CAN: Bus On...\n");
	ret = canBusOn(hCAN[ch]);
	if (ret < 0) return -3;
	printf("\t- Done\n");
	Sleep(200);

	return 0;
}

int command_can_open_ex(int ch, int type, int index)
{
	return command_can_open(ch);
}

int command_can_reset(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	canStatus ret;

	printf("<< CAN: Reset Bus...\n");
	ret = canResetBus(hCAN[ch]);
	if (ret < 0) return ret;
	printf("\t- Done\n");
	Sleep(200);

	return 0;
}

int command_can_close(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	canStatus ret;

	printf("<< CAN: Close...\n");
	ret = canClose(hCAN[ch]);
	if (ret < 0) return ret;
	hCAN[ch] = 0;
	printf("\t- Done\n");
	Sleep(200);
	return 0;
}

int command_can_set_id(int ch, unsigned char can_id)
{
	CAN_ID = can_id;
	return 0;
}

int command_servo_on(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_ON;
	ret = canWrite(hCAN[ch - 1], (Txid << 2) | CAN_ID, data, 0, STD);

	return ret;
}

int command_servo_off(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_OFF;
	ret = canWrite(hCAN[ch - 1], (Txid << 2) | CAN_ID, data, 0, STD);

	return ret;
}

int command_set_torque(int ch, int findex, short* pwm)
{
	assert(ch >= 0 && ch < CH_COUNT);
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

		ret = canWrite(hCAN[ch - 1], (Txid << 2) | CAN_ID, (unsigned char *)duty, 8, STD);
	}
	else
		return -1;

	return ret;
}

int command_set_pose(int ch, int findex, short* jposition)
{
	assert(ch >= 0 && ch < CH_COUNT);
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

		ret = canWrite(hCAN[ch - 1], (Txid << 2) | CAN_ID, (unsigned char *)pose, 8, STD);
	}
	else
		return -1;

	return ret;
}

int command_set_period(int ch, short* period)
{
	assert(ch >= 0 && ch < CH_COUNT);

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
	ret = canWrite(hCAN[ch - 1], (Txid << 2) | CAN_ID, (unsigned char *)&msg, 6, STD);

	return ret;
}

int command_set_device_id(int ch, unsigned char did)
{
	assert(ch >= 0 && ch < CH_COUNT);

	long Txid;
	int ret;
	can_config_msg_t msg;

	Txid = ID_CMD_CONFIG;
	msg.set = 0x01;
	msg.did = did;
	msg.baudrate = 0;
	ret = canWrite(hCAN[ch - 1], (Txid << 2) | CAN_ID, (unsigned char *)&msg, 6, STD);

	return ret;
}

int command_set_rs485_baudrate(int ch, unsigned int baudrate)
{
	assert(ch >= 0 && ch < CH_COUNT);

	long Txid;
	int ret;
	can_config_msg_t msg;

	Txid = ID_CMD_CONFIG;

	msg.set = 0x02;
	msg.did = 0;
	msg.baudrate = baudrate;
	ret = canWrite(hCAN[ch - 1], (Txid << 2) | CAN_ID, (unsigned char *)&msg, 6, STD);

	return ret;
}

int request_hand_information(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	long Txid = ID_RTR_HAND_INFO;
	int ret = canWrite(ch, (Txid << 2) | CAN_ID, NULL, 0, STD);

	return ret;
}

int request_hand_serial(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	long Txid = ID_RTR_SERIAL;
	int ret = canWrite(ch, (Txid << 2) | CAN_ID, NULL, 0, STD);

	return ret;
}

int request_finger_pose(int ch, int findex)
{
	assert(findex >= 0 && findex < NUM_OF_FINGERS);

	long Txid = ID_RTR_FINGER_POSE + findex;
	int ret = canWrite(ch, (Txid << 2) | CAN_ID, NULL, 0, STD);

	return ret;
}

int request_imu_data(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	long Txid = ID_RTR_IMU_DATA;
	int ret = canWrite(ch, (Txid << 2) | CAN_ID, NULL, 0, STD);

	return ret;
}

int request_temperature(int ch, int sindex)
{
	assert(ch >= 0 && ch < CH_COUNT);

	assert(sindex >= 0 && sindex < NUM_OF_TEMP_SENSORS);

	long Txid = ID_RTR_TEMPERATURE + sindex;
	int ret = canWrite(ch, (Txid << 2) | CAN_ID, NULL, 0, STD);

	return ret;
}

int get_message(int ch, int* id, int* len, unsigned char* data, int blocking)
{
	long Rxid;
	unsigned char rdata[8];
	unsigned int dlc;
	unsigned int flag;
	unsigned long time;
	canStatus ret;

	memset(rdata, NULL, sizeof(rdata));
	ret = canRead(hCAN[ch], &Rxid, rdata, &dlc, &flag, &time);
	if (ret != canOK) return ret;
	//printf("    %ld+%ld (%d)", Rxid-Rxid%128, Rxid%128, dlc);
	//for(int nd=0; nd<(int)dlc; nd++) printf(" %3d ", rdata[nd]);
	//printf("\n");

	*id = (Rxid & 0xfffffffc) >> 2;
	*len = (int)dlc;
	for(int nd=0; nd<(int)dlc; nd++) data[nd] = rdata[nd];

	return 0;
}



CANAPI_END
