

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
#else
#include <windows.h>
#endif
#include <malloc.h>
#include <assert.h>
//project headers
extern "C" {
#include "EasySYNC/USBCanPlusDllF.h"
}
#include "canDef.h"
#include "canAPI.h"


CANAPI_BEGIN

/*=========================================*/
/*       Global file-scope variables       */
/*=========================================*/

CANHANDLE canDev[MAX_BUS] = { 0, };

/*==========================================*/
/*       Private functions prototypes       */
/*==========================================*/
CANHANDLE initCAN(int bus){
	char szAdapter[10];
	const char szBitrate[] = "1000";
	const char szAcceptanceCode[] = "000"; 
	const char szAcceptanceMask[] = "000"; 
	int numAdapters;
	CANHANDLE handle;
	CAN_STATUS status = canplus_getFirstAdapter(szAdapter, 10);
	if (status <= 0) {
		printf("initCAN(): canplus_getFirstAdapter() failed with error %ld\n", status);
		switch (status) {
			case ERROR_CANPLUS_MEMORY_ERROR:		printf("szAdapter buffer size (size variable) is less than 9Bytes.");
			case ERROR_CANPLUS_FAIL:				printf("Unable to get information about adapters.");
			case ERROR_CANPLUS_NO_DEVICE:			printf("No devices found.");
			default: printf("Unknown error");
		}
		return -1;
	}
	numAdapters = status;
	printf("initCAN(): Number of CanPlus Adapters %d\n", numAdapters);
	printf("initCAN(): Serial number of first adapter %s\n", szAdapter);

	while (bus > 0) {
		status = canplus_getNextAdapter(szAdapter, 10);
		if (status <= 0) {
			printf("initCAN(): canplus_getNextAdapter() failed with error %ld\n", status);
			switch (status) {
				case ERROR_CANPLUS_MEMORY_ERROR:		printf("szAdapter buffer size (size variable) is less than 9Bytes.");
				case ERROR_CANPLUS_FAIL:				printf("canplus_getFirstAdapter was not called.");
				case ERROR_CANPLUS_NO_DEVICE:			printf("No devices found.");
				default: printf("Unknown error");
			}
			return -1;
		}
		printf("initCAN(): Serial number of next adapter %s\n", szAdapter);
		bus--;
	}

	handle = canplus_Open(
		szAdapter, //NULL: select the first USB2-F-7x01 adapter. 
		szBitrate, 
		NULL/*szAcceptanceCode*/, //11-bit (0x000 to 0x7FF) or 29-bit (0x00000000 to 0x1FFFFFFF) code used in filtering specific or ranges of CAN messages.
		NULL/*szAcceptanceMask*/, //11-bit (0x000 to 0x7FF) or 29-bit (0x00000000 to 0x1FFFFFFF) code used in filtering specific or ranges of CAN messages
		0x0 // CANPLUS_FLAG_TIMESTAMP: The timestamp function will be enabled by the USB2-F-7x01
		);
	if (handle <= 0) {
		printf("initCAN(): canplus_Open() failed with error %ld\n", handle);
		switch (status) {
			case ERROR_CANPLUS_FAIL:				printf("Unable to open communication to USB2-F-7x01 device.");
			case ERROR_CANPLUS_OPEN_SUBSYSTEM:		printf("Unable to open CAN channel.");
			case ERROR_CANPLUS_COMMAND_SUBSYSTEM:	printf("Failed in setting other parameters (e.g. setting timestamp, bitrate, acceptance code or acceptance mask)");
			default: printf("Unknown error");
		}
		return -1;
	}

	status = canplus_SetTimeouts(handle, 1, 1);
	if (status <= 0) {
		printf("initCAN(): canplus_SetTimeouts() failed with error %ld\n", status);
		switch (status) {
			case ERROR_CANPLUS_FAIL:				printf("CAN channel timeouts could not be configured.");
			default: printf("Unknown error");
		}
		return -1;
	}

	return handle;
}

int freeCAN(CANHANDLE h){
	CAN_STATUS status;

	status = canplus_Close(h);
	if (status <= 0) {
		printf("freeCAN(): canplus_Close() failed with error %ld\n", status);
		switch (status) {
			case ERROR_CANPLUS_FAIL:				printf("CAN channel could not be closed.");
			default: printf("Unknown error");
		}
		return -1;
	}

	return 0;
}

int resetCAN(CANHANDLE h){
	CAN_STATUS status;

	status = canplus_Reset(h);
	if (status <= 0) {
		printf("resetCAN(): canplus_Reset() failed with error %ld\n", status);
		switch (status) {
			case ERROR_CANPLUS_FAIL:				printf("CAN channel could not be reset.");
			default: printf("Unknown error");
		}
		return -1;
	}

	return 0;
}

int canReadMsg(CANHANDLE h, int *id, int *len, unsigned char *data, int blocking){
	CANMsg msg;
	CAN_STATUS status;
	int i;

	// We execute the "Read" function
	//
	status = canplus_Read(h, &msg);
	if (status == ERROR_CANPLUS_NO_MESSAGE) {
		//printf("canReadMsg(): The receive buffer is empty.\n");
		return 0;
	}
	else if (status < 0) {
		printf("canReadMsg(): canplus_Read() failed with error %ld\n", status);
		switch (status) {
			default: printf("Unknown error");
		}
		return status;
	}

	*id = msg.id;
	*len = msg.len;
	for(i = 0; i < msg.len; i++)
		data[i] = msg.data[i];

	return 0;
}

int canSendMsg(CANHANDLE h, int id, char len, unsigned char *data, int blocking){
	CANMsg msg;
	CAN_STATUS status;
	int i;

	msg.id = id;
	msg.len = len & 0x0F;
	for(i = 0; i < len; i++)
        msg.data[i] = data[i];
	msg.flags = 0x0; // CANMSG_EXTENDED: 0x80, CANMSG_RTR: 0x40

	status = canplus_Write(h, &msg);
	if (status <= 0)
	{
		printf("canSendMsg(): canplus_Write() failed with error %ld\n", status);
		switch (status) {
			case ERROR_CANPLUS_FAIL:				printf("Standard/Extended Frame write Failure.");
			default: printf("Unknown error");
		}
		return status;
	}

	return 0;
}

int canSendMsgRTR(CANHANDLE h, int id, int blocking) {
	CANMsg msg;
	CAN_STATUS status;
	int i;

	msg.id = id;
	msg.len = 0;
	msg.flags = 0x40; // CANMSG_EXTENDED: 0x80, CANMSG_RTR: 0x40

	status = canplus_Write(h, &msg);
	if (status <= 0)
	{
		printf("canSendMsgRTR(): canplus_Write() failed with error %ld\n", status);
		switch (status) {
		case ERROR_CANPLUS_FAIL:				printf("Standard/Extended Frame write Failure.");
		default: printf("Unknown error");
		}
		return status;
	}

	return 0;
}

/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
int command_can_open(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	CANHANDLE ret;

	printf("<< CAN: Open Channel...\n");
	ret = initCAN(ch);
	if (ret < 0) return ret;
	canDev[ch] = ret;
	printf("\t- Ch.%2d (OK)\n", ch);
	printf("\t- Done\n");

	return 0;
}

int command_can_open_ex(int ch, int type, int index)
{
	return command_can_open(ch);
}

int command_can_reset(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	printf("<< CAN: Reset...\n");

	int status = resetCAN(canDev[ch]);
	if (status < 0)
		return status;

	printf("\t- Done\n");
	return -1;
}

int command_can_close(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	printf("<< CAN: Close...\n");

	int status = freeCAN(canDev[ch]);
	if (status < 0)
		return status;

	printf("\t- Done\n");
	canDev[ch] = -1;
	return 0;
}

int command_can_set_id(int ch, unsigned char can_id)
{
	return 0;
}

int command_servo_on(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_ON;
	ret = canSendMsg(canDev[ch], Txid, 0, data, TRUE);

	return ret;
}

int command_servo_off(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_OFF;
	ret = canSendMsg(canDev[ch], Txid, 0, data, TRUE);

	return ret;
}

int command_set_torque(int ch, int findex, short* pwm)
{
	assert(ch >= 0 && ch < MAX_BUS);
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

		ret = canSendMsg(canDev[ch], Txid, 8, (unsigned char *)duty, TRUE);
	}
	else
		return -1;

	return ret;
}

int command_set_pose(int ch, int findex, short* jposition)
{
	assert(ch >= 0 && ch < MAX_BUS);
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

		ret = canSendMsg(canDev[ch], Txid, 8, (unsigned char *)pose, TRUE);
	}
	else
		return -1;

	return ret;
}

int command_set_period(int ch, short* period)
{
	assert(ch >= 0 && ch < MAX_BUS);

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
	ret = canSendMsg(canDev[ch], Txid, 6, (unsigned char *)&msg, TRUE);

	return ret;
}

int command_set_device_id(int ch, unsigned char did)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid;
	int ret;
	can_config_msg_t msg;

	Txid = ID_CMD_CONFIG;
	msg.set = 0x01;
	msg.did = did;
	msg.baudrate = 0;
	ret = canSendMsg(canDev[ch], Txid, 6, (unsigned char *)&msg, TRUE);

	return ret;
}

int command_set_rs485_baudrate(int ch, unsigned int baudrate)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid;
	int ret;
	can_config_msg_t msg;

	Txid = ID_CMD_CONFIG;

	msg.set = 0x02;
	msg.did = 0;
	msg.baudrate = baudrate;
	ret = canSendMsg(canDev[ch], Txid, 6, (unsigned char *)&msg, TRUE);

	return ret;
}

int request_hand_information(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid = ID_RTR_HAND_INFO;
	int ret = canSendMsgRTR(canDev[ch], Txid, TRUE);

	return ret;
}

int request_hand_serial(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid = ID_RTR_SERIAL;
	int ret = canSendMsgRTR(canDev[ch], Txid, TRUE);

	return ret;
}

int request_finger_pose(int ch, int findex)
{
	assert(ch >= 0 && ch < MAX_BUS);
	assert(findex >= 0 && findex < NUM_OF_FINGERS);

	long Txid = ID_RTR_FINGER_POSE + findex;
	int ret = canSendMsgRTR(canDev[ch], Txid, TRUE);

	return ret;
}

int request_imu_data(int ch)
{
	assert(ch >= 0 && ch < MAX_BUS);

	long Txid = ID_RTR_IMU_DATA;
	int ret = canSendMsgRTR(canDev[ch], Txid, TRUE);

	return ret;
}

int request_temperature(int ch, int sindex)
{
	assert(ch >= 0 && ch < MAX_BUS);
	assert(sindex >= 0 && sindex < NUM_OF_TEMP_SENSORS);

	long Txid = ID_RTR_TEMPERATURE + sindex;
	int ret = canSendMsgRTR(canDev[ch], Txid, TRUE);

	return ret;
}

int get_message(int ch, int* id, int* len, unsigned char* data, int blocking)
{
	int err;
	unsigned long Rxid;

	err = canReadMsg(canDev[ch], id, len, data, TRUE);
	if (!err)
	{
		/*printf("    %ld+%ld (%d)", Rxid-Rxid%128, Rxid%128, len);
		for(int nd=0; nd<(*len); nd++) printf(" %3d ", data[nd]);
		printf("\n");*/
	}
	else
	{
		return err;
	}
	return 0;
}



CANAPI_END
