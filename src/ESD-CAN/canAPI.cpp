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
#include "ESD-CAN/ntcan.h"

CANAPI_BEGIN


#define CH_COUNT			(int)4 // number of CAN channels

unsigned char CAN_ID = 0;

static NTCAN_HANDLE canDev[CH_COUNT] = { // CAN channel handles
	(NTCAN_HANDLE)-1,
	(NTCAN_HANDLE)-1,
	(NTCAN_HANDLE)-1,
	(NTCAN_HANDLE)-1
}; 

/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
void allowMessage(int bus, int id, int mask){
    int i;
    DWORD retvalue;
	for(i=0;i<2048;i++){
		if((i & ~mask)==id){
			retvalue = canIdAdd(canDev[bus],i);
			if(retvalue != NTCAN_SUCCESS){
#ifndef _WIN32
			  syslog(LOG_ERR, "allowMessage(): canIdAdd() failed with error %d", retvalue);
#endif
			  printf("allowMessage(): canIdAdd() failed with error %ld", retvalue);
			}
		}
	}
}

int initCAN(int bus){
    DWORD retvalue;
#ifndef _WIN32
    pthread_mutex_init(&commMutex, NULL);
#endif
    
    retvalue = canOpen(bus, 0, TX_QUEUE_SIZE, RX_QUEUE_SIZE, TX_TIMEOUT, RX_TIMEOUT, &canDev[bus]);
    if(retvalue != NTCAN_SUCCESS){
#ifndef _WIN32
        syslog(LOG_ERR, "initCAN(): canOpen() failed with error %d", retvalue);
#endif
		printf("initCAN(): canOpen() failed with error %ld", retvalue);
        return(1);
    }
    
    retvalue = canSetBaudrate(canDev[bus], 0); // 1 = 1Mbps, 2 = 500kbps, 3 = 250kbps
    if(retvalue != 0)
    {
#ifndef _WIN32
        syslog(LOG_ERR, "initCAN(): canSetBaudrate() failed with error %d", retvalue);
#endif
		printf("initCAN(): canSetBaudrate() failed with error %ld", retvalue);
        return(1);
    }
    
    // Mask 7FC: 0000 0111 1111 1100
    allowMessage(bus, ID_RTR_HAND_INFO, 0x07fc);
	allowMessage(bus, ID_RTR_SERIAL, 0x07fc);
	allowMessage(bus, ID_RTR_FINGER_POSE, 0x07fc);
	allowMessage(bus, ID_RTR_IMU_DATA, 0x07fc);
	allowMessage(bus, ID_RTR_TEMPERATURE, 0x07e0);

    return(0);
}

void freeCAN(int bus){
    canClose(canDev[bus]);
}

int canReadMsg(int bus, int *id, int *len, unsigned char *data, int blocking){
    CMSG    msg;
    DWORD   retvalue;
    long    msgCt = 1;
    int     i;
    
    if(blocking){
        retvalue = canRead(canDev[bus], &msg, &msgCt, NULL);
    }else{
        retvalue = canTake(canDev[bus], &msg, &msgCt);
    }
    if(retvalue != NTCAN_SUCCESS){
#ifndef _WIN32
        syslog(LOG_ERR, "canReadMsg(): canRead/canTake error: %ld", retvalue);
#endif
		//printf("canReadMsg(): canRead/canTake error: %ld", retvalue);
        if(retvalue == NTCAN_RX_TIMEOUT)
            return(1);
        else
            return(2);
    }
    if(msgCt == 1){
		*id = (msg.id & 0xfffffffc) >> 2;
        *len = msg.len;
        for(i = 0; i < msg.len; i++)
            data[i] = msg.data[i];
            
        return(0);
    }
    
    return(1); // No message received, return err
}

int canSendMsg(int bus, int id, char len, unsigned char *data, int blocking){
    CMSG    msg;
    DWORD   retvalue;
    long    msgCt = 1;
    int     i;
    
	msg.id = (id << 2) | CAN_ID;
    msg.len = len & 0x0F;
    for(i = 0; i < len; i++)
        msg.data[i] = data[i];
    
    if(blocking){
        retvalue = canWrite(canDev[bus], &msg, &msgCt, NULL);
    }else{
        retvalue = canSend(canDev[bus], &msg, &msgCt);
    }
    
    if(retvalue != NTCAN_SUCCESS){
#ifndef _WIN32
        syslog(LOG_ERR, "canSendMsg(): canWrite/Send() failed with error %d", retvalue);
#endif
		printf("canSendMsg(): canWrite/Send() failed with error %ld", retvalue);
        return(1);
    }
    return 0;
}

int canSendMsgRTR(int bus, int id, int blocking)
{
	return 0;
}

/*========================================*/
/*       CAN API                          */
/*========================================*/
int command_can_open(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	DWORD ret;

	printf("<< CAN: Open Channel...\n");
	ret = initCAN(ch);
	if (ret != 0) return ret;
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
	return -1;
}

int command_can_close(int ch)
{
	assert(ch >= 0 && ch < CH_COUNT);

	DWORD ret;

	printf("<< CAN: Close...\n");
	ret = canClose(canDev[ch]);
	if (ret != 0) return ret;
	canDev[ch] = (NTCAN_HANDLE)-1;
	printf("\t- Done\n");

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
	ret = canSendMsg(ch, Txid, 0, data, TRUE);
	
	return ret;
}

int command_servo_off(int ch)
{
	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_OFF;
	ret = canSendMsg(ch, Txid, 0, data, TRUE);

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

		ret = canSendMsg(ch, Txid, 8, (unsigned char*)duty, TRUE);
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

		ret = canSendMsg(ch, Txid, 8, (unsigned char*)pose, TRUE);
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
	ret = canSendMsg(ch, Txid, 6, (unsigned char*)&msg, TRUE);

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
	ret = canSendMsg(ch, Txid, 6, (unsigned char*)&msg, TRUE);

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
	ret = canSendMsg(ch, Txid, 6, (unsigned char*)&msg, TRUE);

	return ret;
}

int request_hand_information(int ch)
{
	long Txid = ID_RTR_HAND_INFO;
	int ret = canSendMsgRTR(ch, Txid, TRUE);

	return ret;
}

int request_hand_serial(int ch)
{
	long Txid = ID_RTR_SERIAL;
	int ret = canSendMsgRTR(ch, Txid, TRUE);

	return ret;
}

int request_finger_pose(int ch, int findex)
{
	assert(findex >= 0 && findex < NUM_OF_FINGERS);

	long Txid = ID_RTR_FINGER_POSE + findex;
	int ret = canSendMsgRTR(ch, Txid, TRUE);

	return ret;
}

int request_imu_data(int ch)
{
	long Txid = ID_RTR_IMU_DATA;
	int ret = canSendMsgRTR(ch, Txid, TRUE);

	return ret;
}

int request_temperature(int ch, int sindex)
{
	assert(sindex >= 0 && sindex < NUM_OF_TEMP_SENSORS);

	long Txid = ID_RTR_TEMPERATURE + sindex;
	int ret = canSendMsgRTR(ch, Txid, TRUE);

	return ret;
}

int get_message(int ch, int* id, int* len, unsigned char* data, int blocking)
{
	int Rxid;
	unsigned char rdata[8];
	int dlc;
	int ret;

	memset(rdata, NULL, sizeof(rdata));
	ret = canReadMsg(ch, &Rxid, &dlc, rdata, TRUE);
	if (ret != 0) return ret;
	//printf("    %ld+%ld (%d)", Rxid-Rxid%128, Rxid%128, dlc);
	//for(int nd=0; nd<(int)dlc; nd++) printf(" %3d ", rdata[nd]);
	//printf("\n");

	*id = Rxid;
	*len = (int)dlc;
	for(int nd=0; nd<(int)dlc; nd++) data[nd] = rdata[nd];

	return 0;
}



CANAPI_END
