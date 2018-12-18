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
#include "Softing/Can_def.h"
#include "Softing/CANL2.h"
}
#include "canDef.h"
#include "canAPI.h"

CANAPI_BEGIN


#define CH_COUNT			(int)2 // number of CAN channels
unsigned char CAN_ID = 0;
static CAN_HANDLE hCAN[CH_COUNT] = {-1, -1}; // CAN channel handles

#define	STD		(bool)0
#define	EXT		(bool)1

const char* szCanDevType[] = {
	"",
	"CANcard2",
	"CAN-ACx-PCI",
	"CAN-ACx-PCI/DN",
	"CAN-ACx-104",
	"CANusb",
	"CAN-PROx-PCIe", 
	"CAN-PROx-PC104+",
	"CANpro USB", 
	"EDICcard2",
};

// gets the device name from the device type
char *getDeviceType(int u32DeviceType)
{
  static char u8Type[100];

  switch(u32DeviceType)
  {
  case D_CANCARD2:
    strcpy(u8Type,"CANcard2");
    break;

  case D_CANACPCI:
    strcpy(u8Type,"CAN-AC PCI");
    break;

  case D_CANACPCIDN:
    strcpy(u8Type,"CAN-AC PCI/DN");
    break;

  case D_CANAC104:
    strcpy(u8Type,"CAN-AC PC/104");
    break;

  case D_CANUSB:
    strcpy(u8Type,"CANusb");
    break;

  case D_EDICCARD2:
    strcpy(u8Type,"EDICcard2");
    break;

  case D_CANPROXPCIE:
    strcpy(u8Type,"CANpro PCI Express");
    break;

  case D_CANPROX104:
    strcpy(u8Type,"CANpro PC/104plus");
    break;

  case D_CANECO104:
    strcpy(u8Type,"CAN-ECOx-104");
    break;

  case D_CANFANUPC8:
    strcpy(u8Type,"PC8 onboard CAN");
    break;

  case D_CANPROUSB:
    strcpy(u8Type,"CANpro USB");
    break;

  default:
    strcpy(u8Type,"UNKNOWN");
    break;

  }
  return u8Type;
}

/*==========================================*/
/*       Private functions                  */
/*==========================================*/
int canWrite(CAN_HANDLE handle,
			 unsigned long id, 
			 void * msg,
			 unsigned int dlc,
			 int mode)
{
	if (handle < 0)
		return -1;

	int ret = CANL2_send_data(handle, (id << 2) | CAN_ID, mode, dlc, (unsigned char*)msg);
	if (ret)
	{
		printf("CAN write error\n");
		return ret;
	}

	return 0;
}

int canWriteRTR(CAN_HANDLE handle, unsigned long id, int mode)
{
	if (handle < 0)
		return -1;

	int ret = CANL2_send_remote(handle, (id << 2) | CAN_ID, mode, 0);
	if (ret)
	{
		printf("CAN write error\n");
		return ret;
	}

	return 0;
}


/*========================================*/
/*       Public functions (CAN API)       */
/*========================================*/
int command_can_open(int ch)
{
	assert(ch >= 1 && ch <= CH_COUNT);

	int ret = 0;
	
	////////////////////////////////////////////////////////////////////////
	// Init Channel
	char ch_name[256];
	sprintf_s(ch_name, 256, "CAN-ACx-PCI_%d", ch);
	printf("Open CAN channel: %s...\n", ch_name);

	ret = INIL2_initialize_channel(&hCAN[ch-1], ch_name);
	if (ret)
	{
		printf("\tError: CAN open\n");
		return ret;
	}

	///////////////////////////////////////////////////////////////////////
	// Reset Chip
//	ret = CANL2_reset_chip(hCAN[ch-1]);
//	if (ret)
//	{
//		printf("\tError: CAN reset chip\n");
//		INIL2_close_channel(hCAN[ch-1]);
//		hCAN[ch-1] = 0;
//		return ret;
//	}

	///////////////////////////////////////////////////////////////////////
	// Init Chip
//	ret = CANL2_initialize_chip(hCAN[ch-1], 1, 1, 4, 3, 0);
//	if (ret)
//	{
//		printf("\tError: CAN set baud rate\n");
//		INIL2_close_channel(hCAN[ch-1]);
//		hCAN[ch-1] = 0;
//		return ret;
//	}
	
	///////////////////////////////////////////////////////////////////////
	// Set Out Control
//	ret = CANL2_set_output_control(hCAN[ch-1], -1);

	///////////////////////////////////////////////////////////////////////
	// Enable FIFO
	L2CONFIG L2Config;
	L2Config.fBaudrate = 1000.0;
	L2Config.bEnableAck = false;
	L2Config.bEnableErrorframe = false;
	L2Config.s32AccCodeStd = GET_FROM_SCIM;
	L2Config.s32AccMaskStd = GET_FROM_SCIM;
	L2Config.s32AccCodeXtd = GET_FROM_SCIM;
	L2Config.s32AccMaskXtd = GET_FROM_SCIM;
	L2Config.s32OutputCtrl = GET_FROM_SCIM;
	L2Config.s32Prescaler = GET_FROM_SCIM;
	L2Config.s32Sam = GET_FROM_SCIM;
	L2Config.s32Sjw = GET_FROM_SCIM;
	L2Config.s32Tseg1 = GET_FROM_SCIM;
	L2Config.s32Tseg2 = GET_FROM_SCIM;
	L2Config.hEvent = (void*)-1;
	ret = CANL2_initialize_fifo_mode(hCAN[ch-1], &L2Config);
	if (ret)
	{
		printf("\tError: CAN set fifo mode\n");
		INIL2_close_channel(hCAN[ch-1]);
		hCAN[ch-1] = 0;
		return ret;
	}

	///////////////////////////////////////////////////////////////////////
	// Set Acceptance (Filter)
	unsigned long acc_code[] = {
		(unsigned long)ID_RTR_HAND_INFO,
		(unsigned long)ID_RTR_SERIAL,
		(unsigned long)ID_RTR_FINGER_POSE,
		(unsigned long)ID_RTR_IMU_DATA,
		(unsigned long)ID_RTR_TEMPERATURE
	};
	unsigned int acc_code_count = 5;// sizeof(acc_code);
	for (int i = 0; i < acc_code_count; i++)
	{
		ret = CANL2_set_acceptance(
			hCAN[ch - 1],
			acc_code[i],
			0xfffc,
			0, 0x1fffffff);
		if (ret)
		{
			printf("\tError: CAN set acceptance\n");
			INIL2_close_channel(hCAN[ch - 1]);
			hCAN[ch - 1] = 0;
			return ret;
		}
	}

	return 0;
}

int command_can_open_ex(int ch, int type, int index)
{
	assert(ch >= 1 && ch <= CH_COUNT);
	assert(type >= 1 && type < 9/*eCanDevType_COUNT*/);
	assert(index >= 1 && index <= 8);

	int ret = 0;
	PCHDSNAPSHOT pBuffer = NULL;
	unsigned long u32NeededBufferSize, u32NumOfChannels, u32ProvidedBufferSize, channelIndex;
	int sw_version, fw_version, hw_version, license, chip_type;

	////////////////////////////////////////////////////////////////////////
	// Set buffer size
	u32ProvidedBufferSize = 0;

	// call the function without a valid buffer size first to get the needed buffersize in "u32NeededBufferSize"
	ret = CANL2_get_all_CAN_channels(0, &u32NeededBufferSize, &u32NumOfChannels, NULL);

	if(!u32NumOfChannels)
	{
		printf("you have no Softing CAN interface card plugged in your Computer!\n");
		printf("plug a interface card first and start this program again after this.\n");
		return -1;
	}
	if(ret)
	{
		printf("The driver reported a problem: Error Code %x\n", ret);
		return -1;
	}

	pBuffer = (PCHDSNAPSHOT)malloc(u32NeededBufferSize);
	u32ProvidedBufferSize = u32NeededBufferSize;

	ret = CANL2_get_all_CAN_channels(u32ProvidedBufferSize, &u32NeededBufferSize, &u32NumOfChannels, pBuffer);

	if(ret)
	{
		printf("The driver reported a problem: Error Code %x\n", ret);
		return -1;
	}

	printf("You have %u Softing CAN channels in your system\n\n", u32NumOfChannels);

	printf("\tname\t\t serialnumber\t type\t\t chan.\t    open\n");
	printf("------------------------------------------------------------------------\n");
	printf("\n");

	for (channelIndex=0; channelIndex<u32NumOfChannels; channelIndex++)
	{
		PCHDSNAPSHOT pCh = &pBuffer[channelIndex];

		printf("% 17s\t %09u  % 18s\t %2u\t % 5s\n",
			pCh->ChannelName,
			pCh->u32Serial,
			getDeviceType(pCh->u32DeviceType),
			pCh->u32PhysCh,
			(pCh->bIsOpen) ? "yes" : "no");
	}

	////////////////////////////////////////////////////////////////////////
	// Init Channel
	char ch_name[256];
	sprintf_s(ch_name, 256, "%s_%d", szCanDevType[type], index);
	printf("Open CAN channel[%d]: %s...\n", ch, ch_name);
//hCAN[0] = -1;
	ret = INIL2_initialize_channel(&hCAN[ch-1], ch_name);
	if (ret)
	{
		switch (ret) {
			case -536215551: printf("  Internal Error.\n"); break;
			case -536215550: printf("  General Error.\n"); break;
			case -536215546: printf("  Illegal driver call.\n"); break;
			case -536215542: printf("  Driver not loaded / not installed, or device is not plugged.\n"); break;
			case -536215541: printf("  Out of memory.\n"); break;
			case -536215531: printf("  An error occurred while hooking the interrupt service routine.\n"); break;
			case -536215523: printf("  Device not found.\n"); break;
			case -536215522: printf("  Can not get a free address region for DPRAM from system.\n"); break;
			case -536215521: printf("  Error while accessing hardware.\n"); break;
			case -536215519: printf("  Can not access the DPRAM memory.\n"); break;
			case -536215516: printf("  Interrupt does not work/Interrupt test failed!\n"); break;
			case -536215514: printf("  Device is already open.\n"); break;
			case -536215512: printf("  An incompatible firmware is running on that device. (CANalyzer/CANopen/DeviceNet firmware)\n"); break;
			case -536215511: printf("  Channel can not be accessed, because it is not open.\n"); break;
			case -536215500: printf("  Error while calling a Windows function.\n"); break;
			case -1002:      printf("  Too many open channels.\n"); break;
			case -1003:      printf("  Wrong DLL or driver version.\n"); break;
			case -1004:      printf("  Error while loading the firmware. (This may be a DPRAM access error)\n"); break;
			case -1:         printf("  Function not successful.\n"); break;
		}

		printf("\tError: CAN open\n");
		return ret;
	}

	///////////////////////////////////////////////////////////////////////
	// Reset Chip
//	ret = CANL2_reset_chip(hCAN[ch-1]);
//	if (ret)
//	{
//		printf("\tError: CAN reset chip\n");
//		INIL2_close_channel(hCAN[ch-1]);
//		hCAN[ch-1] = 0;
//		return ret;
//	}

	///////////////////////////////////////////////////////////////////////
	// Init Chip
//	ret = CANL2_initialize_chip(hCAN[ch-1], 1, 1, 4, 3, 0);
//	if (ret)
//	{
//		printf("\tError: CAN set baud rate\n");
//		INIL2_close_channel(hCAN[ch-1]);
//		hCAN[ch-1] = 0;
//		return ret;
//	}
	
	///////////////////////////////////////////////////////////////////////
	// Set Out Control
//	ret = CANL2_set_output_control(hCAN[ch-1], -1);
	
	///////////////////////////////////////////////////////////////////////
	// Enable FIFO
	L2CONFIG L2Config;
	L2Config.fBaudrate = 1000.0;
	L2Config.bEnableAck = false;
	L2Config.bEnableErrorframe = false;
	L2Config.s32AccCodeStd = GET_FROM_SCIM;
	L2Config.s32AccMaskStd = GET_FROM_SCIM;
	L2Config.s32AccCodeXtd = GET_FROM_SCIM;
	L2Config.s32AccMaskXtd = GET_FROM_SCIM;
	L2Config.s32OutputCtrl = GET_FROM_SCIM;
	L2Config.s32Prescaler = GET_FROM_SCIM;
	L2Config.s32Sam = GET_FROM_SCIM;
	L2Config.s32Sjw = GET_FROM_SCIM;
	L2Config.s32Tseg1 = GET_FROM_SCIM;
	L2Config.s32Tseg2 = GET_FROM_SCIM;
	L2Config.hEvent = (void*)-1;
	ret = CANL2_initialize_fifo_mode(hCAN[ch-1], &L2Config);
	if (ret)
	{
		printf("\tError: CAN set fifo mode\n");
		INIL2_close_channel(hCAN[ch-1]);
		hCAN[ch-1] = 0;
		return ret;
	}

	///////////////////////////////////////////////////////////////////////
	// Print driver version info
	ret = CANL2_get_version(hCAN[ch-1], &sw_version, &fw_version, &hw_version, &license, &chip_type);
	if (ret)
	{
		printf("Error %u in CANL2_get_version()\n",ret);
	}
	else
	{
		printf("\n VERSION INFO: \n\n");
		printf("    - Software version: %u.%02u\n", sw_version/100, sw_version%100);
		printf("    - Firmware version: %u.%02u\n", fw_version/100, fw_version%100);
		printf("    - Hardware version: %x.%02x\n", hw_version/0x100, hw_version%0x100);
		printf("    - CAN chip        : %s\n", (chip_type==1000)? "SJA1000": (chip_type==161) ? "Infineon XC161" : "Infineon XE164");
	}

	///////////////////////////////////////////////////////////////////////
	// Set Acceptance (Filter)
	unsigned long acc_code[] = {
		(unsigned long)ID_RTR_HAND_INFO,
		(unsigned long)ID_RTR_SERIAL,
		(unsigned long)ID_RTR_FINGER_POSE,
		(unsigned long)ID_RTR_IMU_DATA,
		(unsigned long)ID_RTR_TEMPERATURE
	};
	unsigned int acc_code_count = sizeof(acc_code);
	for (int i = 0; i < acc_code_count; i++)
	{
		ret = CANL2_set_acceptance(
			hCAN[ch - 1],
			acc_code[i],
			0xfffc,
			0, 0x1fffffff);
		if (ret)
		{
			printf("\tError: CAN set acceptance\n");
			INIL2_close_channel(hCAN[ch - 1]);
			hCAN[ch - 1] = 0;
			return ret;
		}
	}

	return 0;
}

int command_can_reset(int ch)
{
	return -1;
}

int command_can_close(int ch)
{
	INIL2_close_channel(hCAN[ch-1]);
	hCAN[ch-1] = 0;
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
	ret = canWrite(hCAN[ch - 1], Txid, data, 0, STD);

	return ret;
}

int command_servo_off(int ch)
{
	long Txid;
	unsigned char data[8];
	int ret;

	Txid = ID_CMD_SYSTEM_OFF;
	ret = canWrite(hCAN[ch - 1], Txid, data, 0, STD);

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

		ret = canWrite(hCAN[ch - 1], Txid, (unsigned char *)duty, 8, STD);
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

		ret = canWrite(hCAN[ch - 1], Txid, (unsigned char *)pose, 8, STD);
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
	ret = canWrite(hCAN[ch - 1], Txid, (unsigned char *)&msg, 6, STD);

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
	ret = canWrite(hCAN[ch - 1], Txid, (unsigned char *)&msg, 6, STD);

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
	ret = canWrite(hCAN[ch - 1], Txid, (unsigned char *)&msg, 6, STD);

	return ret;
}

int request_hand_information(int ch)
{
	long Txid = ID_RTR_HAND_INFO;
	int ret = canWriteRTR(ch, Txid, STD);

	return ret;
}

int request_hand_serial(int ch)
{
	long Txid = ID_RTR_SERIAL;
	int ret = canWriteRTR(ch, Txid, STD);

	return ret;
}

int request_finger_pose(int ch, int findex)
{
	assert(findex >= 0 && findex < NUM_OF_FINGERS);

	long Txid = ID_RTR_FINGER_POSE + findex;
	int ret = canWriteRTR(ch, Txid, STD);

	return ret;
}

int request_imu_data(int ch)
{
	long Txid = ID_RTR_IMU_DATA;
	int ret = canWriteRTR(ch, Txid, STD);

	return ret;
}

int request_temperature(int ch, int sindex)
{
	assert(sindex >= 0 && sindex < NUM_OF_TEMP_SENSORS);

	long Txid = ID_RTR_TEMPERATURE + sindex;
	int ret = canWriteRTR(ch, Txid, STD);

	return ret;
}

int get_message(int ch, int* id, int* len, unsigned char* data, int blocking)
{
	int ret;
	PARAM_STRUCT param;
	
	ret = CANL2_read_ac(hCAN[ch-1], &param);

	switch (ret)
	{
	case CANL2_RA_XTD_DATAFRAME :
		*id = (param.Ident & 0xfffffffc) >> 2;
		*len = param.DataLength;
		data[0] = param.RCV_data[0];
		data[1] = param.RCV_data[1];
		data[2] = param.RCV_data[2];
		data[3] = param.RCV_data[3];
		data[4] = param.RCV_data[4];
		data[5] = param.RCV_data[5];
		data[6] = param.RCV_data[6];
		data[7] = param.RCV_data[7];
		break;

	case CANL2_RA_DATAFRAME:
		*id = (param.Ident & 0xfffffffc) >> 2;
		*len = param.DataLength;
		data[0] = param.RCV_data[0];
		data[1] = param.RCV_data[1];
		data[2] = param.RCV_data[2];
		data[3] = param.RCV_data[3];
		data[4] = param.RCV_data[4];
		data[5] = param.RCV_data[5];
		data[6] = param.RCV_data[6];
		data[7] = param.RCV_data[7];
		break;

	case CANL2_RA_NO_DATA:
		return -1;

	default:
		return -1;
	}

	return 0;
}



CANAPI_END
