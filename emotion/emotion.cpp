#include <jni.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include "com_robot_et_core_hardware_emotion_Emotion.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define EMOTION_BLINK                (1)
#define EMOTION_PLEASURE             (2)
#define EMOTION_ANGER                (3)
#define EMOTION_SORROW               (4)
#define EMOTION_JOY                  (5)
#define EMOTION_LOOKLEFT             (6)
#define EMOTION_LOOKRIGHT            (7)
#define EMOTION_TEARS                (8)

int g_uart2_fileid;
int g_uart3_fileid;

int SerialPort_init()
{
    int fd = -1;
    struct termios options;
    
    //115200, 8N1
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME]=0;
    options.c_cc[VMIN]=1;    

    fd = open("/dev/ttyAMA2", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Open Serial Port Error!\n");
        return -1;
    }

	tcgetattr(fd, &options);
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);
	g_uart2_fileid = fd;

    fd = open("/dev/ttyAMA3", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Open Serial Port Error!\n");
        return -1;
    }

	tcgetattr(fd, &options);
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);
	g_uart3_fileid = fd;

    return 0;
}

void emotion_blink()
{
	unsigned char tx_buffer[] = {0xAA, 0x70, 0x00, 0xCC, 0x33, 0xC3, 0x3C};
	
	tx_buffer[2] = 0x00;
	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	
	usleep(400000);
	
	tx_buffer[2] = 0x01;
	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));	
	
	usleep(400000);
	
	tx_buffer[2] = 0x00;
	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	
//	usleep(400000);
	
//	tx_buffer[2] = 0x01;
//	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
//	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));	
	
//	usleep(400000);
	
//	tx_buffer[2] = 0x00;
//	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
//	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));		
}

void emotion_pleasure()
{
	unsigned int index;
	unsigned char tx_buffer[] = {0xAA, 0x70, 0x00, 0xCC, 0x33, 0xC3, 0x3C};
	
	for (index = 0; index < 2; index++)
	{
		tx_buffer[2] = 0x02;
		write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
		write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	
		usleep(400000);

		tx_buffer[2] = 0x03;
		write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
		write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	
		usleep(400000);

		tx_buffer[2] = 0x04;
		write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
		write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	
		usleep(400000);

		tx_buffer[2] = 0x05;
		write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
		write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	
		usleep(400000);
	}

	tx_buffer[2] = 0x00;
	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));	
}

void emotion_anger()
{
	unsigned char tx_buffer[] = {0xAA, 0x70, 0x00, 0xCC, 0x33, 0xC3, 0x3C};

	tx_buffer[2] = 0x06;
	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));

	usleep(500000);

	tx_buffer[2] = 0x00;
	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));

}

void emotion_sorrow()
{
	unsigned int index;
	unsigned char tx_buffer[] = {0xAA, 0x70, 0x00, 0xCC, 0x33, 0xC3, 0x3C};

	for (index = 0; index < 2; index++)
	{
		tx_buffer[2] = 0x07;
		write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
		write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));

		usleep(400000);

		tx_buffer[2] = 0x08;
		write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
		write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));

		usleep(400000);
	}

	tx_buffer[2] = 0x00;
	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));	
}

void emotion_joy()
{

}

void emotion_lookleft()
{
	unsigned char tx_buffer[] = {0xAA, 0x70, 0x00, 0xCC, 0x33, 0xC3, 0x3C};

	tx_buffer[2] = 0x09;
	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));
}

void emotion_lookright()
{
	unsigned char tx_buffer[] = {0xAA, 0x70, 0x00, 0xCC, 0x33, 0xC3, 0x3C};

	tx_buffer[2] = 0xa;
	write(g_uart2_fileid, (void*)tx_buffer, sizeof(tx_buffer));
	write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer));
}

/*
 * 1: 平常（眨眼）
 * 2：高兴
 * 3：生气
 * 4：难过
 * 5：开心
 * 6：往左看
 * 7：往右看
 * 8：流泪
 */
int Robot_SetEmotion(int emotion_id)
{
    int count = 0;
    int index = 0;
                  
	switch ((char)emotion_id & 0xFF)
	{
		case EMOTION_BLINK:
			emotion_blink();
			break;
			
		case EMOTION_PLEASURE:
			emotion_pleasure();
			break;

		case EMOTION_ANGER:
			emotion_anger();
			break;	

		case EMOTION_SORROW:
			emotion_sorrow();
			break;

		case EMOTION_JOY:
			emotion_joy();
			break;

		case EMOTION_LOOKLEFT:
			emotion_lookleft();
			break;

		case EMOTION_LOOKRIGHT:
			emotion_lookright();
			break;

		case EMOTION_TEARS:
			//emotion_tears();
			break;

		default:
			break;
	}         
          
    return 0;
}

JNIEXPORT jint JNICALL Java_com_robot_et_core_hardware_emotion_Emotion_init
(JNIEnv *env, jclass cls)
{
    return SerialPort_init();
}

JNIEXPORT jint JNICALL Java_com_robot_et_core_hardware_emotion_Emotion_setEmotion
(JNIEnv *env, jclass cls, jint emotion_id)
{
    return Robot_SetEmotion(emotion_id);
}

JNIEXPORT jint JNICALL Java_com_robot_et_core_hardware_emotion_Emotion_close
(JNIEnv *env, jclass cls)
{
    return close(g_uart3_fileid) | close(g_uart2_fileid);
}

#ifdef __cplusplus
}
#endif

