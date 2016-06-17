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

int g_uart3_fileid;

int SerialPort_init()
{
    int fd = -1;
    fd = open("/dev/ttyAMA3", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Open Serial Port Error!\n");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    //115200, 8N1
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME]=0;
    options.c_cc[VMIN]=1;
    tcflush(fd, TCIFLUSH);

    tcsetattr(fd, TCSANOW, &options);

    g_uart3_fileid = fd;

    return 0;
}

int Robot_SetEmotion(int emotion_id)
{
    int count = 0;
    int index = 0;
    unsigned char tx_buffer[] = {0xAA, 0x70, 0x00, 0xCC, 0x33, 0xC3, 0x3C};        
   
    tx_buffer[2] = emotion_id & 0xFF;
    
    write(g_uart3_fileid, (void*)tx_buffer, sizeof(tx_buffer)); 
          
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
    return close(g_uart3_fileid);
}

#ifdef __cplusplus
}
#endif

