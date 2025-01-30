/* user_interface_h */
#include "asr_offline_record_sample.h"
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

/***************************参数配置区域，用户可通过修改这些词来进行
 * ********************************/
#define whether_print_log 0 //是否打印log
#define TIMEOUT 10 //在客户端获取服务端结果时，超时时间

/******麦克风基础功能参数******/
// 传入字节数固定16384B，如果修改长度，则相当于只取一部分音频
int PCM_MSG_LEN = 16384; // //在录音时会发布音频流,单次发布大小为2048B
bool save_pcm_local = false; //保存音频到本地.
int max_pcm_size = 102400000; //最大为10M,超过10M后自动删除,以节省空间.
//录音文件保存的地址,最好设置为绝对地址
// char *ORIGINAL_SOUND_PATH = (char*)"/audio/vvui_ori.pcm";
char *ORIGINAL_SOUND_PATH = (char *)"/audio/";
char *DENOISE_SOUND_PATH = (char *)"/audio/vvui_deno.pcm";
//资源文件存储地址
char *SYSTEM_PATH = (char *)"/tmp/system.tar";
char *SYSTEM_CONFIG_PATH = (char *)"/tmp/config.txt";

/******与离线命令词识别相关参数******/
std::string source_path = "/home/york1to/ws/catkin_ws/src/xf_mic_online";

//运行效果调试参数
int confidence;
char awake_words[30] = "你好小微"; //"hello jack";//"你好小微";
