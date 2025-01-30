/*******************************************************
 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.
********************************************************/
//clang-format off
#include <codecvt>
#include <ctime>
#include <iostream>
#include <locale>
#include <pwd.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <user_interface.h>
#include <xf_mic_online/Pcm_Msg.h>
#include <joint.h>
//clang-format on

ros::Publisher pub_pcm;
// ros::Publisher pub_awake_angle;
// ros::Subscriber sub_record_start;
// ros::Subscriber sub_targrt_led_on;
// ros::Subscriber sub_get_major_mic;

ros::Publisher major_mic_pub;
// ros::Publisher recognise_result_pub;

std::string pcm_topic = "/master/mic/pcm/deno";

std::vector<char> pcm_buf; //音频流缓冲区

bool Get_request_mic_id = false;
bool Set_request_mic_id = false;
bool Set_request_led_id = false;
bool Set_request_awake_word = false;
bool Get_request_awake_angle = false;
using namespace std;

extern UserData asr_data;
extern int whether_finised;
extern char *whole_result;
int write_first_data = 0;
int set_led_id;

char *fileName_ori = join(source_path, ORIGINAL_SOUND_PATH);

// 获取当前系统时间
time_t now = time(0);                         // 时间
std::string time_stamp = std::to_string(now); // 时间转char

/*获取文件大小*/
int FileSize(const char *fname) {
  struct stat statbuf;
  if (stat(fname, &statbuf) == 0)
    return statbuf.st_size;
  return -1;
}

// 获取本地用户名
std::string getUserName() {
  uid_t userid;
  struct passwd *pwd;
  userid = getuid();
  pwd = getpwuid(userid);
  return pwd->pw_name;
}

//判断是否是整数
int isnumber(char *a, int count_need) {
  int len = strlen(a);
  if (len > count_need) {
    return -1;
  }
  int j = 0;
  for (int i = 0; i < len; i++) {
    if (a[i] <= 57 && a[i] >= 48) {
      j++;
    }
  }
  if (j == len) {
    return 0;
  } else {
    return -1;
  }
}

//麦克风通信回调函数
int business_proc_callback(business_msg_t businessMsg) {
  int res = 0;
  // 录制音频的文件名
  char *fileName = join(source_path, DENOISE_SOUND_PATH); // 降噪文件

  // 文件名： 用户名_时间.pcm
  // 获取hostname
  char hostname[1024];
  gethostname(hostname, 1024);
  std::string user = hostname; // 用户名

  char *time_stamp_char;
  time_stamp_char = &time_stamp[0];
  char *file = join(user, time_stamp_char); // 用户名时间
  char *file_format = (char *)".pcm";       // 文件格式
  char *full_name = join(file, file_format);
  char *save_path = join(fileName_ori, full_name);

  // printf("save path : %s", save_path);
  static int index = 0;
  unsigned char buf[4096];
  // printf("business proc modId = %d, msgId = %d, size = %d\n",
  // businessMsg.modId, businessMsg.msgId, businessMsg.length); cout <<
  // save_path << endl;
  switch (businessMsg.modId) {
  case 0x01:
    if (businessMsg.msgId == 0x01) {
      unsigned char key[] = "errcode";
      int status = whether_set_succeed(businessMsg.data, key);
      if (status == 0) {
        printf(">>>>>您已开启录音\n");
      }
    } else if (businessMsg.msgId == 0x02) {
      break;
    } else if (businessMsg.msgId == 0x03) {
      unsigned char key[] = "errcode";
      int status = whether_set_succeed(businessMsg.data, key);
      if (status == 0) {
        //发布关闭前剩余的音频流
        xf_mic_online::Pcm_Msg pcm_data;
        vector<char>::iterator it;
        for (it = pcm_buf.begin(); it != pcm_buf.end(); it++) {
          pcm_data.pcm_buf.push_back(*it);
        }
        pcm_data.length = pcm_buf.size();
        pub_pcm.publish(pcm_data);
        pcm_buf.clear();
        printf(">>>>>您已停止录音\n");
      }
    } else if (businessMsg.msgId == 0x04) {
      unsigned char key[] = "errcode";
      int status = whether_set_succeed(businessMsg.data, key);
      if (status == 0) {
        printf(">>>>>开/关原始音频成功\n");
      }
    } else if (businessMsg.msgId == 0x05) {
      unsigned char key[] = "errcode";
      int status = whether_set_succeed(businessMsg.data, key);
      if (status == 0) {
        // printf(">>>>>设置主麦克风成功\n");
      }
    } else if (businessMsg.msgId == 0x06) {
      get_original_sound(save_path, businessMsg.data);
      int len = PCM_MSG_LEN;
      char *pcm_buffer = new char[len]; //在堆中创建空间
#if whether_print_log
      if (pcm_buffer == NULL) {
        cout << "buffer is null" << endl;
      } else {
        cout << "buffer alloced successfully" << endl;
      }
      // cout << "data size:" << businessMsg.length << "len:" << len << endl;
#endif
      try {
        memcpy(pcm_buffer, businessMsg.data, len);
      } catch (...) {
        cout << ">>>>>拷贝失败" << endl;
      }
      try {
        for (int i = 0; i < len; i++) {
          pcm_buf.push_back(pcm_buffer[i]);
        }
      } catch (...) {
        cout << ">>>>>赋值失败" << endl;
      }
      if (businessMsg.length < len) {
        len = businessMsg.length;
        cout << "businessMsg size is not enough" << endl;
      }
      // 获取音频流，并保存在本地
      // if (save_pcm_local)
      // {
      // 	char *denoise_sound_path = join(source_path,
      // DENOISE_SOUND_PATH);
      // 	// if (-1 != FileSize(denoise_sound_path))
      // 	// {
      // 	// 	int file_size = FileSize(denoise_sound_path);
      // 	// 	if (file_size > max_pcm_size)
      // //超出最大文件限制,将删除,以节省磁盘空间
      // 	// 	{
      // 	// 		remove(denoise_sound_path);
      // 	// 	}
      // 	// }
      // 	get_original_sound(denoise_sound_path, businessMsg.data);
      // }
      if (pcm_buf.size() == PCM_MSG_LEN) //满足指定帧率要求的字节数才发送
      {
        xf_mic_online::Pcm_Msg pcm_data;
        vector<char>::iterator it;
        for (it = pcm_buf.begin(); it != pcm_buf.end(); it++) {
          pcm_data.pcm_buf.push_back(*it);
        }
        pcm_data.length = len;
        pub_pcm.publish(pcm_data);
        pcm_buf.clear();
      }
      delete[] pcm_buffer; //释放创建的堆空间
    } else if (businessMsg.msgId == 0x07) {
      unsigned char key2[] = "beam";
      try {
        int major_id = whether_set_succeed(businessMsg.data, key2);
        major_mic_id = major_id;
        Get_request_mic_id = true;
        printf(">>>>>主麦克风id为%d号麦克风\n", major_mic_id);
      } catch (...) {
        Get_request_mic_id = false;
      }
    } else if (businessMsg.msgId == 0x08) {
      unsigned char key[] = "errcode";
      int status = whether_set_succeed(businessMsg.data, key);
      if (status == 0) {
        Set_request_mic_id = true;
        // printf("\n>>>>>设置主麦克风成功\n");
      } else {
        Set_request_mic_id = false;
      }
    } else if (businessMsg.msgId == 0x09) {
      unsigned char key[] = "errcode";
      int status = whether_set_succeed(businessMsg.data, key);
      if (status == 0) {
        Set_request_led_id = true;
        // printf("\n>>>>>设置灯光成功\n");
      } else {
        Set_request_led_id = false;
      }
    }

    break;
  case 0x02:
    if (businessMsg.msgId == 0x01) {
      unsigned char key1[] = "beam";
      unsigned char key2[] = "angle";
      major_mic_id = get_awake_mic_id(businessMsg.data, key1);
      mic_angle = get_awake_mic_angle(businessMsg.data, key2);
      if (major_mic_id <= 5 && major_mic_id >= 0 && mic_angle <= 360 &&
          mic_angle >= 0) {
        if_awake = 1;
        led_id = get_led_based_angle(mic_angle);
        int ret1 = set_major_mic_id(major_mic_id);
        int ret2 = set_target_led_on(led_id);
        if (ret1 == 0 && ret2 == 0) {
          printf(">>>>>第%d个麦克风被唤醒\n", major_mic_id);
          printf(">>>>>唤醒角度为:%d\n", mic_angle);
          printf(">>>>>已点亮%d灯\n", led_id);
          Get_request_awake_angle = true;
          std_msgs::Int32 awake_angle;
          awake_angle.data = mic_angle;
          // pub_awake_angle.publish(awake_angle);

          std_msgs::Int8 awake_flag_msg;
          awake_flag_msg.data = 1;
          // awake_flag_pub.publish(awake_flag_msg);
          time_t now = time(0);
          time_stamp = std::to_string(now);
          std_msgs::Int8 majormic;
          majormic.data = major_mic_id;
          // major_mic_pub.publish(majormic);

          std_msgs::String msg;
          msg.data = "小车唤醒";
          // voice_words_pub.publish(msg);

          whether_finised = 1;
          set_led_id = led_id;
        }
      }
    }
    break;
  case 0x03:
    if (businessMsg.msgId == 0x01) {
      unsigned char key[] = "status";
      int status = whether_set_succeed(businessMsg.data, key);
      char protocol_version[40];
      int ret = get_protocol_version(businessMsg.data, protocol_version);
      printf(">>>>>麦克风%s,软件版本为:%s,协议版本为:%s\n",
             (status == 0 ? "正常工作" : "正在启动"), get_software_version(),
             protocol_version);
      if (status == 1) {
        char *fileName = join(source_path, SYSTEM_CONFIG_PATH);
        send_resource_info(fileName, 0);
      } else {
        is_boot = 1;
      }
    }
    break;
  case 0x04:
    if (businessMsg.msgId == 0x01) {
      whether_set_resource_info(businessMsg.data);
    } else if (businessMsg.msgId == 0x03) //文件接收结果
    {
      whether_set_resource_info(businessMsg.data);
    } else if (businessMsg.msgId == 0x04) //查看设备升级结果
    {
      whether_upgrade_succeed(businessMsg.data);
    } else if (businessMsg.msgId == 0x05) //下发文件
    {
      char *fileName_system_path = join(source_path, SYSTEM_PATH);
      ;
      send_resource(businessMsg.data, fileName_system_path, 1);
    } else if (businessMsg.msgId == 0x08) //获取升级配置文件
    {
      printf("config.json: %s", businessMsg.data);
    }
    break;
  default:
    break;
  }
  return 0;
}

// 话题形式录制音频
class Listener {
public:
  bool record_state = false;
  int index = 0;

public:
  void callback(const std_msgs::Header::ConstPtr &msg);
};

void Listener::callback(const std_msgs::Header::ConstPtr &msg) {
  if (msg->seq == 1 & record_state == false) {
    record_state = true;
    start_to_record_original_sound();
  } else if (msg->seq == 0 & record_state == true) {
    finish_to_record_original_sound();
    record_state = false;
  }
}

/*程序入口*/
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "voice_control");
  ros::NodeHandle ndHandle("~");
  ndHandle.param("source_path", source_path);
  ros::NodeHandle n;

  /*　topic 发布实时音频文件*/
  pub_pcm = ndHandle.advertise<xf_mic_online::Pcm_Msg>(pcm_topic, 1);

  // 话题形式录制音频
  Listener listener;
  ros::Subscriber sub =
      n.subscribe("switch", 1, &Listener::callback, &listener);

  hid_device *handle = NULL;
  handle = hid_open(); //开启麦克风设备

  if (!handle) {
    printf(">>>>>无法打开麦克风设备，尝试重新连接进行测试\n");
    return -1;
  }
  printf(">>>>>成功打开麦克风设备\n");
  protocol_proc_init(send_to_usb_device, recv_from_usb_device,
                     business_proc_callback, err_proc);
  get_system_status(); //获取麦克风状态，是否正常工作
  sleep(1);
  if (!is_boot) {
    printf(">>>>>开机中，请稍等！\n");
  }
  while (!is_boot) {
    if (is_reboot) {
      break;
    }
  }
  printf(">>>>>开机成功！\n");
  set_awake_word(awake_words);
  set_major_mic_id(0);
  set_target_led_on(0);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  printf(">>>>>设置主麦成功！\n");
  ros::waitForShutdown();
  hid_close();
  return 0;
}
