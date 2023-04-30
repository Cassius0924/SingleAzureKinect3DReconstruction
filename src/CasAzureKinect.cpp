//
// Created by Cassius0924 on 4/30/23.
//

#include "CasAzureKinect.h"

#include <alsa/asoundlib.h>
#include <iostream>

#define BUFFER_SIZE 8192
#define MICROPHONE_NAME "plughw:2,0"// 麦克风名称：声卡2的0号设备


using namespace std;

bool cas::kinect::checkKinectNum(int num) {
    const uint32_t device_count = k4a::device::get_installed_count();
    if (0 == device_count) {
        cerr << "错误：没有发现 K4A 设备。" << endl;
        return false;
    } else {
        cout << "发现 " << device_count << " 个已连接的设备。" << endl;
        if (1 != device_count) {// 超过1个设备，也输出错误信息。
            cerr << "错误：发现多个 K4A 设备。" << endl;
            return false;
        } else {
            cout << "发现 1 个 K4A 设备。" << endl;
        }
    }
    return true;
}

bool cas::kinect::stabilizeCamera(k4a::device &device) {
    k4a::capture capture;
    int i_auto = 0;      // 用来稳定，类似自动曝光
    int i_auto_error = 0;// 统计自动曝光的失败次数
    while (true) {
        if (device.get_capture(&capture)) {
            cout << i_auto << " 稳定相机，用于自动曝光" << endl;
            // 跳过前 n 个（成功的数据采集）循环，用来稳定
            if (i_auto != 30) {
                i_auto++;
                continue;
            } else {
                cout << "自动曝光完成" << endl;
                break;// 跳出该循环，完成相机的稳定过程
            }
        } else {
            cerr << i_auto_error << "自动曝光失败" << endl;
            if (i_auto_error != 30) {
                i_auto_error++;
                continue;
            } else {
                cerr << "错误：无法自动曝光。" << endl;
                return false;
            }
        }
    }
    return true;
}

// 使用 Azure Kinect 的麦克风阵列进行声源定位
void cas::kinect::startSoundSourceLocalization() {
    snd_pcm_t *pcm_handle;
    snd_pcm_hw_params_t *params;
    char buffer[BUFFER_SIZE];
    unsigned int sample_rate = 44100;// 采样率
    int channels = 7;                // 声道数

    // 打开Azure Kinect的麦克风阵列
    if (snd_pcm_open(&pcm_handle, MICROPHONE_NAME, SND_PCM_STREAM_CAPTURE, 0) < 0) {
        cerr << "错误：无法打开麦克风阵列。" << endl;
        return;
    }

    if (snd_pcm_hw_params_malloc(&params) < 0) {
        cerr << "错误：无法分配硬件参数结构。" << endl;
        return;
    }

    if (snd_pcm_hw_params_any(pcm_handle, params) < 0) {
        cerr << "错误：无法初始化硬件参数结构。" << endl;
        return;
    }

    if (snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED) < 0) {
        cerr << "错误：无法设置访问类型。" << endl;
        return;
    }

    if (snd_pcm_hw_params_set_rate_near(pcm_handle, params, &sample_rate, 0) < 0) {
        cerr << "错误：无法设置采样率。" << endl;
        return;
    }

    if (snd_pcm_hw_params_set_channels(pcm_handle, params, channels) < 0) {
        cerr << "错误：无法设置声道数。" << endl;
        return;
    }

    if (snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE) < 0) {
        cerr << "错误：无法设置格式。" << endl;
        return;
    }

    if (snd_pcm_hw_params(pcm_handle, params) < 0) {
        cerr << "错误：无法设置硬件参数。" << endl;
        return;
    }

    snd_pcm_hw_params_free(params);

    int frame_size = channels * 2;      // 一个采样点占用的字节数
    int channels_size = BUFFER_SIZE / frame_size; // 一个缓冲区有多少个采样点

    while (true) {
        if (snd_pcm_readi(pcm_handle, buffer, channels_size) == -EPIPE) {
            cerr << "错误：无法读取数据。" << endl;
            snd_pcm_prepare(pcm_handle);
        } else {
            //此时 buffer 中存储的就是麦克风阵列采集到的声音数据
            //分离声道
            short *data = (short *) buffer;
            short *data1 = new short[channels_size];
            short *data2 = new short[channels_size];
            short *data3 = new short[channels_size];
            short *data4 = new short[channels_size];
            short *data5 = new short[channels_size];
            short *data6 = new short[channels_size];
            short *data7 = new short[channels_size];

            for (int i = 0; i < channels_size; i++) {
                data1[i] = data[i * channels];
                data2[i] = data[i * channels + 1];
                data3[i] = data[i * channels + 2];
                data4[i] = data[i * channels + 3];
                data5[i] = data[i * channels + 4];
                data6[i] = data[i * channels + 5];
                data7[i] = data[i * channels + 6];
            }

            // 计算平均值
            double average1 = 0;
            double average2 = 0;
            double average3 = 0;
            double average4 = 0;
            double average5 = 0;
            double average6 = 0;
            double average7 = 0;

            for (int i = 0; i < channels_size; i++) {
                average1 += data1[i];
                average2 += data2[i];
                average3 += data3[i];
                average4 += data4[i];
                average5 += data5[i];
                average6 += data6[i];
                average7 += data7[i];
            }

            cout << "1: " << average1 / channels_size << " " << "2: " << average2 / channels_size << " "
                 << "3: " << average3 / channels_size << " " << "4: " << average4 / channels_size << " "
                 << "5: " << average5 / channels_size << " " << "6: " << average6 / channels_size << " "
                 << "7: " << average7 / channels_size << endl;
        }
    }

    snd_pcm_close(pcm_handle);

    return;
}
