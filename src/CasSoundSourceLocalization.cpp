//
// Create by HoChihchou on 2023/5/6
//
#include "CasSoundSourceLocalization.h"

#include <cmath>
#include <complex>
#include <fftw3.h>
#include <iostream>
#include <numeric>
#include <vector>

using namespace std;
using namespace cas::ssl;

SoundSourceDetector::SoundSourceDetector(unsigned int smaple_rate = 44100,
                                         const int samples = 2205,
                                         const int channels = 7,
                                         string microphone_name = "plughw:2,0") : sample_rate(smaple_rate),
                                                                                  samples(samples),
                                                                                  channels(channels),
                                                                                  microphone_name(microphone_name){};

SoundSourceDetector::~SoundSourceDetector() {
    stop();
};

bool SoundSourceDetector::start() {
    this->format = SND_PCM_FORMAT_FLOAT_LE;// 数据格式为float类型
    this->period_size = 256;
    this->fmt_size = 4;

    // 打开Azure Kinect的麦克风阵列
    if (snd_pcm_open(&this->pcm_handle, this->microphone_name.c_str(), SND_PCM_STREAM_CAPTURE, 0) < 0) {
        cerr << "错误：无法打开麦克风阵列。" << endl;
        return false;
    }

    if (snd_pcm_sw_params_malloc(&this->sw_params) < 0) {
        cerr << "错误：无法分配软件参数结构。" << endl;
        return false;
    }

    snd_pcm_sw_params_current(this->pcm_handle, this->sw_params);
    snd_pcm_hw_params_alloca(&this->hw_params);

    if (snd_pcm_hw_params_any(this->pcm_handle, this->hw_params) < 0) {
        cerr << "错误：无法初始化硬件参数结构。" << endl;
        return false;
    }

    if (snd_pcm_hw_params_set_access(this->pcm_handle, this->hw_params, SND_PCM_ACCESS_RW_INTERLEAVED) < 0) {
        cerr << "错误：无法设置访问类型。" << endl;
        return false;
    }

    if (snd_pcm_hw_params_set_format(this->pcm_handle, this->hw_params, this->format) < 0) {
        cerr << "错误：无法设置格式。" << endl;
        return false;
    }

    if (snd_pcm_hw_params_set_channels(this->pcm_handle, this->hw_params, this->channels) < 0) {
        cerr << "错误：无法设置声道数。" << endl;
        return false;
    }

    if (snd_pcm_hw_params_set_rate_near(this->pcm_handle, this->hw_params, &this->sample_rate, 0) < 0) {
        cerr << "错误：无法设置采样率。" << endl;
        return false;
    }

    if (snd_pcm_hw_params_set_period_size_near(this->pcm_handle, this->hw_params, &this->period_size, 0) < 0) {
        cerr << "错误：无法设置周期大小。" << endl;
        return false;
    }

    if (snd_pcm_hw_params(this->pcm_handle, this->hw_params) < 0) {
        cerr << "错误：无法设置硬件参数。" << endl;
        return false;
    }

    this->buffer_size = this->samples * this->channels * this->fmt_size;

    cout << "声源捕获开启成功" << endl;
    cout << "=========================" << endl;
    cout << "采样率: " << sample_rate << endl;
    cout << "采样点数: " << samples << endl;
    cout << "通道数: " << channels << endl;
    cout << "缓冲区大小: " << buffer_size << endl;
    cout << "=========================" << endl;

    return true;
}

void SoundSourceDetector::stop() {
    snd_pcm_close(pcm_handle);
}

// 分帧
void enframeChannel(float *data, int channel_size, int frame_length, int overlap, float **frames_enframe) {
    int step_length = frame_length - overlap;              // 步长
    int frame_num = (channel_size - overlap) / step_length;// 帧数

    for (int i = 0; i < frame_num; i++) {
        int start = i * step_length;
        int end = start + frame_length - 1;
        // 将数组中从 start 到 end 的元素赋值给 frames_enframe 数组的第 i 行
        for (int j = start; j <= end; j += 10) {
            frames_enframe[i][j - start + 0] = data[j + 0];
            frames_enframe[i][j - start + 1] = data[j + 1];
            frames_enframe[i][j - start + 2] = data[j + 2];
            frames_enframe[i][j - start + 3] = data[j + 3];
            frames_enframe[i][j - start + 4] = data[j + 4];
            frames_enframe[i][j - start + 5] = data[j + 5];
            frames_enframe[i][j - start + 6] = data[j + 6];
            frames_enframe[i][j - start + 7] = data[j + 7];
            frames_enframe[i][j - start + 8] = data[j + 8];
            frames_enframe[i][j - start + 9] = data[j + 9];
        }
    }
}

int nextpow2(int x) {// 计算大于等于 x 的最小的 2 的幂
    int i = 0;
    while (pow(2, i) < x) {
        i++;
    }
    return pow(2, i);
}

void gccPhat(float *frames_enframe1, float *frames_enframe2, int frame_length, Eigen::Ref<Eigen::VectorXf, 0, Eigen::InnerStride<>> y) {
    int ncorr = 2 * frame_length - 1;
    int nfft = nextpow2(ncorr);

    fftw_complex *in1, *in2, *out1, *out2, *out3;
    fftw_plan p1, p2, p3;

    in1 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * nfft);
    in2 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * nfft);
    out1 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * nfft);
    out2 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * nfft);

    for (int i = 0; i < frame_length; i++) {
        in1[i][0] = frames_enframe1[i];
        in1[i][1] = 0;
        in2[i][0] = frames_enframe2[i];
        in2[i][1] = 0;
    }
    for (int i = frame_length; i < nfft; i++) {
        in1[i][0] = 0;
        in1[i][1] = 0;
        in2[i][0] = 0;
        in2[i][1] = 0;
    }

    p1 = fftw_plan_dft_1d(nfft, in1, out1, FFTW_FORWARD, FFTW_ESTIMATE);
    p2 = fftw_plan_dft_1d(nfft, in2, out2, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(p1);
    fftw_execute(p2);

    // 计算 Gss
    vector<complex<float>> Gss(nfft);
    for (int i = 0; i < nfft; i++) {
        complex<float> c1(out1[i][0], out1[i][1]);
        complex<float> c2(out2[i][0], out2[i][1]);
        Gss[i] = c1 * conj(c2);
    }

    // 计算 exp(1i*angle(Gss))
    vector<complex<float>> expGss(nfft);
    for (int i = 0; i < nfft; i++) {
        expGss[i] = exp(complex<float>(0, 1) * arg(Gss[i]));
        in1[i][0] = expGss[i].real();
        in1[i][1] = expGss[i].imag();
    }

    // 计算 ifft(exp(1i*angle(Gss)))
    out3 = (fftw_complex *) fftw_malloc(sizeof(fftw_complex) * nfft);
    p3 = fftw_plan_dft_1d(nfft, in1, out3, FFTW_BACKWARD, FFTW_ESTIMATE);
    fftw_execute(p3);

    // 计算 xcorr_cac
    vector<complex<float>> xcorr_cac(nfft);
    for (int i = 0; i < nfft; i++) {
        xcorr_cac[i] = complex<float>(out3[i][0], out3[i][1]) / static_cast<float>(nfft);
    }

    // 计算 fftshift
    int half_size = nfft / 2;
    for (int i = 0; i < half_size; i++) {
        swap(xcorr_cac[i], xcorr_cac[i + half_size]);
    }//对啦

    // 计算 y
    int start = nfft / 2 + 1 - (ncorr - 1) / 2;
    int end = nfft / 2 + 1 + (ncorr - 1) / 2;
    for (int i = start - 1; i <= end - 1; i++) {
        y[i - start + 1] = xcorr_cac[i].real();
    }//对啦

    fftw_destroy_plan(p1);
    fftw_destroy_plan(p2);
    fftw_destroy_plan(p3);
    fftw_free(in1);
    fftw_free(in2);
    fftw_free(out1);
    fftw_free(out2);
    fftw_free(out3);
}

void sinc_interp(Eigen::VectorXf input_signal, Eigen::VectorXf input_time, int num, Eigen::Ref<Eigen::VectorXf> output_signal, Eigen::Ref<Eigen::VectorXf> output_time) {
    int n_length = input_time.size();       // 原始信号的长度
    int N = n_length + num * (n_length - 1);// 总点数

    float y_cac = 0;   // 累加变量
    float m_before = 0;// 当前插值点之前的原始时间位置
    float m_back = 0;  // 当前插值点之后的原始时间位置
    float dt = 0;      // 该插值区间的分辨率
    float m_cac = 0;   // 新的插值点对应的时间位置

    for (int i = 1; i <= N; i++) {
        if (i % (num + 1) == 1) {
            output_signal[i - 1] = input_signal[ceil(i / (num + 1.0)) - 1];
            output_time[i - 1] = input_time[ceil(i / (num + 1.0)) - 1];
        } else {
            y_cac = 0;                                                                // 累加变量
            m_before = input_time[ceil(i / (num + 1.0)) - 1];                         // 当前插值点之前的原始时间位置
            m_back = input_time[ceil(i / (num + 1.0))];                               // 当前插值点之后的原始时间位置
            dt = (m_back - m_before) / (num + 1.0);                                   // 该插值区间的分辨率
            m_cac = m_before + dt * (i - (num + 1) * (ceil(i / (num + 1.0) - 1)) - 1);// 新的插值点对应的时间位置
            for (int j = 0; j < n_length; j++) {
                y_cac += input_signal[j] * sin((m_cac - input_time[j]) * M_PI) / ((m_cac - input_time[j]) * M_PI);// 进行sinc插值累加
            }
            output_signal[i - 1] = y_cac;// 信号赋值
            output_time[i - 1] = m_cac;  // 时间赋值
        }
    }
}

Eigen::Vector3f SoundSourceDetector::locate() {
    int frames_read = 0;
    float *buffer = new float[this->buffer_size];
    frames_read = snd_pcm_readi(this->pcm_handle, buffer, this->samples);
    if (frames_read == -EPIPE) {
        snd_pcm_prepare(this->pcm_handle);
        frames_read = snd_pcm_readi(this->pcm_handle, buffer, this->samples);
    } else if (frames_read < 0) {
        cout << "错误：无法从PCM设备读取数据。" << endl;
        return Eigen::Vector3f(0, 0, 0);
    } else if (frames_read != this->samples) {
        cout << "错误：从PCM设备读取的数据不完整。" << endl;
        return Eigen::Vector3f(0, 0, 0);
    }

    // 此时 buffer 中存储的就是麦克风阵列采集到的声音数据
    // 步骤1: 分离声道
    float *data1 = new float[this->samples];
    float *data2 = new float[this->samples];
    float *data3 = new float[this->samples];
    float *data4 = new float[this->samples];
    float *data5 = new float[this->samples];
    float *data6 = new float[this->samples];
    float *data7 = new float[this->samples];

    for (int i = 0; i < this->samples; i++) {
        data1[i] = buffer[i * this->channels];
        data2[i] = buffer[i * this->channels + 1];
        data3[i] = buffer[i * this->channels + 2];
        data4[i] = buffer[i * this->channels + 3];
        data5[i] = buffer[i * this->channels + 4];
        data6[i] = buffer[i * this->channels + 5];
        data7[i] = buffer[i * this->channels + 6];
    }

    // 计算声源定位
    // 步骤2: 分帧加窗
    int frame_length = 1200;                                // 帧长
    int overlap = 400;                                      // 帧移
    int step_length = frame_length - overlap;               // 步长
    int frame_num = (this->samples - overlap) / step_length;// 帧数
    float **frames_enframe1 = new float *[frame_num];
    float **frames_enframe2 = new float *[frame_num];
    float **frames_enframe3 = new float *[frame_num];
    float **frames_enframe4 = new float *[frame_num];
    float **frames_enframe5 = new float *[frame_num];
    float **frames_enframe6 = new float *[frame_num];
    float **frames_enframe7 = new float *[frame_num];
    for (int i = 0; i < frame_num; i++) {
        frames_enframe1[i] = new float[frame_length];
        frames_enframe2[i] = new float[frame_length];
        frames_enframe3[i] = new float[frame_length];
        frames_enframe4[i] = new float[frame_length];
        frames_enframe5[i] = new float[frame_length];
        frames_enframe6[i] = new float[frame_length];
        frames_enframe7[i] = new float[frame_length];
    }
    enframeChannel(data1, this->samples, frame_length, overlap, frames_enframe1);
    enframeChannel(data2, this->samples, frame_length, overlap, frames_enframe2);
    enframeChannel(data3, this->samples, frame_length, overlap, frames_enframe3);
    enframeChannel(data4, this->samples, frame_length, overlap, frames_enframe4);
    enframeChannel(data5, this->samples, frame_length, overlap, frames_enframe5);
    enframeChannel(data6, this->samples, frame_length, overlap, frames_enframe6);
    enframeChannel(data7, this->samples, frame_length, overlap, frames_enframe7);

    // for (int i = 0; i < frame_num; i++) {
    //     for (int j = 0; j < frame_length; j++) {
    //          cout << frames_enframe2[i][j] << " ";
    //     }
    //     cout << endl;
    // }    //DONE!!!

    // 步骤3: 计算GCC-PHAT
    int cac_length = 2 * frame_length - 1;
    Eigen::MatrixXf xcorr_cac12(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac13(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac14(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac15(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac16(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac17(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac23(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac24(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac25(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac26(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac27(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac34(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac35(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac36(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac37(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac45(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac46(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac47(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac56(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac57(cac_length, frame_num);
    Eigen::MatrixXf xcorr_cac67(cac_length, frame_num);

    for (int i = 0; i < frame_num; i++) {
        gccPhat(frames_enframe1[i], frames_enframe2[i], frame_length, xcorr_cac12.col(i));
        gccPhat(frames_enframe1[i], frames_enframe3[i], frame_length, xcorr_cac13.col(i));
        gccPhat(frames_enframe1[i], frames_enframe4[i], frame_length, xcorr_cac14.col(i));
        gccPhat(frames_enframe1[i], frames_enframe5[i], frame_length, xcorr_cac15.col(i));
        gccPhat(frames_enframe1[i], frames_enframe6[i], frame_length, xcorr_cac16.col(i));
        gccPhat(frames_enframe1[i], frames_enframe7[i], frame_length, xcorr_cac17.col(i));
        gccPhat(frames_enframe2[i], frames_enframe3[i], frame_length, xcorr_cac23.col(i));
        gccPhat(frames_enframe2[i], frames_enframe4[i], frame_length, xcorr_cac24.col(i));
        gccPhat(frames_enframe2[i], frames_enframe5[i], frame_length, xcorr_cac25.col(i));
        gccPhat(frames_enframe2[i], frames_enframe6[i], frame_length, xcorr_cac26.col(i));
        gccPhat(frames_enframe2[i], frames_enframe7[i], frame_length, xcorr_cac27.col(i));
        gccPhat(frames_enframe3[i], frames_enframe4[i], frame_length, xcorr_cac34.col(i));
        gccPhat(frames_enframe3[i], frames_enframe5[i], frame_length, xcorr_cac35.col(i));
        gccPhat(frames_enframe3[i], frames_enframe6[i], frame_length, xcorr_cac36.col(i));
        gccPhat(frames_enframe3[i], frames_enframe7[i], frame_length, xcorr_cac37.col(i));
        gccPhat(frames_enframe4[i], frames_enframe5[i], frame_length, xcorr_cac45.col(i));
        gccPhat(frames_enframe4[i], frames_enframe6[i], frame_length, xcorr_cac46.col(i));
        gccPhat(frames_enframe4[i], frames_enframe7[i], frame_length, xcorr_cac47.col(i));
        gccPhat(frames_enframe5[i], frames_enframe6[i], frame_length, xcorr_cac56.col(i));
        gccPhat(frames_enframe5[i], frames_enframe7[i], frame_length, xcorr_cac57.col(i));
        gccPhat(frames_enframe6[i], frames_enframe7[i], frame_length, xcorr_cac67.col(i));
    }

    // 输出 xcorr_cac12
    // cout << "xcorr_cac12: " << endl;
    // for (int i = 0; i < frame_num; i++) {
    //     for (int j = 0; j < 2 * frame_length - 1; j++) {
    //         cout << xcorr_cac12(j, i) << " ";
    //     }
    //     cout << endl;
    // }// DONE!!! ///YESYESYES！！！

    Eigen::VectorXf xcorr12 = xcorr_cac12.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr13 = xcorr_cac13.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr14 = xcorr_cac14.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr15 = xcorr_cac15.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr16 = xcorr_cac16.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr17 = xcorr_cac17.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr23 = xcorr_cac23.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr24 = xcorr_cac24.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr25 = xcorr_cac25.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr26 = xcorr_cac26.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr27 = xcorr_cac27.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr34 = xcorr_cac34.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr35 = xcorr_cac35.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr36 = xcorr_cac36.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr37 = xcorr_cac37.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr45 = xcorr_cac45.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr46 = xcorr_cac46.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr47 = xcorr_cac47.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr56 = xcorr_cac56.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr57 = xcorr_cac57.rowwise().sum().cwiseAbs();
    Eigen::VectorXf xcorr67 = xcorr_cac67.rowwise().sum().cwiseAbs();

    vector<Eigen::VectorXf> gcc_all(21);
    gcc_all[0] = xcorr12;
    gcc_all[1] = xcorr13;
    gcc_all[2] = xcorr14;
    gcc_all[3] = xcorr15;
    gcc_all[4] = xcorr16;
    gcc_all[5] = xcorr17;
    gcc_all[6] = xcorr23;
    gcc_all[7] = xcorr24;
    gcc_all[8] = xcorr25;
    gcc_all[9] = xcorr26;
    gcc_all[10] = xcorr27;
    gcc_all[11] = xcorr34;
    gcc_all[12] = xcorr35;
    gcc_all[13] = xcorr36;
    gcc_all[14] = xcorr37;
    gcc_all[15] = xcorr45;
    gcc_all[16] = xcorr46;
    gcc_all[17] = xcorr47;
    gcc_all[18] = xcorr56;
    gcc_all[19] = xcorr57;
    gcc_all[20] = xcorr67;

    int gcc_length = xcorr12.size();
    float frame_len = (gcc_length + 1) / 2;
    float delayPointMax[21];
    int inter_points = 20;
    int inter_radius = 20;
    for (int i = 0; i < 21; i++) {
        delayPointMax[i] = 0;
    }
    for (int i = 0; i < 21; i++) {
        Eigen::VectorXf gcc_cac = gcc_all[i];
        Eigen::VectorXf::Index max_index;
        gcc_cac.maxCoeff(&max_index);
        while (max_index + 1 == frame_len || max_index >= 2398 - inter_radius || max_index <= inter_radius) {
            gcc_cac(max_index) = 0;
            gcc_cac.maxCoeff(&max_index);
        }

        int start = max_index - inter_radius + 1;
        int end = max_index + inter_radius + 1;
        Eigen::VectorXf input_time = Eigen::VectorXf::LinSpaced(end - start + 1, start, end);
        Eigen::VectorXf input_signal = gcc_cac.segment(start - 1, end - start + 1);

        int n_length = input_time.size();                // 原始信号的长度
        int N = n_length + inter_points * (n_length - 1);// 总点数
        Eigen::VectorXf output_signal(N);
        Eigen::VectorXf output_time(N);

        sinc_interp(input_signal, input_time, inter_points, output_signal, output_time);

        // cout << "output_signal: " << endl;
        // for (int j = 0; j < N; j++) {
        //     cout << output_signal[j] << " ";
        // }    //YESYESYES！！！

        int max_index2;
        output_signal.maxCoeff(&max_index2);

        delayPointMax[i] = output_time[max_index2];
    }

    Eigen::VectorXf delay_ref(21);
    delay_ref << delayPointMax[0] - frame_len, delayPointMax[1] - frame_len, delayPointMax[2] - frame_len,
            delayPointMax[3] - frame_len, delayPointMax[4] - frame_len, delayPointMax[5] - frame_len,
            delayPointMax[6] - frame_len, delayPointMax[7] - frame_len, delayPointMax[8] - frame_len,
            delayPointMax[9] - frame_len, delayPointMax[10] - frame_len, delayPointMax[11] - frame_len,
            delayPointMax[12] - frame_len, delayPointMax[13] - frame_len, delayPointMax[14] - frame_len,
            delayPointMax[15] - frame_len, delayPointMax[16] - frame_len, delayPointMax[17] - frame_len,
            delayPointMax[18] - frame_len, delayPointMax[19] - frame_len, delayPointMax[20] - frame_len;

    Eigen::Vector3f obj(1, 1, 1);

    float alpha = 0.005;
    float error_thresh = 1e-1;
    int number = 0;
    int number_thresh = 500;
    float error = 100;
    Eigen::MatrixXf factor_matrix(21, 7);
    factor_matrix << 1, -1, 0, 0, 0, 0, 0,
            1, 0, -1, 0, 0, 0, 0,
            1, 0, 0, -1, 0, 0, 0,
            1, 0, 0, 0, -1, 0, 0,
            1, 0, 0, 0, 0, -1, 0,
            1, 0, 0, 0, 0, 0, -1,
            0, 1, -1, 0, 0, 0, 0,
            0, 1, 0, -1, 0, 0, 0,
            0, 1, 0, 0, -1, 0, 0,
            0, 1, 0, 0, 0, -1, 0,
            0, 1, 0, 0, 0, 0, -1,
            0, 0, 1, -1, 0, 0, 0,
            0, 0, 1, 0, -1, 0, 0,
            0, 0, 1, 0, 0, -1, 0,
            0, 0, 1, 0, 0, 0, -1,
            0, 0, 0, 1, -1, 0, 0,
            0, 0, 0, 1, 0, -1, 0,
            0, 0, 0, 1, 0, 0, -1,
            0, 0, 0, 0, 1, -1, 0,
            0, 0, 0, 0, 1, 0, -1,
            0, 0, 0, 0, 0, 1, -1;

    Eigen::Matrix<float, 7, 3> mic;//麦克风拓扑结构
    mic << 0, 0, 0,
            0, 0.04, 0,
            0.034641, 0.02, 0,
            0.034641, -0.02, 0,
            0, -0.04, 0,
            -0.034641, -0.02, 0,
            -0.034641, 0.02, 0;
    float v = 340;//声速


    // 梯度下降法
    while (error > error_thresh && number < number_thresh) {// 当误差或次数达到门限的时候退出
        Eigen::VectorXf range = (mic.rowwise() - obj.transpose()).rowwise().norm();
        Eigen::VectorXf delay = factor_matrix * range * 2 / v * this->sample_rate;
        error = (delay - delay_ref).array().square().sum();
        Eigen::MatrixXf range_gradient = (obj.transpose().colwise().replicate(7) - mic).array().colwise() / range.array();
        Eigen::MatrixXf delay_gradient = factor_matrix * range_gradient;
        Eigen::MatrixXf delay_error = (delay - delay_ref).rowwise().replicate(3);
        Eigen::VectorXf error_gradient = (delay_gradient.array() * delay_error.array()).colwise().sum() * 4 * sample_rate / v;
        obj -= alpha * error_gradient.transpose();
        number++;
    }

    // cout << "obj: " << obj[0] << " " << obj[1] << " " << obj[2] << endl;//YESYESYES！！！
    return obj;
    //结果格式：x y z
    //          ^ y
    //          |
    //          |
    //          |
    // -------Kinect-------> x
    //          |
    //          |
    //          |
}
