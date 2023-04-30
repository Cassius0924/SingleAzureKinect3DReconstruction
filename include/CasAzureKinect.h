//
// Created by Cassius0924 on 4/30/23.
//

#ifndef _CASAZUREKINECT_H_
#define _CASAZUREKINECT_H_

#include <k4a/k4a.hpp>


namespace cas {
    namespace kinect {

        // 检测Kinect数量是否大于某个值
        bool checkKinectNum(int num);

        // 稳定相机
        bool stabilizeCamera(k4a::device &device);

        // 开启声源定位
        void startSoundSourceLocalization();

    }
}


#endif