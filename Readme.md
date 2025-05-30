# Ros2Go2Estimator 🦾
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

- 一种高精度里程计解决方案，
- 基于纯运动学的双足/四足机器人位置估计算法，
- 不依赖相机或Lidar，但可将信号融合进去，进一步提高估计精度，
- 目前仅使用IMU、足压力传感器、关节角度和角速度，
- 里程计被发布于话题/SMXFE_odom，frame_id是"SMXFE_odom"。

## 📚 项目亮点
- 切换两足、四足无需在估计器内做模式切换
- 两足站立行走误差率1%  
- 动态行走模式误差率0.5%
- 支持长距离定位
- 目前没有调整参数做补偿，工程使用时可进一步提升精度
- 增加了voice_chat_py用于语音交流，用不到的同志直接删除这个package即可，不影响其他包的运行。

## 🎥 视频演示
### 最新进展(点击图片进入视频)
1. 站立行走误差1%，四足行走误差0.5%
[![主演示视频](https://i1.hdslb.com/bfs/archive/10e501bc7a93c77c1c3f41f163526b630b0afa3f.jpg)](https://www.bilibili.com/video/BV18Q9JYEEdn/)

#### 实验记录
2. 爬楼梯高度误差小于5cm
[![实验1](https://i0.hdslb.com/bfs/archive/c469a3dd37522f6b7dcdbdbb2c135be599eefa7b.jpg)](https://www.bilibili.com/video/BV1VV9ZYZEcH/)

3. 长距离测试，受磁场变化影响，380米运动偏差3.3%
[![实验2](https://i0.hdslb.com/bfs/archive/481731d2db755bbe087f44aeb3f48db29c159ada.jpg)](https://www.bilibili.com/video/BV1BhRAYDEsV/)

4. 语音控制机器狗，DeepSeek大模型语音交流
[![实验3](https://i0.hdslb.com/bfs/archive/6aaac2a8d2726fa2c7d77f20544c9692f9fb752f.jpg)](https://www.bilibili.com/video/BV1YjQVYcEdX/)

## ⚙️ 安装指南

- Use Ubuntu 22.04, ROS2 Humble
```bash
sudo apt install ros-humble-joy
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
git clone --recursive https://github.com/ShineMinxing/Ros2Go2Estimator.git
cd Ros2Go2Estimator
colcon build
ros2 launch joystick_control joystick_control_launch.py
```
```bash for voice chat
sudo apt-get install portaudio19-dev libportaudio2
sudo apt-get install ffmpeg
sudo apt-get install libsdl2-dev libsdl2-mixer-dev
pip install PyAudio-0.2.11-cp310-cp310-linux_armv7l.whl
pip install pyaudio
pip install SpeechRecognition
pip install pyaudio speechrecognition
pip install openai
pip install pydub
pip install pygame
# Voice_chat_py需要挂VPN，速度比较慢，还需要安装各种环境。里面有的api key是我个人的付费key，转语音有点贵。请长期使用的同志点击～https://cloud.siliconflow.cn/i/5kSHnwpA～申请API密钥获得免费额度，将src/voice_chat_py/voice_chat_py/voice_chat_node.py的第21行替换成您的密钥。
```
- 记得在src/joystick_control/launch/joystick_control_launch.py中，修改机器狗的网口名，我的是“enx00e04c8d0eff”。
- 同时按下手柄的LT、RT，解锁/锁定手柄；按住RT+左摇杆进行移动；按住RT+右摇杆进行旋转；更多操作请看joystick_control_node.cpp。

## 📄 相关文档
- 核心算法原理: [技术白皮书](https://github.com/ShineMinxing/FusionEstimation.git)
- 历史项目参考: [FusionEstimation项目](https://github.com/ShineMinxing/FusionEstimation.git)

## 📧 联系我们
``` 
博士团队: 401435318@qq.com  
研究所: 中国科学院光电技术研究所
```

> 📌 注意：当前为开发预览版，完整文档正在编写中
``