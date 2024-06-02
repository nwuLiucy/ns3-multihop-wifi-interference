# ns3-multihop-wifi-interference
基于NS-3模拟受到干扰的Wi-Fi网络，可以实现指定路由和多跳传输
NS-3的下载、安装和使用，请参考[官方文档](https://www.nsnam.org/documentation/)
本代码基于NS-3.40编写，实现的主要功能如下：
- N个specturm Wi-Fi节点进行UDP通信
- 设置了M个干扰节点（波形发生器 waveformGeneratorHelper 实现），通过 waveformPower 和所处位置(不同的随机种子)控制干扰强度
- 可以分析网络中每一条链路的吞吐率，并以矩阵形式保存到一个TXT文件中; 或者只在源节点和汇节点之间发送数据
- 根据路由表文件，手动设置静态路由
