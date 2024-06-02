/*
    本代码实现的是：N个specturm Wi-Fi节点进行UDP通信;
    设置了M个干扰节点，通过waveformPower和所处位置(不同的随机种子)控制干扰强度;
    可以分析网络中每一条链路的吞吐率，并以矩阵形式保存到一个TXT文件中; 或者只在源节点和汇节点之间发送数据
    根据路由表文件，手动设置静态路由；
*/
#include "UtilityFunctions.h"

using namespace ns3;
using namespace std;

// 全局变量
static vector<string> modes = 
{
    "HtMcs0",  "HtMcs1",  "HtMcs2",  "HtMcs3",  "HtMcs4",  "HtMcs5",  "HtMcs6",  "HtMcs7",
};
static vector<string> datarates =
{
    "6.5Mb/s",  "13Mb/s",   "19.5Mb/s", "26Mb/s",   "39Mb/s",   "52Mb/s",   "58.5Mb/s",   "65Mb/s",
};
uint16_t N = 10; // 无线节点的数量
vector<vector<double>> psr(N, vector<double>(N, 0));

// 计算吞吐率和psr
void CalculateThroughput(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier, 
    vector<vector<double>>* throughput, uint16_t count, uint16_t sourceNode, uint16_t sinkNode)
{
    map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    auto i = stats.begin();
    advance(i, count > 0 ? count-1 : 0); // 将迭代器前进到第(count-1)个元素
    for (; i != stats.end(); ++i) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        cout << "Flow " << i->first << " (" << t.sourceAddress << " -> "
                << t.destinationAddress << ")" << endl;
        cout << "  Packet success rate from " << t.sourceAddress << " to " 
                << t.destinationAddress << " is " 
                << i->second.rxPackets*1.0/i->second.txPackets << endl;
        cout << "  Throughput to " << t.destinationAddress << " :"
                << i->second.rxBytes * 8.0 /
                (i->second.timeLastRxPacket.GetSeconds() - 
                i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024
                << " Mbps" << endl;
        double psr_value = 0.0;
        double throughput_value = 0.0;
        double timeInterval = i->second.timeLastRxPacket.GetSeconds() - 
                            i->second.timeFirstTxPacket.GetSeconds();

        // 确保时间间隔大于零
        if (timeInterval > 0) {
            throughput_value = i->second.rxBytes * 8.0 / timeInterval / 1024 / 1024;
            // 四舍五入到三位小数
            throughput_value = std::round(throughput_value * 1000.0) / 1000.0;
            psr_value = i->second.rxPackets*1.0/i->second.txPackets;
            psr_value = std::round(psr_value * 100000.0) / 1000.0; // 百分数表示，保留三位小数
        } else {
            // 处理时间间隔不大于零的情况，例如设置吞吐率为零
            throughput_value = 0.0;
            psr_value = 0.0;
        }

        (*throughput)[sourceNode][sinkNode] = throughput_value;
        psr[sourceNode][sinkNode] = psr_value;

    }
}

void 
InitializeDirectRoutes(NodeContainer &nodes, Ipv4StaticRoutingHelper &ipv4RoutingHelper, 
Ipv4InterfaceContainer &interfaces) 
{
  Ptr<UniformRandomVariable> random = CreateObject<UniformRandomVariable>();
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    Ptr<Ipv4> ipv4 = nodes.Get(i)->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting(ipv4);
    // 获取路由表中的路由数量
    int32_t numRoutes = staticRouting->GetNRoutes();
    // 从后向前遍历路由表，删除除了前两条默认路由之外的所有路由
    for (int32_t j = numRoutes - 1; j >= 2; --j) {
      staticRouting->RemoveRoute(j);
    }
    // 初始化直接路由
    for (uint32_t j = 0; j < nodes.GetN(); ++j) {
      if (i != j) { // 确保不是指向自己的路由
        Ipv4Address destAddress = interfaces.GetAddress(j, 0);
        Ipv4Address nextHop = interfaces.GetAddress(j, 0); // 下一跳地址设置为目的地址
        uint32_t interface = random->GetInteger(1, ipv4->GetNInterfaces() - 1);
        staticRouting->AddHostRouteTo(destAddress, nextHop, interface);
      }
    }
  }
}
void 
UpdateStaticRoutingTable(NodeContainer &nodes, Ipv4StaticRoutingHelper &ipv4RoutingHelper, 
Ipv4InterfaceContainer &interfaces, vector<vector<int>> routingTable) 
{
    for (uint32_t i = 0; i < nodes.GetN(); ++i) {
        Ptr<Ipv4> ipv4 = nodes.Get(i)->GetObject<Ipv4>();
        Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting(ipv4);

        for (uint32_t j = 0; j < nodes.GetN(); ++j) {
        if (i != j) { // 确保不是指向自己的路由
            Ipv4Address destAddress = interfaces.GetAddress(j, 0);
            int nextHopIndex = routingTable[i][j];
            Ipv4Address nextHop = interfaces.GetAddress(nextHopIndex, 0);
            int32_t interface = 2;
            // 检查是否已存在路由
            bool routeExists = false;
            int32_t routeIndex = staticRouting->GetNRoutes();
            while (routeIndex--) {
            Ipv4RoutingTableEntry routeEntry = staticRouting->GetRoute(routeIndex);
            if (routeEntry.GetDest() == destAddress) {
                interface = routeEntry.GetInterface(); //采用原来的接口
                routeExists = true;
                // 如果新路由的metric更低，或者相同但我们想要更新路由，则删除旧路由
                if (routeEntry.GetGateway() != nextHop || staticRouting->GetMetric(routeIndex) >= 1) {
                staticRouting->RemoveRoute(routeIndex);
                routeExists = false; // 允许添加新路由
                }
                break;
            }
            }
            // 如果不存在相同目的地的路由，或者旧路由已被删除，则添加新路由
            if (!routeExists) {
            staticRouting->AddHostRouteTo(destAddress, nextHop, interface, 0);
            }
        }
        }
    }
}

NS_LOG_COMPONENT_DEFINE("wifi-spectrum-interference-routing");

int main(int argc, char* argv[])
{
    // 启用Wi-Fi日志记录
    LogComponentEnable("wifi-spectrum-interference-routing", LOG_LEVEL_INFO);

    // 常量和配置
    const double simulationTime = 1; // seconds
    const double T = 1; // 间隔时间
    const uint32_t packetSize = 1420;
    const double minX = 10;
    const double maxX = 100;
    const double minY = 10;
    const double maxY = 100;
    const double frequencyMode = 2.4; 

    // 动态配置变量
    uint16_t M = 2; // 干扰节点的数量
    uint16_t sourceNode = 0;
    uint16_t sinkNode = N - 1;
    uint16_t power = 10;
    uint32_t seed = 2000; // 设置随机种子
    uint8_t mcsIndex = 3; // 设置MCS索引值
    double datarate = 6.5; // Mbps
    const double waveformPower = power * 1e-4;

    // 状态标志
    bool channelBonding = false;
    bool linkTest = false;
    bool updateRoutes = true;
    bool reset = false;

    // 命令行解析
    CommandLine cmd(__FILE__);
    cmd.AddValue("sinkNode", "信号的接收节点", sinkNode);
    cmd.AddValue("sourceNode", "信号的发送节点", sourceNode);
    cmd.AddValue("N", "无线网络中节点的数量", N);
    cmd.AddValue("M", "无线网络中干扰节点的数量", M);
    cmd.AddValue("power", "干扰功率", power);
    cmd.AddValue("seed", "随机种子", seed);
    cmd.AddValue("mcsIndex","Wi-Fi的MCS索引",mcsIndex);
    cmd.AddValue("datarate","app的发送速率",datarate);
    cmd.AddValue("linkTest", "是否进行网络中的链路状态测试", linkTest);
    cmd.AddValue("updateRoutes", "是否更新路由", updateRoutes);
    cmd.AddValue("reset", "是否重置吞吐率和psr文件", reset);
    cmd.Parse(argc, argv);

    if (mcsIndex < 0 || mcsIndex > 7) { // 检查MCS索引值的范围，可以通过增加Wi-Fi天线数量的方式，使用更大的MCS索引
        cerr << "MCS索引值的范围为0-7" << endl;
        return 0;
    }

    // 文件名和数据结构
    string file_prefix = "txtfiles/wifi/wifi_" + to_string(N) + "_" + to_string(seed);
    string throughputFileName = file_prefix + "_tht_matrix.txt";
    string throughputLinkTestFileName = file_prefix + "_tht_init_matrix.txt";
    string psrFileName = file_prefix + "_psr_matrix.txt";
    string psrLinkTestFileName = file_prefix + "_psr_init_matrix.txt";
    string routingFileName = file_prefix + "_RoutingTable.txt";
    string outfileName = file_prefix + "_position";

    vector<vector<double>> throughput(N, vector<double>(N, 0));
    
    // 如果不存在路由文件，则创建并初始化为直接路由
    if(!fileExists(routingFileName)){
        InitRouteMatrix(routingFileName, N);
    }
    vector<vector<int>> routingTable = ReadMatrixFromFile<int>(routingFileName); // 数据读取

    // 创建Wi-Fi和干扰节点
    vector<NodeContainer> nodeContainers;
    vector<string> nodeTypes;
    NodeContainer nodes, interferingNodes;
    nodes.Create(N);
    interferingNodes.Create(M);
   
    nodeContainers.insert(nodeContainers.end(), {nodes, interferingNodes});
    nodeTypes.insert(nodeTypes.end(), {"Wi-Fi", "Interference"});

    SpectrumWifiPhyHelper wifiPhy;
    // 信道设置
    Ptr<MultiModelSpectrumChannel> spectrumChannel;
    spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
    // 传播时延模型
    Ptr<ConstantSpeedPropagationDelayModel> delayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    spectrumChannel->SetPropagationDelayModel(delayModel);
    // 传播损失模型
    Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
    lossModel->SetFrequency(2412 * 1e6);//
    spectrumChannel->AddPropagationLossModel(lossModel);
    
    wifiPhy.SetChannel(spectrumChannel);
    wifiPhy.Set("ChannelSettings",
                    StringValue(string("{0, ") + (channelBonding ? "40, " : "20, ") +
                    (frequencyMode == 2.4 ? "BAND_2_4GHZ" : "BAND_5GHZ") + ", 0}"));

    // 创建一个Wi-Fi网络
    WifiHelper wifi;
    Ssid ssid = Ssid("ns3-80211n");
    wifi.SetStandard(WIFI_STANDARD_80211n);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", 
                                "DataMode",StringValue(modes[mcsIndex]),
                                "ControlMode",StringValue(modes[mcsIndex]));
    // 创建MAC层属性
    WifiMacHelper adhocMac;
    adhocMac.SetType("ns3::AdhocWifiMac", "Ssid", SsidValue(ssid));// adhocmac

    NetDeviceContainer wifiAdHocDevices = wifi.Install(wifiPhy, adhocMac, nodes);
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/"
                "ShortGuardIntervalSupported", BooleanValue(false));
    
    // 获取频率
    Ptr<NetDevice> devicePtr = wifiAdHocDevices.Get(0);
    Ptr<WifiPhy> wifiPhyPtr = devicePtr->GetObject<WifiNetDevice>()->GetPhy();
    uint16_t frequency = wifiPhyPtr->GetFrequency();
    
    // 创建移动模型
    MobilityHelper mobility;
    RngSeedManager::SetSeed(seed);
    Ptr<UniformRandomVariable> xVal = CreateObject<UniformRandomVariable> ();
    xVal->SetAttribute ("Min", DoubleValue (minX));
    xVal->SetAttribute ("Max", DoubleValue (maxX));
    Ptr<UniformRandomVariable> yVal = CreateObject<UniformRandomVariable> ();
    yVal->SetAttribute ("Min", DoubleValue (minY));
    yVal->SetAttribute ("Max", DoubleValue (maxY));

    // 设置RandomRectanglePositionAllocator的属性
    mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                  "X", PointerValue (xVal),
                                  "Y", PointerValue (yVal));

    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
    mobility.Install(interferingNodes);

    PlotMultipleNodePositionsGnuplot(nodeContainers, nodeTypes, seed, outfileName); //绘制节点分布图

    // Configure waveform generator
    Ptr<SpectrumValue> wgPsd = Create<SpectrumValue>(generator_Spectrum_Model(frequency));
    *wgPsd = waveformPower / 20e6; // PSD spread across 20 MHz
    WaveformGeneratorHelper waveformGeneratorHelper;
    waveformGeneratorHelper.SetChannel(spectrumChannel);
    waveformGeneratorHelper.SetTxPowerSpectralDensity(wgPsd);
    waveformGeneratorHelper.SetPhyAttribute("Period", TimeValue(Seconds(0.0007)));
    waveformGeneratorHelper.SetPhyAttribute("DutyCycle", DoubleValue(1));
    NetDeviceContainer waveformGeneratorDevices =
        waveformGeneratorHelper.Install(interferingNodes);
    for(uint32_t i=0; i<waveformGeneratorDevices.GetN(); i++){
        Simulator::Schedule(Seconds(0.002),
                        &WaveformGenerator::Start,
                        waveformGeneratorDevices.Get(i)
                            ->GetObject<NonCommunicatingNetDevice>()
                            ->GetPhy()
                            ->GetObject<WaveformGenerator>());
    }

    // 配置路由和安装网络协议
    InternetStackHelper stack;

    stack.Install(nodes);
    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer ip = address.Assign(wifiAdHocDevices);

    Ipv4StaticRoutingHelper staticRouting;
    InitializeDirectRoutes(nodes, staticRouting, ip);//没必要保存到路由表中
    if(updateRoutes){ // default value is true
        UpdateStaticRoutingTable(nodes, staticRouting, ip, routingTable);
    }
    stack.SetRoutingHelper(staticRouting);

    if(!fileExists(throughputLinkTestFileName)){//如果没有进行过链路测试的话，先进行测试
        cout<<"starting link test..."<<endl;
        linkTest = true;
    }
    else if(reset){//准备重置吞吐率文件 default value is false
        throughput = ReadMatrixFromFile<double>(throughputLinkTestFileName);
        psr = ReadMatrixFromFile<double>(psrLinkTestFileName);
    }
    else{
        throughput = ReadMatrixFromFile<double>(throughputFileName);
        psr = ReadMatrixFromFile<double>(psrFileName);
    }
    
    // Calculate Throughput using Flowmonitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    ApplicationContainer apps_source;
    ApplicationContainer apps_sink;
    uint16_t count = 0;
    uint16_t port = 9;
    uint16_t initialDelay = 30; // 初始化延迟
    double startTime = initialDelay;
    double stopTime = startTime + simulationTime;

    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    Ipv4Address sinkAddress; // 用来存储sink节点的地址
    // 用于创建数据流的通用函数
    auto createDataFlow = [&](uint16_t source, uint16_t sink) {
        sinkAddress = ip.GetAddress(sink); // 获取sink节点的地址
        OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port));
        onoff.SetConstantRate(DataRate(to_string(datarate)+"Mb/s"), packetSize); // 设置数据生成速率
        onoff.SetAttribute("StartTime", TimeValue(Seconds(startTime)));
        onoff.SetAttribute("StopTime", TimeValue(Seconds(stopTime))); // 设置开始和结束时间
        apps_source.Add(onoff.Install(nodes.Get(source)));
        Simulator::Schedule(Seconds(stopTime + T / 2),
            &CalculateThroughput, monitor, classifier, &throughput, count, source, sink);
        startTime = stopTime + T;
        stopTime = startTime + simulationTime;
    };

    if(linkTest) { // default value is false
        for(sinkNode = 0; sinkNode < N; sinkNode++) {
            apps_sink.Add(sink.Install(nodes.Get(sinkNode)));
            for(sourceNode = 0; sourceNode < N; sourceNode++) {
                if(sourceNode != sinkNode) {
                    count++;
                    createDataFlow(sourceNode, sinkNode);
                }
            }
        }
    } else {
        apps_sink.Add(sink.Install(nodes.Get(sinkNode)));
        if(sourceNode != sinkNode) {
            count++;
            createDataFlow(sourceNode, sinkNode);
        } else {
            cerr << "sourceNode和sinkNode不能相同" << endl;
        }
    }    
    // 启动仿真器
    Simulator::Stop(Seconds(count*(T+simulationTime)+initialDelay));
    Simulator::Run();
    if(linkTest){
        SaveMatrixToFile(throughput, throughputLinkTestFileName);
        SaveMatrixToFile(psr, psrLinkTestFileName);
    }
    SaveMatrixToFile(throughput,throughputFileName);
    SaveMatrixToFile(psr,psrFileName);
    Simulator::Destroy();
    return 0;
}