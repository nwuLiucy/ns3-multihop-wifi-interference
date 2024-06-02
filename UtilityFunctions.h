#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include "ns3/abort.h"
#include "ns3/applications-module.h"
#include "ns3/callback.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/gnuplot.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/log.h"
#include "ns3/lr-wpan-error-model.h"
#include "ns3/lr-wpan-helper.h"
#include "ns3/lr-wpan-mac.h"
#include "ns3/lr-wpan-net-device.h"
#include "ns3/lr-wpan-spectrum-value-helper.h"
#include "ns3/mac16-address.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/net-device-container.h"
#include "ns3/net-device.h"
#include "ns3/network-module.h"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/non-communicating-net-device.h"
#include "ns3/nstime.h"
#include "ns3/object-factory.h"
#include "ns3/packet.h"
#include "ns3/position-allocator.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/single-model-spectrum-channel.h"
#include "ns3/spectrum-value.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/uinteger.h"
#include "ns3/vector.h"
#include "ns3/waveform-generator-helper.h"
#include "ns3/waveform-generator.h"
#include "ns3/wifi-module.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <filesystem>


using namespace ns3;
namespace fs = std::filesystem;

// 从文件中读取矩阵
template <typename T>
std::vector<std::vector<T>> ReadMatrixFromFile(const std::string& inputFileName) {
    std::ifstream inputFile(inputFileName.c_str());
    std::vector<std::vector<T>> matrix;
    std::string line;
    if (!inputFile.is_open()) {
        throw std::runtime_error("File " + inputFileName + " not found");
    }
    while (getline(inputFile, line)) {
        std::istringstream iss(line);
        std::vector<T> row;
        T value;
        while (iss >> value) { // 使用 >> 操作符读取
            row.push_back(value);
            // 如果你的值是通过逗号分隔的，你需要去掉逗号
            if (iss.peek() == ',') iss.ignore();
        }
        matrix.push_back(row);
    }
    inputFile.close();
    return matrix;
}
// 将吞吐率矩阵保存到TXT文件中。函数模板，以支持任何类型的二维vector
template <typename T>
void SaveMatrixToFile(const std::vector<std::vector<T>>& matrix, const std::string& filename) {
    std::ofstream file(filename); 
    if (!file.is_open()) {
        std::cerr << "无法打开文件：" << filename << std::endl;
        return;
    }
    file << std::fixed << std::setprecision(3); // 设置浮点数打印格式为固定的小数点表示法，并保留三位小数
    for (const auto& row : matrix) {
        for (const auto& element : row) {
            file << std::setw(8) << element; // 设置宽度为8，并默认右对齐
        }
        file << "\n";
    }
    file.close();
}

// 初始化路由矩阵
void InitRouteMatrix(const std::string& outputFile, size_t nodes) {
    // 确保输出文件的目录存在，如果不存在则创建
    fs::create_directories(fs::path(outputFile).parent_path());
    std::ofstream out(outputFile);
    if (!out.is_open()) {
        throw std::runtime_error("Unable to open file " + outputFile);
    }
    
    for (size_t i = 0; i < nodes; ++i) {
        for (size_t j = 0; j < nodes; ++j) {
            out << ((i == j) ? "-1" : std::to_string(j));
            if (j < nodes - 1) out << " "; // 使用空格分隔
        }
        out << std::endl;
    }

    out.close();
}

bool fileExists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

// 节点位置可视化函数, 支持多种类型的节点绘制
void PlotMultipleNodePositionsGnuplot(
    const std::vector<NodeContainer> &nodeContainers, 
    const std::vector<std::string> &nodeTypes, 
    uint32_t &seed, 
    const std::string &outputFileName)
{
    // 检查参数长度是否一致
    if (nodeContainers.size() != nodeTypes.size()) {
        throw std::runtime_error("Node containers and node types size mismatch");
    }

    double minX = std::numeric_limits<double>::max(), maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max(), maxY = std::numeric_limits<double>::lowest();

    // 使用lambda函数简化遍历和更新坐标值的过程
    auto updateMinMaxCoordinates = [&](Ptr<Node> node) {
        Ptr<MobilityModel> mobilityModel = node->GetObject<MobilityModel>();
        Vector pos = mobilityModel->GetPosition();
        minX = std::min(minX, pos.x);
        maxX = std::max(maxX, pos.x);
        minY = std::min(minY, pos.y);
        maxY = std::max(maxY, pos.y);
    };

    for (const auto &nodeContainer : nodeContainers) {
        for (uint32_t j = 0; j < nodeContainer.GetN(); ++j) {
            updateMinMaxCoordinates(nodeContainer.Get(j));
        }
    }

    // 在最小值和最大值的基础上扩大10
    minX -= 10;
    maxX += 10;
    minY -= 10;
    maxY += 10;

    // 创建Gnuplot对象和Gnuplot散点图数据集
    Gnuplot plot(outputFileName + "_seed_" + std::to_string(seed) + ".pdf");
    plot.SetTitle("Node Positions with Random Seed " + std::to_string(seed));
    plot.SetLegend("X", "Y");
    plot.SetExtra("set xrange [" + std::to_string(minX) + ":" + std::to_string(maxX) + "]\n\
        set yrange [" + std::to_string(minY) + ":" + std::to_string(maxY) + "]\n\
        set grid\n\
        set key outside below");

    std::vector<Gnuplot2dDataset> datasets;
    std::vector<std::string> pointTypes = {"7", "9", "11", "13"}; // 实心圆、实心三角形、实心五角星、实心菱形
    std::vector<std::string> colors = {"\"black\"", "\"red\"", "\"blue\"", "\"green\""}; // 黑色、红色、蓝色、绿色
    for (size_t i = 0; i < nodeContainers.size(); ++i) {
        Gnuplot2dDataset dataset(nodeTypes[i]);
        dataset.SetStyle(Gnuplot2dDataset::POINTS);
        // 根据节点类型设置不同的颜色和点的样式
        std::string extra = "pt " + pointTypes[i % pointTypes.size()] + " lc rgb " + colors[i % colors.size()] + " pointsize 0.5";
        dataset.SetExtra(extra);

        // 遍历所有节点，获取位置并添加到数据集
        for (uint32_t j = 0; j < nodeContainers[i].GetN(); ++j) {
            Ptr<Node> node = nodeContainers[i].Get(j);
            Ptr<MobilityModel> mobilityModel = node->GetObject<MobilityModel>();
            Vector pos = mobilityModel->GetPosition(); // 获取位置
            dataset.Add(pos.x, pos.y); // 添加到数据集
            plot.AppendExtra("set label \"" + nodeTypes[i].substr(0, 1) + "-" + std::to_string(j) + "\" at " + std::to_string(pos.x) + "," + std::to_string(pos.y) + " font \",6\" textcolor rgb " + colors[i % colors.size()] + " offset char 0.5,0.5");
        }

        datasets.push_back(dataset);
    }

    // 将数据集添加到图表并生成图像文件
    for (auto &dataset : datasets) {
        plot.AddDataset(dataset);
    }
    std::ofstream plotFile(outputFileName + "_seed_" + std::to_string(seed) + ".plt");
    plot.GenerateOutput(plotFile);
    plotFile.close();
}

//// 计算两点之间的距离
//double CalculateCustomDistance(const ns3::Vector &a, const ns3::Vector &b) {
//    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
//}
//
//// 计算一个ZigBee节点距离所有Wi-Fi节点的距离倒数的和
//std::vector<double> CalculateInterference(const std::vector<ns3::Vector> &zigbeeNodePositions, const std::vector<ns3::Vector> &wifiNodePositions) {
//    std::vector<double> interference(zigbeeNodePositions.size(), 0.0);
//
//    for (size_t i = 0; i < zigbeeNodePositions.size(); ++i) {
//        for (size_t j = 0; j < wifiNodePositions.size(); ++j) {
//            double distance = CalculateCustomDistance(zigbeeNodePositions[i], wifiNodePositions[j]);
//            if (distance > 0) { // 避免除以0的情况
//                interference[i] += 10 / distance; // 将距离倒数加到受干扰程度上
//            }
//        }
//    }
//
//    return interference;
//}
//// 计算节点彼此之间的距离
//std::vector<std::vector<double>> CalculateNodeDistances(const std::vector<ns3::Vector> &nodePositions) {
//    std::vector<std::vector<double>> distances(nodePositions.size(), std::vector<double>(nodePositions.size(), 0.0));
//    for (size_t i = 0; i < nodePositions.size(); ++i) {
//        for (size_t j = 0; j < nodePositions.size(); ++j) {
//            if (i != j) { // 避免计算节点与自身的距离
//                double distance = CalculateCustomDistance(nodePositions[i], nodePositions[j]);
//                distances[i][j] = distance;
//            }
//        }
//    }
//    return distances;
//}
// 频谱生成函数
Ptr<SpectrumModel> generator_Spectrum_Model(uint16_t frequency)
{
    BandInfo bandInfo;
    bandInfo.fc = frequency*1e6;
    bandInfo.fl = bandInfo.fc - 10e6;
    bandInfo.fh = bandInfo.fc + 10e6;
    Bands bands;
    bands.push_back(bandInfo);
    Ptr<SpectrumModel> spectrumModel = Create<SpectrumModel>(bands);
    return spectrumModel;
}
#endif // UTILITY_FUNCTIONS_H