#include <iostream>
#include <string>
#include <serial/serial.h>
#include <chrono>
#include <thread>

using namespace std;

// 16进制转10进制（北微传感器格式）
double convert16to10(unsigned char chr1, unsigned char chr2, int np)
{
    int data1 = int(chr1);
    int data2 = int(chr2);

    double test2 = (int(data1 / 16 * 10) + data1 % 16 +
                   double(data2 / 16 * 10 + data2 % 16) / 100) * np;

    return test2;
}

// 将十六进制字节当作十进制数字解析
int hexByteToDecimal(unsigned char hex)
{
    int high = (hex >> 4) & 0x0F;  // 高4位
    int low = hex & 0x0F;          // 低4位
    return high * 10 + low;        // 当作十进制数字
}

// 解析北微传感器数据（14字节格式）
void DecodeIBEWISData(unsigned char chrTemp[])
{
    int np1 = 1;
    int np2 = 1;

    // 检查符号位（0x10表示负数）
    if(chrTemp[4] == 0x10)
        np1 = -1;
    if(chrTemp[7] == 0x10)
        np2 = -1;

    int sensor_num = int(chrTemp[2]);
    // 符号说明：前仰为正(Pitch)，右翻为正(Roll)
    double anglex = convert16to10(chrTemp[5], chrTemp[6], np1);  // Pitch: 前仰为正
    double angley = convert16to10(chrTemp[8], chrTemp[9], np2);  // Roll: 右翻为正

    // 解析航向角（Heading）- 将字节10-12当作十进制数字解析
    double anglez = hexByteToDecimal(chrTemp[10]) * 100.0 +
                    hexByteToDecimal(chrTemp[11]) +
                    hexByteToDecimal(chrTemp[12]) / 100.0;

    cout << "[解码] 传感器" << sensor_num
         << " - X: " << anglex << "°, Y: " << angley << "°"
         << ", Z(Heading): " << anglez << "°" << endl;
}

int main()
{
    cout << "=== 北微传感器串口测试工具 ===" << endl;

    string port = "/dev/ttyUSB0";
    unsigned long baud = 115200;

    cout << "打开串口: " << port << " @ " << baud << " baud" << endl;

    serial::Serial my_serial;
    my_serial.setPort(port);
    my_serial.setBaudrate(baud);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    my_serial.setTimeout(to);

    try {
        my_serial.open();
    } catch (serial::IOException &e) {
        cerr << "无法打开串口: " << e.what() << endl;
        return 1;
    }

    if(my_serial.isOpen()) {
        cout << "串口打开成功！" << endl;
    } else {
        cerr << "串口打开失败！" << endl;
        return 1;
    }

    cout << "\n开始读取数据 (按 Ctrl+C 退出)...\n" << endl;

    // 缓冲区
    const int BUFFER_SIZE = 1000;
    unsigned char buffer[BUFFER_SIZE];
    unsigned short rxLength = 0;

    int packet_count = 0;
    auto start_time = chrono::steady_clock::now();

    while(true) {
        // 读取可用数据
        size_t available = my_serial.available();
        if(available > 0) {
            // 读取数据
            size_t n = my_serial.read(buffer + rxLength, min(available, size_t(BUFFER_SIZE - rxLength)));
            rxLength += n;

            // 查找并解析数据包
            while(rxLength >= 14) {
                // 查找数据包头 0x77 0x0d
                if(buffer[0] != 0x77 || buffer[1] != 0x0d) {
                    // 移除第一个字节，继续查找
                    for(int i = 1; i < rxLength; i++)
                        buffer[i - 1] = buffer[i];
                    rxLength--;
                    continue;
                }

                // 显示原始数据（14字节标准格式）
                cout << "[原始14字节] ";
                for(int i = 0; i < 14; i++) {
                    printf("%02X ", buffer[i]);
                }
                cout << endl;

                // 解析数据
                DecodeIBEWISData(buffer);
                packet_count++;

                // 移除已处理的数据
                for(int i = 14; i < rxLength; i++)
                    buffer[i - 14] = buffer[i];
                rxLength -= 14;
            }
        }

        // 每5秒显示统计信息
        auto now = chrono::steady_clock::now();
        auto elapsed = chrono::duration_cast<chrono::seconds>(now - start_time).count();
        if(elapsed > 0 && elapsed % 5 == 0) {
            cout << "[统计] 已接收 " << packet_count << " 个数据包, 平均频率: "
                 << (packet_count / (double)elapsed) << " Hz" << endl;
        }

        this_thread::sleep_for(chrono::milliseconds(10));
    }

    my_serial.close();
    return 0;
}
