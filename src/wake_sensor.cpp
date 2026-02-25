#include <iostream>
#include <string>
#include <serial/serial.h>
#include <chrono>
#include <thread>

using namespace std;

// 发送命令到传感器
void send_command(serial::Serial &ser, const string &cmd)
{
    ser.write(cmd);
    cout << "[发送] " << cmd << endl;
}

int main()
{
    cout << "=== 北微传感器唤醒工具 ===" << endl;

    string port = "/dev/ttyUSB0";
    unsigned long baud = 115200;

    serial::Serial my_serial;
    my_serial.setPort(port);
    my_serial.setBaudrate(baud);
    serial::Timeout to = serial::Timeout::simpleTimeout(500);
    my_serial.setTimeout(to);

    try {
        my_serial.open();
    } catch (serial::IOException &e) {
        cerr << "无法打开串口: " << e.what() << endl;
        return 1;
    }

    if(!my_serial.isOpen()) {
        cerr << "串口打开失败！" << endl;
        return 1;
    }

    cout << "串口打开成功！" << endl;

    // 等待传感器稳定
    this_thread::sleep_for(chrono::milliseconds(500));

    // 根据数据手册，尝试不同的唤醒命令
    cout << "\n尝试发送唤醒命令..." << endl;

    // 命令格式：0x77 0x04 0x00 0x00 (读取数据命令)
    // 根据数据手册第3页的输出协议
    unsigned char wake_cmd[] = {0x77, 0x04, 0x00, 0x00};
    my_serial.write(wake_cmd, 4);
    cout << "[发送] 77 04 00 00 (读取数据命令)" << endl;

    this_thread::sleep_for(chrono::milliseconds(100));

    // 尝试请求输出
    // 0x77 0x05 0x00 0x00 0x00 (请求输出)
    unsigned char output_cmd[] = {0x77, 0x05, 0x00, 0x00, 0x00};
    my_serial.write(output_cmd, 5);
    cout << "[发送] 77 05 00 00 00 (请求输出)" << endl;

    this_thread::sleep_for(chrono::milliseconds(100));

    // 读取响应
    cout << "\n等待传感器响应..." << endl;
    unsigned char buffer[100];
    size_t total_read = 0;

    auto start = chrono::steady_clock::now();
    while(chrono::duration_cast<chrono::seconds>(
        chrono::steady_clock::now() - start).count() < 5)
    {
        size_t available = my_serial.available();
        if(available > 0) {
            size_t n = my_serial.read(buffer + total_read, min(available, size_t(100 - total_read)));
            total_read += n;
            cout << "[收到] " << n << " 字节" << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    if(total_read > 0) {
        cout << "\n总接收 " << total_read << " 字节：" << endl;
        for(size_t i = 0; i < total_read; i++) {
            printf("%02X ", buffer[i]);
            if((i + 1) % 16 == 0) cout << endl;
        }
        cout << endl;
    } else {
        cout << "\n未收到任何数据！" << endl;
        cout << "\n可能的原因：" << endl;
        cout << "1. 传感器未上电" << endl;
        cout << "2. 串口连接错误" << endl;
        cout << "3. 波特率不匹配" << endl;
        cout << "4. 传感器需要特定的初始化命令" << endl;
    }

    my_serial.close();
    return 0;
}
