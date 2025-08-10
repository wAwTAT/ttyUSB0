#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>

volatile sig_atomic_t keep_running = 1;

void sigint_handler(int sig) {
    (void)sig;
    keep_running = 0;
}

int main() {
    const char *device = "/dev/ttyUSB0";
    int fd;
    
    // 设置Ctrl+C信号处理
    signal(SIGINT, sigint_handler);
    
    // 打开串口设备
    fd = open(device, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        perror("无法打开串口设备");
        return EXIT_FAILURE;
    }
    
    // 配置串口参数
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(fd, &tty) != 0) {
        perror("获取串口配置失败");
        close(fd);
        return EXIT_FAILURE;
    }
    
    // 设置波特率为9600
    cfsetispeed(&tty, B9600);
    
    // 配置串口为阻塞模式
    tty.c_cflag |= (CLOCAL | CREAD);  // 启用接收器，忽略调制解调器控制线
    tty.c_cflag &= ~CSIZE;            // 清除数据位掩码
    tty.c_cflag |= CS8;               // 8位数据位
    tty.c_cflag &= ~PARENB;           // 禁用奇偶校验
    tty.c_cflag &= ~CSTOPB;           // 1位停止位
    tty.c_cflag &= ~CRTSCTS;          // 禁用硬件流控
    
    // 配置为原始输入模式（无回显，无信号处理）
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;            // 原始输出
    
    // 设置阻塞模式（无超时）
    tty.c_cc[VMIN] = 1;   // 至少读取1个字符
    tty.c_cc[VTIME] = 0;  // 无限期等待
    
    // 应用配置
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("配置串口失败");
        close(fd);
        return EXIT_FAILURE;
    }
    
    printf("正在阻塞模式监听 %s (按Ctrl+C退出)...\n", device);
    
    // 读取数据循环
    char buf[256];
    ssize_t n;
    while (keep_running) {
        n = read(fd, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';  // 添加字符串终止符
            printf("收到 %zd 字节: %s\n", n, buf);
        } else if (n < 0) {
            perror("读取错误");
            break;
        }
        // n == 0 在阻塞模式下不会发生
    }
    
    close(fd);
    printf("\n程序已退出\n");
    return EXIT_SUCCESS;
}
