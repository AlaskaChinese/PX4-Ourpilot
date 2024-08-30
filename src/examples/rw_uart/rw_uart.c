#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <string.h>
#include <systemlib/err.h>
#include <nuttx/config.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <poll.h>

__EXPORT int rw_uart_main(int argc, char *argv[]);

static int uart_init(const char *uart_name);
static int set_uart_baudrate(int fd, unsigned int baud);

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
    case 9600:   speed = 9600;   break;
    case 19200:  speed = 19200;  break;
    case 38400:  speed = 38400;  break;
    case 57600:  speed = 57600;  break;
    case 115200: speed = 115200; break;
    default:
        warnx("ERR: baudrate: %d\n", baud);
        return -EINVAL;
    }

    struct termios uart_config;
    /*
    termios 函数族提供了一个常规的终端接口，用于控制非同步通信端口。 这个结构包含了至少下列成员：
    tcflag_t c_iflag;      输入模式
    tcflag_t c_oflag;      输出模式
    tcflag_t c_cflag;      控制模式
    tcflag_t c_lflag;      本地模式
    cc_t c_cc[NCCS];       控制字符
    */

    int termios_state;

    /* 以新的配置填充结构体 */
    /* 设置某个选项，那么就使用"|="运算，
     * 如果关闭某个选项就使用"&="和"~"运算
     * */
    tcgetattr(fd, &uart_config); // 获取终端参数

    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;// 将NL转换成CR(回车)-NL后输出。

    /* 无偶校验，一个停止位 */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB 使用两个停止位，PARENB 表示偶校验

    /* 设置波特率 */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }
    // 设置与终端相关的参数，TCSANOW 立即改变参数
    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }
    return true;
}

int uart_init(const char *uart_name) {
    /*Linux中，万物皆文件，打开串口设备和打开普通文件一样，使用的是open（）系统调用*/
    // 选项 O_NOCTTY 表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行
    /*
     * O_CREAT 要打开的文件名不存在时自动创建改文件。
     * O_EXCL 要和 O_CREAT 一起使用才能生效， 如果文件存在则 open()调用失败。
     * O_RDONLY 只读模式打开文件。
     * O_WRONLY 只写模式打开文件。
     * O_RDWR 可读可写模式打开文件。
     * O_APPEND 以追加模式打开文件。
     * O_NONBLOCK 以非阻塞模式打开。
     * */
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

int rw_uart_main(int argc, char *argv[]) {
    char data = '0';
    char buffer[4] = "";
    /*
     * USART1	/dev/ttyS0	GPS
     * USART2	/dev/ttyS1	TELEM3
     * USART3	/dev/ttyS2	Debug Console
     * UART4	/dev/ttyS3	UART4
     * UART5	/dev/ttyS4	TELEM2
     * USART6	/dev/ttyS5	PX4IO/RC
     * UART7	/dev/ttyS6	TELEM1
     * UART8	/dev/ttyS7	GPS2
     */
    int uart_fd = uart_init("/dev/ttyS3");

    if(false == uart_fd){
        printf("read uart failed\n");
        return -1;
    }

    if (false == set_uart_baudrate(uart_fd, 115200)) {
        printf("set_uart_baudrate is failed\n");
        return -1;
    }

    printf("[JXF]UART init is successful\n");

    while (true) {
        read(uart_fd, &data, 1);

	// 如果读取到的数据是字符 'm'，则继续读取接下来的4个字节并打印出来
	// mr 0f 000005a4 000004c8 00000436 000003f9 0958 c0 40424042 a0:0
        if (data == 'm') {
            for(int i = 0;i <4;++i){
                read(uart_fd, &data, 1);
                buffer[i] = data;
                data = '0';
            }
            printf("%s\n", buffer);
        }
    }

    close(uart_fd);
    return 0;
}
