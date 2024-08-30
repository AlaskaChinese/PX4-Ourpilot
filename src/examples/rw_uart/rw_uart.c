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
    termios �������ṩ��һ��������ն˽ӿڣ����ڿ��Ʒ�ͬ��ͨ�Ŷ˿ڡ� ����ṹ�������������г�Ա��
    tcflag_t c_iflag;      ����ģʽ
    tcflag_t c_oflag;      ���ģʽ
    tcflag_t c_cflag;      ����ģʽ
    tcflag_t c_lflag;      ����ģʽ
    cc_t c_cc[NCCS];       �����ַ�
    */

    int termios_state;

    /* ���µ��������ṹ�� */
    /* ����ĳ��ѡ���ô��ʹ��"|="���㣬
     * ����ر�ĳ��ѡ���ʹ��"&="��"~"����
     * */
    tcgetattr(fd, &uart_config); // ��ȡ�ն˲���

    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;// ��NLת����CR(�س�)-NL�������

    /* ��żУ�飬һ��ֹͣλ */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);// CSTOPB ʹ������ֹͣλ��PARENB ��ʾżУ��

    /* ���ò����� */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }
    // �������ն���صĲ�����TCSANOW �����ı����
    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }
    return true;
}

int uart_init(const char *uart_name) {
    /*Linux�У�������ļ����򿪴����豸�ʹ���ͨ�ļ�һ����ʹ�õ���open����ϵͳ����*/
    // ѡ�� O_NOCTTY ��ʾ���ܰѱ����ڵ��ɿ����նˣ������û��ļ���������Ϣ��Ӱ������ִ��
    /*
     * O_CREAT Ҫ�򿪵��ļ���������ʱ�Զ��������ļ���
     * O_EXCL Ҫ�� O_CREAT һ��ʹ�ò�����Ч�� ����ļ������� open()����ʧ�ܡ�
     * O_RDONLY ֻ��ģʽ���ļ���
     * O_WRONLY ֻдģʽ���ļ���
     * O_RDWR �ɶ���дģʽ���ļ���
     * O_APPEND ��׷��ģʽ���ļ���
     * O_NONBLOCK �Է�����ģʽ�򿪡�
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

	// �����ȡ�����������ַ� 'm'���������ȡ��������4���ֽڲ���ӡ����
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
