# PX4-Ourpilot
 
<!-- @import "[TOC]" {cmd="toc" depthFrom=1 depthTo=6 orderedList=false} -->

<!-- code_chunk_output -->

- [PX4-Ourpilot](#px4-ourpilot)
- [PX4开发步骤](#px4开发步骤)
  - [1. 快速上手](#1-快速上手)
    - [1.1 提示](#11-提示)
  - [2. 通过 VS Code 开发](#2-通过-vs-code-开发)
    - [2.1 安装docker](#21-安装docker)
  - [3. 第一个Demo](#3-第一个demo)
    - [3.1 工程建立](#31-工程建立)
    - [3.2 编译、生成固件和仿真](#32-编译-生成固件和仿真)
    - [3.3 烧录固件与应用程序运行](#33-烧录固件与应用程序运行)
    - [3.4 应用程序类型](#34-应用程序类型)
      - [3.4.1 工作队列任务](#341-工作队列任务)
      - [3.4.2 任务](#342-任务)
  - [4. 传感器](#4-传感器)
    - [4.1 GPS](#41-gps)
  - [5. 为固件添加一个串口读取程序](#5-为固件添加一个串口读取程序)
    - [5.1 开始](#51-开始)
    - [5.2 代码分析](#52-代码分析)
    - [5.3 测试](#53-测试)
    - [5.4 总结](#54-总结)
    - [5.5 读取任意长度字符串](#55-读取任意长度字符串)
      - [5.5.1 测试](#551-测试)
- [开发时会用到的命令](#开发时会用到的命令)
    - [更多内容施工中](#更多内容施工中)
    - [任务](#任务)

<!-- /code_chunk_output -->


# PX4开发步骤
## 1. 快速上手

首先安装`Ubuntu 22.04 LTS`虚拟机，安装完成后在终端输入：

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

以克隆固件二次开发源码，然后输入：

```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
等待全部提示运行通过后，重启虚拟机。然后在终端输入：

```bash
source /opt/ros/humble/setup.bash
export GZ_SIM_RESOURCE_PATH=~/.gz/models
```

将`QGroundControl`地面站下载到系统中，[这里是下载链接](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)。

下载完成后根据链接里的指导添加运行权限后，双击运行。然后在终端输入：

```bash
make px4_sitl gz_x500
```

此为`Gazebo`仿真，官方推荐使用。或者输入：

```bash
make px4_sitl jmavsim
```

编译运行`jmavsim`仿真，该仿真平台由社区开发维护。

操作完成后等待片刻即可看到仿真图形化界面，并且地面站显示已连接，随后终端提示`ready to takeoff`。

### 1.1 提示

固件编译需要下载一系列工具链，而ROS2环境中已经包含了大部分工具链，所以在进行以上操作前，可以先进行ROS2的安装，使用小鱼ROS一键安装，在终端输入：

```bash
wget http://fishros.com/install -O fishros && sudo bash fishros
```

根据提示安装`ROS2 Humble`即可。

## 2. 通过 VS Code 开发
[官方指南](https://docs.px4.io/main/zh/dev_setup/vscode.html)

安装好`VS Code`后左上角选择`"文件-打开文件夹-PX4-Autopilot"`即可。

根据提示安装全部插件，以及NET框架等。

大部分操作根据提示即可完成，不过多赘述。此处只指出官方指南无法解决的问题。

### 2.1 安装docker

通过官方指南安装docker会遇到无法与官方源建立连接的问题，可以使用阿里镜像源。在终端输入：

```bash
# step 1: 安装必要的一些系统工具
sudo apt-get update
sudo apt-get -y install apt-transport-https ca-certificates curl software-properties-common
# step 2: 安装GPG证书
curl -fsSL https://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg | sudo apt-key add -
# Step 3: 写入软件源信息
sudo add-apt-repository "deb [arch=amd64] https://mirrors.aliyun.com/docker-ce/linux/ubuntu $(lsb_release -cs) stable"
# Step 4: 更新并安装Docker-CE
sudo apt-get -y update
sudo apt-get -y install docker-ce
```

然后[在此处](https://docs.docker.com/desktop/install/ubuntu/)下载`DEB PACKAGE`。*`提示：你可能需要使用代理才能打开此网站。`*

下载完毕后在终端输入：

```bash
sudo apt install `将刚才的安装包拖到终端然后回车`
```

## 3. 第一个Demo

### 3.1 工程建立

在`./src/examples`中创建`hello_sky`文件夹。

在`hello_sky`中创建一个`hello_sky.c`，按照官方要求，二次开发需要添加以下注释：

```c
/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
```

然后开始写代码，注意主函数必须命名为`module_main`(`module`自定义)。

```c
/**
 * @file hello_sky.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/log.h>

__EXPORT int hello_sky_main(int argc, char *argv[]);

int hello_sky_main(int argc, char *argv[])
{
   PX4_INFO("Hello Sky!");
   return OK;
}
```

创建一个名为`CMakeLists.txt`的`cmake`定义文件，添加以下代码：

```txt
############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
px4_add_module(
 MODULE examples__hello_sky
 MAIN hello_sky
 STACK_MAIN 2000
 SRCS
     hello_sky.c
 DEPENDS
 )
```

创建一个`Kconfig`文件，添加以下代码：

```
menuconfig EXAMPLES_HELLO_SKY
 bool "hello_sky"
 default n
 ---help---
     Enable support for hello_sky
```

### 3.2 编译、生成固件和仿真

代码写好后，在终端中输入：

```bash
make px4_sitl_default boardconfig
```

在`examples`里面勾选上我们刚才写好的`hello_sky`。

保存后在终端中输入：

```bash
make px4_sitl_default jmavsim
```

即可编译并开启仿真，此时在终端中输入`hello_sky`或者`hello_sky start`即可运行我们写的程序。

### 3.3 烧录固件与应用程序运行

打开QGroundControl地面站，用USB将电脑和飞控连接好后，点击左上角打开`Vehicle Setup`，点击`firmware`，将飞控断开连接再插上，会提示烧录固件，此时选择`自定义固件`，将`../PX4-Autopilot/build/px4_fmu-v6x_default`中的`.bin`或者`.px4`选中烧录即可。

退出到主界面，点击左上角打开`Analyse Tools`，`MAVLink Console`即为飞控的系统控制终端，可以输入`<所编译的应用程序的名称> start`来运行我们所编写的应用程序。

### 3.4 应用程序类型

#### 3.4.1 工作队列任务

一个运行在工作队列线程上的模块, 与工作队列上的其他任务分享堆栈和线程优先级。在大多数情况下，可以使用工作队列任务，因为这会减少资源的使用。它需要指定它是一个工作队列任务，并在初始化期间运行调度它本身。

代码模板在以下目录：`src/examples/work_item`.

在以下初始化函数中指定要添加任务的队列：
```cpp
WorkItemExample::WorkItemExample() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
  //...
}
```
`platforms/common/include/px4_platform_common/px4_work_queue/WorkQueueManager.hpp`中列出了可用的工作队列。

#### 3.4.2 任务

一个有自己的堆栈和处理优先级的模块。该应用程序是在其自己的堆栈上作为任务运行。

代码模板在以下目录：`src/templates/template_module`.

一个任务包含
- 访问参数并对参数更新做出反应。
- 订阅、等待topic更新。
- 通过`start`/`stop`/`status`控制任务。而`module start [<arguments>]`可以直接加入[启动脚本](https://docs.px4.io/main/zh/concept/system_startup.html)中，使程序自启动。
- 命令行参数解析。
- 文档记录：`PRINT_MODULE_*`，[了解更多](https://github.com/PX4/Firmware/blob/v1.8.0/src/platforms/px4_module.h#L381)。

## 4. 传感器

### 4.1 GPS

Neo 3 即插即用，无需配置，可在地面站设置是否使用GPS、北斗、伽利略、格洛纳斯定位。

Neo 3 Pro 则需要在地面站的`Parameters`中作以下设置：
- 将`UAVCAN_ENABLESet`设置为`Sensors Automatic config`
- 将`UAVCAN_SUB_GPS`设置为`Enable`

在MAVLink终端中可以通过输入`gps status`查看 Neo 3 的信息。
而GPS产生的数据可以在应用程序中订阅并发布，但具体到编程还需要时间熟悉。

## 5. 为固件添加一个串口读取程序

### 5.1 开始

首先在PX4源码目录中`/src/examples`里面创建文件夹，命名为`rw_uart`，并在该文件夹下添加以下文件：`rw_uart.c`、`Kconfig`和`CMakeLists.txt`。

然后输入`make px4_fmu-v6x_default boardconfig`，在`examples`里面勾选`rw_uart`，勾选后保存并退出。然后输入`make px4_fmu-v6x_default`编译并生成固件。再将固件通过QGC上传至飞控即可。

### 5.2 代码分析

`Nuttx`是一个嵌入式实时操作系统，它的运作方式以及开发方式都与Linux特别相似。

而在`Linux`中，万物皆文件，打开串口设备和打开普通文件一样，使用的是`open()`函数。而使用选项`O_NOCTTY`表示不能把本串口当成控制终端，否则用户的键盘输入信息将影响程序的执行。如果没有找到该设备则输出错误信息。设备可以在终端中使用`ls /dev`查看。而对于我们的`CUAV Pixhawk V6X`设备树详见下表。

| Port | Path | Driver |
| :---: | :---: | :---: |
| USART1 | /dev/ttyS0 | GPS
| USART2 | /dev/ttyS1 | TELEM3
| USART3 | /dev/ttyS2 | Debug Console
| UART4 | /dev/ttyS3 | UART4
| UART5	| /dev/ttyS4 | TELEM2
| USART6 | /dev/ttyS5 | PX4IO/RC
| UART7 | /dev/ttyS6 | TELEM1
| UART8	| /dev/ttyS7 | GPS2

```c
// 串口初始化函数
int uart_init(const char *uart_name) {
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}
```

初始化串口后还需要设置相关参数，比如波特率、有无校验位、停止位。
```c
// 波特率设置函数
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

    tcgetattr(fd, &uart_config); // 获取终端参数

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
```

以上函数都写好后，就不需要再改动了，数据接收与预处理都在主函数中进行修改，后处理需要在`module`和QGC的二次开发中进行，比如我们要接收`mr 0f 000005a4 000004c8 00000436 000003f9 0958 c0 40424042 a0:0`这样一连串并保存，我们可以创建一个结构体，把每个部分都创建一个成员，这些成员的数据长度都是不会变化的，在串口读取时我们可以很简单地使用for循环来遍例读取给定长度的字节，并保存到结构体成员中，再进行后续的uORB相关数据处理。
```c
// 主函数
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
```

### 5.3 测试

将飞控上的UART4与STlink V3上的串口相连，打开串口助手和QGC，首先在QGC的MAVLINK终端上输入`rw_uart`运行程序，然后使用串口助手打开STlink的串口，向飞控的UART4发送数据，飞控接收后在QGC的MAVLINK终端上把数据打印出来。
![test](/assets/53test.png)

### 5.4 总结

这部分主要难点还是在于之前都没有接触过Linux的系统开发，后来学习了一些嵌入式Linux的内容后，思路也变得清晰了，结合着网上的资料还是成功把这一块做了出来。

程序需要改进的地方：
* 适配UWB数据格式
* 程序什么时候退出

### 5.5 读取任意长度字符串

在`while`循环内加入如下代码，并在主函数内定义相关变量。完整代码见`rw_uart2.c`
```c
// int uart_fd;  // 串口文件描述符
// char buffer[256];  // 缓冲区
// int read_bytes;  // 读取的字节数
memset(buffer, '\0', sizeof(buffer));  // 初始化缓冲区
read_bytes = read(uart_fd, buffer, sizeof(buffer));
if (read_bytes > 0)
{
    printf("read %d bytes: %s\n", read_bytes, buffer);
}
```
#### 5.5.1 测试

测试发现数据是完整的，但不能一次性打印完毕。***（亟待解决）***

![test](/assets/5511test.png)
![test](/assets/551test.png)

---

# 开发时会用到的命令

```bash
make px4_sitl gz_x500
make px4_sitl jmavsim
make px4_sitl_default boardconfig
make px4_sitl_default jmavsim
make px4_fmu-v6x_default boardconfig
make px4_fmu-v6x_default
```

---

### 更多内容施工中

[参考链接](https://docs.px4.io/main/zh/)

by Alaska

### 任务

1. 用终端显示外部传感器的数据
