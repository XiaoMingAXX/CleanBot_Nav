<h2 id="6ab90836"><font style="color:rgb(0, 0, 0);">一、任务目标</font></h2>
<font style="color:rgb(0, 0, 0);">基于用户定义的 USB 通讯协议（虚拟串口 / CDC），修改 STM32 端代码，实现：</font>

1. <font style="color:rgb(0, 0, 0);">与树莓派（ROS2）的双向数据交互（下行控制命令接收、上行传感器 / 状态数据发送）；</font>
2. <font style="color:rgb(0, 0, 0);">符合协议规范的帧打包、解析、CRC 校验、ACK 反馈；</font>
3. <font style="color:rgb(0, 0, 0);">基于 FreeRTOS 的实时任务调度（满足高频数据传输与低延迟控制）；</font>
4. <font style="color:rgb(0, 0, 0);">与电机、IMU、编码器、传感器的硬件交互适配；</font>
5. <font style="color:rgb(0, 0, 0);">容错与安全策略（丢包检测、超时重连、紧急停机）。</font>

<h2 id="eec599ba"><font style="color:rgb(0, 0, 0);">二、核心设计原则（必须遵守）</font></h2>
1. <font style="color:rgb(0, 0, 0);">单一二进制帧格式：统一头 + 长度 + ID+seq+payload+CRC16，解析高效无冗余；</font>
2. <font style="color:rgb(0, 0, 0);">频率分级：轮速数据 200Hz、IMU 数据 100Hz、传感器状态 20-50Hz、系统状态 1-5Hz，高频数据独立帧、低频数据合并帧；</font>
3. <font style="color:rgb(0, 0, 0);">ACK 策略：仅关键命令（模式切换、开始对位、dock）需 ACK，速度指令为高频无 ACK（取最新值）；</font>
4. <font style="color:rgb(0, 0, 0);">数据格式：小端字节序（Little-endian），浮点用 IEEE754 float32，状态类数据用 uint8/uint16 压缩；</font>
5. <font style="color:rgb(0, 0, 0);">校验规则：CRC-16-CCITT（多项式 0x1021，初始值 0xFFFF），计算范围为 VER 到 PAYLOAD 的所有字节。</font>

<h2 id="646c4b8f"><font style="color:rgb(0, 0, 0);">三、协议规范（开发依据）</font></h2>
<h3 id="49704d42"><font style="color:rgb(0, 0, 0);">1. 统一帧格式（字节序：小端）</font></h3>
| **<font style="color:rgb(0, 0, 0) !important;">字段</font>** | **<font style="color:rgb(0, 0, 0) !important;">字节数</font>** | **<font style="color:rgb(0, 0, 0) !important;">内容说明</font>** |
| :--- | :--- | :--- |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">HEADER</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">2</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">固定标识 0x55 0xAA</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">VER</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">1</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">协议版本（固定为 1）</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">LEN</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">2</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">payload 字节数（不含头、VER、LEN、MSG_ID、SEQ、CRC）</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">MSG_ID</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">1</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">消息类型（见下方定义）</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">SEQ</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">1</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">帧计数（0-255 循环，用于丢包检测）</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">PAYLOAD</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">LEN</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">消息具体数据（按 MSG_ID 定义）</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">CRC</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">2</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">校验值（计算 VER~PAYLOAD 字节，小端存储）</font> |


<h3 id="a92b7599"><font style="color:rgb(0, 0, 0);">2. 消息 ID 与 Payload 定义</font></h3>
<h4 id="dc0a569b"><font style="color:rgb(0, 0, 0);">（1）下行消息（树莓派→STM32）</font></h4>
| **<font style="color:rgb(0, 0, 0) !important;">MSG_ID</font>** | **<font style="color:rgb(0, 0, 0) !important;">消息名称</font>** | **<font style="color:rgb(0, 0, 0) !important;">频率</font>** | **<font style="color:rgb(0, 0, 0) !important;">Payload 定义（总 14 字节）</font>** | **<font style="color:rgb(0, 0, 0) !important;">说明</font>** |
| :--- | :--- | :--- | :--- | :--- |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">0x10</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">CONTROL_CMD</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">20-50Hz</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">float32 left_wheel_speed_mps（左轮速度 m/s）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">主控制帧，含速度 + 状态命令</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">float32 right_wheel_speed_mps（右轮速度 m/s）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 work_mode（0=Idle/1 = 自动全屋 / 2 = 沿边 / 3 = 弓形 / 4 = 遥控 / 5 = 回充）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 side_brush_left_level（0-3 档）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 side_brush_right_level（0-3 档）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 fan_level（0-3 档）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 water_level（0-3 档）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">bool need_ack（True = 需 ACK/False = 无需）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">关键命令时置为 True</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 reserved（填充 0）</font> | |


<h4 id="15f9f741"><font style="color:rgb(0, 0, 0);">（2）上行消息（STM32→树莓派）</font></h4>
| **<font style="color:rgb(0, 0, 0) !important;">MSG_ID</font>** | **<font style="color:rgb(0, 0, 0) !important;">消息名称</font>** | **<font style="color:rgb(0, 0, 0) !important;">频率</font>** | **<font style="color:rgb(0, 0, 0) !important;">Payload 定义</font>** | **<font style="color:rgb(0, 0, 0) !important;">说明</font>** |
| :--- | :--- | :--- | :--- | :--- |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">0x20</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">IMU</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">100Hz</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">float32 ax/ay/az（线加速度 m/s²）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">运动姿态数据</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">float32 gx/gy/gz（角速度 rad/s）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">float32 roll/pitch/yaw（姿态角 rad）</font> | |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">0x21</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">WHEEL</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">200Hz</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">float32 left_wheel_angle_deg（左轮角度 °）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">编码器反馈数据</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">float32 left_wheel_speed_mps（左轮实际速度 m/s）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">float32 right_wheel_angle_deg（右轮角度 °）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">float32 right_wheel_speed_mps（右轮实际速度 m/s）</font> | |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">0x22</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">SENSORS_STATUS</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">20-50Hz</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 bumper_left（0 = 无碰撞 / 1 = 碰撞）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">传感器与状态反馈</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 bumper_right（0 = 无碰撞 / 1 = 碰撞）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 ir_down0/ir_down1/ir_down2（0 = 正常 / 1 = 触发）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">下视红外传感器</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 fault_flags（故障位掩码）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">各 bit 对应不同故障</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 heartbeat_counter（心跳计数）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">周期递增，用于断线检测</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 dock_status（0 = 无 / 1 = 接近 / 2 = 成功 / 3 = 失败）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">回充状态</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 reserved（填充 0）</font> | |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">0x23</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">SYSTEM_STATUS</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">1-5Hz</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">（可选）电池电压（uint16，单位 mV）+ 当前 mode + 报警码</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">系统级状态（可合并到 0x22）</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">0x24</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">ACK_REPLY</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">按需</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 cmd_id（对应下行命令 MSG_ID）</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">关键命令确认回复</font> |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">uint8 status（0 = 成功 / 1 = 失败 / 2 = 忙碌）</font> | |
| | | | <font style="color:rgba(0, 0, 0, 0.85) !important;">（可选）uint8 info（附加信息）</font> | |


<h3 id="9a364466"><font style="color:rgb(0, 0, 0);">3. CRC 实现代码（直接复用）</font></h3>
**<font style="color:rgba(0, 0, 0, 0.85);">c</font>**

<font style="color:rgba(0, 0, 0, 0.5);">运行</font>

```c
#include <stdint.h>

// CRC-16-CCITT：poly=0x1021，初始值0xFFFF，计算范围：VER~PAYLOAD
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

<h2 id="ee6fddf4"><font style="color:rgb(0, 0, 0);">四、STM32 端实现要求</font></h2>
<h3 id="2d0a7926"><font style="color:rgb(0, 0, 0);">1. 硬件资源配置</font></h3>
+ <font style="color:rgb(0, 0, 0);">通讯接口：USB CDC（虚拟串口）或 UART+USB 转串口，波特率≥230400（推荐 921600）；</font>
+ <font style="color:rgb(0, 0, 0);">DMA 配置：UART RX 用 DMA 循环模式 + IDLE 中断，TX 用 DMA 发送（避免阻塞）；</font>
+ <font style="color:rgb(0, 0, 0);">定时器：用于生成固定频率的传感器采集与数据发送触发（100Hz/200Hz/50Hz）；</font>
+ <font style="color:rgb(0, 0, 0);">中断：编码器计数中断、传感器触发中断（优先级高于普通任务）。</font>

<h3 id="55a6292e"><font style="color:rgb(0, 0, 0);">2. FreeRTOS 任务架构（含优先级）</font></h3>
| **<font style="color:rgb(0, 0, 0) !important;">任务名称</font>** | **<font style="color:rgb(0, 0, 0) !important;">优先级</font>** | **<font style="color:rgb(0, 0, 0) !important;">核心职责</font>** | **<font style="color:rgb(0, 0, 0) !important;">周期 / 触发方式</font>** |
| :--- | :--- | :--- | :--- |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">Sensors 采集任务</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">高</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">1. 100Hz 读取 IMU 数据（ax/ay/az/gx/gy/gz/ 姿态角）；2. 200Hz 读取编码器数据（角度 + 速度）；3. 20-50Hz 读取碰撞、红外、故障状态</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">定时器周期触发</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">通讯 TX 发送任务</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">中高</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">1. 从发送环形队列取打包好的帧；2. 通过 DMA 发送（按频率优先级：WHEEL>IMU>SENSORS>SYSTEM）；3. 关键命令 ACK 的超时重发</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">队列非空触发 + 周期兜底</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">通讯 RX 解析任务</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">中</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">1. 从接收环形缓冲读取字节；2. 有限状态机（FSM）解析帧（HEADER→VER→LEN→Payload→CRC 校验）；3. 解析后的数据写入线程安全共享结构</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">IDLE 中断触发 + 队列非空触发</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">控制执行任务</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">高</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">1. 从共享结构读取下行 CONTROL_CMD（速度 + 模式命令）；2. 将速度目标值传入电机 PID 控制器；3. 执行模式切换、刷子 / 风扇 / 水量控制</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">20-50Hz 周期触发</font> |
| <font style="color:rgba(0, 0, 0, 0.85) !important;">安全监控任务</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">最高</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">1. 监控心跳与串口状态（N 秒无心跳则安全停机）；2. 碰撞 / 故障触发时立即停止电机；3. 处理紧急停机逻辑</font> | <font style="color:rgba(0, 0, 0, 0.85) !important;">100Hz 周期触发 + 中断触发</font> |


<h3 id="810c9179"><font style="color:rgb(0, 0, 0);">3. 关键模块实现细节</font></h3>
<h4 id="9306c5a6"><font style="color:rgb(0, 0, 0);">（1）帧打包与解析</font></h4>
+ <font style="color:rgb(0, 0, 0);">打包函数：按 MSG_ID 生成对应 Payload，填充帧头、LEN、SEQ、CRC，支持批量打包高频数据；</font>
+ <font style="color:rgb(0, 0, 0);">解析 FSM 状态：IDLE→WAIT_HEADER1（0x55）→WAIT_HEADER2（0xAA）→READ_VER→READ_LEN→READ_MSGID→READ_SEQ→READ_PAYLOAD→READ_CRC→VERIFY_CRC→DISPATCH_DATA；</font>
+ <font style="color:rgb(0, 0, 0);">SEQ 管理：每个 MSG_ID 独立维护 SEQ 计数器（0-255 循环），接收端通过 SEQ 差判断丢包。</font>

<h4 id="da063634"><font style="color:rgb(0, 0, 0);">（2）数据交互与硬件适配</font></h4>
+ <font style="color:rgb(0, 0, 0);">电机控制：接收 CONTROL_CMD 中的 left/right_wheel_speed_mps，转换为电机目标转速，传入现有 PID 控制器；</font>
+ <font style="color:rgb(0, 0, 0);">编码器读取：通过硬件中断或定时器捕获，计算轮子角度（累计）和实际速度（瞬时），用于上行 WHEEL 消息；</font>
+ <font style="color:rgb(0, 0, 0);">IMU 读取：通过 I2C/SPI 读取原始数据，按需进行姿态融合（或直接输出原始数据），按 100Hz 打包为 0x20 消息；</font>
+ <font style="color:rgb(0, 0, 0);">传感器读取：碰撞传感器（ bumper ）、下视红外（ ir_down ）通过 GPIO 中断或轮询读取，故障状态通过寄存器查询，打包为 0x22 消息。</font>

<h4 id="8f13d173"><font style="color:rgb(0, 0, 0);">（3）ACK 与容错逻辑</font></h4>
+ <font style="color:rgb(0, 0, 0);">ACK 触发：当接收的 0x10 帧中 need_ack=True 时，执行命令后（如模式切换完成、回充启动），立即发送 0x24 ACK_REPLY；</font>
+ <font style="color:rgb(0, 0, 0);">超时重发：关键命令（如回充、模式切换）发送后，等待 ACK 超时（建议 500ms），重试 3 次，失败则置故障标志；</font>
+ <font style="color:rgb(0, 0, 0);">丢包处理：通过 SEQ 检测丢包，高频数据（WHEEL/IMU）无需重发（取最新帧），关键状态数据（故障、回充结果）可主动补发；</font>
+ <font style="color:rgb(0, 0, 0);">串口重连：USB 断开后，自动重启 CDC / 串口初始化，重连期间停止电机，上报故障状态。</font>

<h4 id="de46e4a9"><font style="color:rgb(0, 0, 0);">（4）安全策略</font></h4>
+ <font style="color:rgb(0, 0, 0);">紧急停机：碰撞传感器触发、串口断线超时（建议 3 秒）、故障标志置位时，立即停止电机（速度指令置 0），禁用所有执行器；</font>
+ <font style="color:rgb(0, 0, 0);">心跳机制：SENSORS_STATUS 中的 heartbeat_counter 每 100ms 递增 1，树莓派无心跳则触发安全停机；</font>
+ <font style="color:rgb(0, 0, 0);">故障上报：通过 fault_flags 位掩码记录电机故障、传感器故障、通讯故障，上行至树莓派。</font>



<h2 id="e6231cb6"><font style="color:rgb(0, 0, 0);">五、验收标准</font></h2>
1. <font style="color:rgb(0, 0, 0);">协议兼容性：发送 / 接收的帧符合定义格式，CRC 校验 100% 通过，树莓派端可正确解析所有上行消息；</font>
2. <font style="color:rgb(0, 0, 0);">实时性：高频数据发送频率达标（IMU≥100Hz，WHEEL≥200Hz），控制命令延迟≤50ms；</font>
3. <font style="color:rgb(0, 0, 0);">可靠性：连续运行 1 小时，无丢包导致的控制失效，断线重连成功率 100%，关键命令 ACK 响应率 100%；</font>
4. <font style="color:rgb(0, 0, 0);">安全性：碰撞、断线、故障时能立即停机，无电机失控情况；</font>
5. <font style="color:rgb(0, 0, 0);">硬件适配：所有传感器数据正常上报，电机能按速度指令准确运行，执行器（刷子、风扇、水泵）按命令切换档位。</font>

<h2 id="6a2a9b6c"><font style="color:rgb(0, 0, 0);">六、依赖与注意事项</font></h2>
1. <font style="color:rgb(0, 0, 0);">依赖资源：STM32 的 USB CDC 驱动、FreeRTOS 内核、现有电机 PID 控制代码、传感器硬件驱动；</font>
2. <font style="color:rgb(0, 0, 0);">字节序校验：严格使用小端字节序，float32 打包 / 解包需确认 IEEE754 兼容性；</font>
3. <font style="color:rgb(0, 0, 0);">内存管理：使用静态内存分配（避免 malloc），环形队列大小需适配最大发送频率（如 WHEEL 200Hz×25 字节 = 5KB/s，队列深度建议 100）；</font>
4. <font style="color:rgb(0, 0, 0);">调试支持：预留 USART 调试口，输出帧解析状态、任务运行状态、故障信息，便于问题定位。</font>

