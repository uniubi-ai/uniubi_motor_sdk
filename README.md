# UNIUBI6913 关节电机 C SDK

UNIUBI6913 关节电机对外接入库，版本 `0.5.0`，采用 C99 标准编写，实现 UNIUBI 自研关节电机 CAN FD 协议，包含头文件、控制实现、平台适配接口和示例。

“可移植”是指核心协议代码不依赖特定芯片或操作系统。接入不同平台时，由应用适配 CAN FD 发送回调，并将接收到的帧交给 SDK 解析；核心协议代码可复用。目标平台需提供兼容 C99 的编译器和 CAN FD 通信能力，完成驱动适配、编译及实机验证后使用。

支持两种接入方式：

- **CAN FD 主控直连电机**：实现发送回调，将收到的帧交给 SDK 解析；STM32 用户可使用随库提供的 FDCAN 适配接口。
- **电脑通过 USB 调试板连接电机**：使用串口适配接口，见 [USB 串口接入](docs/debug-board.md)。

建议先运行报文示例，再接入收发接口查询版本和状态，最后配置运动模式并使能电机。

## 功能

| 接口 | 功能 |
|---|---|
| `u6913_enable / disable` | 使能、失能 |
| `u6913_set_mode` | 速度、位置、扭矩、PD/MIT 四种模式 |
| `u6913_speed_control` | 目标速度 rad/s + 输出扭矩限幅 Nm |
| `u6913_position_control` | 目标位置 rad + 位置环输出速度限幅 rad/s |
| `u6913_torque_control` | 扭矩目标 Nm；D 轴字段固定零 |
| `u6913_build_pd` | 显式提供六轴位置/速度/Kp/Kd/前馈扭矩 |
| `u6913_set_zero / set_can_id` | 设置机械零位、CAN 节点 ID |
| `u6913_request_status / request_version` | 请求完整状态、版本 |
| `u6913_start_auto_calibration` | AF 一键自动标定，由电机固件执行流程 |
| `u6913_request_calibration_status` | E1标定完成标志查询 |
| `u6913_request_serial_number` | E8 SN及产品型号、生产日期、固件版本、MCU类型查询 |
| `u6913_request_standard_status` | 0xCB 标准状态，含使能/故障位和母线电压 |
| `u6913_parse_feedback` | 标准状态、完整状态、版本、身份信息和标定状态解析 |
| `u6913_board_send / feed` | 调试板 A8 请求封装、A0/AC 字节流接收及 CRC 检查 |

扭矩控制接口的目标单位为 **N·m**，D 轴电流字段固定为零。SDK 不提供对象字典参数读写、参数保存、恢复出厂、软件复位、单项标定、SN 写入或固件升级接口。

## 标定和 SN 查询

初始化电机句柄后，调用以下查询接口。每次查询都应等待并匹配对应设备的反馈，再发起下一次查询。

```c
/* 查询均按motor.joint_id寻址，发出请求后由应用接收并匹配反馈。 */
int rc = u6913_request_serial_number(&motor);          /* E8 */
/* 在上一事务完成后再发起下一事务： */
rc = u6913_request_calibration_status(&motor);         /* E1 */
```

E8解析结果为`feedback.value.identity`，含`serial_number`（16字节SN加一个保证为零的终止字节）、`serial_length`、产品类型、生产年月日、固件版本和MCU代码。SN长度0代表空值，不能作为有效身份。保留原始字节，不把未知型号或未配置日期伪装成有效数据。

E1解析结果为`feedback.value.calibration.flags`，保留16位完成标志。未完成位不能区分未标定、正在标定或失败，不能据此提供逐项四态信息。位含义见[协议说明](docs/protocol.md)。

```c
/* 只有台架满足标定所需的工装、运动行程和配置覆盖条件时才执行。 */
int rc = u6913_start_auto_calibration(&motor);         /* AF */
/* rc==U6913_OK仅表示已发送；AF普通状态应答也不表示标定完成。
 * 后续串行查询E1及CB/FE，按固件流程核对各项结果、故障和运行状态。 */
```

自动标定会驱动电机运动，并可能保存标定数据。仅在满足标定工装、完整运动行程及配置覆盖条件时显式调用；SDK 不会在初始化或查询时启动标定，也不会自动重试。普通状态应答只表示命令响应，不能用它或已有的完成标志判定本次标定成功。流程步骤和时间取决于目标固件。

SN 查询支持 ID1～6 的单节点或掩码寻址，使用 64 字节请求和 32 字节回复。反馈布局及标定标志见 [协议说明](docs/protocol.md)。

## 目录

```text
uniubi_motor_sdk/
├── include/uniubi6913.h              公共 API、数据类型和单位
├── include/uniubi6913_board.h        调试板主机侧串口 API
├── controllers/uniubi6913.c          协议构帧、发送封装、反馈解析
├── controllers/uniubi6913_board.c    A8/A0/AC 封装及字节流解析
├── ports/stm32/                      STM32 HAL FDCAN 适配参考
├── examples/host_demo.c              无硬件报文打印示例
├── examples/board_demo.c             无硬件串口报文打印示例
├── tests/                           单元测试与测试记录
├── docs/protocol.md                  报文格式、单位和反馈字段
├── docs/debug-board.md               USB 调试板串口接入方式
└── CMakeLists.txt
```

## 构建与验证

在本目录执行（CMake >= 3.16，C99 编译器）：

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/uniubi6913_demo
./build/uniubi6913_board_demo
```

无 CMake 时可以直接构建主机示例：

```sh
cc -std=c99 -Wall -Wextra -Iinclude controllers/uniubi6913.c examples/host_demo.c -o /tmp/uniubi6913_demo
/tmp/uniubi6913_demo
```

示例仅打印查询帧，并解析一条模拟版本反馈，不打开 CAN 设备。`cmake --install build --prefix <目录>` 可安装静态库和公共头文件。嵌入式工程可以直接加入 `controllers/uniubi6913.c` 和 `include/`；核心不依赖 HAL、Python、操作系统、堆分配或本仓库其他模块。需要 IEEE-754 32 位 `float`，不要启用 `-ffast-math`/`-ffinite-math-only`，否则可能破坏 NaN/Inf 校验。

## 接入发送与接收

```c
#include "uniubi6913.h"

/* 实现由应用提供：把整个 frame 复制到 CAN FD 驱动队列，成功返回0。 */
extern int platform_send_canfd(void *user, const u6913_frame_t *frame);

int query_motor(void *can_driver)
{
    u6913_motor_t motor;
    int rc = u6913_init(&motor, 1, platform_send_canfd, can_driver);
    if (rc != U6913_OK) return rc;
    return u6913_request_version(&motor);
}
```

所有 API 同步构帧并调用一次发送回调，不等待电机回复、不自动重试、不自动使能或切模式。`U6913_OK` 仅代表回调已接收报文。回调不能保留传入帧指针；异步驱动必须复制帧。应用负责发送节奏、接收队列、超时、总线错误和线程同步；在 RTOS 任务中调用，接收中断只入队，再由任务解析。

接收侧先筛选标准 CAN FD 数据帧，然后调用：

```c
u6913_feedback_t feedback;
int rc = u6913_parse_feedback(1, received_can_id, received_data,
                             received_byte_count, &feedback);
if (rc == U6913_OK && feedback.kind == U6913_FEEDBACK_VERSION) {
    /* feedback.value.version.major / minor / patch */
}
```

`received_byte_count` 是真实字节数，不是硬件 DLC 编码；反馈 ID 和载荷设备 ID 必须与目标一致。未知反馈返回 `U6913_EUNSUPPORTED`，不会冒充标准状态。协议没有通用事务序号，同一电机的重复查询应串行处理，超时后的旧回复需由应用隔离。构帧和解析失败时输出对象保持不变。

## 运动控制顺序

在已有接收/超时处理的应用状态机中按以下步骤调用，每一步确认对应反馈后才进入下一步：

1. 查询版本、标准状态和完整状态，核对固件、使能/故障位、母线电压与当前模式。
2. `u6913_disable(&motor)`，确认已失能。
3. `u6913_set_mode(&motor, U6913_MODE_SPEED)`，通过完整状态确认模式。
4. `u6913_speed_control(&motor, 0.0f, 1.0f)` 设置初始零速度及小扭矩限幅；确认设备接收。
5. `u6913_enable(&motor)`，确认使能状态。
6. 根据台架允许范围调用 `u6913_speed_control(&motor, target_rad_s, torque_limit_nm)`；周期由应用控制。
7. 正常结束先设置零目标，确认停止后失能；反馈超时或发送错误交给应用停机策略处理。

位置模式使用 `U6913_MODE_POSITION + u6913_position_control`；扭矩模式使用 `U6913_MODE_TORQUE + u6913_torque_control`。SDK 不在每一帧中重复切模式。失能态目标接受行为和状态位含义应在目标固件上确认。

设零和更改 ID 由调用方显式调用。更改 ID 后 SDK 不会自动改写 `motor.joint_id`；先从新 ID 查询确认，再重新初始化句柄。总线上的 ID 必须唯一，支持范围为 1～6。

## 多轴与 PD/MIT

普通命令的构帧接口提供 `mask`：bit0 对应 ID1，bit5 对应 ID6。例：

```c
u6913_frame_t frame;
int rc = u6913_build_simple(&frame, U6913_ALL_JOINTS, U6913_CMD_DISABLE);
if (rc == U6913_OK) rc = u6913_send_frame(&motor, &frame);
```

**`0xAE` PD/MIT 帧没有“忽略此槽”的含义。** 必须显式准备总线上全部电机的目标，并统一发送一帧；不要为不同电机分别调用单轴 PD，否则其他节点目标也会被覆盖。

```c
u6913_pd_target_t targets[U6913_JOINT_COUNT] = {{0}};
u6913_frame_t frame;
/* 为所有实际接入节点填入本周期目标。
 * 初始化为0的槽表示 p/v/kp/kd/ff均为0，不表示保持原目标或失能。 */
targets[0].position_rad = 0.0f;
targets[0].kp = 1.0f;
targets[0].kd = 0.1f;
int rc = u6913_build_pd(&frame, targets);
if (rc == U6913_OK) rc = u6913_send_frame(&motor, &frame);
```

上述数值仅说明 API，实际目标需由应用根据当前位置和负载生成。所有目标节点须先进入 PD 模式。无目标节点的槽也编码零目标：`00 80 00 80 00 00 00 00 00 80`，避免全零字节被解码为负端量程。

## STM32 FDCAN

加入 `ports/stm32/uniubi6913_stm32.c` 及其头文件搜索路径，按 MCU 选择 HAL 头文件（默认 `stm32g4xx_hal.h`，可定义 `U6913_STM32_HAL_HEADER` 覆盖）。使用静态或长期有效的端口/句柄：

```c
#include "uniubi6913_stm32.h"
static u6913_motor_t motor;
static u6913_stm32_port_t port;

int motor_init(FDCAN_HandleTypeDef *hfdcan)
{
    port.hfdcan = hfdcan;
    port.bit_rate_switch = FDCAN_BRS_ON;
    return u6913_init(&motor, 1, u6913_stm32_send, &port);
}
```

CubeMX/应用负责时钟、引脚、CAN FD 收发器、终端、位时序、过滤器、Message RAM、FIFO 和 `HAL_FDCAN_Start`。CAN FD 通信速率为仲裁段 **1 Mbps**、数据段 **5 Mbps**；须匹配实际固件与硬件，SDK 不硬编码寄存器时序。Tx/Rx 数据区均须能容纳 64 字节。

`HAL_FDCAN_GetRxMessage` 取出 `header` 和 64 字节接收缓冲后，用 `u6913_stm32_parse` 处理 DLC 和帧类型。端口支持本 SDK 已实现的 12/32/48 字节反馈；其他扩展反馈尚未支持。

主控需使用支持 CAN FD 的 MCU 或外部控制器，并支持 64 字节数据帧。接入后应使用所选 Cube HAL 版本编译，并在目标硬件验证收发。

## 单位与错误处理

运动接口的位置、速度、扭矩使用输出轴单位：rad、rad/s、N·m。完整状态的 `output_velocity_rad_s` 为输出轴速度，`mechanical_angle_rad` 为电机侧单圈机械角度。SDK 不额外乘减速比。普通状态的 `predicted_temperature_c` 为预测温度，完整状态的 `ntc_temperature_c` 为电机 NTC 测量温度；母线电流与相电流/Iq 不可混用。

| 返回值 | 含义与处理 |
|---|---|
| `U6913_OK` | 调用成功；发送接口仍需等待设备反馈 |
| `U6913_EINVAL` | 检查指针、ID、掩码、模式及长度参数 |
| `U6913_ERANGE` | 输入含 NaN/Inf 或超出协议编码范围，修正目标后再发送 |
| `U6913_EIO` | 发送回调失败，检查驱动或串口连接 |
| `U6913_EFRAME` | 接收长度或设备 ID 不匹配，丢弃该帧 |
| `U6913_EUNSUPPORTED` | 未支持的命令或反馈，按应用需要记录并继续接收 |
| `U6913_ECRC` | 调试板串口帧校验失败，检查通信并重新同步 |

SDK 对非法值返回错误，不自动将目标限制到边界。协议编码范围不等于电机允许运行范围；应用需根据电机规格、负载和机构行程设置位置、速度及扭矩限制，并处理通信超时和故障停机。完整状态不含标准状态的故障位和母线电压，运行时应同时查询标准状态。

## USB 调试板只读检查

可使用 `controllers/hardware_validation.py` 检查连接。脚本需要 Python 3、`pyserial` 和可用的 C 编译器；这些仅是检查脚本的依赖，C SDK 本身不依赖 Python。

在 SDK 根目录执行，将串口名称替换为实际设备：

```sh
python3 -m pip install pyserial
python3 controllers/hardware_validation.py --port /dev/ttyACM0 --report /tmp/motor-status.json --extended
```

脚本查询 ID1～6 的版本，对响应节点读取标准状态和完整状态；`--extended` 额外查询 SN 和标定完成标志。脚本不会使能电机、发送运动目标或启动标定。串口应由该脚本独占；结果及收发记录写入 `--report` 指定的 JSON 文件。
