# UNIUBI6913 关节电机 C SDK

UNIUBI6913 关节电机对外接入库，版本 `0.5.0`，采用 C99 标准编写，实现 UNIUBI 自研关节电机 CAN FD 协议，包含头文件、控制实现、平台适配接口和示例。

“可移植”是指核心协议代码不依赖特定芯片或操作系统。接入不同平台时，由应用适配 CAN FD 发送回调，并将接收到的帧交给 SDK 解析；核心协议代码可复用。目标平台需提供兼容 C99 的编译器和 CAN FD 通信能力，完成驱动适配、编译及实机验证后使用。

已结合用户提供的 `UNIUBI6913_关节电机使用说明书_V0.5.md` 和 `motorTools_gd32c113` 调试板源码核对。电机命令编码基线为 `motor_canfd_test/new_board_protocol` 提交 `b7675beabca6f4181768e08833125e436b482926`；手册明确的物理字段定义用于修正接口。手册文件名为 V0.5，但正文标题/版本记录仍为 V0.3，且限幅取整等条目与上位机实现存在差异，详见 [协议差异](docs/protocol.md)。0.3.0按用户要求移除F9读参数、F8写参数、F6保存、F7恢复出厂及C6软件复位，0.5.0移除D0逐项状态查询后，保留自动标定、E1完成标志查询及SN查询，当前支持15类电机命令。此前0.2.0的17类指令实测保留为 [历史实机报告](docs/hardware-validation-2026-10-10/REPORT.md)，不作为当前接口清单。

支持两种接法：**客户 CAN FD 主控直连电机**，或 **电脑 USB 串口 → GD32C113 调试板 → CAN FD → 电机**。使用你的调试板时见 [USB 串口接入](docs/debug-board.md)，无需 STM32 HAL。

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

本版提供AF一键自动标定及结果查询，不提供单项标定启动、SN写入、OTA、私有厂测和GUI。固件历史名称“Current”对应命令 `0xF1`，第一个字段是 **Nm 而非 A**，因此对外接口使用 `torque_control`；第二字段按手册为 Id 电流，本版固定零。

## 标定和SN接口

支持`AF / E1 / E8`，当前SDK共15类命令。0.5.0已移除D0接口、命令枚举、解析及其四态类型/字段；旧调用方需要更新并重新编译。此前移除的F9、F8、F6、F7、C6仍不支持。这三个接口完成离线验证；2026-10-10已实机验证E1/E8查询，AF尚未执行，等待完整行程标定授权。

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

AF会由固件驱动电机并可能保存标定数据；历史测试的±2.9°运动许可不覆盖自动标定。SDK不在初始化或查询时启动标定、不自动重试AF，也不以历史成功位判定本次标定完成；流程步骤和时间取决于目标固件。手册的“电机标定”在本版映射为一键自动标定AF，未额外开放单项工厂校准命令。

SN使用64字节六槽请求，支持ID1～6单节点/掩码寻址；区别于旧Python工具的12字节广播。本地电机固件已确认有E8级联槽处理路径，32字节回复按设备ID校验。详细来源及差异见协议文档。

## 0.3.0 接口变更

0.3.0保留的12类命令为：`AE / CB / E9 / EF / F0 / F1 / F4 / F5 / FB / FC / FD / FE`。移除了五类命令的枚举、构帧和发送函数，以及参数反馈类型和解析分支；`u6913_build_simple`、`u6913_send_frame`和反馈解析对已移除命令返回`U6913_EUNSUPPORTED`。板级串口及STM32端口仍是通用传输适配，不负责筛选电机功能。

这是不兼容的接口更新，使用方需要删除旧接口调用并重新编译；依赖结构体布局或反馈枚举数值的绑定也需同步更新。历史测试JSON保留原始枚举值，不可当作当前ABI直接加载。

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
├── tests/                           单元测试、HAL 替身、Python 协议对照
├── docs/protocol.md                  协议范围和来源差异
├── docs/debug-board.md               你的 GD32C113 调试板接入方式
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

仓库开发者可额外运行协议对照（发行包单独使用时不需要子仓）：

```sh
python3 tests/compare_reference.py --protocol-root ../core/motor_canfd_test
```

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

CubeMX/应用负责时钟、引脚、CAN FD 收发器、终端、位时序、过滤器、Message RAM、FIFO 和 `HAL_FDCAN_Start`。协议文档给出的参考速率是仲裁段 **1 Mbps**、数据段 **5 Mbps**；须匹配实际固件与硬件，SDK 不硬编码寄存器时序。Tx/Rx 数据区均须能容纳 64 字节。

`HAL_FDCAN_GetRxMessage` 取出 `header` 和 64 字节接收缓冲后，用 `u6913_stm32_parse` 处理 DLC 和帧类型。端口支持本 SDK 已实现的 12/32/48 字节反馈；其他扩展反馈尚未支持。

主控需使用支持 CAN FD 的 MCU 或外部控制器，并支持 64 字节数据帧。STM32 端口已做模拟 HAL 接口测试；仍需在所选 Cube HAL 版本编译并实机验证。

## 单位与验证边界

运动 API 的位置、速度、扭矩使用输出轴单位。按产品说明书，FE 的 `output_velocity_rad_s` 也是输出轴速度，`mechanical_angle_rad` 才是电机侧单圈机械角度，SDK 不额外乘减速比。普通状态的 `predicted_temperature_c` 是预测温度，FE 的 `ntc_temperature_c` 是电机 NTC 测量温度。

用户提供的说明书列明：21:1、额定48 V（30～60 V）、额定输出18 Nm、峰值60 Nm/3 s、额定转速15.7 rad/s、最高22 rad/s、额定母线电流6 A。母线电流不能与相电流/Iq混用。SDK不将这些额定值或峰值持续时间当作自动保护功能；编码范围 ±100 Nm、±65 rad/s 也不代表实际允许运行范围。

0.2.0 修正了初版三个反馈成员名称（`motor_velocity_rad_s` → `output_velocity_rad_s`、完整状态 `temperature_c` → `ntc_temperature_c`、普通状态 `temperature_c` → `predicted_temperature_c`），版本结构增加 `encoder_type`。已有使用方需更新字段访问并重新编译。

本 SDK 拒绝非有限值和超协议范围输入，不做静默饱和。应用仍需施加经确认的产品和台架运行限值。当前验证包括主机 C 编译、离线报文/解析测试、模拟 HAL 测试、与 Python 实现对照、真实GD32C113协议源码的主机互操作测试，以及0.2.0历史版本的单电机17类命令实测。0.5.0在迁移后的新目录完成14类指令实机复测（AF除外），E8读出SN、E1返回0x03D7，最大记录偏移约0.033°，详见[0.5.0复测报告](docs/hardware-retest-v0.5.0-2026-10-10/REPORT.md)。四模式使能态测试使用低限矩和小目标，记录最大偏移约0.07°，证明指令收发与反馈响应，不代表位置/速度精度或输出扭矩标定通过。STM32实板、多节点联动及满载性能不在本次验证范围内。

实机入口为 `controllers/hardware_validation.py`，需要Python与pyserial，仅负责验证，不是C SDK的运行依赖。默认查询版本、标准状态和完整状态；添加`--extended`可查询E8身份和E1标定完成标志。脚本不提供AF启动入口。依赖已移除参数接口的运动及配置测试阶段已删除。

0.5.0验证结果：4项CTest（核心协议、新增身份/标定、串口封装、STM32 mock）、2项Python绑定/只读脚本测试通过；4700组请求载荷与既有Python参考一致。E8采用手册与电机固件确认的级联格式，独立覆盖63种mask，未与旧广播格式混同比较。

0.5.0复测结束：ID1、位置模式、失能、CB状态字0；保留新零位，以及低输出测试设置（速度环限矩请求0.2 N·m、位置限速0.04 rad/s、PD Kp=3/Kd=0.15）。当前SDK无参数读写接口，本轮没有声称原控制参数已恢复。
