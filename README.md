# CY_WATCH

STM32F411CEU6 最小可编程模板（裸机 / HAL / Keil MDK-ARM）。

## 状态

已可编译链接，实测产出镜像 **Code 9478 B + RO 66 B + RW 16 B + ZI 1112 B**（含 printf + UART；
未加串口时为 Code 4254 B + RO 34 B + RW 12 B + ZI 1640 B）。
工程文件：[Project/MDK_ARM/CY_WATCH.uvprojx](Project/MDK_ARM/CY_WATCH.uvprojx)

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `Core/main.c` | 入口，`HAL_Init` → `SystemClock_Config` → `delay_init` → 主循环 |
| `Core/Inc/main.h` | 公共头（引入 HAL、`Error_Handler` 声明） |
| `Core/Inc/stm32f4xx_hal_conf.h` | HAL 裁剪配置，当前只开 8 个模块 |
| `Core/Inc/stm32f4xx_it.h` | 中断服务函数声明 |
| `Core/stm32f4xx_it.c` | 异常/SysTick 中断，`SysTick_Handler` 内喂 HAL 1ms 时基 |
| `Core/stm32f4xx_hal_msp.c` | 全局 MSP 初始化 |
| `Core/system_stm32f4xx.c` | CMSIS 系统初始化（`SystemInit`） |
| `Core/system/delay/` | 基于 DWT 的 `delay_us` / `delay_ms` |
| `Core/system/uart/` | USART1 初始化 + printf 重定向（PA9/PA10，115200） |
| `Driver/` | CMSIS + STM32F4xx HAL（原始库，未改动） |
| `Project/MDK_ARM/` | Keil 工程 |

## 时钟配置

`SystemClock_Config()` 位于 [Core/main.c](Core/main.c)，为 **HSI 16MHz → PLL → 100MHz**
（`PLLM=8 / PLLN=100 / PLLP=2 / PLLQ=4`，`FLASH_LATENCY_3`，APB1 = HCLK/2，APB2 = HCLK）。
不依赖板上晶振，改时钟只动这一个函数。

⚠️ `delay_init()` 已移到 `SystemClock_Config()` **之后**调用。它会按当前主频换算 DWT
计数（`SystemCoreClock / 1000000`），若在时钟配置前调用会读到复位默认的 16MHz，
实际 100MHz 下延时偏快 6.25 倍。

## 编译与烧录

Keil uVision 打开 `Project/MDK_ARM/CY_WATCH.uvprojx` 直接 Build。
编译器为 **Arm Compiler 6（armclang，实测 V6.19）**，启动文件用 CMSIS 的
`startup_stm32f411xe.s`（legacy armasm 语法，靠工程里的 `ClangAsOpt=1` 汇编，
会有一条 A1950W 弃用告警，属正常）。

已配好：`USE_HAL_DRIVER, STM32F411xE`；包含路径指向 `Core`、`Core/Inc`、
HAL 的 `Inc` 与 `Inc/Legacy`、CMSIS 的 `Device/.../Include` 与 `Include`。

## 警告配置（0 Error / 0 Warning）

原工程是 C90 语言标准（`v6Lang=1`），而 HAL 大量使用 C99 特性，导致
`-Wc99-extensions`（枚举尾逗号，18 个文件几乎每个都报）和 `-Wcomment`（`//` 注释）
刷屏。现按 Dirver_Test 那套已调好的配置改了 Cads：

| 项 | 原值 | 现值 | 作用 |
| --- | --- | --- | --- |
| `v6Lang` | `1`（c90） | `4`（gnu99） | 消除 c99-extensions / comment |
| `uGnu` | `1` | `0` | 与 AC6 的 `v6Lang` 保持一致 |
| `MiscControls` | 空 | 一组 `-Wno-*` + `-fno-short-enums` | 压掉 HAL 源码的风格类警告 |

`-Wno-*` 清单与 Dirver_Test 完全一致（`padded` / `unused-parameter` /
`bad-function-cast` / `covered-switch-default` / `switch-enum` / `extra-semi-stmt` /
`missing-variable-declarations` / `implicit-int-conversion` / `missing-noreturn` 等），
保留了 `unused-variable`、`conditional-uninitialized` 这类有真实信号的警告。
`-fno-short-enums` 是 ABI 选项，与 Dirver_Test 保持一致。

验证：用 `-Weverything` 关掉 `reserved-macro-identifier` 等刷屏项（比 MDK 档位更严）
跑全部 18 个文件，仅剩 8 条 `-Wunused-macros`（HAL 内部宏），而该警告原本就不在
MDK 的档位里 —— 即 MDK 实际构建应为 0 Warning。

## ⚠️ 改工程配置后要让 Keil 重新加载

直接用 Keil 打开工程构建时，uVision 会把工程文件重写一遍。**在 Keil 已经打开该工程
的情况下从外部改了 `.uvprojx`，必须在 Keil 里关掉工程再重新打开**（或 `Project →
Reload`），否则 Keil 内存里的旧设置会在保存时覆盖掉改动。

## 加一个外设（两步）

模板故意只编了核心 HAL，保持最小。要用某个外设：

1. 在 [Core/Inc/stm32f4xx_hal_conf.h](Core/Inc/stm32f4xx_hal_conf.h) 里取消对应
   `#define HAL_xxx_MODULE_ENABLED` 的注释（例如 `HAL_I2C_MODULE_ENABLED`）。
2. 在 Keil 工程的 `HAL_Driver` 分组里添加对应的
   `Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_xxx.c`
   （I2C 记得连带 `stm32f4xx_hal_i2c_ex.c`，TIM 连带 `_tim_ex.c`）。

外设的 `HAL_xxx_MspInit` 放哪都行，但**别重复定义**：本工程 UART 的时钟/GPIO
内联在 `uart.c` 的 `UART_Init()` 里，其余外设留在 `Core/stm32f4xx_hal_msp.c`。

当前已启用的 HAL 模块：`HAL / DMA / FLASH / GPIO / EXTI / PWR / RCC / CORTEX / UART`。

## 串口与 printf

`Core/system/uart/` 提供 USART1 初始化与 printf 重定向：

| 项 | 值 |
| --- | --- |
| 外设 | USART1 |
| 引脚 | PA9 = TX，PA10 = RX |
| 参数 | 115200 baud，8 数据位，1 停止位，无校验，无流控 |
| 复用 | `GPIO_AF7_USART1` |

`main()` 的 `USER CODE BEGIN 2` 里调用 `UART_Init()`，之后 `printf` 即输出到 PA9。
主循环里有一句心跳：`printf("CY_WATCH alive: %lu ms\r\n", HAL_GetTick())`。

重定向靠 MicroLIB 的 `fputc` 钩子，工程已开 `<useUlib>1</useUlib>`（MicroLIB）——
**这是 printf 能工作的前提**，若关掉 MicroLIB，`fputc` 不会被调用，printf 静默无输出。
`g_uart1_ready` 在初始化完成后置 1，就绪前的 `printf` 丢弃字符（不会因未初始化句柄而卡住）。

注意时钟使能与 GPIO 配置内联在 `UART_Init()` 里（**没有**用 `HAL_UART_MspInit`
这个弱函数钩子），改引脚时去 `uart.c` 改。`HAL_UART_MspDeInit` 未实现，
若用不到 `HAL_UART_DeInit()` 则无影响。printf 为阻塞发送，115200 下
每字节约 87 µs，主循环里频繁打印会明显占用 CPU。



## 备注

- **VS Code IntelliSense 会误报**：`main.c` 里可能出现 `RCC`、`RCC_CFGR_SW_PLL` 等
  "未定义标识符"。那是 C/C++ 扩展的 `includePath` 没配（默认解析到了别处的
  `stm32f4xx.h`），**不影响 Keil 编译**。补一个 `.vscode/c_cpp_properties.json`
  指向 `Core/Inc` 和 `Driver/CMSIS/Device/ST/STM32F4xx/Include` 即可消除。
- 备份文件：`Core/stm32f4xx_it.c.bak_aht21_refs`（清理前的原文件，含 AHT21_TEST 的
  MAX30102 / CST816T / EXTI / FreeRTOS 残留引用）、
  `Project/MDK_ARM/CY_WATCH.uvprojx.bak_empty`（改造前的空工程）。
  确认无误后可删。
- 启动文件栈大小 1KB（`Stack_Size EQU 0x00000400`）。上 LVGL 或深递归前
  记得调大（Dirver_Test 那份改到了 4KB）。
