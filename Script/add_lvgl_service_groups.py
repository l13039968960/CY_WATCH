# -*- coding: utf-8 -*-
"""给 CY_WATCH.uvprojx 接上 LVGL 服务链路所需的全部组 / 源文件 / IncludePath.

幂等: 每次跑都整体重建 <Groups> 段与 <IncludePath>, 可重复执行.

LVGL 本体那 419 个 src/*.c 与 6 个 assets 的文件清单不从零写死, 而是从参考工程
Dirver_Test/MDK-ARM/AHT21_TEST.uvprojx 里读出来再重映射路径 —— 避免手抄出错,
也保证与参考工程编译的文件集合完全一致(该工程已实测构建通过).

用法(在本脚本所在目录下):
    python add_lvgl_service_groups.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))       # <仓库>/Script
REPO = os.path.dirname(HERE)                            # <仓库>
PROJ_DIR = os.path.join(REPO, "Project", "MDK_ARM")     # uvprojx 所在目录
UVPROJX = os.path.join(PROJ_DIR, "CY_WATCH.uvprojx")
REF_UVPROJX = os.path.abspath(os.path.join(
    REPO, "..", "Dirver_Test", "MDK-ARM", "AHT21_TEST.uvprojx"))

# 参考工程 -> CY_WATCH 的路径重映射(uvprojx 里 FilePath 均为相对 .uvprojx 的路径)
# 注意: 这里不能用 raw string —— r"...\\" 是"两个真反斜杠", 不是转义后的一个
PATH_REMAP = [
    ("..\\LVGL\\lvgl\\",   "..\\..\\MIddleWares\\LVGL\\lvgl\\"),
    ("..\\LVGL\\assets\\", "..\\..\\MIddleWares\\LVGL\\assets\\"),
]


def read_ref_group(group_name):
    """从参考工程读出某个 <Group> 的全部 <FilePath>."""
    with open(REF_UVPROJX, "r", encoding="utf-8") as f:
        xml = f.read()

    m = re.search(
        r"<Group>\s*<GroupName>%s</GroupName>(.*?)</Group>" % re.escape(group_name),
        xml, re.S)
    if not m:
        sys.exit("参考工程里找不到组: %s" % group_name)

    paths = re.findall(r"<FilePath>([^<]+)</FilePath>", m.group(1))
    if not paths:
        sys.exit("参考工程的组 %s 里没有文件" % group_name)

    out = []
    for p in paths:
        for old, new in PATH_REMAP:
            p = p.replace(old, new)
        out.append(p)
    return out


def remap_ref_head():
    """参考工程里 LVGL/src 与 assets 的文件路径重映射结果."""
    return {
        "LVGL/src": read_ref_group("LVGL/src"),
        "LVGL/assets": read_ref_group("LVGL/assets"),
    }


# ---------------------------------------------------------------- 组定义
# 每项: (组名, [相对 Project/MDK_ARM 的 FilePath, ...])
def build_groups():
    ref = remap_ref_head()

    groups = [
        ("Core", [
            r"..\..\Core\main.c",
            r"..\..\Core\stm32f4xx_it.c",
            r"..\..\Core\stm32f4xx_hal_msp.c",
            r"..\..\Core\system_stm32f4xx.c",
        ]),
        ("System", [
            r"..\..\Core\system\delay\delay.c",
            r"..\..\Core\system\uart\uart.c",
            r"..\..\Core\system\Rtc\cywatch_rtc.c",
        ]),
        ("Core/GPIO", [
            r"..\..\Core\GPIO\gpio_hal.c",
        ]),
        ("Core/EXTI", [
            r"..\..\Core\EXTI\exti_hal.c",
        ]),
        ("Core/SPI", [
            r"..\..\Core\SPI\spi_hal.c",
        ]),
        ("Core/IIC", [
            r"..\..\Core\IIC\iic_hal.c",
        ]),
        ("Startup", [
            r"..\..\Driver\CMSIS\Device\ST\STM32F4xx\Source\Templates\arm\startup_stm32f411xe.s",
        ]),
        ("HAL_Driver", [
            r"..\..\Driver\STM32F4xx_HAL_Driver\Src\%s" % n for n in (
                "stm32f4xx_hal.c",
                "stm32f4xx_hal_cortex.c",
                "stm32f4xx_hal_rcc.c",
                "stm32f4xx_hal_rcc_ex.c",
                "stm32f4xx_hal_gpio.c",
                "stm32f4xx_hal_exti.c",
                "stm32f4xx_hal_dma.c",
                "stm32f4xx_hal_dma_ex.c",
                "stm32f4xx_hal_pwr.c",
                "stm32f4xx_hal_pwr_ex.c",
                "stm32f4xx_hal_flash.c",
                "stm32f4xx_hal_flash_ex.c",
                "stm32f4xx_hal_flash_ramfunc.c",
                "stm32f4xx_hal_uart.c",
                "stm32f4xx_hal_usart.c",
                "stm32f4xx_hal_spi.c",
                "stm32f4xx_hal_rtc.c",
                "stm32f4xx_hal_rtc_ex.c",
            )
        ]),
        ("OS/FreeRTOS", [
            r"..\..\MIddleWares\FreeRTOS\tasks.c",
            r"..\..\MIddleWares\FreeRTOS\queue.c",
            r"..\..\MIddleWares\FreeRTOS\list.c",
            r"..\..\MIddleWares\FreeRTOS\timers.c",
            r"..\..\MIddleWares\FreeRTOS\event_groups.c",
            r"..\..\MIddleWares\FreeRTOS\stream_buffer.c",
            r"..\..\MIddleWares\FreeRTOS\portable\GCC\ARM_CM4F\port.c",
            r"..\..\MIddleWares\FreeRTOS\portable\MemMang\heap_4.c",
            r"..\..\MIddleWares\FreeRTOS\CMSIS_RTOS_V2\cmsis_os2.c",
        ]),
        ("LVGL/src", ref["LVGL/src"]),
        ("LVGL/port", [
            r"..\..\MIddleWares\LVGL\port\lv_port_disp.c",
            r"..\..\MIddleWares\LVGL\port\lv_port_indev.c",
        ]),
        ("LVGL/assets", ref["LVGL/assets"]),
        ("Bsp", [
            r"..\..\BSP\ST7789T3\cywatch_bsp_st7789t3_driver.c",
            r"..\..\BSP\CST816T\cywatch_bsp_cst816t_driver.c",
        ]),
        ("APP/EasyAPP", [
            r"..\..\MIddleWares\EasyAPP\src\easyapp_core.c",
            r"..\..\MIddleWares\EasyAPP\src\easyapp_event.c",
            r"..\..\MIddleWares\EasyAPP\src\easyapp_page.c",
            r"..\..\MIddleWares\EasyAPP\port\easyapp_port.c",
        ]),
        ("App/DataModel", [
            r"..\..\APP\APP\DataModel\cywatch_app_datamodel.c",
        ]),
        ("Service/lvgl", [
            r"..\..\APP\Service\lvgl\cywatch_service_Lvgl.c",
        ]),
        ("Service/lvgl/adapter", [
            r"..\..\APP\Service\lvgl\adapter\cywatch_adapter_disp.c",
            r"..\..\APP\Service\lvgl\adapter\cywatch_adapter_indev.c",
        ]),
        ("Service/lvgl/PageMgr", [
            r"..\..\APP\Service\lvgl\Lvgl_PageManger\PageMem.c",
        ]),
        ("Service/lvgl/ui", [
            r"..\..\APP\Service\lvgl\ui\lv_watch_ui.c",
            r"..\..\APP\Service\lvgl\ui\lv_watch_page_home.c",
            r"..\..\APP\Service\lvgl\ui\lv_watch_page_menu.c",
            r"..\..\APP\Service\lvgl\ui\lv_watch_page_heart.c",
            r"..\..\APP\Service\lvgl\ui\lv_watch_page_spo2.c",
            r"..\..\APP\Service\lvgl\ui\lv_watch_page_ota.c",
            r"..\..\APP\Service\lvgl\ui\lv_watch_selftest.c",
        ]),
    ]

    # 自检: 每个文件都得真实存在, 否则 Keil 会静默跳过
    missing = []
    for name, files in groups:
        for fp in files:
            absfp = os.path.normpath(os.path.join(PROJ_DIR, fp))
            if not os.path.isfile(absfp):
                missing.append("%s -> %s" % (name, fp))
    if missing:
        sys.exit("以下文件不存在, 接线中止:\n  " + "\n  ".join(missing))

    return groups


INCLUDE_PATHS = [
    r"..\..\Core",
    r"..\..\Core\Inc",
    r"..\..\Core\system",
    r"..\..\Core\system\delay",
    r"..\..\Core\system\uart",
    r"..\..\Core\system\Rtc",
    r"..\..\Core\GPIO",
    r"..\..\Core\EXTI",
    r"..\..\Core\SPI",
    r"..\..\Core\IIC",
    r"..\..\Driver\STM32F4xx_HAL_Driver\Inc",
    r"..\..\Driver\STM32F4xx_HAL_Driver\Inc\Legacy",
    r"..\..\Driver\CMSIS\Device\ST\STM32F4xx\Include",
    r"..\..\Driver\CMSIS\Include",
    r"..\..\BSP",
    r"..\..\BSP\ST7789T3",
    r"..\..\BSP\CST816T",
    r"..\..\MIddleWares",
    r"..\..\MIddleWares\LVGL",
    r"..\..\MIddleWares\LVGL\lvgl",
    r"..\..\MIddleWares\LVGL\port",
    r"..\..\MIddleWares\LVGL\assets\fonts",
    r"..\..\MIddleWares\FreeRTOS",
    r"..\..\MIddleWares\FreeRTOS\include",
    r"..\..\MIddleWares\FreeRTOS\portable\GCC\ARM_CM4F",
    r"..\..\MIddleWares\FreeRTOS\CMSIS_RTOS_V2",
    # EasyAPP 根目录必须在内: easyapp_core.c/easyapp_page.c 写的是
    # #include "./inc/xxx.h" —— 引号包含先按"源文件所在目录"找(src/inc/, 不存在),
    # 再按 -I 列表找, 于是要命中 <EasyAPP根>/inc/xxx.h。参考工程同样有这条
    r"..\..\MIddleWares\EasyAPP",
    r"..\..\MIddleWares\EasyAPP\inc",
    r"..\..\MIddleWares\EasyAPP\src",
    r"..\..\MIddleWares\EasyAPP\port",
    r"..\..\APP",
    r"..\..\APP\APP\DataModel",
    r"..\..\APP\Service\lvgl",
    r"..\..\APP\Service\lvgl\adapter",
    r"..\..\APP\Service\lvgl\ui",
    r"..\..\APP\Service\lvgl\Lvgl_PageManger",
]


def group_xml(name, files):
    out = ["        <Group>", "          <GroupName>%s</GroupName>" % name]
    if files:
        out.append("          <Files>")
        for fp in files:
            fname = fp.replace("\\", "/").rsplit("/", 1)[-1]
            ftype = "2" if fname.endswith(".s") else "1"
            out += [
                "            <File>",
                "              <FileName>%s</FileName>" % fname,
                "              <FileType>%s</FileType>" % ftype,
                "              <FilePath>%s</FilePath>" % fp,
                "            </File>",
            ]
        out.append("          </Files>")
    out.append("        </Group>")
    return "\n".join(out)


def main():
    with open(UVPROJX, "r", encoding="utf-8") as f:
        xml = f.read()

    groups = build_groups()
    body = "\n".join(group_xml(n, fs) for n, fs in groups)
    new_groups = "      <Groups>\n%s\n      </Groups>" % body

    xml, n = re.subn(r"      <Groups>.*?</Groups>", lambda m: new_groups,
                     xml, count=1, flags=re.S)
    if n != 1:
        sys.exit("替换 <Groups> 失败(找到 %d 处)" % n)

    # uvprojx 里有三处 <IncludePath>, 必须按父元素区分, 不能按出现顺序取第一个:
    #   1) <TargetCommonOption> 里那份是"环境 include"(Keil 界面没用到) —— 必须保持空,
    #      误填进去既不报错也不生效, 只会让人以为配好了;
    #   2) <Cads> 里那份才是 C 编译器的 —— 这才是要写的;
    #   3) <Aads> 里那份是汇编器的 —— 保持空。
    # 用 lambda 而非字符串替换: 替换串里全是 `\C` 这种 Windows 路径, 直接交给 re
    # 会被当成非法转义而报 "bad escape \C"
    inc_paths = ";".join(INCLUDE_PATHS)

    xml, n = re.subn(r"(<TargetCommonOption>.*?<IncludePath>)[^<]*(</IncludePath>)",
                     lambda m: m.group(1) + m.group(2), xml, count=1, flags=re.S)
    if n != 1:
        sys.exit("清空 <TargetCommonOption> 的 <IncludePath> 失败(找到 %d 处)" % n)

    xml, n = re.subn(r"(<Cads>.*?<IncludePath>)[^<]*(</IncludePath>)",
                     lambda m: m.group(1) + inc_paths + m.group(2),
                     xml, count=1, flags=re.S)
    if n != 1:
        sys.exit("替换 <Cads> 的 <IncludePath> 失败(找到 %d 处)" % n)

    with open(UVPROJX, "w", encoding="utf-8") as f:
        f.write(xml)

    total = sum(len(fs) for _, fs in groups)
    print("已接线: %d 个组, %d 个文件" % (len(groups), total))
    for n, fs in groups:
        print("  %-22s %d" % (n, len(fs)))


if __name__ == "__main__":
    main()
