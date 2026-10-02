# -*- coding: utf-8 -*-
"""给 CY_WATCH.uvprojx 接上 MPU6050 / MAX30102 两条服务链的组 / 源文件 / IncludePath.

**只增不改, 幂等**: 组已存在就跳过, IncludePath 已存在就不重复加, 可反复执行.
(不同于 add_lvgl_service_groups.py —— 那个是整体重建 <Groups>, 而它写完之后
 uvprojx 又被手工加过 fatfs/adkey/nordic 等组, 重跑会把那些组删掉.)

用法(在本脚本所在目录下):
    python add_sensor_service_groups.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))       # <仓库>/Script
REPO = os.path.dirname(HERE)                            # <仓库>
PROJ_DIR = os.path.join(REPO, "Project", "MDK_ARM")     # uvprojx 所在目录
UVPROJX = os.path.join(PROJ_DIR, "CY_WATCH.uvprojx")

# 两块 BSP 驱动加进既有的 "Bsp" 组(该组放着 ST7789T3/CST816T, 是"所有 BSP 驱动"之家)
BSP_GROUP = "Bsp"
BSP_FILES = [
    r"..\..\BSP\MPU6050\cywatch_bsp_mpu6050_driver.c",
    r"..\..\BSP\MAX30102\cywatch_bsp_max30102_driver.c",
]

# 服务组: 沿用 Service/<名> + Service/<名>/adapter 的既有分法
NEW_GROUPS = [
    ("Service/attitude", [
        r"..\..\APP\Service\attitude\cywatch_service_AttitudeCalculation.c",
    ]),
    ("Service/attitude/adapter", [
        r"..\..\APP\Service\attitude\adapter\cywatch_adapter_mpu6050.c",
    ]),
    ("Service/attitude/ServiceFunction", [
        r"..\..\APP\Service\attitude\ServiceFunction\AttitudeCalculation.c",
    ]),
    ("Service/heartrate", [
        r"..\..\APP\Service\heartrate\cywatch_service_HeartRate.c",
    ]),
    ("Service/heartrate/adapter", [
        r"..\..\APP\Service\heartrate\adapter\cywatch_adapter_max30102.c",
    ]),
    ("Service/heartrate/ServiceFunction", [
        r"..\..\APP\Service\heartrate\ServiceFunction\HeartRate.c",
        r"..\..\APP\Service\heartrate\ServiceFunction\SpO2.c",
    ]),
]

NEW_INCLUDES = [
    r"..\..\BSP\MPU6050",
    r"..\..\BSP\MAX30102",
    r"..\..\APP\Service\attitude",
    r"..\..\APP\Service\attitude\adapter",
    r"..\..\APP\Service\attitude\ServiceFunction",
    r"..\..\APP\Service\heartrate",
    r"..\..\APP\Service\heartrate\adapter",
    r"..\..\APP\Service\heartrate\ServiceFunction",
]


def file_xml(fp, indent="            "):
    fname = fp.replace("\\", "/").rsplit("/", 1)[-1]
    ftype = "2" if fname.endswith(".s") else "1"
    return "\n".join([
        indent + "<File>",
        indent + "  <FileName>%s</FileName>" % fname,
        indent + "  <FileType>%s</FileType>" % ftype,
        indent + "  <FilePath>%s</FilePath>" % fp,
        indent + "</File>",
    ])


def group_xml(name, files):
    out = ["        <Group>", "          <GroupName>%s</GroupName>" % name,
           "          <Files>"]
    for fp in files:
        out.append(file_xml(fp))
    out += ["          </Files>", "        </Group>"]
    return "\n".join(out)


def check_exist(pairs):
    missing = []
    for fp in pairs:
        absfp = os.path.normpath(os.path.join(PROJ_DIR, fp))
        if not os.path.isfile(absfp):
            missing.append(fp)
    if missing:
        sys.exit("以下文件不存在, 接线中止:\n  " + "\n  ".join(missing))


def main():
    with open(UVPROJX, "r", encoding="utf-8") as f:
        xml = f.read()

    check_exist(BSP_FILES + [fp for _, fs in NEW_GROUPS for fp in fs])

    added_groups, added_files = [], 0

    # ---- 1) Bsp 组补两个驱动文件 ----
    m = re.search(r"<Group>\s*<GroupName>%s</GroupName>(.*?)</Group>"
                  % re.escape(BSP_GROUP), xml, re.S)
    if not m:
        sys.exit("uvprojx 里找不到组: %s" % BSP_GROUP)
    body = m.group(1)
    for fp in BSP_FILES:
        if fp in body:
            continue
        body = body.replace("          </Files>",
                            file_xml(fp) + "\n          </Files>", 1)
        added_files += 1
    xml = xml[:m.start(1)] + body + xml[m.end(1):]

    # ---- 2) 追加新组(已存在则跳过) ----
    exist = set(re.findall(r"<GroupName>([^<]*)</GroupName>", xml))
    blob = []
    for name, files in NEW_GROUPS:
        if name in exist:
            continue
        blob.append(group_xml(name, files))
        added_groups.append(name)
    if blob:
        xml = xml.replace("      </Groups>",
                          "\n".join(blob) + "\n      </Groups>", 1)

    # ---- 3) 追加 IncludePath(Cads 段那份, 不是 TargetCommonOption/Aads) ----
    m = re.search(r"(<Cads>.*?<IncludePath>)([^<]*)(</IncludePath>)", xml, re.S)
    if not m:
        sys.exit("找不到 <Cads> 的 <IncludePath>")
    cur = m.group(2).split(";") if m.group(2) else []
    added_inc = [p for p in NEW_INCLUDES if p not in cur]
    new_paths = cur + added_inc
    # 用 lambda 而非字符串替换: 替换串里全是 \C 这种 Windows 路径, 交给 re 会报 bad escape
    xml = xml[:m.start(2)] + ";".join(new_paths) + xml[m.end(2):]

    with open(UVPROJX, "w", encoding="utf-8") as f:
        f.write(xml)

    print("Bsp 组新增文件: %d" % added_files)
    print("新增组: %s" % (", ".join(added_groups) if added_groups else "(无, 已存在)"))
    print("新增 IncludePath: %s" % (", ".join(added_inc) if added_inc else "(无, 已存在)"))


if __name__ == "__main__":
    main()
