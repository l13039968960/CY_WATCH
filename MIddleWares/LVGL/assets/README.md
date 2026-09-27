# LVGL/assets —— 手表UI(产品UI)用的字库与图标

本目录是**产品UI的资源目录**。2026-09-17 之前这些资源放在一个 lvgl_ui_designer
导出的资源目录里(整块导出,每个字库 86~125KB),现已全部换成下面的子集 ——
那个旧目录连同它的生成脚本都已从工程中删除。

目录:

| 路径 | 内容 |
|---|---|
| `fonts/` | 3 个子集字库,由 `MDK-ARM/gen_watch_fonts.py` 生成 |
| `image/` | 3 个 18x18 图标(home 三张卡片 + menu 心率按钮) |

## fonts/ —— 生成出来的,不要手改

三个字库都是 `lv_font_conv` 生成的**子集**,只含页面实际用到的字符:

| 字库 | 谁在用 | 字符集 | 大小 |
|---|---|---|---|
| `lv_font_montserrat_regular_16` | home 的 `"08-28"` 日期、三张卡片数值 | ASCII + `°` + `•` | ~41 KB |
| `lv_font_montserrat_regular_20` | home 的 `"00:00:00"` 时钟、heart 的心率数值 | ASCII + `°` + `•` | ~54 KB |
| `lv_font_alibaba_puhuiti_14` | menu 的六个中文标签、heart 的 `"心率"` 标题 | ASCII + 12 个汉字 | ~50 KB |

源字体是 Montserrat Regular(OFL)与 Alibaba PuHuiTi(免费商用)——
选这两个也有授权上的理由: **Windows 自带的宋体/雅黑不允许嵌进设备固件**。

> ### ⚠ 改菜单上的中文文案 → 必须重新生成字库
>
> 汉字子集是**静态**的。页面里写出子集之外的汉字,那个字**直接不显示**——不报错,
> 编译和链接都能过,只是字没了。改动步骤:
>
> 1. 把新汉字补进 `MDK-ARM/gen_watch_fonts.py` 里的 `CJK_SYMBOLS`
> 2. `python MDK-ARM/gen_watch_fonts.py`
> 3. 重新编译
>
> 同理,`line_height` 被脚本**钉回**了原度量(见脚本头部注释):不钉的话三页文字
> 会整体上移 1px。改动 `gen_watch_fonts.py` 的钉值前先读那段注释。

源 ttf 只作输入、**不放进工程**(在 `E:/Downloads/.../lvgl_ui_designer_2.0.5/`,
见脚本顶部的 `DEFAULT_MONTSERRAT` / `DEFAULT_PUHUITI`,也可用命令行参数覆盖)。

## image/ —— 逐字节搬过来的

`icon_{steps,heart,flame}_18x18_RGB565A8_NONE.c`,LVGL 图片格式,无外部依赖。
menu 的心率按钮用它是因为 **LVGL 9.3 没有 `LV_SYMBOL_HEART`**。

页面对这三个图标的引用方式是 `extern const lv_image_dsc_t ...`,声明在
`LVGL/port/lv_watch_page.h`——**改文件名要连那里一起改**。
