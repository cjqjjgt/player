/**
 * @file lv_conf.h
 * Configuration file for v9.6.0 - AArch64 Cross-Compilation
 */

#ifndef LV_CONF_H
#define LV_CONF_H

/* If you need to include anything here, do it inside the `__ASSEMBLY__` guard */
#if 0 && defined(__ASSEMBLY__)
#include "my_include.h"
#endif

/*============================================================================
 * MEMORY AND STANDARD LIBRARY
 *============================================================================*/

#define LV_USE_STDLIB_MALLOC LV_STDLIB_CLIB
#define LV_USE_STDLIB_STRING LV_STDLIB_CLIB
#define LV_USE_STDLIB_SPRINTF LV_STDLIB_CLIB

#define LV_STDINT_INCLUDE "stdint.h"
#define LV_STDDEF_INCLUDE "stddef.h"
#define LV_STDBOOL_INCLUDE "stdbool.h"
#define LV_INTTYPES_INCLUDE "inttypes.h"
#define LV_LIMITS_INCLUDE "limits.h"
#define LV_STDARG_INCLUDE "stdarg.h"

/*============================================================================
 * OPERATING SYSTEM (OS)
 *============================================================================*/

#define LV_USE_OS LV_OS_NONE

/*============================================================================
 * RENDERING CONFIGURATION
 *============================================================================*/

#define LV_COLOR_DEPTH 32
#define LV_COLOR_MIX_ROUND_OFS 0
#define LV_DEF_REFR_PERIOD 33
#define LV_DPI_DEF 130
#define LV_DRAW_BUF_STRIDE_ALIGN 1
#define LV_DRAW_BUF_ALIGN 4
#define LV_USE_MATRIX 0
#define LV_DRAW_LAYER_SIMPLE_BUF_SIZE 24576
#define LV_DRAW_LAYER_MAX_MEMORY 0
#define LV_USE_VECTOR_GRAPHIC 0
#define LV_USE_SNAPSHOT 0
#define LV_USE_THORVG 0

#define LV_USE_DRAW_SW 1

#if LV_USE_DRAW_SW
#define LV_DRAW_SW_SUPPORT_RGB565 1
#define LV_DRAW_SW_SUPPORT_RGB565_SWAPPED 1
#define LV_DRAW_SW_SUPPORT_RGB565A8 1
#define LV_DRAW_SW_SUPPORT_RGB888 1
#define LV_DRAW_SW_SUPPORT_XRGB8888 1
#define LV_DRAW_SW_SUPPORT_ARGB8888 1
#define LV_DRAW_SW_SUPPORT_ARGB8888_PREMULTIPLIED 1
#define LV_DRAW_SW_SUPPORT_L8 1
#define LV_DRAW_SW_SUPPORT_AL88 1
#define LV_DRAW_SW_SUPPORT_A8 1
#define LV_DRAW_SW_SUPPORT_I1 1

#if LV_DRAW_SW_SUPPORT_I1
#define LV_DRAW_SW_I1_LUM_THRESHOLD 127
#endif

#define LV_DRAW_SW_DRAW_UNIT_CNT 1
#define LV_DRAW_SW_COMPLEX 1
#define LV_GRADIENT_MAX_STOPS 2
#define LV_USE_DRAW_SW_COMPLEX_GRADIENTS 0

#if LV_DRAW_SW_COMPLEX
#define LV_DRAW_SW_SHADOW_CACHE_SIZE 0
#define LV_DRAW_SW_CIRCLE_CACHE_SIZE 4
#endif

#define LV_USE_DRAW_SW_ASM LV_DRAW_SW_ASM_NONE
#endif /*LV_USE_DRAW_SW*/

#define LV_USE_DRAW_VG_LITE 0
#define LV_USE_DRAW_DAVE2D 0
#define LV_USE_NEMA_GFX 0
#define LV_USE_DRAW_PXP 0

/*============================================================================
 * FEATURES
 *============================================================================*/

#define LV_USE_LOG 1
#if LV_USE_LOG
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#define LV_LOG_PRINTF 1
#define LV_USE_LOG_TRACE 0
#endif

#define LV_USE_ASSERT_NULL 0
#define LV_USE_ASSERT_MALLOC 0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ 0
#define LV_USE_ASSERT_STYLE 0

#define LV_USE_TICK 1
#if LV_USE_TICK
#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
#define LV_TICK_CUSTOM_INCLUDE "unistd.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR ((uint64_t)0)
#endif
#endif

#define LV_USE_ANIMATION 1
#if LV_USE_ANIMATION
#define LV_USE_ANIMATION_MONO 0
#define LV_BEZIER_VAL_SHIFT 10
#endif

#define LV_USE_INDEV 1
#if LV_USE_INDEV
#define LV_INDEV_DEF_SCROLL_THROW 50
#define LV_INDEV_DEF_SCROLL_LIMIT 10
#endif

#define LV_USE_GROUP 1
#define LV_USE_FLEX 1
#define LV_USE_GRID 1

/*============================================================================
 * WIDGETS
 *============================================================================*/

#define LV_WIDGETS_HAS_DEFAULT_VALUE 1

#define LV_USE_ANIMIMG 1
#define LV_USE_ARC 1
#define LV_USE_ARCLABEL 1
#define LV_USE_BAR 1
#define LV_USE_BARCODE 0
#define LV_USE_BUTTON 1
#define LV_USE_BUTTONMATRIX 1
#define LV_USE_CALENDAR 1
#if LV_USE_CALENDAR
#define LV_CALENDAR_WEEK_STARTS_MONDAY 0
#define LV_USE_CALENDAR_HEADER_ARROW 1
#define LV_USE_CALENDAR_HEADER_DROPDOWN 1
#define LV_USE_CALENDAR_CHINESE 0
#endif
#define LV_USE_CANVAS 1
#define LV_USE_CHART 1
#define LV_USE_CHECKBOX 1
#define LV_USE_DROPDOWN 1
#define LV_USE_FFMPEG 0
#define LV_USE_GIF 0
#define LV_USE_IMAGE 1
#define LV_USE_IMAGEBUTTON 1
#define LV_USE_IME_PINYIN 0
#define LV_USE_KEYBOARD 1
#define LV_USE_LABEL 1
#if LV_USE_LABEL
#define LV_LABEL_TEXT_SELECTION 1
#define LV_LABEL_LONG_TXT_HINT 1
#define LV_LABEL_WAIT_CHAR_COUNT 3
#endif
#define LV_USE_LED 1
#define LV_USE_LINE 1
#define LV_USE_LIST 1
#define LV_USE_LOTTIE 0
#define LV_USE_MENU 1
#define LV_USE_MSGBOX 1
#define LV_USE_QRCODE 0
#define LV_USE_ROLLER 1
#define LV_USE_SCALE 1
#define LV_USE_SLIDER 1
#define LV_USE_SPAN 1
#if LV_USE_SPAN
#define LV_SPAN_SNIPPET_STACK_SIZE 64
#endif
#define LV_USE_SPINBOX 1
#define LV_USE_SPINNER 1
#define LV_USE_SWITCH 1
#define LV_USE_TABLE 1
#define LV_USE_TABVIEW 1
#define LV_USE_TEXTAREA 1
#if LV_USE_TEXTAREA
#define LV_TEXTAREA_DEF_PWD_SHOW_TIME 1500
#endif
#define LV_USE_TILEVIEW 1
#define LV_USE_WIN 1

/*============================================================================
 * THEMES
 *============================================================================*/

#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT
#define LV_THEME_DEFAULT_DARK_MODE 0
#define LV_THEME_DEFAULT_GROW 1
#endif

#define LV_USE_THEME_SIMPLE 1
#define LV_USE_THEME_MONO 1

/*============================================================================
 * LAYOUTS
 *============================================================================*/

#define LV_USE_FLEX 1
#define LV_USE_GRID 1

/*============================================================================
 * LIBRARIES
 *============================================================================*/

#define LV_USE_FS_STDIO 0
#define LV_USE_FS_POSIX 0
#define LV_USE_FS_WIN32 0
#define LV_USE_FS_LITTLEFS 0
#define LV_USE_FS_ARDUINO_SD 0
#define LV_USE_FS_ARDUINO_LITTLEFS 0
#define LV_USE_FS_ARDUINO_FFAT 0
#define LV_USE_FS_MEMFS 0

#define LV_USE_LODEPNG 0
#define LV_USE_LIBPNG 0
#define LV_USE_BMP 0
#define LV_USE_RLE 0
#define LV_USE_TJPGD 0
#define LV_USE_LIBJPEG_TURBO 0
#define LV_USE_PNG 0
#define LV_USE_SJPG 0

/*============================================================================
 * OTHERS
 *============================================================================*/

#define LV_USE_GRIDNAV 0
#define LV_USE_FRAGMENT 0
#define LV_USE_IMGFONT 0
#define LV_USE_MONKEY 0
#define LV_USE_OBSERVER 0
#define LV_USE_FILE_EXPLORER 0

/*============================================================================
 * FONTS
 *============================================================================*/

#define LV_FONT_MONTSERRAT_8 0
#define LV_FONT_MONTSERRAT_10 0
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 0
#define LV_FONT_MONTSERRAT_20 0
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 0
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 0
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0
#define LV_FONT_MONTSERRAT_12_SUBPX 0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0
#define LV_FONT_SOURCE_HAN_SANS_SC_14_CJK 0
#define LV_FONT_SOURCE_HAN_SANS_SC_16_CJK 0
#define LV_FONT_UNSCII_8 0
#define LV_FONT_UNSCII_16 0

#define LV_FONT_USE_CUSTOM_INCLUDE 0
#define LV_USE_CUSTOM_FONT_DEFAULT 0

#if !LV_USE_CUSTOM_FONT_DEFAULT
#define LV_FONT_DEFAULT LV_FONT_DEFAULT_MONTSERRAT_14
#endif

#define LV_FONT_FMT_TXT_LARGE 0
#define LV_USE_FONT_COMPRESSED 0
#define LV_USE_FONT_PLACEHOLDER 1
#define LV_USE_FONT_MANAGER 0

#define LV_USE_FREETYPE 0
#define LV_USE_TINY_TTF 0

/*============================================================================
 * COMPILE OPTIONS
 *============================================================================*/

#define LV_USE_PRIVATE_API 0
#define LV_USE_OBJ_ID 0
#define LV_USE_OBJ_ID_B32 0
#define LV_OBJ_ID_AUTO_ASSIGN 0
#define LV_USE_OBJ_NAME 0
#define LV_USE_OBJ_NAME_STR 0
#define LV_USE_PERF_MONITOR 0
#define LV_USE_MEM_MONITOR 0

/*============================================================================
 * DRIVERS
 *============================================================================*/

#define LV_USE_LINUX_DRM 1
#define LV_USE_LINUX_FBDEV 0
#define LV_USE_FT81X 0
#define LV_USE_LOVYAN_GFX 0
#define LV_USE_GENERIC_MIPI 0
#define LV_USE_ST7735 0
#define LV_USE_ST7789 0
#define LV_USE_ST7796 0
#define LV_USE_ILI9341 0
#define LV_USE_NV3007 0
#define LV_USE_NXP_ELCDIF 0
#define LV_USE_RENESAS_GLCDC 0
#define LV_USE_ST_LTDC 0
#define LV_USE_TFT_ESPI 0
#define LV_USE_EVDEV 0
#define LV_USE_LIBINPUT 0
#define LV_USE_NUTTX 0
#define LV_USE_OPENGLES 0
#define LV_USE_GLFW 0
#define LV_USE_QNX 0
#define LV_USE_SDL 0
#define LV_USE_EGL 0
#define LV_USE_WAYLAND 0

/*============================================================================
 * 3D
 *============================================================================*/

#define LV_USE_DRAW_HAS_3D_SUPPORT 0
#define LV_USE_3DTEXTURE 0
#define LV_USE_GLTF 0

#endif /*LV_CONF_H*/