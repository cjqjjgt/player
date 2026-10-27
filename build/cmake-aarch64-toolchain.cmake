# ============================================================
# aarch64 (Buildroot SDK) 交叉编译 CMake 工具链
# 供 faad2 / mp4v2 第三方库静态交叉编译复用
# ============================================================
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(TOOLCHAIN_DIR /home/dell/aarch64-buildroot-linux-gnu_sdk-buildroot)
set(BINDIRE ${TOOLCHAIN_DIR}/bin)
set(CMAKE_SYSROOT ${TOOLCHAIN_DIR}/aarch64-buildroot-linux-gnu/sysroot)

set(CMAKE_C_COMPILER   ${BINDIRE}/aarch64-buildroot-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${BINDIRE}/aarch64-buildroot-linux-gnu-g++)

# ar/ranlib 仅做"打包 .o"操作, 与目标架构无关, 直接用宿主系统工具最稳.
# (Buildroot 的 ar 是 toolchain-wrapper 脚本链接, 常因 wrapper 找不到 *.br_real 而失败)
set(CMAKE_AR          /usr/bin/ar)
set(CMAKE_RANLIB      /usr/bin/ranlib)
set(CMAKE_NM          /usr/bin/nm)
set(CMAKE_STRIP       ${BINDIRE}/aarch64-buildroot-linux-gnu-strip)

set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)