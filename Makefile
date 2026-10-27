# ============================================================
# RK3572 本地视频播放器 - Makefile
# 飞凌 OK3572 aarch64 交叉编译
# ============================================================

# 交叉编译工具链 (Buildroot SDK)
TOOLCHAIN_DIR = /home/dell/aarch64-buildroot-linux-gnu_sdk-buildroot
TOOLCHAIN_BIN = $(TOOLCHAIN_DIR)/bin
SYSROOT       = $(TOOLCHAIN_DIR)/aarch64-buildroot-linux-gnu/sysroot
CROSS_COMPILE = aarch64-buildroot-linux-gnu-
CC            = $(TOOLCHAIN_BIN)/$(CROSS_COMPILE)gcc
STRIP         = $(TOOLCHAIN_DIR)/aarch64-buildroot-linux-gnu/bin/strip
export PATH   := $(TOOLCHAIN_BIN):$(TOOLCHAIN_DIR)/aarch64-buildroot-linux-gnu/bin:$(PATH)

# C++ 编译器: buildroot 的 g++ 是坏的 (toolchain-wrapper), 用系统 aarch64-linux-gnu-g++
# (GCC 11, 与 buildroot libstdc++ 13 向后兼容, 参考项目已验证)
CXX_SYS = aarch64-linux-gnu-g++

# 项目目录
PROJECT_ROOT  = $(shell pwd)
SRC_DIR       = $(PROJECT_ROOT)/src
INC_DIR       = $(PROJECT_ROOT)/include
LIB_DIR       = $(PROJECT_ROOT)/lib
THIRDLIB_DIR  = $(PROJECT_ROOT)/thirdlib

# live555 (RTSP 拉流, 替代 ffmpeg 的 RTSP demuxer)
# 动态库方案: lib/live555 下 4 个 .so (aarch64, 由 config.aarch64-shared 编译)
LIVE555_DIR    = /home/dell/live555-mirror-main
LIVE555_LIBDIR = $(LIB_DIR)/live555
LIVE555_LIBS  = -L$(LIVE555_LIBDIR) -lliveMedia -lgroupsock \
                -lBasicUsageEnvironment -lUsageEnvironment

# 输出
TARGET        = rk3572_player
BUILD_DIR     = $(PROJECT_ROOT)/build/obj

# 源文件 (递归收集 src/ 下所有 .c 和 .cpp)
SRCS = $(shell find $(SRC_DIR) -name '*.c')
CXXSRCS = $(shell find $(SRC_DIR) -name '*.cpp')
OBJS = $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SRCS))
CXXOBJS = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(CXXSRCS))

# libmov + libflv (MP4 封装/解封装, 替代 mp4v2)
LIBMOV_SRC = $(wildcard $(THIRDLIB_DIR)/mov/libmov/source/*.c)
LIBFLV_SRC = $(wildcard $(THIRDLIB_DIR)/mov/libflv/source/*.c)
THIRDLIB_SRCS = $(LIBMOV_SRC) $(LIBFLV_SRC)
THIRDLIB_OBJS = $(patsubst $(THIRDLIB_DIR)/%.c,$(BUILD_DIR)/thirdlib/%.o,$(THIRDLIB_SRCS))

# 包含路径
INCLUDES = \
    -I$(INC_DIR) \
    -I$(INC_DIR)/playback \
    -I$(INC_DIR)/network \
    -I$(INC_DIR)/video \
    -I$(INC_DIR)/gui \
    -I$(THIRDLIB_DIR)/mov/libmov/include \
    -I$(THIRDLIB_DIR)/mov/libflv/include \
    -I$(THIRDLIB_DIR)/mov/libmov/source \
    -I$(THIRDLIB_DIR)/faad2 \
    -I$(THIRDLIB_DIR)/mpp \
    -I$(THIRDLIB_DIR)/mpp/rockchip \
    -I$(THIRDLIB_DIR)/rga \
    -I$(THIRDLIB_DIR)/lvgl \
    -I$(THIRDLIB_DIR)/lvgl/include \
    -I$(THIRDLIB_DIR)/lvgl/src \
    -I$(LIVE555_DIR)/liveMedia/include \
    -I$(LIVE555_DIR)/groupsock/include \
    -I$(LIVE555_DIR)/UsageEnvironment/include \
    -I$(LIVE555_DIR)/BasicUsageEnvironment/include

# C++ 编译选项 (live555 模块)
CXXFLAGS_SYS = -O2 -g -std=c++11 -fexceptions -D_GNU_SOURCE -DLOG_LEVEL=3 -DNO_OPENSSL=1 \
    $(INCLUDES)

# 库路径 (每库一子目录)
LDFLAGS = \
    -L$(LIB_DIR)/faad2 \
    -L$(LIB_DIR)/mpp \
    -L$(LIB_DIR)/rga \
    -L$(LIB_DIR)/lvgl

# 链接库
LIBS = \
    -lfaad \
    -lrockchip_mpp \
    -lrga \
    -llvgl \
    -ldrm \
    -lasound \
    -lstdc++ \
    -lz \
    -lpthread \
    -lm \
    -ldl \
    -lrt

# 编译选项
CFLAGS = \
    -Wall \
    -Wextra \
    -O2 \
    -g \
    -D_GNU_SOURCE \
    -DLOG_LEVEL=3 \
    --sysroot=$(SYSROOT) \
    $(INCLUDES)

# 链接选项
LDFLAGS += \
    -rdynamic \
    -Wl,-rpath,'$$ORIGIN/../lib' \
    -Wl,--copy-dt-needed-entries

# AddressSanitizer 调试
ifdef SAN
CFLAGS += -fsanitize=address -fno-omit-frame-pointer
LDFLAGS += -fsanitize=address -L$(TOOLCHAIN_DIR)/aarch64-buildroot-linux-gnu/lib64 -lasan
endif

.PHONY: all clean install package

all: $(TARGET)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX_SYS) $(CXXFLAGS_SYS) -c $< -o $@

$(BUILD_DIR)/thirdlib/%.o: $(THIRDLIB_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJS) $(CXXOBJS) $(THIRDLIB_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LIBS) $(LIVE555_LIBS)
	@echo "=========================================="
	@echo "  Build complete: $(TARGET)"
	@echo "=========================================="

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# 安装到开发板
DEVICE_IP ?= 192.168.1.100
INSTALL_DIR  = /userdata/rk3572_player
install: $(TARGET)
	@echo "Installing to $(DEVICE_IP):$(INSTALL_DIR)..."
	ssh root@$(DEVICE_IP) "mkdir -p $(INSTALL_DIR)/bin $(INSTALL_DIR)/lib"
	scp $(TARGET) root@$(DEVICE_IP):$(INSTALL_DIR)/bin/
	scp $(LIB_DIR)/faad2/libfaad.so* root@$(DEVICE_IP):$(INSTALL_DIR)/lib/
	scp $(LIB_DIR)/mpp/*.so* root@$(DEVICE_IP):$(INSTALL_DIR)/lib/
	scp $(LIB_DIR)/rga/*.so* root@$(DEVICE_IP):$(INSTALL_DIR)/lib/
	scp $(LIB_DIR)/lvgl/*.so* root@$(DEVICE_IP):$(INSTALL_DIR)/lib/
	scp $(LIB_DIR)/live555/*.so root@$(DEVICE_IP):$(INSTALL_DIR)/lib/
	ssh root@$(DEVICE_IP) "echo '#!/bin/sh' > $(INSTALL_DIR)/run.sh && \
		echo 'export LD_LIBRARY_PATH=\$$(dirname \$$0)/lib:\$$LD_LIBRARY_PATH' >> $(INSTALL_DIR)/run.sh && \
		echo 'exec \$$(dirname \$$0)/bin/rk3572_player \$$@' >> $(INSTALL_DIR)/run.sh && \
		chmod +x $(INSTALL_DIR)/run.sh"

# 打包发布 (本地生成 .tar.gz, 拷贝到开发板解压即用)
PACKAGE_NAME = rk3572_player_release
package: $(TARGET)
	@echo "Packaging $(PACKAGE_NAME)..."
	@rm -rf $(PROJECT_ROOT)/$(PACKAGE_NAME) $(PROJECT_ROOT)/$(PACKAGE_NAME).tar.gz
	@mkdir -p $(PROJECT_ROOT)/$(PACKAGE_NAME)/bin $(PROJECT_ROOT)/$(PACKAGE_NAME)/lib
	@$(STRIP) -s $(TARGET) -o $(PROJECT_ROOT)/$(PACKAGE_NAME)/bin/$(TARGET)
	@# 拷贝动态库时用 cp -a 保留符号链接, 避免丢失 libxxx.so / libxxx.so.N 软链导致"动态库不完整"
	@cp -a $(LIB_DIR)/faad2/libfaad.so*  $(PROJECT_ROOT)/$(PACKAGE_NAME)/lib/
	@cp -a $(LIB_DIR)/mpp/librockchip_mpp.so* $(PROJECT_ROOT)/$(PACKAGE_NAME)/lib/
	@cp -a $(LIB_DIR)/rga/librga.so* $(PROJECT_ROOT)/$(PACKAGE_NAME)/lib/
	@cp -a $(LIB_DIR)/lvgl/liblvgl.so* $(LIB_DIR)/lvgl/libdrm.so* $(PROJECT_ROOT)/$(PACKAGE_NAME)/lib/
	@# ★ 剔除备份库 (.bak): cp 通配符会带上旧备份, 软链循环会把
	@#   库链到 .bak 旧库, 导致符号缺失。
	@rm -f $(PROJECT_ROOT)/$(PACKAGE_NAME)/lib/*.bak
	@# live555 动态库 (RTSP 拉流)
	@cp -a $(LIB_DIR)/live555/libliveMedia.so $(LIB_DIR)/live555/libgroupsock.so \
	       $(LIB_DIR)/live555/libUsageEnvironment.so $(LIB_DIR)/live555/libBasicUsageEnvironment.so \
	       $(PROJECT_ROOT)/$(PACKAGE_NAME)/lib/
	@echo '#!/bin/sh' > $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo 'BASEDIR=$$(cd "$$(dirname "$$0")" && pwd)' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo 'export LD_LIBRARY_PATH=$$BASEDIR/lib:$$LD_LIBRARY_PATH' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '# 清理上次残留进程 (异常退出会占用 NAU8822 声卡 hw:1,0, 导致新实例回退 HDMI 无声)' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo 'killall -9 rk3572_player 2>/dev/null; sleep 1' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo 'cd "$$BASEDIR/lib"' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '# 自动为带版本号的动态库重建软链 (兼容 busybox tar 解包丢软链)' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo 'for so in lib*.so.*; do' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '  [ -e "$$so" ] || continue' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '  case "$$so" in' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '    *.bak) continue ;;' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '    librockchip_mpp.so.*)' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '      ln -sf "$$so" librockchip_mpp.so.1' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '      ln -sf librockchip_mpp.so.1     librockchip_mpp.so' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '      ;;' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '    *)' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '      name=$${so%%.so.*}.so' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '      ver=$${so#*.so.}; ver=$${ver%%.*}' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '      ln -sf "$$so" "$$name.$$ver"' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '      ln -sf "$$name.$$ver" "$$name"' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '      ;;' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo '  esac' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo 'done' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo 'cd "$$BASEDIR"' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@echo 'exec "$$BASEDIR/bin/$(TARGET)" "$$@"' >> $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@chmod +x $(PROJECT_ROOT)/$(PACKAGE_NAME)/run.sh
	@tar -czf $(PROJECT_ROOT)/$(PACKAGE_NAME).tar.gz -C $(PROJECT_ROOT) $(PACKAGE_NAME)
	@rm -rf $(PROJECT_ROOT)/$(PACKAGE_NAME)
	@echo "=========================================="
	@echo "  Package: $(PACKAGE_NAME).tar.gz"
	@ls -lh $(PROJECT_ROOT)/$(PACKAGE_NAME).tar.gz
	@echo "  Usage on board:"
	@echo "    tar -xzf $(PACKAGE_NAME).tar.gz"
	@echo "    cd $(PACKAGE_NAME) && ./run.sh"
	@echo "=========================================="
