#!/bin/bash
# 快速测试 MEB 平台日志

LOG_PATH="/home/jeff/realdata/00000001--989b07c1e0--20/rlog.zst"

echo "=========================================="
echo "MEB 平台日志快速测试"
echo "=========================================="
echo ""

# 1. 测试日志分析
echo "1. 运行日志分析..."
python3 test_meb_log.py "$LOG_PATH"
echo ""

# 2. 检查并编译 libsafety.so
echo "2. 检查 libsafety.so..."
LIBSAFETY_SO="opendbc_repo/opendbc/safety/tests/libsafety/libsafety.so"
if [ ! -f "$LIBSAFETY_SO" ]; then
    echo "   libsafety.so 不存在，尝试编译..."
    if command -v scons &> /dev/null; then
        cd opendbc_repo
        if [ -f "setup.sh" ]; then
            source setup.sh
        fi
        echo "   从 opendbc_repo 根目录编译..."
        scons -j$(nproc) -D opendbc/safety/tests/libsafety/libsafety.so
        cd ..
        if [ -f "$LIBSAFETY_SO" ]; then
            echo "   ✅ libsafety.so 编译成功"
        else
            echo "   ⚠️  编译失败，跳过 Safety Replay 测试"
            echo "   提示: 需要安装 scons 和编译工具: pip install scons"
            echo "   或者运行: ./build_libsafety.sh"
        fi
    else
        echo "   ⚠️  scons 未安装，跳过 Safety Replay 测试"
        echo "   提示: 安装 scons: pip install scons"
        echo "   或者运行: ./build_libsafety.sh"
    fi
else
    echo "   ✅ libsafety.so 已存在"
fi

# 3. Safety Replay 测试（如果 libsafety.so 存在）
if [ -f "$LIBSAFETY_SO" ]; then
    echo ""
    echo "3. 运行 Safety Replay 测试..."
    echo "   这可能需要一些时间..."
    python3 opendbc_repo/opendbc/safety/tests/safety_replay/replay_drive.py \
      "$LOG_PATH" \
      --mode 34 2>&1 | tail -20
else
    echo ""
    echo "3. 跳过 Safety Replay 测试（libsafety.so 不可用）"
fi
echo ""

echo "=========================================="
echo "测试完成！"
echo ""
echo "下一步："
echo "1. 如果日志分析显示消息未找到，检查日志是否来自 MEB 平台"
echo "2. 如果 Safety Replay 有错误，检查安全代码实现"
echo "3. 使用完整 replay 测试 UI 显示："
echo "   tools/replay/replay 00000001--989b07c1e0--20 --data_dir=\"/home/jeff/realdata\""
echo "   cd selfdrive/ui && ./ui"
echo "=========================================="

