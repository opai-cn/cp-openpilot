#!/bin/bash
# 编译 libsafety.so 用于 safety replay 测试

set -e

echo "=========================================="
echo "编译 libsafety.so"
echo "=========================================="
echo ""

# 进入 opendbc_repo 目录
cd "$(dirname "$0")/opendbc_repo"

# 检查 scons 是否安装
if ! command -v scons &> /dev/null; then
    echo "错误: scons 未安装"
    echo "请安装 scons: pip install scons"
    exit 1
fi

# 检查 setup.sh 是否存在并加载环境
if [ -f "setup.sh" ]; then
    echo "加载环境配置..."
    source setup.sh
fi

# 编译 libsafety.so（不需要 coverage，只需要基本编译）
echo "开始编译 libsafety.so..."
echo "   工作目录: $(pwd)"
echo "   使用 scons 编译..."

# 从 opendbc_repo 根目录运行 scons
# SConstruct 会调用 SConscript，SConscript 会包含 tests/libsafety/SConscript
# 构建目标：opendbc/safety/tests/libsafety/libsafety.so
scons -j$(nproc) -D opendbc/safety/tests/libsafety/libsafety.so

# 检查是否编译成功
LIBSAFETY_PATH="opendbc/safety/tests/libsafety/libsafety.so"
if [ -f "$LIBSAFETY_PATH" ]; then
    echo ""
    echo "✅ libsafety.so 编译成功！"
    echo "   位置: $(pwd)/$LIBSAFETY_PATH"
    ls -lh "$LIBSAFETY_PATH"
else
    echo ""
    echo "❌ 编译失败: libsafety.so 未找到"
    echo "   提示: 检查编译错误信息"
    echo "   可能需要安装编译工具: gcc, scons"
    exit 1
fi

echo ""
echo "=========================================="
echo "编译完成！"
echo "=========================================="

