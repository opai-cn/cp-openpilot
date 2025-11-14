#!/bin/bash
# 验证 MEB 平台代码的脚本

echo "=========================================="
echo "大众 ID.4 MEB 平台代码验证脚本"
echo "=========================================="
echo ""

# 检查安全模式定义
echo "1. 检查安全模式定义..."
if grep -q "SAFETY_VOLKSWAGEN_MEB.*34" opendbc_repo/opendbc/safety/safety.h; then
    echo "   ✓ SAFETY_VOLKSWAGEN_MEB = 34 已定义"
else
    echo "   ✗ 未找到 SAFETY_VOLKSWAGEN_MEB 定义"
fi

# 检查安全 hooks 注册
echo ""
echo "2. 检查安全 hooks 注册..."
if grep -q "volkswagen_meb_hooks" opendbc_repo/opendbc/safety/safety.h; then
    echo "   ✓ volkswagen_meb_hooks 已注册"
else
    echo "   ✗ 未找到 volkswagen_meb_hooks 注册"
fi

# 检查车型配置
echo ""
echo "3. 检查车型配置..."
if grep -q "VOLKSWAGEN_ID4_MK1\|ID\.4" opendbc_repo/opendbc/car/volkswagen/values.py; then
    echo "   ✓ ID.4 车型配置已找到"
else
    echo "   ✗ 未找到 ID.4 车型配置"
fi

# 检查 DBC 文件
echo ""
echo "4. 检查 DBC 文件..."
if [ -f "opendbc_repo/opendbc/dbc/vw_meb.dbc" ]; then
    echo "   ✓ vw_meb.dbc 文件存在"
    # 检查关键消息
    if grep -q "Motor_51\|Motor_54\|ESC_51" opendbc_repo/opendbc/dbc/vw_meb.dbc; then
        echo "   ✓ 关键 CAN 消息已定义"
    else
        echo "   ⚠ 未找到部分关键消息"
    fi
else
    echo "   ✗ vw_meb.dbc 文件不存在"
fi

# 检查编译产物
echo ""
echo "5. 检查编译产物..."
if [ -f "panda/board/obj/safety.o" ] || [ -f "panda/board/obj/safety_panda.o" ]; then
    echo "   ✓ 安全模块已编译"
else
    echo "   ⚠ 未找到编译产物，可能需要先编译"
fi

echo ""
echo "=========================================="
echo "验证完成！"
echo ""
echo "下一步："
echo "1. 如果有日志文件，运行 safety_replay 测试："
echo "   python3 opendbc_repo/opendbc/safety/tests/safety_replay/replay_drive.py <日志文件> --mode 34"
echo ""
echo "2. 或者使用完整 replay 工具："
echo "   tools/replay/replay <路由名称>"
echo "   cd selfdrive/ui && ./ui"
echo "=========================================="

