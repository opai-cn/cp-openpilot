#!/usr/bin/env python3
"""
测试 MEB 平台日志，检查巡航速度和挡位信息
"""
import sys
import os

# 添加路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'opendbc_repo'))

try:
    from tools.lib.logreader import LogReader
except ImportError:
    print("错误: 无法导入 LogReader，请确保在项目根目录运行此脚本")
    sys.exit(1)

def test_meb_log(log_path):
    print(f"正在分析日志: {log_path}")
    print("=" * 60)

    lr = LogReader(log_path)

    # 检查车辆参数
    try:
        cp = lr.first('carParams')
        print(f"\n车辆信息:")
        print(f"  车型名称: {cp.carName}")
        print(f"  车辆指纹: {cp.carFingerprint}")
        print(f"  安全模式: {cp.safetyConfigs[-1].safetyModel.raw}")
        print(f"  网络位置: {cp.networkLocation}")
    except Exception as e:
        print(f"  无法读取车辆参数: {e}")

    # 检查关键消息
    print(f"\n检查关键 CAN 消息:")

    # MEB 平台关键消息
    meb_messages = {
        'MEB_ACC_01': {'address': 0x300, 'bus': None, 'signal': 'ACC_Wunschgeschw_02'},
        'Gateway_73': {'address': 0x3DC, 'bus': None, 'signal': 'GE_Fahrstufe'},
        'Motor_51': {'address': 0x10B, 'bus': None, 'signal': 'TSK_Status'},
        'ESC_51': {'address': 0x0FC, 'bus': None, 'signal': 'VL_Radgeschw'},
    }

    found_messages = {}
    message_counts = {}

    for msg in lr:
        if msg.which() == 'can':
            for can_msg in msg.can:
                addr = can_msg.address
                bus = can_msg.src

                # 检查 MEB 关键消息
                for msg_name, msg_info in meb_messages.items():
                    if addr == msg_info['address']:
                        if msg_name not in found_messages:
                            found_messages[msg_name] = {
                                'address': addr,
                                'bus': bus,
                                'data_len': len(can_msg.dat),
                                'first_seen': msg.logMonoTime
                            }
                        if msg_name not in message_counts:
                            message_counts[msg_name] = 0
                        message_counts[msg_name] += 1

    # 打印结果
    for msg_name, msg_info in meb_messages.items():
        if msg_name in found_messages:
            info = found_messages[msg_name]
            count = message_counts[msg_name]
            print(f"  ✓ {msg_name} (0x{msg_info['address']:03X})")
            print(f"     总线: {info['bus']}, 数据长度: {info['data_len']}, 消息数: {count}")
        else:
            print(f"  ✗ {msg_name} (0x{msg_info['address']:03X}) - 未找到")

    # 检查 CarState
    print(f"\n检查 CarState 消息:")
    carstate_count = 0
    cruise_speed_samples = []
    gear_samples = []

    for msg in lr:
        if msg.which() == 'carState':
            carstate_count += 1
            cs = msg.carState

            # 采样巡航速度
            if hasattr(cs, 'cruiseState') and hasattr(cs.cruiseState, 'speed'):
                speed = cs.cruiseState.speed
                if speed > 0:
                    cruise_speed_samples.append(speed * 3.6)  # 转换为 km/h

            # 采样挡位
            if hasattr(cs, 'gearShifter'):
                gear_samples.append(cs.gearShifter)

    print(f"  CarState 消息数: {carstate_count}")
    if cruise_speed_samples:
        print(f"  巡航速度样本: {len(cruise_speed_samples)} 个")
        print(f"    范围: {min(cruise_speed_samples):.1f} - {max(cruise_speed_samples):.1f} km/h")
        print(f"    最后值: {cruise_speed_samples[-1]:.1f} km/h")
    else:
        print(f"  ⚠ 未找到巡航速度数据")

    if gear_samples:
        print(f"  挡位样本: {len(gear_samples)} 个")
        unique_gears = set(gear_samples)
        print(f"    唯一挡位: {unique_gears}")
        print(f"    最后值: {gear_samples[-1]}")
    else:
        print(f"  ⚠ 未找到挡位数据")

    print("\n" + "=" * 60)
    print("分析完成！")

    # 建议
    if not cruise_speed_samples or not gear_samples:
        print("\n建议:")
        if 'MEB_ACC_01' not in found_messages:
            print("  - MEB_ACC_01 消息未找到，检查日志是否来自 MEB 平台")
        if 'Gateway_73' not in found_messages:
            print("  - Gateway_73 消息未找到，检查 CAN 总线配置")
        print("  - 检查 carstate.py 中的消息解析逻辑")
        print("  - 确认日志是在启用巡航控制时录制的")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 test_meb_log.py <日志文件路径>")
        print("示例: python3 test_meb_log.py /home/jeff/realdata/00000001--989b07c1e0--20/rlog.zst")
        sys.exit(1)

    log_path = sys.argv[1]
    if not os.path.exists(log_path):
        print(f"错误: 日志文件不存在: {log_path}")
        sys.exit(1)

    test_meb_log(log_path)

