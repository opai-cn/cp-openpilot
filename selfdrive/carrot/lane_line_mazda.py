#!/usr/bin/env python3
"""
马自达车道线类型检测服务

在 onroad 模式下自动运行，通过 CV 检测车道线类型（实线/虚线）
并将结果写入 Params 系统，供 carstate.py 读取
"""
import time
from msgq.visionipc.visionipc_pyx import VisionIpcClient, VisionStreamType
import cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params
from openpilot.selfdrive.modeld.lane_line_detector import LaneLineDetector
from openpilot.common.realtime import set_realtime_priority


def main():
    """车道线检测服务主循环"""
    # 设置实时优先级
    try:
        set_realtime_priority(2)
    except Exception:
        pass

    cloudlog.info("Lane line detection service for Mazda starting...")

    detector = LaneLineDetector()
    sm = messaging.SubMaster(['modelV2', 'liveCalibration', 'deviceState', 'roadCameraState'])
    vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)

    # 等待 vision stream 连接
    cloudlog.info("Waiting for vision stream...")
    while not vipc_client.connect(False):
        time.sleep(0.2)
    cloudlog.info("Vision stream connected")

    # 等待相机初始化
    while True:
        sm.update(0)
        if detector.init_camera(sm, vipc_client):
            break
        time.sleep(0.1)

    cloudlog.info(f"Camera initialized: {detector.w}x{detector.h}, starting detection loop...")

    frame_count = 0
    while True:
        sm.update(0)
        yuv_buf = vipc_client.recv()

        if yuv_buf is None:
            time.sleep(0.1)
            continue

        # 执行检测
        result = detector.update(sm, yuv_buf, vipc_client)

        # 发布结果到 Params（供 carstate.py 读取）
        detector.publish_result(None, result)

        frame_count += 1

        # 每100帧输出一次调试信息
        if frame_count % 100 == 0:
            left_type_str = ['虚线', '实线', '不确定'][result['left'] if result['left'] >= 0 else 2]
            right_type_str = ['虚线', '实线', '不确定'][result['right'] if result['right'] >= 0 else 2]
            cloudlog.info(f"Lane detection - Left: {left_type_str}({result['left']}), "
                         f"Right: {right_type_str}({result['right']}), "
                         f"RelStd: L={result['left_rel_std']:.4f}, R={result['right_rel_std']:.4f}")

        time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        cloudlog.info("Lane line detection service interrupted")
    except Exception as e:
        cloudlog.error(f"Lane line detection service failed: {e}")
        import traceback
        traceback.print_exc()

