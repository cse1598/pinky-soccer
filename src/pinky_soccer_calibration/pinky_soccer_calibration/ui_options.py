"""Command-line-only selection of the local webcam rehearsal source."""

import argparse
import math


def device_value(value):
    """Accept a local video index or device path, never a remote video URL."""
    if value.isdecimal():
        return int(value)
    if value.startswith('/dev/'):
        return value
    raise argparse.ArgumentTypeError('웹캠 번호(예: 0) 또는 /dev/video 장치 경로를 입력하세요.')


def parse_ui_options(argv):
    parser = argparse.ArgumentParser(description='Pinky calibration UI / local webcam rehearsal')
    parser.add_argument('-webcam', '--webcam', nargs='?', const=0, default=None, type=device_value,
                        help='노트북 웹캠 전용 리허설. 생략한 장치 번호는 0입니다.')
    parser.add_argument('--webcam-width', type=int, default=640)
    parser.add_argument('--webcam-height', type=int, default=480)
    parser.add_argument('--webcam-fps', type=float, default=15.)
    parser.add_argument('--webcam-hfov-deg', type=float, default=None,
                        help='미보정 거리 추정을 위한 명시적 HFOV 가정. 생략하면 내부 보정 전에는 픽셀만 표시.')
    options, ros_args = parser.parse_known_args(argv)
    if not 32 <= options.webcam_width <= 7680 or not 32 <= options.webcam_height <= 4320:
        parser.error('웹캠 요청 해상도를 확인하세요.')
    if not math.isfinite(options.webcam_fps) or not 0 < options.webcam_fps <= 120:
        parser.error('웹캠 FPS는 0 초과 120 이하입니다.')
    if options.webcam_hfov_deg is not None and (
            not math.isfinite(options.webcam_hfov_deg) or not 1 < options.webcam_hfov_deg < 179):
        parser.error('웹캠 HFOV 가정은 1~179도 사이여야 합니다.')
    if options.webcam is None and any(v.startswith('--webcam-') for v in argv):
        parser.error('--webcam-* 옵션은 -webcam과 함께 사용하세요.')
    if options.webcam is not None:
        from rclpy.utilities import remove_ros_args
        unknown = remove_ros_args(args=['calibration_ui', *ros_args])[1:]
        if unknown:
            parser.error('알 수 없는 옵션: '+' '.join(unknown))
    return options, ros_args
