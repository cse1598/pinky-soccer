"""Command line entry point for the camera intrinsics experiment."""

import argparse
from pathlib import Path
import subprocess
import sys

import cv2

from .core import check, export, import_archive, load, positive, summarize


def gui_command(session):
    meta = load(Path(session) / 'session.yaml')
    if meta['purpose'] != 'calibration':
        raise ValueError('Do not fit a model on the validation dataset')
    # No --commit or SetCameraInfo call: save first, independently verify before use.
    return ['ros2', 'run', 'camera_calibration', 'cameracalibrator',
            '--size', '9x6', '--square', str(positive(meta['square_mm']) / 1000),
            '--no-service-check', '--camera_name', meta['camera_id'],
            '--ros-args', '-r', 'image:=' + meta['image_topic']]


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest='command', required=True)
    capture = commands.add_parser('record', help='Record raw ROS bag, CSV and sampled PNGs')
    capture.add_argument('--session', required=True, help='New directory, never reused')
    capture.add_argument('--purpose', choices=['calibration', 'validation'], required=True)
    capture.add_argument('--square-mm', type=positive, required=True)
    capture.add_argument('--camera-id', required=True)
    capture.add_argument('--camera-settings', required=True,
                         help='Resolution, focus, exposure and camera mode used')
    capture.add_argument('--image-topic', default='/camera/image_raw')
    capture.add_argument('--info-topic', default='/camera/camera_info')
    capture.add_argument('--interval', type=positive, default=1.0)
    capture.add_argument('--duration', type=positive, default=180.0)
    capture.add_argument('--notes', default='')
    gui = commands.add_parser('gui', help='Run the standard ROS calibration GUI')
    gui.add_argument('--session', required=True)
    imp = commands.add_parser('import', help='Preserve the archive produced by GUI SAVE')
    imp.add_argument('--session', required=True)
    imp.add_argument('--archive', required=True)
    verify = commands.add_parser('check', help='Check imported training or held-out images')
    verify.add_argument('--session', required=True)
    verify.add_argument('--calibration', required=True)
    verify.add_argument('--max-rms-px', type=positive, default=0.7)
    verify.add_argument('--min-views', type=int)
    approve = commands.add_parser('export', help='Export only after both checks and visual review')
    approve.add_argument('--training', required=True)
    approve.add_argument('--validation', required=True)
    approve.add_argument('--visual-check', action='store_true')
    repeat = commands.add_parser('summarize', help='Mean and sample standard deviation across runs')
    repeat.add_argument('--reports', nargs='+', required=True)
    repeat.add_argument('--output', required=True)
    return result


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    try:
        if args.command == 'record':
            from .record import record
            print(record(args))
        elif args.command == 'gui':
            return subprocess.call(gui_command(args.session))
        elif args.command == 'import':
            print(import_archive(args.session, args.archive))
        elif args.command == 'check':
            report = check(args.session, args.calibration, args.max_rms_px, args.min_views)
            print('PASS' if report['passed'] else 'FAIL: ' + '; '.join(report['reasons']))
            return 0 if report['passed'] else 2
        elif args.command == 'export':
            print(export(args.training, args.validation, args.visual_check))
        elif args.command == 'summarize':
            summarize(args.reports, args.output)
            print(args.output)
    except (ValueError, OSError, KeyError, TypeError, RuntimeError, cv2.error) as error:
        print(f'Calibration error: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
