"""Live pixel-only chessboard feedback, independent of calibration recording."""

import cv2

from .board_geometry import Board


def annotate_board(frame, columns, rows):
    """Return an annotated copy, Korean status, and full-grid detection success."""
    preview = frame.copy()
    try:
        # Square size is irrelevant for pixel detection; no metric values are inferred here.
        board = Board(int(columns), int(rows))
    except (ValueError, TypeError, OverflowError):
        return preview, '설정 확인: 내부 코너 가로·세로 수를 정수 3~30으로 입력하세요.', False
    corners = board.detect(cv2.cvtColor(preview, cv2.COLOR_BGR2GRAY), fast=True)
    shape = f'{board.columns}×{board.rows}'
    if corners is None:
        return preview, (f'미검출 · 내부 코너 {shape} 전체가 보여야 합니다. '
                         '보드 전체·초점·조명과 코너 수(칸 수 아님)를 확인하세요.'), False
    cv2.drawChessboardCorners(preview, board.pattern, corners, True)
    for index, label in [(0, '0'), (board.columns-1, '+X'),
                         ((board.rows-1)*board.columns, '+Y')]:
        point = tuple(corners[index, 0].astype(int))
        cv2.putText(preview, label, point, cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 255), 2)
    return preview, (f'인식 성공 · 내부 코너 {shape} = {len(corners)}개 · '
                     '포인트 표시 중, 사진은 촬영 버튼으로 저장'), True
