"""One camera workspace: board settings, capture/import, split, preview and validation."""

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageTk
import yaml

from .project_view import ProjectView


class CameraCalibrationView(ttk.Frame):
    """Keep all camera dataset operations and the final calculation on one tab."""

    def __init__(self, parent, app):
        super().__init__(parent, padding=8)
        self.app = app
        self.photo_rows = {}
        self.heading = ttk.Label(self, text='프로젝트를 만들거나 불러온 뒤 카메라 사진을 관리하세요.', wraplength=560)
        self.heading.pack(fill='x')
        board = ttk.Frame(self)
        board.pack(fill='x', pady=5)
        for key, label, default in [('columns', '내부 코너 가로', '9'), ('rows', '내부 코너 세로', '6'),
                                     ('square', '한 칸 실측 길이(mm)', '30')]:
            cell = ttk.Frame(board)
            cell.pack(side='left', padx=(0, 14))
            ttk.Label(cell, text=label).pack(anchor='w')
            app.fields[key] = tk.StringVar(value=default)
            ttk.Entry(cell, textvariable=app.fields[key], width=10).pack(anchor='w')
        ttk.Label(self, text='① 촬영/파일 추가 → ② 학습·독립 검증 분류 → ③ 현재 사진으로 보정·검증\n'
                  '학습 15장 이상 / 검증 8장 이상 · 위치·거리·기울기를 다양하게 촬영하세요.',
                  wraplength=560).pack(fill='x', pady=4)
        actions = ttk.Frame(self)
        actions.pack(fill='x')
        ttk.Button(actions, text='원본 사진 촬영',
                   command=lambda: app.job(lambda: app.capture('training'))).pack(side='left', padx=(0, 4))
        ttk.Button(actions, text='사진 파일 추가', command=self.import_photos).pack(side='left', padx=4)
        ttk.Button(actions, text='현재 사진으로 보정·독립 검증',
                   command=lambda: app.job(app.validate)).pack(side='left', padx=4)
        self.photo_count = ttk.Label(self, text='학습 0장 / 독립 검증 0장')
        self.photo_count.pack(fill='x', pady=4)
        split = ttk.Frame(self)
        split.pack(fill='x')
        for title, callback in [('선택 → 검증', lambda: self.assign('validation')),
                                ('선택 → 학습', lambda: self.assign('training')),
                                ('선택 사진 삭제', lambda: self.assign(None))]:
            ttk.Button(split, text=title, command=callback).pack(side='left', padx=(0, 4))
        gallery = ttk.Panedwindow(self, orient='horizontal')
        gallery.pack(fill='both', expand=True, pady=4)
        listing, selected = ttk.Frame(gallery), ttk.Frame(gallery, width=220)
        gallery.add(listing, weight=3)
        gallery.add(selected, weight=2)
        self.photos = ProjectView.table(listing, ('분류', '파일'), (65, 180), height=7)
        self.photos.bind('<<TreeviewSelect>>', self.preview)
        ttk.Label(selected, text='선택 사진 미리보기').pack(fill='x')
        self.image_label = ttk.Label(selected, text='목록에서 사진을 선택하세요.', anchor='center')
        self.image_label.pack(fill='both', expand=True)
        self.photo_details = ttk.Label(selected, text='', wraplength=220)
        self.photo_details.pack(fill='x')
        self.result = ttk.Label(self, text='보정 결과: 미계산', wraplength=560, padding=(0, 5))
        self.result.pack(fill='x')

    def refresh(self):
        """Restore the current split and calculation status after any dataset operation."""
        session = self.app.session
        selected = {self.photo_rows[i] for i in self.photos.selection() if i in self.photo_rows}
        self.photos.delete(*self.photos.get_children())
        self.photo_rows = {}
        self.image_label.configure(image='', text='목록에서 사진을 선택하세요.')
        self.photo_details.configure(text='')
        if not session:
            return
        self.heading.configure(text=f"{session.meta['robot_id']} · 프로젝트 {session.root.name}")
        restore = []
        for purpose, paths in session.frames.items():
            for path in paths:
                iid = self.photos.insert('', 'end', values=('학습' if purpose == 'training' else '검증', path.name))
                relative = session.relative(path)
                self.photo_rows[iid] = relative
                if relative in selected:
                    restore.append(iid)
        self.photo_count.configure(text=f"학습 {len(session.frames['training'])}장 / "
                                       f"독립 검증 {len(session.frames['validation'])}장 · 다중 선택 가능")
        if restore:
            self.photos.selection_set(restore)
            self.preview()
        report = session.results.get('camera')
        if report:
            text = '검증 통과' if report.get('passed') else '검증 실패 · 사진/분류 확인 후 재계산'
            if all(key in report for key in ('mean', 'sample_std', 'max')):
                text += f" · RMS 평균 {report['mean']:.3f}px / 최대 {report['max']:.3f}px"
                if report['sample_std'] is not None:
                    text += f" / 표준편차 {report['sample_std']:.3f}px"
            self.result.configure(text=text, foreground='#16733c' if report.get('passed') else '#a05a00')
        else:
            self.result.configure(text='사진 변경됨 · 현재 사진으로 보정·독립 검증을 다시 계산하세요.'
                                  if 'camera' in session.stale else '보정 결과: 미계산', foreground='#a05a00')

    def preview(self, event=None):
        selection = self.photos.selection()
        if not selection or selection[0] not in self.photo_rows or not self.app.session:
            return
        path = self.app.session.local_path(self.photo_rows[selection[0]])
        try:
            with Image.open(path) as image:
                image.thumbnail((220, 180))
                self.photo = ImageTk.PhotoImage(image.copy())
            self.image_label.configure(image=self.photo, text='')
            sidecar = path.with_suffix('.capture.yaml')
            data = yaml.safe_load(sidecar.read_text()) if sidecar.exists() else {}
            self.photo_details.configure(text=f"{path.name}\n출처: "
                                             f"{data.get('source', data.get('format', '이전 사진'))}")
        except (OSError, ValueError) as error:
            self.image_label.configure(image='', text=str(error))

    def assign(self, purpose):
        paths = [self.photo_rows[i] for i in self.photos.selection()]
        if not paths:
            return
        if purpose is None and not messagebox.askyesno('사진 제외', '선택 사진을 보정 목록에서 삭제할까요? 원본은 보존합니다.'):
            return
        self.app.job(lambda: self.app.require().assign_photos(paths, purpose))

    def import_photos(self):
        paths = filedialog.askopenfilenames(title='추가할 체커보드 사진',
                                          filetypes=[('Images', '*.png *.jpg *.jpeg *.bmp *.tif *.tiff')])
        if paths:
            def import_current_board():
                self.app.configure_board()
                return self.app.require().import_photos(paths)
            self.app.job(import_current_board)
