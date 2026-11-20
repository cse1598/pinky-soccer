"""Private Jupyter kernel transport for pinkylib.Camera; no notebook edits."""

import base64
from datetime import datetime, timezone
import http.cookiejar
import json
from pathlib import Path
import time
import urllib.parse
import urllib.request
import uuid

import cv2
import numpy as np


class JupyterCamera:
    def __init__(self, url='http://192.168.4.1:8888', token=''):
        parsed = urllib.parse.urlsplit(url)
        if parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.query:
            raise ValueError('Jupyter 주소는 http(s)://주소:포트 형식으로 입력하세요.')
        if parsed.username or parsed.password or parsed.fragment:
            raise ValueError('인증 정보는 주소 대신 토큰 입력란을 사용하세요.')
        self.base = url.rstrip('/')
        self.token = token
        self.cookies = http.cookiejar.CookieJar()
        self.http = urllib.request.build_opener(
            urllib.request.ProxyHandler({}), urllib.request.HTTPCookieProcessor(self.cookies))
        self.kernel = self.socket = None
        self.info = None
        self.service_ready = False

    def request(self, path, method='GET', data=None):
        headers = {'Content-Type': 'application/json'}
        if self.token:
            headers['Authorization'] = 'token ' + self.token
        for cookie in self.cookies:
            if cookie.name == '_xsrf':
                headers['X-XSRFToken'] = urllib.parse.unquote(cookie.value)
        body = None if data is None else json.dumps(data).encode()
        req = urllib.request.Request(self.base + path, data=body, headers=headers, method=method)
        with self.http.open(req, timeout=8) as response:
            content = response.read()
            if response.headers.get_content_type() == 'application/json' and content:
                return json.loads(content)
            return None

    def connect(self, width=640, height=480):
        import websocket
        try:
            self.request('/tree')
            self.kernel = self.request('/api/kernels', 'POST', {'name': 'python3'})['id']
            url = self.base.replace('https://', 'wss://', 1).replace('http://', 'ws://', 1)
            headers = {'Authorization': 'token ' + self.token} if self.token else {}
            self.socket = websocket.create_connection(
                url + '/api/kernels/' + self.kernel + '/channels', timeout=8,
                origin=self.base, header=headers,
                cookie='; '.join(c.name + '=' + c.value for c in self.cookies),
                http_no_proxy=[urllib.parse.urlsplit(self.base).hostname])
            source = Path(__file__).with_name('robot_service.py').read_text()
            self.execute(source + '\n_pinky_service = RobotCameraService(' +
                         str(int(width)) + ', ' + str(int(height)) + ')')
            self.service_ready = True
            self.info = self.call('info')
            return self.info
        except Exception:
            self.close()
            raise

    def execute(self, code, timeout=90):
        from websocket import WebSocketTimeoutException
        if self.socket is None:
            raise RuntimeError('먼저 로봇에 연결하세요.')
        msg_id = uuid.uuid4().hex
        self.socket.send(json.dumps({
            'header': {'msg_id': msg_id, 'username': 'pinky-calibration',
                       'session': uuid.uuid4().hex, 'date': datetime.now(timezone.utc).isoformat(),
                       'msg_type': 'execute_request', 'version': '5.3'},
            'parent_header': {}, 'metadata': {}, 'channel': 'shell',
            'content': {'code': code, 'silent': False, 'store_history': False,
                        'user_expressions': {}, 'allow_stdin': False, 'stop_on_error': True},
        }))
        deadline, output, failure = time.monotonic() + timeout, '', None
        while time.monotonic() < deadline:
            self.socket.settimeout(min(8, max(.1, deadline - time.monotonic())))
            try:
                raw = self.socket.recv()
            except (TimeoutError, WebSocketTimeoutException):
                continue
            if not raw:
                raise ConnectionError('Jupyter 연결이 끊어졌습니다.')
            if not isinstance(raw, str):
                continue
            msg = json.loads(raw)
            if msg.get('parent_header', {}).get('msg_id') != msg_id:
                continue
            content = msg['content']
            if msg['msg_type'] == 'stream':
                output += content['text']
            elif msg['msg_type'] == 'error':
                failure = content['ename'] + ': ' + content['evalue']
            elif msg['msg_type'] == 'status' and content['execution_state'] == 'idle':
                if failure:
                    raise RuntimeError(failure)
                for line in output.splitlines():
                    if line.startswith('PINKY_JSON:'):
                        return json.loads(line[len('PINKY_JSON:'):])
                return None
        raise TimeoutError('로봇 작업 제한 시간을 초과했습니다. 다시 연결하세요.')

    def call(self, action, **params):
        payload = json.dumps(params, ensure_ascii=True)
        return self.execute('import json\nprint("PINKY_JSON:" + json.dumps('
                            f'_pinky_service.run({action!r}, json.loads({payload!r}))))')

    def frame(self, purpose=None, min_sharpness=30.):
        result = self.call('frame', purpose=purpose, min_sharpness=min_sharpness)
        result['png_bytes'] = base64.b64decode(result.pop('png'), validate=True)
        result['image'] = cv2.imdecode(np.frombuffer(result['png_bytes'], np.uint8),
                                     cv2.IMREAD_COLOR)
        if result['image'] is None:
            raise ValueError('로봇 영상이 손상되었습니다.')
        if 'jpeg' in result:
            result['jpeg_bytes'] = base64.b64decode(result.pop('jpeg'), validate=True)
        return result

    def fit(self, square_mm):
        return base64.b64decode(self.call('fit', square_mm=square_mm)['npz'], validate=True)

    def close(self):
        """Only stop the kernel created by this instance, even after a failed connect."""
        try:
            if self.socket is not None:
                if self.service_ready:
                    try:
                        self.execute('_pinky_service.run("close", {})', timeout=3)
                    except Exception:
                        pass
                self.socket.close()
        finally:
            self.socket = None
            self.service_ready = False
            if self.kernel is not None:
                kernel, self.kernel = self.kernel, None
                try:
                    self.request('/api/kernels/' + kernel, 'DELETE')
                except Exception:
                    # Robot watchdog still releases the camera after loss of contact.
                    pass
