import os, re, sys, json, shlex, time, tempfile, subprocess, http.server, socketserver

# 설치 위치에 상관없이 동작하도록 이 스크립트가 있는 디렉터리를 기준으로 삼는다.
ROOT = os.path.dirname(os.path.abspath(__file__))
URDF_OUT_DIR = os.path.join(ROOT, "urdf")
os.makedirs(URDF_OUT_DIR, exist_ok=True)
PORT = int(os.environ.get("RANGER_CALIB_PORT", "8080"))

os.chdir(ROOT)

CAPTURE_SCRIPT = os.path.join(ROOT, "capture_once.py")
# 자동 정렬은 ROS 를 쓰지 않는다 (numpy/scipy 만 필요). 그래서 이 서버와 같은 인터프리터로 돌린다.
AUTO_SCRIPT = os.path.join(ROOT, "auto_calib.py")
AUTO_TIMEOUT = 120

# 로봇 반영(실사용 URDF 교체 + 서비스 재시작)은 되돌리기 어려운 동작이다.
# 이 서버는 인증이 없고 0.0.0.0 에 열리므로, 같은 네트워크의 누구나 호출할 수 있다.
# 그래서 기본은 막아 두고 환경변수로 명시적으로 켠 경우에만 허용한다.
#   RANGER_ALLOW_DEPLOY=1 python3 calib_server.py
DEPLOY_SCRIPT = os.path.join(ROOT, "deploy_urdf.sh")
DEPLOY_TIMEOUT = 90
ALLOW_DEPLOY = os.environ.get("RANGER_ALLOW_DEPLOY") == "1"
CAPTURE_CMD = (
    "source /opt/ros/humble/setup.bash; "
    "export ROS_DOMAIN_ID=18; "
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
    "python3 " + shlex.quote(CAPTURE_SCRIPT)
)

# 로봇이 실제로 쓰는 URDF 는 기체 설정파일이 $CONA_URDF_PATH 로 지정한다.
#   /etc/coga-robotics/cona/setup.bash  ->  export CONA_URDF_PATH=...
# ranger.launch.py 도 이 변수로 robot_description 을 읽으므로 이게 유일한 원본이다.
#
# 문제: .bashrc 는 비대화형 셸에서 조기 return 하므로, systemd/ssh 로 띄우면
# 이 변수가 프로세스 환경에 안 실려 온다. 그래서 설정파일을 직접 source 해서 읽는다.
ROBOT_SETUP = os.environ.get("ROBOT_SETUP", "/etc/coga-robotics/cona/setup.bash")
DEFAULT_URDF_NAME = "ranger_new_a01.urdf"   # 위 두 경로가 모두 실패했을 때의 최후 폴백
DEFAULT_URDF_REL = os.path.join("..", "data", "CoNA", "urdf", DEFAULT_URDF_NAME)

def read_cona_urdf_path():
    """CONA_URDF_PATH 를 환경 -> 기체 설정파일 순으로 찾는다."""
    v = (os.environ.get("CONA_URDF_PATH") or "").strip()
    if v:
        return v, "env"
    if os.path.isfile(ROBOT_SETUP):
        try:
            cmd = ". " + shlex.quote(ROBOT_SETUP) + ' >/dev/null 2>&1; printf %s "$CONA_URDF_PATH"'
            proc = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=10)
            v = proc.stdout.strip()
            if v:
                return v, "ROBOT_SETUP"
        except (OSError, subprocess.SubprocessError):
            pass
    return "", "unset"

def resolve_live_urdf():
    # 상대·절대 둘 다 받는다 (os.path.join 은 절대경로를 만나면 앞을 버린다).
    p, src = read_cona_urdf_path()
    if not p:
        return os.path.normpath(os.path.join(ROOT, DEFAULT_URDF_REL)), "fallback-default"
    full = os.path.normpath(os.path.join(ROOT, p))
    # 변수가 파일이 아니라 디렉터리를 가리키는 경우도 받아준다.
    if os.path.isdir(full):
        return os.path.join(full, DEFAULT_URDF_NAME), src + "(dir)"
    return full, src

LIVE_URDF_PATH, LIVE_URDF_SRC = resolve_live_urdf()
# 실사용 URDF의 파일명/스템. 화면 표시와 저장 파일명이 모두 이걸 따라가므로
# 기체가 a01 -> a02 로 바뀌면 코드 수정 없이 같이 따라온다.
LIVE_URDF_NAME = os.path.basename(LIVE_URDF_PATH)
LIVE_URDF_STEM = os.path.splitext(LIVE_URDF_NAME)[0]

class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path.split("?")[0] == "/latest_urdf":
            try:
                with open(LIVE_URDF_PATH, "r", encoding="utf-8") as f:
                    data = f.read().encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                # 어느 기체의 URDF를 읽었는지 화면이 알 수 있게 실제 파일명을 함께 보낸다.
                # 헤더는 latin-1 로만 인코딩되므로 비ASCII 파일명은 치환해서 보낸다.
                self.send_header("X-Live-Urdf", LIVE_URDF_NAME.encode("ascii", "replace").decode("ascii"))
                self.end_headers()
                self.wfile.write(data)
            except OSError as e:
                self.send_response(500)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(str(e).encode("utf-8"))
            return
        super().do_GET()

    def do_POST(self):
        if self.path == "/refresh_snapshot":
            try:
                proc = subprocess.run(["bash", "-lc", CAPTURE_CMD], capture_output=True, text=True, timeout=15)
                out = proc.stdout + proc.stderr
                print(out, end="", flush=True)   # 서버 콘솔에도 남긴다 (브라우저만 보면 놓치므로)
                # capture_once.py 는 실패해도 "DONE 0/6" 을 찍으므로 개수를 봐야 한다.
                # 6개 전부 성공했을 때만 200. 부분 성공(4/6)도 실패로 보고해야
                # 사용자가 옛 점군에 맞춰 보정하는 일이 없다.
                m = re.search(r"DONE (\d+)/(\d+)", out)
                all_ok = bool(m) and m.group(1) == m.group(2) and m.group(1) != "0"
                self.send_response(200 if all_ok else 500)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(out.encode("utf-8"))
            except subprocess.TimeoutExpired:
                self.send_response(504)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"capture timed out")
        elif self.path == "/auto_align":
            # 화면의 현재 상태(초기 origin + 마운트 체인)를 그대로 받아 auto_calib.py 에 넘긴다.
            # 서버가 상수를 들고 있으면 index.html 과 어긋나므로 중계만 한다.
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length)
            tmp_path = None
            try:
                with tempfile.NamedTemporaryFile("wb", suffix=".json", delete=False) as tf:
                    tf.write(body)
                    tmp_path = tf.name
                # 인코딩을 명시해야 한다. 생략하면 부모가 로케일 인코딩(예: Windows cp949)으로
                # 디코딩을 시도하고, 한글이 든 JSON 에서 UnicodeDecodeError 가 나면서
                # proc.stdout 이 조용히 None 이 된다.
                proc = subprocess.run([sys.executable, AUTO_SCRIPT, tmp_path],
                                      capture_output=True, text=True,
                                      encoding="utf-8", errors="replace", timeout=AUTO_TIMEOUT)
                if proc.returncode != 0 or not (proc.stdout or "").strip():
                    # stderr 를 그대로 올려야 scipy 미설치 같은 원인이 브라우저에서 바로 보인다.
                    msg = (proc.stderr or proc.stdout or "auto_calib.py 실행 실패").strip()
                    print(msg, flush=True)
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json; charset=utf-8")
                    self.end_headers()
                    self.wfile.write(json.dumps({"error": msg}, ensure_ascii=False).encode("utf-8"))
                    return
                out = proc.stdout.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(out)))
                self.end_headers()
                self.wfile.write(out)
            except subprocess.TimeoutExpired:
                self.send_response(504)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.end_headers()
                self.wfile.write(json.dumps({"error": "자동 정렬 시간 초과 (%ds)" % AUTO_TIMEOUT},
                                            ensure_ascii=False).encode("utf-8"))
            finally:
                if tmp_path:
                    try:
                        os.unlink(tmp_path)
                    except OSError:
                        pass
        elif self.path == "/deploy_urdf":
            self.handle_deploy()
        elif self.path == "/save_urdf":
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length).decode("utf-8")
            ts = time.strftime("%Y%m%d_%H%M%S")
            # 저장 파일명도 실사용 URDF 이름을 따라간다 (a01 -> ranger_new_a01_calib_*.urdf).
            # 기존에는 "ranger_new_" 로 고정돼 있어 어느 기체 결과인지 파일만 봐선 알 수 없었다.
            out_path = os.path.join(URDF_OUT_DIR, f"{LIVE_URDF_STEM}_calib_{ts}.urdf")
            with open(out_path, "w", encoding="utf-8") as f:
                f.write(body)
            latest_path = os.path.join(URDF_OUT_DIR, f"{LIVE_URDF_STEM}_latest.urdf")
            with open(latest_path, "w", encoding="utf-8") as f:
                f.write(body)
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(out_path.encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()

    def handle_deploy(self):
        """deploy_urdf.sh 를 돌려 실사용 URDF 를 교체하고 cona 를 재시작한다.

        스크립트 출력을 그대로 브라우저에 올린다. 실패 사유(백업 실패, sudo 권한,
        서비스가 안 올라옴)가 화면에서 바로 읽혀야 다음 조치를 할 수 있다.
        """
        length = int(self.headers.get("Content-Length", 0))
        try:
            req = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            req = {}
        action = req.get("action", "deploy")
        urdf_file = req.get("file")

        def reply(code, text):
            body = text.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        if not ALLOW_DEPLOY:
            reply(403, "로봇 반영이 꺼져 있습니다. 서버를 다음처럼 다시 띄우십시오:\n"
                       "  RANGER_ALLOW_DEPLOY=1 python3 calib_server.py\n\n"
                       "이 서버는 인증이 없고 0.0.0.0 에 열리므로, 켜 두면 같은 네트워크의\n"
                       "누구나 실사용 URDF 를 바꾸고 서비스를 재시작할 수 있습니다.")
            return
        if not os.path.isfile(DEPLOY_SCRIPT):
            reply(500, "deploy_urdf.sh 가 없습니다: %s" % DEPLOY_SCRIPT)
            return

        # 절대경로를 그대로 넘기면 Windows 백슬래시가 bash 에서 깨진다.
        # cwd 를 저장소로 두고 상대경로만 넘기면 두 OS 에서 같이 동작한다.
        # 대상 경로는 서버가 이미 해석해 둔 값을 넘긴다. 스크립트가 따로 해석하면
        # 두 값이 갈릴 수 있고, 일부 환경에서는 자식 프로세스가 환경변수를 못 받는다.
        cmd = ["bash", os.path.basename(DEPLOY_SCRIPT),
               "--target", LIVE_URDF_PATH.replace("\\", "/")]
        if action == "rollback":
            cmd.append("--rollback")
        elif urdf_file:
            # 경로 주입 방지: urdf/ 안의 파일명만 받는다
            name = os.path.basename(urdf_file)
            if not os.path.isfile(os.path.join(URDF_OUT_DIR, name)):
                reply(400, "urdf/ 안에 그런 파일이 없습니다: %s" % name)
                return
            cmd.append("urdf/" + name)

        try:
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  encoding="utf-8", errors="replace",
                                  timeout=DEPLOY_TIMEOUT, cwd=ROOT)
        except subprocess.TimeoutExpired:
            reply(504, "로봇 반영 시간 초과 (%ds). 서비스 상태를 직접 확인하십시오:\n"
                       "  systemctl status cona" % DEPLOY_TIMEOUT)
            return

        out = (proc.stdout or "") + (proc.stderr or "")
        print(out, end="", flush=True)      # 서버 콘솔에도 남긴다
        reply(200 if proc.returncode == 0 else 500, out or "출력 없음")

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        # 페이지를 다른 오리진에서 열어도 X-Live-Urdf 를 읽을 수 있게 노출한다.
        self.send_header("Access-Control-Expose-Headers", "X-Live-Urdf")
        super().end_headers()

    def log_message(self, fmt, *args):
        pass

socketserver.TCPServer.allow_reuse_address = True
with socketserver.TCPServer(("0.0.0.0", PORT), Handler) as httpd:
    print(f"serving {ROOT} on :{PORT}, POST /save_urdf writes into {URDF_OUT_DIR}")
    # 경로가 틀렸을 때 '최신 URDF 불러오기'가 조용히 실패하는 대신 여기서 바로 보이게 한다.
    mark = "ok" if os.path.isfile(LIVE_URDF_PATH) else "MISSING - CONA_URDF_PATH 를 확인하세요"
    print(f"live urdf: {os.path.relpath(LIVE_URDF_PATH, ROOT)}  (src={LIVE_URDF_SRC}) [{mark}]")
    print(f"           -> {LIVE_URDF_PATH}")
    print(f"save as  : {LIVE_URDF_STEM}_calib_<ts>.urdf, {LIVE_URDF_STEM}_latest.urdf")
    httpd.serve_forever()
