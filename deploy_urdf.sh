#!/usr/bin/env bash
# 캘리브레이션 결과 URDF 를 로봇 실사용 경로에 반영하고 서비스를 재시작한다.
#
#   ./deploy_urdf.sh                      # urdf/<스템>_latest.urdf 를 배포 (복사)
#   ./deploy_urdf.sh urdf/xxx_calib_*.urdf  # 특정 파일을 배포
#   ./deploy_urdf.sh --symlink            # 복사 대신 심볼릭 링크로 연결
#   ./deploy_urdf.sh --no-restart         # 파일만 바꾸고 서비스는 건드리지 않음
#   ./deploy_urdf.sh --rollback           # 가장 최근 백업으로 되돌리고 재시작
#
# 대상 경로는 calib_server.py 와 같은 규칙으로 찾는다:
#   $CONA_URDF_PATH -> $ROBOT_SETUP 을 source 해서 얻은 값 -> 실패 시 중단
# ranger.launch.py 가 이 변수로 robot_description 을 읽으므로 이게 유일한 원본이다.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_SETUP="${ROBOT_SETUP:-/etc/coga-robotics/cona/setup.bash}"
SERVICE="${CONA_SERVICE:-cona}"

MODE="copy"
RESTART=1
ROLLBACK=0
SRC=""
TARGET_ARG=""

while [ $# -gt 0 ]; do
  case "$1" in
    --symlink)    MODE="symlink" ;;
    --no-restart) RESTART=0 ;;
    --rollback)   ROLLBACK=1 ;;
    --target)     TARGET_ARG="${2:-}"; shift ;;
    -h|--help)    sed -n '2,22p' "${BASH_SOURCE[0]}"; exit 0 ;;
    -*)           echo "알 수 없는 옵션: $1" >&2; exit 2 ;;
    *)            SRC="$1" ;;
  esac
  shift
done

die() { echo "오류: $*" >&2; exit 1; }

# 웹 UI 가 이 스크립트를 호출할 때는 입력을 받을 수 없다. sudo -n 으로 암호를 묻지
# 않게 하고, 권한이 없으면 무엇을 설정해야 하는지 알려준다 (프롬프트에 매달리면
# 브라우저 쪽은 이유도 모르고 타임아웃된다).
restart_service() {
  if sudo -n true 2>/dev/null; then
    sudo -n systemctl restart "$SERVICE"
  else
    cat >&2 <<EOF
오류: 암호 없이 sudo 를 쓸 수 없어 $SERVICE 를 재시작하지 못했습니다.
URDF 파일은 이미 교체됐습니다. 아래 중 하나로 마무리하십시오.

  1) 터미널에서 직접:  sudo systemctl restart $SERVICE
  2) 이 명령만 암호 없이 허용 (웹 UI 버튼을 쓰려면 필요):
       sudo tee /etc/sudoers.d/ranger-calib <<'SUDO'
       $(id -un) ALL=(root) NOPASSWD: /bin/systemctl restart $SERVICE
       SUDO
       sudo chmod 440 /etc/sudoers.d/ranger-calib
EOF
    return 1
  fi
}

# ---- 대상 URDF 경로 확인 ----
resolve_target() {
  # 웹 UI 에서 호출될 때는 calib_server.py 가 이미 해석한 경로를 --target 으로 넘긴다.
  # 서버와 스크립트가 각자 해석하면 두 값이 갈릴 수 있으므로 넘어온 값을 그대로 쓴다.
  if [ -n "$TARGET_ARG" ]; then
    TARGET="$TARGET_ARG"
    TARGET_SRC="--target"
    return
  fi
  local p="${CONA_URDF_PATH:-}"
  local src="env"
  if [ -z "$p" ] && [ -f "$ROBOT_SETUP" ]; then
    # .bashrc 는 비대화형 셸에서 조기 return 하므로 설정파일을 직접 source 한다
    p="$(. "$ROBOT_SETUP" >/dev/null 2>&1; printf %s "${CONA_URDF_PATH:-}")"
    src="$ROBOT_SETUP"
  fi
  [ -n "$p" ] || die "CONA_URDF_PATH 를 찾지 못했습니다 ($ROBOT_SETUP 확인)"
  # 상대경로는 이 저장소 기준으로 해석한다 (calib_server.py 와 동일)
  case "$p" in
    /*) TARGET="$p" ;;
    *)  TARGET="$(cd "$ROOT" && cd "$(dirname "$p")" 2>/dev/null && pwd)/$(basename "$p")" \
          || die "경로를 해석할 수 없습니다: $p" ;;
  esac
  TARGET_SRC="$src"
}

resolve_target
echo "대상 URDF : $TARGET   (출처: $TARGET_SRC)"
[ -e "$TARGET" ] || echo "경고: 대상 파일이 아직 없습니다 — 새로 만들게 됩니다"

# ---- 롤백 ----
if [ "$ROLLBACK" = "1" ]; then
  LAST="$(ls -1t "${TARGET}".bak.* 2>/dev/null | head -1 || true)"
  [ -n "$LAST" ] || die "백업 파일이 없습니다 (${TARGET}.bak.*)"
  echo "롤백 : $LAST -> $TARGET"
  cp -f "$LAST" "$TARGET"
  if [ "$RESTART" = "1" ]; then
    restart_service
    sleep 2
    systemctl is-active "$SERVICE" || die "$SERVICE 가 활성 상태가 아닙니다"
    echo "$SERVICE 재시작 완료"
  fi
  exit 0
fi

# ---- 원본 파일 확인 ----
if [ -z "$SRC" ]; then
  STEM="$(basename "$TARGET")"; STEM="${STEM%.urdf}"
  SRC="$ROOT/urdf/${STEM}_latest.urdf"
fi
[ -f "$SRC" ] || die "원본 파일이 없습니다: $SRC (웹 UI 에서 'URDF 만들기' 를 먼저 누르십시오)"

# 같은 파일을 자기 자신에 덮어쓰는 사고 방지
if [ -e "$TARGET" ] && [ "$(readlink -f "$SRC")" = "$(readlink -f "$TARGET")" ]; then
  die "원본과 대상이 같은 파일입니다"
fi

echo "원본 URDF : $SRC"
echo

# ---- 바뀌는 origin 을 먼저 보여준다 ----
if [ -f "$TARGET" ]; then
  echo "camera origin 변경 내용 (- 현재 / + 새로 적용):"
  diff <(grep -A1 'camera_.*RGBD_link"/>' "$TARGET" | grep '<origin') \
       <(grep -A1 'camera_.*RGBD_link"/>' "$SRC"    | grep '<origin') \
    | sed 's/^/  /' || true
  echo
fi

# ---- 백업 후 반영 ----
if [ -f "$TARGET" ] && [ ! -L "$TARGET" ]; then
  BACKUP="${TARGET}.bak.$(date +%Y%m%d_%H%M%S)"
  cp -p "$TARGET" "$BACKUP"
  echo "백업 : $BACKUP"
fi

if [ "$MODE" = "symlink" ]; then
  ln -sfn "$(readlink -f "$SRC")" "$TARGET"
  echo "연결 : $TARGET -> $(readlink -f "$SRC")  (심볼릭 링크)"
else
  # 같은 디렉터리에 임시로 쓰고 rename 한다. 읽는 쪽이 반쯤 쓰인 파일을 보지 않는다
  TMP="$(mktemp "$(dirname "$TARGET")/.urdf.XXXXXX")"
  cat "$SRC" > "$TMP"
  chmod --reference="$SRC" "$TMP" 2>/dev/null || chmod 644 "$TMP"
  mv -f "$TMP" "$TARGET"
  echo "복사 : $SRC -> $TARGET"
fi

# ---- 서비스 재시작 ----
if [ "$RESTART" = "0" ]; then
  echo "재시작 생략 (--no-restart). 반영하려면: sudo systemctl restart $SERVICE"
  exit 0
fi

echo
echo "$SERVICE 재시작..."
sudo systemctl restart "$SERVICE"
sleep 3

if ! systemctl is-active --quiet "$SERVICE"; then
  echo "오류: $SERVICE 가 올라오지 않았습니다. 최근 로그:" >&2
  journalctl -u "$SERVICE" -n 30 --no-pager >&2
  echo >&2
  echo "되돌리려면: $0 --rollback" >&2
  exit 1
fi
echo "$SERVICE active"

# ---- 동작 확인 ----
echo
echo "적용된 origin (실파일):"
grep -B1 '<origin' "$TARGET" | grep -A1 'camera_.*RGBD_link"/>' | sed 's/^/  /' || true

cat <<'EOF'

TF 로 최종 확인 (robot_state_publisher 가 실제로 읽은 값):
  source /opt/ros/humble/setup.bash
  export ROS_DOMAIN_ID=18
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ros2 run tf2_ros tf2_echo base_link camera_frontRGBD_link

위 translation/rotation 이 새 origin 과 같으면 반영된 것입니다.
어긋나면 서비스가 다른 URDF 를 읽고 있습니다 — CONA_URDF_PATH 를 다시 확인하십시오.

문제가 있으면 되돌립니다:
  ./deploy_urdf.sh --rollback
EOF
