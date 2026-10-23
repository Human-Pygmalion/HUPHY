#!/usr/bin/env bash
# HUPHY 설치 — Ubuntu 22.04 + ROS 2 Humble.
#
#   ~/HUPHY/ros/install.sh
#
# 여러 번 실행해도 됨. 이미 된 단계는 건너뜀.
# 바꿀 수 있는 값 (환경변수):
#   HUPHY_WS          ROS 워크스페이스   (기본 ~/ros2_ws)
#   HUPHY_DOMAIN_ID   ROS_DOMAIN_ID     (기본 7, 두 기계가 같아야 함)
#   HUPHY_BASHRC      자동 설정을 넣을 파일 (기본 ~/.bashrc)

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # 이 파일은 ros/ 에 있음
WS="${HUPHY_WS:-$HOME/ros2_ws}"
DOMAIN_ID="${HUPHY_DOMAIN_ID:-7}"
BASHRC="${HUPHY_BASHRC:-$HOME/.bashrc}"
VENV="$REPO/.venv"
SYS_PY=/usr/bin/python3

step() { printf '\n\033[1;34m== %s\033[0m\n' "$*"; }
ok()   { printf '   \033[32m✓\033[0m %s\n' "$*"; }
info() { printf '   %s\n' "$*"; }
die()  { printf '\n\033[1;31m✗ %s\033[0m\n' "$*" >&2; exit 1; }

# ROS 의 setup.bash 는 정의 안 된 변수를 읽어서 set -u 와 안 맞음.
ros_env() { set +u; source /opt/ros/humble/setup.bash; set -u; }

# ---- 0. 확인 ---------------------------------------------------------------
step "0. 환경 확인"
. /etc/os-release
[ "${VERSION_CODENAME:-}" = jammy ] || die "Ubuntu 22.04 (jammy) 가 필요함. 지금: ${PRETTY_NAME:-알 수 없음}"
ok "Ubuntu 22.04"
info "레포        $REPO"
info "워크스페이스 $WS"

# ---- 1. ROS 2 Humble -------------------------------------------------------
step "1. ROS 2 Humble"
need=()
for p in ros-humble-ros-base python3-colcon-common-extensions curl; do
  dpkg -s "$p" >/dev/null 2>&1 || need+=("$p")
done
# venv 는 패키지 이름이 여럿이라 (python3-venv, python3.10-venv) 되는지로 봄.
"$SYS_PY" -c "import ensurepip" >/dev/null 2>&1 || need+=(python3-venv)
if [ ${#need[@]} -eq 0 ]; then
  ok "이미 설치됨"
else
  info "설치할 것: ${need[*]}  (sudo 비밀번호를 물어봄)"
  if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    info "ROS 저장소 추가"
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe
    ver=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    [ -n "$ver" ] || die "ros-apt-source 버전을 못 읽음 (인터넷 연결 확인)"
    deb=$(mktemp --suffix=.deb)
    curl -fsSL -o "$deb" \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.jammy_all.deb"
    sudo dpkg -i "$deb"
    rm -f "$deb"
  fi
  sudo apt-get update
  sudo apt-get install -y ros-humble-ros-base python3-colcon-common-extensions python3-venv curl
  ok "설치함"
fi

# ---- 2. 가상환경 -----------------------------------------------------------
step "2. 가상환경 ($VENV)"
# ROS 가 보이려면 --system-site-packages 여야 하고, 기반은 시스템 파이썬이어야 함.
# venv 를 켠 채 만들면 기반이 자기 자신을 가리키게 되어 이것도 다시 만듦.
venv_ok() {
  local cfg="$VENV/pyvenv.cfg"
  [ -x "$VENV/bin/python" ] && [ -f "$cfg" ] \
    && grep -qx 'include-system-site-packages = true' "$cfg" \
    && grep -qx 'home = /usr/bin' "$cfg"
}
if venv_ok; then
  ok "이미 맞게 있음"
else
  [ -d "$VENV" ] && info "설정이 맞지 않아 다시 만듦"
  rm -rf "$VENV"
  "$SYS_PY" -m venv --system-site-packages "$VENV"
  ok "만듦"
fi
PY="$VENV/bin/python"

# ---- 3. huphy 설치 ---------------------------------------------------------
step "3. huphy 설치"
# setuptools 80 이상은 colcon 과 안 맞음.
"$PY" -m pip install -q --upgrade pip "setuptools>=64,<80"
"$PY" -m pip install -q -e "$REPO[imu]"
ok "pip install -e .[imu]"

# ---- 4. ROS 패키지 빌드 ----------------------------------------------------
step "4. ROS 패키지 빌드 ($WS)"
mkdir -p "$WS/src"
link="$WS/src/huphy"
if [ -L "$link" ]; then
  [ "$(readlink -f "$link")" = "$REPO" ] || die "$link 가 다른 곳을 가리킴: $(readlink -f "$link")"
elif [ -e "$link" ]; then
  die "$link 가 이미 있고 링크가 아님. 옮기거나 지운 뒤 다시 실행"
else
  ln -s "$REPO" "$link"
fi
ok "src/huphy -> $REPO"

# 노드가 시스템 파이썬으로 빌드돼 있으면 지우고 다시 빌드 (그대로 두면 huphy 를 못 찾음).
node="$WS/install/huphy_ros/lib/huphy_ros/control_node"
if [ -f "$node" ] && [ "$(head -1 "$node")" != "#!$PY" ]; then
  info "다른 파이썬으로 빌드돼 있어 build/install/log 를 지움"
  rm -rf "$WS/build" "$WS/install" "$WS/log"
fi

ros_env
# 'colcon build' 가 아니라 가상환경 파이썬으로 돌림 -- 노드가 이 파이썬으로 돌게 됨.
(cd "$WS" && "$PY" -m colcon build --symlink-install) \
  || die "colcon 빌드 실패 (위 로그 확인)"
ok "빌드함"

# ---- 5. 터미널 자동 설정 ---------------------------------------------------
step "5. 터미널 자동 설정 ($BASHRC)"
begin='# >>> huphy >>>'
end='# <<< huphy <<<'
touch "$BASHRC"
# 전에 넣은 블록은 지우고 새로 넣음 -- 여러 번 실행해도 한 번만 들어감.
sed -i "/^$begin\$/,/^$end\$/d" "$BASHRC"
cat >> "$BASHRC" <<EOF
$begin
source /opt/ros/humble/setup.bash
source "$VENV/bin/activate"
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=$DOMAIN_ID
$end
EOF
ok "넣음 (ROS_DOMAIN_ID=$DOMAIN_ID)"

stray=$(sed "/^$begin\$/,/^$end\$/d" "$BASHRC" \
        | grep -nE 'opt/ros/humble/setup|ros2_ws/install/setup|\.venv/bin/activate|ROS_DOMAIN_ID' || true)
if [ -n "$stray" ]; then
  info "블록 밖에도 같은 설정이 있음. 중복이니 지워도 됨:"
  printf '%s\n' "$stray" | sed 's/^/     /'
fi

# ---- 6. 확인 ---------------------------------------------------------------
step "6. 확인"
set +u; source "$WS/install/setup.bash"; set -u
ros2 pkg list 2>/dev/null | grep -qx huphy_ros || die "ros2 가 huphy_ros 를 못 찾음"
ok "ros2 pkg list 에 huphy_ros"
"$PY" -c "import huphy, huphy_ros" || die "import huphy, huphy_ros 실패"
ok "import huphy, huphy_ros"
[ "$(head -1 "$node")" = "#!$PY" ] || die "노드가 가상환경 파이썬으로 빌드되지 않음: $(head -1 "$node")"
ok "노드가 가상환경 파이썬으로 돎"

printf '\n\033[1;32m끝.\033[0m 새 터미널을 열면 바로 ros2 launch 를 쓸 수 있음.\n'
