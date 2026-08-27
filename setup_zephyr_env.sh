#!/usr/bin/env bash
# ============================================================
# Zephyr 开发环境一键搭建脚本（Ubuntu）
# 对应文档：《新手环境搭建.md》第一、二、三部分
# 用法：bash setup_zephyr_env.sh
# 特点：可重复运行，已完成的步骤自动跳过（幂等）
# ============================================================

set -e

# ------------------------------------------------------------
# 配置区（按需修改）
# ------------------------------------------------------------
# 网络代理：下载慢/失败时填写，格式 "http://IP:端口"；不需要则留空
PROXY="http://127.0.0.1:7890"

# Zephyr manifest 仓库地址
MANIFEST_URL="https://github.com/derskop/zephyr.git"

# 拉取仓库方式：full=全部（west update）/ nsing=只拉 nsing 驱动库 / skip=暂不拉取
UPDATE_MODE="full"

# ------------------------------------------------------------
# 进入家目录，用 pwd 获取实际安装根路径
# 所有东西（.venv / SDK / 环境变量）都安装在这个路径下
# ------------------------------------------------------------
cd ~
BASE_DIR=$(pwd)
WORKSPACE="$BASE_DIR/zephyrproject"
echo "[提示] 安装根路径（pwd）：$BASE_DIR"
echo "[提示] Zephyr 工作区：$WORKSPACE"

# ------------------------------------------------------------
# 函数：版本比较（ver_ge 1.2.3 1.2.2 → true，表示前者 ≥ 后者）
# ------------------------------------------------------------
ver_ge() {
  [ "$(printf '%s\n%s\n' "$1" "$2" | sort -V | head -n1)" = "$2" ]
}

# 代理设置（下载 pyenv / SDK 等需要）
if [ -n "$PROXY" ]; then
  export http_proxy="$PROXY"
  export https_proxy="$PROXY"
  echo "[提示] 已设置代理：$PROXY"
fi

echo "============================================================"
echo " Zephyr 环境搭建开始（第一部分：系统准备）"
echo "============================================================"

# ---------- 1.1 更新系统 ----------
echo "[1/9] 更新系统软件包..."
sudo apt update
sudo apt upgrade -y

# ---------- 1.2 安装依赖 ----------
echo "[2/9] 安装编译依赖..."
sudo apt install -y --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget python3-dev python3-venv python3-tk \
  xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

# ---------- 1.3 检查依赖版本 ----------
echo "[3/9] 检查依赖版本（要求：cmake≥3.28.0 / python3≥3.12 / dtc≥1.4.6）..."
CMAKE_VER=$(cmake --version 2>/dev/null | head -n1 | awk '{print $3}' || true)
PYTHON_VER=$(python3 --version 2>/dev/null | awk '{print $2}' || true)
DTC_VER=$(dtc --version 2>/dev/null | awk '{print $NF}' || true)
echo "  当前版本：cmake=${CMAKE_VER:-未安装}  python3=${PYTHON_VER:-未安装}  dtc=${DTC_VER:-未安装}"

echo "============================================================"
echo " Zephyr 环境搭建（第二部分：工具链升级，版本不满足才执行）"
echo "============================================================"

# ---------- 2.1 升级 CMake（版本 < 3.28.0 才执行） ----------
if ! ver_ge "$CMAKE_VER" "3.28.0"; then
  echo "[4/9] 升级 CMake（Kitware 官方源）..."
  sudo apt install -y software-properties-common
  wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
  sudo apt-add-repository -y "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
  sudo apt update
  sudo apt install -y cmake
  CMAKE_VER=$(cmake --version | head -n1 | awk '{print $3}')
  echo "  CMake 已升级到：$CMAKE_VER"
else
  echo "[4/9] CMake 版本满足要求，跳过升级。"
fi

# ---------- 2.2 升级 Python 到 3.12（版本 < 3.12 才执行，pyenv 方式） ----------
if ! ver_ge "$PYTHON_VER" "3.12"; then
  echo "[5/9] 升级 Python 到 3.12（pyenv）..."
  sudo apt update
  sudo apt install -y make build-essential libssl-dev zlib1g-dev libbz2-dev \
    libreadline-dev libsqlite3-dev wget curl llvm libncursesw5-dev xz-utils \
    tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev

  # 安装 pyenv（如已安装则跳过）
  if [ ! -d "$BASE_DIR/.pyenv" ]; then
    curl https://pyenv.run | bash
  fi

  # 写入 ~/.bashrc（已存在则不重复添加）
  if ! grep -Fq 'pyenv init' "$BASE_DIR/.bashrc" 2>/dev/null; then
    cat >> "$BASE_DIR/.bashrc" <<'EOF'

# ---- pyenv（Zephyr 环境搭建脚本自动添加）----
export PATH="$HOME/.pyenv/bin:$PATH"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"   # 可选
EOF
    echo "  已向 ~/.bashrc 添加 pyenv 配置"
  fi

  # 在当前脚本进程内生效
  export PATH="$BASE_DIR/.pyenv/bin:$PATH"
  eval "$(pyenv init -)"

  # 安装并设为全局默认（-s：已安装则跳过）
  pyenv install 3.12.5 -s
  pyenv global 3.12.5
  PYTHON_VER=$(python3 --version | awk '{print $2}')
  echo "  Python 已升级到：$PYTHON_VER"
else
  echo "[5/9] Python 版本满足要求，跳过升级。"
fi

# DTC 没有提供升级步骤，版本过低只给警告
if ! ver_ge "$DTC_VER" "1.4.6"; then
  echo "[警告] DTC 版本低于 1.4.6（当前 $DTC_VER），后续编译报错时请升级 DTC。"
fi

echo "============================================================"
echo " Zephyr 环境搭建（第三部分：获取 Zephyr 并安装依赖）"
echo "============================================================"

# ---------- 3.1 创建虚拟环境 ----------
echo "[6/9] 创建 Python 虚拟环境..."
if [ ! -d "$WORKSPACE/.venv" ]; then
  python3 -m venv "$WORKSPACE/.venv"
  echo "  已创建 $WORKSPACE/.venv"
else
  echo "  虚拟环境已存在，跳过。"
fi
source "$WORKSPACE/.venv/bin/activate"

# ---------- 3.2 / 3.3 安装 west 并获取 Zephyr 源码 ----------
echo "[7/9] 安装 west 并获取 Zephyr 源码..."
pip install west

# 首次运行 west 会询问是否启用统计（analytics），先提醒避免卡住
echo "  [提示] 如果出现 'Do you want to enable west analytics' 之类的询问，直接回车（或输入 n）即可"

if [ ! -d "$WORKSPACE/.west" ]; then
  west init -m "$MANIFEST_URL" "$WORKSPACE"
  echo "  west 已初始化"
else
  echo "  west 已初始化，跳过 init。"
fi
cd "$WORKSPACE"

case "$UPDATE_MODE" in
  full)
    echo "  拉取全部仓库（west update，首次较慢）..."
    west update
    ;;
  nsing)
    echo "  只拉取 nsing 驱动库（west update hal_nsing）..."
    west update hal_nsing
    ;;
  *)
    echo "  已配置跳过拉取（可稍后手动执行 west update）"
    ;;
esac

# ---------- 3.4 / 3.5 Python 依赖与 CMake 导出 ----------
echo "[8/9] 安装 Zephyr Python 依赖并导出 CMake 包..."
case "$UPDATE_MODE" in
  skip)
    echo "  [跳过] UPDATE_MODE=skip 且尚未拉取仓库，Python 依赖稍后手动执行："
    echo "  west update && west packages pip --install && west zephyr-export"
    ;;
  *)
    west packages pip --install
    pip install --upgrade pip
    west zephyr-export
    ;;
esac

# ---------- 3.6 / 3.7 安装 Zephyr SDK 并配置环境变量 ----------
echo "[9/9] 安装 Zephyr SDK（下载约 1GB，视网速可能需要十几分钟）..."
SDK_DIR=$(ls -d "$BASE_DIR"/zephyr-sdk* 2>/dev/null | sort -V | tail -n1 || true)
if [ -z "$SDK_DIR" ]; then
  cd "$WORKSPACE/zephyr"
  west sdk install
  SDK_DIR=$(ls -d "$BASE_DIR"/zephyr-sdk* 2>/dev/null | sort -V | tail -n1 || true)
fi
if [ -z "$SDK_DIR" ]; then
  echo "[错误] 未找到 Zephyr SDK，请手动检查 west sdk install 是否成功。"
  exit 1
fi
echo "  Zephyr SDK 位于：$SDK_DIR"

# 写入环境变量（.profile 和 .bashrc，已存在则不重复添加）
if ! grep -Fq "ZEPHYR_SDK_INSTALL_DIR=$SDK_DIR" "$BASE_DIR/.profile" 2>/dev/null; then
  echo "export ZEPHYR_SDK_INSTALL_DIR=$SDK_DIR" >> "$BASE_DIR/.profile"
fi
if ! grep -Fq "ZEPHYR_SDK_INSTALL_DIR=$SDK_DIR" "$BASE_DIR/.bashrc" 2>/dev/null; then
  echo "export ZEPHYR_SDK_INSTALL_DIR=$SDK_DIR" >> "$BASE_DIR/.bashrc"
fi

echo ""
echo "============================================================"
echo " 环境搭建完成！"
echo " - 安装根路径：$BASE_DIR"
echo " - Zephyr 工作区：$WORKSPACE"
echo " - Zephyr SDK：$SDK_DIR"
echo ""
echo " 下一步："
echo " 1) 重新打开终端（或执行 source ~/.bashrc 使环境变量生效）"
echo " 2) 验证 SDK 变量：bash -lc 'echo \$ZEPHYR_SDK_INSTALL_DIR'"
echo " 3) 激活环境并编译第一个程序："
echo "      source $WORKSPACE/.venv/bin/activate"
echo "      west build -b n32g45xml_stb -p zephyr/samples/basic/blinky"
echo "============================================================"
