# 修改 VS Code 调试配置——保姆级教程

> 共 4 个文件。**哪些要动、哪些别动，都列在下面了。**
> 你只需要按顺序做：查 2 个值 → 改 1 个文件（tasks.json）→ 保存 → 编译 → F5。
>
> **固定项（全团队统一，永远不要改）**：
> - SDK 版本：`zephyr-sdk-1.0.1`（目录名不能变）
> - 板子：`n32g45xml_stb`　芯片：`N32G457ME`
> - 构建目录：`build`
>
> **每台机器不同的（就这 2 个）**：SDK 所在位置、west 所在位置
> **团队会变的（就这 1 个）**：编译的应用（`zephyr/samples/basic/blinky`）

---

## 第 0 步：查 2 个值（打开终端：`Terminal → New Terminal` 或 `Ctrl + \``）

### 0.1 SDK 路径（你 SDK 解压的位置）

```bash
ls -d ~/zephyr-sdk-*
```

- 有输出（如 `/home/nsing/zephyr-sdk-1.0.1`）→ 记下这个**完整路径**
- 没输出 → 问团队你的 SDK 解压在哪

### 0.2 west 路径

```bash
which west
```

- 有输出（如 `/home/nsing/zephyrproject/.venv/bin/west`）→ 记下**去掉最后的 `/west`**，即 `/home/nsing/zephyrproject/.venv/bin`
- 没输出 → 执行 `source ~/zephyrproject/.venv/bin/activate` 再试；还不行就问团队

> 如果 0.1 没输出，也可能是 SDK 不在 home 下：用 `ls -d /*/zephyr-sdk-* 2>/dev/null` 或者直接问团队你的 SDK 在哪。

---

## 文件 1：tasks.json（编译/烧录命令）

**打开**：左侧文件树 → `.vscode` 文件夹 → `tasks.json`

### 1.1 编译命令（`"command"` 开头的那一行）

```json
"command": "export ZEPHYR_SDK_INSTALL_DIR=/home/nsing/zephyr-sdk-1.0.1 && /home/nsing/zephyrproject/.venv/bin/west build -b n32g45xml_stb zephyr/samples/basic/blinky -d build",
```

**改 2 处，其余字符一个都别动**：

| 改哪 | 原值 | 改成 |
|---|---|---|
| ① SDK 路径 | `/home/nsing/zephyr-sdk-1.0.1` | 你第 0.1 步记下的路径（如 `/home/zhangsan/zephyr-sdk-1.0.1`） |
| ② west 路径 | `/home/nsing/zephyrproject/.venv/bin` | 你第 0.2 步记下的路径（注意：`west` 的 `/west` 不要） |

改完示例（SDK=`/home/zhangsan/zephyr-sdk-1.0.1`，west=`/home/zhangsan/zephyrproject/.venv/bin`）：

```json
"command": "export ZEPHYR_SDK_INSTALL_DIR=/home/zhangsan/zephyr-sdk-1.0.1 && /home/zhangsan/zephyrproject/.venv/bin/west build -b n32g45xml_stb zephyr/samples/basic/blinky -d build",
```

> 为什么是绝对路径？VS Code 的任务是在后台 shell 里跑的，不读你的 .bashrc，所以环境变量（ZEPHYR_SDK_INSTALL_DIR）必须在这里直接写死，SDK 放哪就写哪。

### 1.2 烧录命令（`flash (J-Link)` 任务，`args` 里那一大串）

```json
"args": [
  "-lc",
  "printf 'r\nloadfile build/zephyr/zephyr.hex\nr\ng\nq\n' | JLinkExe -device N32G457ME -if SWD -speed 1000 -AutoConnect 1 -ExitOnError 1"
]
```

**无需修改**（`build`、`N32G457ME`、`SWD`、`1000` 都是固定项）。

---

## 换应用（团队会经常做，只改这一处）

改 `tasks.json` 编译命令里的**应用路径**，例如：

```json
west build -b n32g45xml_stb zephyr/samples/hello_world -d build
```

（把 `zephyr/samples/basic/blinky` 换成新应用路径，其余不动）

**改完必须清一次构建缓存**——west 会拒绝用旧应用留下的 build 目录，报错长这样：

> ERROR: Build directory ... is for application ... but source directory ... was specified

清缓存（在终端执行）：

```bash
rm -rf ~/zephyrproject/build
```

然后 `Ctrl+Shift+B` 重新编译（首次要几分钟）。之后 F5 调试、烧录、代码提示自动跟随新应用，**launch.json、c_cpp_properties.json 都不用动**。

---

## 文件 2：launch.json（F5 调试）

**打开**：`.vscode/launch.json`。

- **如果你的 SDK 在 zephyrproject 的上一级目录**（即 `/home/你的用户名/zephyr-sdk-1.0.1` 和 `/home/你的用户名/zephyrproject` 平级）→ **无需修改**（`${workspaceFolder}/../zephyr-sdk-1.0.1` 会自动找到它）
- **如果 SDK 不在那个位置** → 把两处 `"gdbPath"` 里的 `../zephyr-sdk-1.0.1` 改成你 SDK 的实际路径：

```json
"gdbPath": "${workspaceFolder}/../zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb",
```

改成：

```json
"gdbPath": "/你的SDK实际路径/zephyr-sdk-1.0.1/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-gdb",
```

> **Windows 电脑**：除此之外，还要把两处 `"serverpath": "/usr/bin/JLinkGDBServer"` 改成你 Windows 上 JLink 的安装路径（如 `C:\Program Files\SEGGER\JLink_V794\JLinkGDBServer.exe`，版本目录名看你装的版本）

## 文件 3：settings.json

**打开**：`.vscode/settings.json`。**无需修改**。

## 文件 4：c_cpp_properties.json（代码提示）

**打开**：`.vscode/c_cpp_properties.json`。

- SDK 在 zephyrproject 上一级 → **无需修改**
- 不在 → 把 `"compilerPath"` 里的 `../zephyr-sdk-1.0.1` 改成你 SDK 的实际路径（`compileCommands` 指向 `build/compile_commands.json`，编译一次后自动生成，不用管）

---

## 改完怎么验证（按顺序做）

1. 保存所有改过的文件（`Ctrl+S`）
2. 按 `Ctrl+Shift+B` 编译 —— 第一次要几分钟，看到 **Build finished** 就是成功
3. 插好 J-Link（SWD 四根线：GND/SWDIO/SWCLK/VCC）
4. 按 **F5** —— 看到停在 `main`（黄线停在 main 函数第一行）就是全部成功
5. 之后每次改代码：直接 F5（自动编译再调试），或 `Ctrl+Shift+B` 只编译

## 最容易犯的 3 个错

1. **改完没保存**就按 F5 —— 先 `Ctrl+S` 再调试
2. **换应用后没清 build 目录**就编译 —— 会报 "Build directory is for application ..."，先 `rm -rf ~/zephyrproject/build` 再编译
3. **tasks.json 里 SDK/west 路径照抄没改**就编译 —— 报错找不到文件时，先回文件 1 核对这两个路径
