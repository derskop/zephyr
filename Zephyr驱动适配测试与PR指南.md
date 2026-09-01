# Zephyr 驱动适配测试与 PR 提交流程指南（新手向）

> 配套文档：[N32G45X-验证测试方案.md](./N32G45X-验证测试方案.md)（L0-L3 测什么）
> 本文档：怎么测、怎么组织代码、怎么提交才能过 CI 被合并

---

## 目录

- [一、Zephyr 测试体系三件套](#一zephyr-测试体系三件套)
- [二、官方要求总览（PR 能过的底线）](#二官方要求总览pr-能过的底线)
- [三、测试工程目录结构规范](#三测试工程目录结构规范)
- [四、tests.yaml 字段详解（用真实例子）](#四testsyaml-字段详解用真实例子)
- [五、本地自检：提交前必跑的命令](#五本地自检提交前必跑的命令)
- [六、方案文件 L0-L3 落地对照](#六方案文件-l0-l3-落地对照)
- [七、硬件测试如何在 CI 中存活](#七硬件测试如何在-ci-中存活)
- [八、commit 与 PR 检查清单](#八commit-与-pr-检查清单)
- [九、实战踩坑记录](#九实战踩坑记录)
- [十、提交前最后验收表](#十提交前最后验收表)

---

## 一、Zephyr 测试体系三件套

| 组件 | 作用 | 新手要知道的 |
|---|---|---|
| **ztest** | 断言测试框架（`zassert_*`、`ZTEST()`） | 测试代码里用它写断言，不用 printf 判断 |
| **twister** | 测试调度器：扫描 `tests/` 下所有测试目录，按 `tests.yaml` 构建/运行 | CI 和本地都用它；解析失败 = PR 直接挂 |
| **CI（GitHub Actions）** | 每个 PR 自动跑 checkpatch/gitlint/clang-format/twister 等 | "All checks have passed" 才能合并 |

工作流一句话：**用 ztest 写断言 → 用 tests.yaml 告诉 twister 怎么跑 → 本地用 twister 验证 → 过 checkpatch/gitlint → 提交 PR**。

---

## 二、官方要求总览（PR 能过的底线）

来源：`zephyr/doc/contribute/guidelines.rst`、`contributor_expectations.rst`、`doc/develop/twister/index.rst`、`doc/develop/test/ztest.rst`、[Zephyr Contribution Guidelines](https://docs.zephyrproject.org/latest/contribute/guidelines.html)

### 2.1 CI 必查项（少一个过不了）

| 检查 | 检查什么 | 本地怎么提前验证 |
|---|---|---|
| **Gitlint** | commit 消息格式（见 §8） | `gitlint --commits origin/main..HEAD` |
| **Checkpatch (Coding Style)** | 代码风格（行宽 100、花括号、空行等） | `git diff --cached \| scripts/checkpatch.pl -` |
| **ClangFormat** | .c/.h 代码格式 | `clang-format -i <file>` |
| **Kconfig** | Kconfig 语法/格式/符号存在性 | `scripts/ci/check_compliance.py` |
| **Devicetree** | binding YAML 可解析、dts 语法 | 同上（含 DevicetreeBindings/DevicetreeLinting） |
| **License** | SPDX 头（REUSE 检查） | `scripts/ci/check_compliance.py` |
| **Identity/Emails** | Signed-off-by 与 git 作者一致 | 提交前自查 |
| **Integration Tests (twister)** | 受改动影响的测试全绿 | `west twister -T <你的测试目录>` |
| **TestMetadataFilename** | 新测试必须是 `tests.yaml`（旧名 sample.yaml/testcase.yaml 拒绝） | 用对文件名即可 |

> 偷懒方式：本地一次跑完全部检查 —— `./scripts/ci/check_compliance.py -c <commit range>`（它是上面各检查的封装）。

### 2.2 硬性规则（零容忍）

- 每个 commit 必须有 `Signed-off-by: 你的真名 <真实邮箱>`（`git commit -s`），**邮箱必须与 git Author 一致**；不能用化名。
- commit title 格式 `[area]: [summary]`，一行 ≤72 字符，**body 必须非空**（写 what/why/how-tested），每行 ≤75 字符。
- 不允许 fixup commit 和 merge commit。
- 每个 commit 必须能独立构建（可 bisect）。
- 文件顶部必须 SPDX：`SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors` + `SPDX-License-Identifier: Apache-2.0`。
- 新功能/修改必须带测试；修 bug 的测试要求"修复前 FAIL、修复后 PASS"。
- 新增驱动要进 release notes（`doc/releases/release-notes-4.x.rst` 的 New Drivers 一节，`* :dtcompatible:` 条目 + PR 号）。

---

## 三、测试工程目录结构规范

测试放在 `zephyr/tests/<子系统>/<名字>/`。以本项目为例：

```
zephyr/tests/drivers/gpio/nsing_l3/
├── CMakeLists.txt          # 构建定义（顺序很重要，见下）
├── Kconfig                 # 测试自己的 config（可选）
├── prj.conf                # CONFIG_ZTEST=y 等
├── tests.yaml              # ★ twister 的入口，必须有
├── src/
│   └── main.c              # ztest 用例
└── boards/                 # 板级 overlay（可选，测试用到板上资源时放这）
    └── n32g45xml_stb.overlay
```

### 3.1 CMakeLists.txt（`find_package` 必须在 `project` 之前）

```cmake
cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(gpio_nsing_l3)

target_sources(app PRIVATE src/main.c)
```

### 3.2 prj.conf

```ini
CONFIG_ZTEST=y
```

被测驱动按需开启（`CONFIG_GPIO=y` 等）。**测试应用不用写 main()**，框架自动提供 `test_main`。

### 3.3 src/main.c —— ztest 规范

- 用例函数名必须以 **`test_`** 开头，带 doxygen 注释。
- 用 `ZTEST(suite_name, test_name)` 注册，`ZTEST_SUITE(suite, NULL, NULL, NULL, NULL, NULL)` 声明套件。
- 断言用 `zassert_*`（失败即 FAIL）、`zexpect_*`（失败继续跑，最后统一 FAIL）、`zassume_*`（失败即 SKIP）。
- **平台不支持时必须显式跳过**：`ztest_test_skip()` 或 `Z_TEST_SKIP_IFDEF(CONFIG_x)`，不许静默什么都不报。
- 一个用例内多个小检查用 `zassert` 串起来即可；twister 会把每个 `ZTEST` 用例单独上报。

```c
static volatile uint32_t irq_count_a;

ZTEST(gpio_nsing_l3, test_real_edge_count)
{
	zassert_ok(gpio_pin_configure(...));
	/* 驱动一个上升沿 */
	zassert_ok(gpio_pin_set(GPIOA_DEV, 0, 1));
	k_busy_wait(100);
	zassert_equal(irq_count_a, 1);   /* 断言回调真的被调用了 */
}

ZTEST_SUITE(gpio_nsing_l3, NULL, NULL, NULL, NULL, NULL);
```

### 3.4 Kconfig（测试专属 config 才需要）

```kconfig
mainmenu "GPIO NSing L3 Test"

source "Kconfig.zephyr"

config TEST_NSING_L3_EDGES
	int "number of edges to drive"
	default 100
```

命名：测试用 `TEST_` 前缀（驱动用 `{驱动类型}_{驱动名}`）。

---

## 四、tests.yaml 字段详解（用真实例子）

官方全文：`zephyr/doc/develop/twister/index.rst`。**我们的 4 个套件就是活教材**（`tests/drivers/gpio/nsing*`、`tests/drivers/pinctrl/nsing*`）。

```yaml
# tests/drivers/gpio/nsing_l3/tests.yaml
tests:
  drivers.gpio.nsing_l3:          # 场景名：必须"点分三段"风格，全局唯一
    tags:                         # ★ 必填
      - drivers
      - gpio
    platform_allow:               # 只能在这些平台跑（语义见 §7）
      - n32g45xml_stb
    vendor_allow:                 # ★ 必须数组形式，写标量会直接解析报错
      - nsing
```

常用字段速查：

| 字段 | 类型 | 含义 |
|---|---|---|
| `tags` | list | ★ 必填。CI 按 tag 筛测试；惯例：`drivers` + 驱动名 |
| `harness` | str | 默认 `test`；ztest 测试显式写 `harness: ztest` 更明确 |
| `platform_allow` | list | 只在这些平台**能跑**（真功能限制） |
| `integration_platforms` | list | 想省 CI 时间时用这个（配合 `--integration`），**别用 platform_allow 省时间** |
| `depends_on` | list | 板子必须有该 feature（如 `gpio`、`i2c`），对不上自动跳过 |
| `filter` | str | 表达式：`ARCH`, `PLATFORM`, `CONFIG_*`, `dt_compat_enabled("...")` 等 |
| `build_only` | bool | 只构建不运行（用于驱动能正确编译的验证，**不能用来验证功能**） |
| `extra_configs` | list | 追加配置，可命名空间：`platform:qemu_x86:CONFIG_X=y` |
| `timeout` | int | 默认 60s，超过自动杀 |
| `min_ram` / `min_flash` | int | 板子容量不够则跳过 |
| `slow` | bool | 需 `--enable-slow` 才跑 |
| `arch_allow` / `arch_exclude` / `toolchain_allow` | list | 架构/工具链过滤（`vendor_allow` 同理） |
| `common:` 顶层块 | — | 多个场景共享 tags/filter/harness |

---

## 五、本地自检：提交前必跑的命令

### 5.1 测试能否被 twister 解析（最容易挂的一步）

```bash
cd zephyr
./scripts/twister --list-tests -T tests/drivers/gpio/nsing_l3
# 预期：列出用例名（drivers.gpio.nsing_l3.gpio_nsing_l3.real_edge_count ...）+ "2 total."
```

**报错即 PR 必挂**。本项目真实踩过两个解析错误：
- 板卡 `boards/nsing/n32g45xml_stb/n32g45xml_stb.yaml` 里写了 `flash_runner: jlink` → 不在 twister platform schema 允许属性里（`scripts/schemas/twister/platform-schema.yaml`），**全仓库 twister 直接崩溃**。已删（runner 由 board.cmake 的 `board_runner_args(jlink ...)` 配置）。
- `vendor_allow: nsing` 写成了标量 → schema 要求数组。已改为：
  ```yaml
  vendor_allow:
    - nsing
  ```

### 5.2 构建 + 运行

```bash
# 构建（可跑平台）
west build -b n32g45xml_stb -d build-test tests/drivers/gpio/nsing_l3

# twister 跑（可模拟平台）
west twister -p native_sim -T tests/drivers/pinctrl/api
# 或本板 twister（需要 --device-testing 接硬件，见 §7）

# 全量合规检查（checkpatch + gitlint + Kconfig + DTS + License...）
./scripts/ci/check_compliance.py -c origin/main..HEAD
```

---

## 六、方案文件 L0-L3 落地对照

| 层 | 测试位置 | 运行方式 | CI 覆盖 | 状态 |
|---|---|---|---|---|
| L0 静态 | 构建即验证 | `west build` | ✅ CI 自动构建 | 完成 |
| L1 框架级 | `tests/drivers/pinctrl/api`（官方）+ native_sim | `west twister -p native_sim` | ✅ CI 直接跑 | 完成 |
| L2 真机断言 | `tests/drivers/gpio/nsing`（3 用例）、`tests/drivers/pinctrl/nsing`（5 用例） | 板载 `west build && west flash` + 串口看 SUITE PASS | ⚠️ 硬件平台 CI 跳过，靠 L1 + 构建兜底 | 完成 |
| L3 功能级 | `tests/drivers/gpio/nsing_l3`（2 用例）、`tests/drivers/pinctrl/nsing_l3`（1 用例） | 同上 + PA0→PA2 跳线 | ⚠️ 同上 | 完成 |

**核心事实**：L2/L3 的 `platform_allow: n32g45xml_stb` 意味着 CI 的 twister 不会在模拟平台上构建它（平台不存在）。所以：
1. 硬件测试进 PR **不会拖垮 CI**（直接被跳过）；
2. 但**驱动逻辑的正确性必须靠 L1（native_sim）或代码审查兜底**——这就是 L1 层存在的意义；
3. 想让驱动测试在 CI 里真跑，唯一办法是写一个不依赖硬件的模拟/仿真测试（native_sim + 驱动 stub），或让测试逻辑可平台无关（例如 pinctrl 的纯编码断言）。

---

## 七、硬件测试如何在 CI 中存活

官方规则（`doc/develop/twister/index.rst` 原文要点）：

> `platform_allow`："Do not use this option to limit testing or building in CI due to time or resource constraints, this option should only be used if the test or sample can only be run on the allowed platform and nothing else."

> `integration_platforms`："limits the scope to the listed platforms when twister is invoked with the --integration option. Use this instead of platform_allow if the goal is to limit scope due to timing or resource constraints."

翻译：
- **真·只能在这板跑**（接了跳线、真硬件中断）→ `platform_allow: [n32g45xml_stb]`（我们 L2/L3 的现状，正确）。
- **只是嫌平台太多跑得慢** → 用 `integration_platforms`，不要用 platform_allow。

硬件依赖测试的官方姿势：
1. `harness: ztest` + 接硬件用 twister 的 `--device-testing --device-serial /dev/ttyUSB0 -p n32g45xml_stb`；
2. 声明外部设备依赖用 `harness_config: fixture:`（如 `i2c_hts221`）；
3. **模拟外设**（`zephyr,fake-*` 节点 + FFF mock 库）是官方推荐的方向——"Emulators for off-chip peripherals are an effective way to test driver APIs"（contributor_expectations.rst），这能让测试真正进 CI。

---

## 八、commit 与 PR 检查清单

### 8.1 commit message（gitlint 强制）

```
drivers: gpio: n32: fix irq enable path in interrupt_configure

The NVIC vector for EXTI lines was left un-enabled when a pin was
configured for interrupt, so interrupts never fired on real edges.

Signed-off-by: 你的真名 <you@example.com>
```

- title：`[area]: [summary]`，≤72 字符；area 例如 `drivers: gpio:`、`soc: nsing:`、`boards: nsing:`、`tests:`。
- body 必须非空（一行也行），每行 ≤75 字符，说明 what/why/how-tested。
- `Signed-off-by` 最后一行（`git commit -s` 自动加）。
- 本地验证：`gitlint --commits origin/main..HEAD`。

### 8.2 拆分与历史

- 一个 PR 拆成多个逻辑 commit（驱动 / 测试 / 文档分开），每个 commit 独立可构建。
- 提交前 `git rebase -i` 合并中间过程，不留 fixup。
- 大功能/新 API 先发 RFC 讨论。

### 8.3 PR 描述

写清楚：改了什么、为什么、**怎么测的**（测试命令 + 串口输出片段）、硬件需求（板子 + 跳线）。

### 8.4 评审与合并门槛

- 至少 2 个 approval（含 assignee）；最少 2 个工作日评审期（琐碎改动 4 小时）。
- 硬件支持改动（driver/SoC/boards）要求 merger 与提交者**不同组织**（四眼原则）。

---

## 九、实战踩坑记录（本项目真实发生）

1. **ztest 用例按字母序执行**（cross < real）——前一个用例留下的寄存器状态会污染后一个。测试间要自恢复（如 `test_real_edge_count` 开头把 AFIO 线源从 PB 改回 PA）。
2. **SWIE 触发绕过真实边沿**——EXTI 的 `SWIE` 直接置 PEND 位、不经引脚采样/边沿检测器，所以"SWIE 触发成功"**不能**证明真实边沿路径工作。真实验证必须驱动引脚电平。
3. **`PINCTRL_STATE_*` 自定义宏要自己定义**（`PINCTRL_STATE_DRIVE=PINCTRL_STATE_PRIV_START`），且要保证共享源文件（如 `tests/drivers/pinctrl/common/test_device.c`）也看得到——用 `target_compile_definitions` 注入比在源文件里定义更稳。
4. **pinctrl 的 GPIO_OUT 默认 slew-rate 是 2MHz**（CNFMODE=0x1），和 GPIO 驱动默认 50MHz（0x3）不同——断言前先查驱动语义，别想当然。
5. **`vendor_allow` / `platform_allow` 必须是数组**（YAML list），标量直接让 twister 解析失败。
6. **板卡 yaml 不能有 schema 外字段**（如 `flash_runner`）——runner 在 board.cmake 配置，不是板卡 yaml。
7. **EXTI PEND 是单锁存器**：ISR 清掉前到达的新边沿会丢。测试驱动边沿必须"等 ISR 处理完再驱动下一个"（wait_count 串行化）。
8. **硬件测试的断言失败会跳过后面的恢复代码**（console "hang" 假象）——失败分支里留恢复路径，或让串口捕获窗口覆盖测试全程。

---

## 十、提交前最后验收表

- [ ] 4 个测试目录 `twister --list-tests` 全部解析成功、无 ERROR
- [ ] 所有改动文件有 SPDX 头
- [ ] `clang-format -i` 跑过改动的 .c/.h
- [ ] 本地 `west build` 通过（相关 board + native_sim 至少一个）
- [ ] 硬件测试在板子上实跑：串口输出 `SUITE PASS`（附截图/日志到 PR）
- [ ] 拔跳线/改参数能制造 FAIL（证明测试真的在测硬件）
- [ ] `gitlint --commits origin/main..HEAD` 通过
- [ ] commit 有 Signed-off-by 且与 git 作者一致，无 fixup/merge
- [ ] `./scripts/ci/check_compliance.py -c <range>` 无 ERROR（本地跑一次）
- [ ] 新驱动已列入 release notes；文档同步更新
- [ ] PR 描述含：改动说明、测试方法、硬件需求

---

*参考：Zephyr 官方文档 `doc/develop/test/ztest.rst`、`doc/develop/twister/index.rst`、`doc/contribute/guidelines.rst`、`doc/contribute/style/*`、`scripts/ci/check_compliance.py`；网上版 [Contribution Guidelines](https://docs.zephyrproject.org/latest/contribute/guidelines.html)、[Contributor Expectations](https://docs.zephyrproject.org/latest/contribute/contributor_expectations.html)。*
