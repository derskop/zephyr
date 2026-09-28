.. _n32_i2c_at24_test:

N32G45x I2C AT24 EEPROM 测试
############################

基于 Nations N32G45x 的 Zephyr ``i2c`` 驱动 + ``eeprom`` 子系统 +
``atmel,at24`` 驱动的 AT24 系列 EEPROM 硬件测试程序。

接线
====

==============  ==================  ==================================
信号            引脚                说明
==============  ==================  ==================================
I2C1_SCL        PB6                 板上 I2C1 默认复用
I2C1_SDA        PB7                 板上 I2C1 默认复用
A2              GND                 7 位从机基地址 = 0x50
WP              GND                 允许写入（接高则写保护）
VCC / GND       3.3V / GND          不要接 5V
==============  ==================  ==================================

I2C 总线需要上拉电阻（典型 4.7 kΩ）。PB6/PB7 配置为开漏输出，
如果模块上已经带上拉，则无需外接。

芯片容量：先跑一遍再改设备树
============================

AT24C04 与 AT24C08 在总线上**长得一模一样**：都是 8 位字地址、16 字节
页，高位地址都靠从机地址携带。唯一的区别是容量：

================  ==========  ====================  ============
型号               容量        应答的从机地址        256 字节页数
================  ==========  ====================  ============
AT24C04           512 B       0x50, 0x51            2
AT24C08           1 KiB       0x50 ~ 0x53           4
================  ==========  ====================  ============

所以这两者**无法靠扫描全片来区分容量之外的任何东西**，而 ``size`` 填错
就会去访问不存在的页，表现为写整片时一片 ``ACKFAIL``。

测试 1 会扫描 0x50~0x57 并打印哪些地址应答、据此判断型号：

.. code-block:: console

   bus scan: 0x50 ACKs (word 0x00 -> 0x5a)
   bus scan: 0x51 ACKs (word 0x00 -> 0x6b)
   bus scan: 2 address(es) answered -> 512-byte AT24C04

本仓库这块板子只应答 0x50 / 0x51，因此按 **512 字节**描述。
如果换上真正的 AT24C08，把 ``size`` 改成 ``1024`` 即可，其余不用动。

设备树建模
==========

**本样例自身不带设备树**，``&i2c1`` 下的 EEPROM 节点就在板级文件
``boards/nsing/n32g45xml_stb/n32g45xml_stb.dts`` 里：

.. code-block:: dts

   &i2c1 {
           scl-gpios = <&gpiob 6 (GPIO_OPEN_DRAIN | GPIO_ACTIVE_HIGH)>;
           sda-gpios = <&gpiob 7 (GPIO_OPEN_DRAIN | GPIO_ACTIVE_HIGH)>;

           at24: eeprom@50 {
                   compatible = "atmel,at24";
                   reg = <0x50>;
                   size = <512>;        /* AT24C08 用 1024 */
                   pagesize = <16>;
                   address-width = <8>;
                   timeout = <20>;
                   status = "okay";
           };
   };

板子的 ``aliases`` 里还有 ``eeprom-0 = &at24;``，样例靠
``DT_ALIAS(eeprom_0)`` 找设备 —— 因为它已经是板级固件配置的一部分，
其它样例也能直接用，不必各自再写一遍 overlay。

页选位在从机地址里，Zephyr 的 at24 驱动正是按“8 位字地址 + 页选择进入
从机地址”实现的，因此上面把它描述成**单个设备**而不是四块 256 字节的
设备。``address-width`` 必须是 **8**，页大小必须是 **16**；这两个参数填错
会导致跨页写入回绕或读回数据错乱。``size`` 是唯一体现容量的参数。

测试内容
========

1. 总线扫描：访问 0x50~0x57，打印应答的地址并据此判定型号；再逐个访问
   本芯片应有的页。
2. 几何参数：``eeprom_get_size()`` 应等于设备树里的 ``size``。
3. 掉电计数器：读取上次运行写入的记录。
4. 跨页写入：在页边界处写入 32 字节并读回比对，验证驱动自动分页。
5. 全片读写：写入整片（图案里混入高位偏移），读回逐字节比对，并对每页
   首字节抽样 —— 页选位译码错误会在这里暴露。
6. 越界保护：越过芯片末尾的读写应返回 ``-EINVAL``。
7. 裸 I2C 对照：绕过 ``eeprom`` API，直接用 ``i2c_write()`` /
   ``i2c_write_read()`` 按手算的从机地址 + 字地址在低页和高页各写一个
   字节再读回，与 ``eeprom_read()`` 的结果比对。第 1~6 项走的是驱动自己
   的偏移翻译，读写同时出错时它们可能一起"自洽"地通过；这一项切断闭环。
8. 逐字节走片：对全部 512 个偏移逐个"写一个字节、立刻读回该字节"，确认
   每一个位置都能正常读写，而不只是某几个挑出来的偏移。
9. 掉电计数器更新：写回并读回校验，复位一次计数 +1。

第 5 和第 8 项是破坏性的，会在写之前备份掉电计数器、写完后恢复，否则第 9
项每次拿到的都是被冲掉的记录，计数永远停在 1。

每项检查打印 ``PASS`` / ``FAIL``，最后打印检查总数与失败数，以及
``ALL TESTS PASSED`` 或 ``TESTS FAILED``。

驱动侧的告警走 logging API（例如总线被从机拉住、驱动不得不手动打时钟
解除时），而不是 ``printk``。``prj.conf`` 里开了
``CONFIG_LOG=y`` + ``CONFIG_LOG_MODE_MINIMAL=y`` + ``CONFIG_I2C_LOG_LEVEL_WRN=y``；
**如果不打开这三项，那些告警会被整个编译掉，驱动出问题时是静默的**。
选 minimal 模式是因为它把 ``LOG_*`` 直接接到 ``printk``，不额外起日志线程、
不占环形缓冲，驱动输出的行也就和样例自己的输出保持同样顺序。

总线恢复
========

样例开 ``CONFIG_I2C_BUS_RECOVERY=y``。如果 MCU 挂在一次读的中途被复位，
从机可能一直把 SDA 拉低等它永远等不到的时钟，此后外设一直报 BUSY、任何
传输都发不出 START。此时驱动会把这根线用 GPIO 手动打最多 9 个时钟再补一个
STOP，把从机放回空闲，然后重新初始化外设 —— 而不是一路超时到重启板子为止。

这需要 ``scl-gpios`` / ``sda-gpios``，板级文件
``boards/nsing/n32g45xml_stb/n32g45xml_stb.dts`` 的 ``&i2c1`` 里已经给出
（和 ``pinctrl-0`` 是同一对引脚，开漏）：

.. code-block:: dts

   &i2c1 {
           scl-gpios = <&gpiob 6 (GPIO_OPEN_DRAIN | GPIO_ACTIVE_HIGH)>;
           sda-gpios = <&gpiob 7 (GPIO_OPEN_DRAIN | GPIO_ACTIVE_HIGH)>;
   };

没有这两个属性时驱动**依然能编译能跑**，只是总线卡住时只能重置外设，
``i2c_recover_bus()`` 返回 ``-ENOSYS``。这两个属性是 I2C 控制器节点的
通用属性（``st,stm32-i2c``、``espressif,esp32-i2c`` 等都支持），所以
换成别的 I2C 从机、别的板子也一样适用。

样例中**没有任何 ``k_sleep``**：AT24 在内部写周期（tWR，最大 5 ms）期间不
应答自己的地址，而 ``eeprom_at24_read()`` / ``eeprom_at24_write()`` 本身
就会在 ``timeout`` 毫秒内反复重发直到从机应答，所以调用方不需要补延时。
设备树里 ``timeout`` 给的是 20（而不是贴着上限的 5），留出余量。

编译与运行
==========

.. code-block:: console

   source /home/hu/zephyrproject/.venv/bin/activate
   export ZEPHYR_SDK_INSTALL_DIR=/home/hu/zephyr-sdk-1.0.1

   west build -b n32g45xml_stb -d build-at24 \
       zephyr/samples/nsing/n32_i2c_at24_test -p always

   west flash -d build-at24

.. warning::

   测试在 ``main()`` 里一次跑完就结束，全部输出集中在**上电后约 1.5 秒内**。
   之后再打开串口监视器只会看到空白 —— 输出已经过去了，不是什么都没跑。
   请**先开监视器占住串口，再烧录或按复位键**：

   .. code-block:: console

      # 终端 1：先开监视器
      python -m serial.tools.miniterm /dev/ttyACM0 115200
      # 或 screen /dev/ttyACM0 115200

      # 终端 2：烧录（烧完自动复位，输出被终端 1 接住）
      west flash -d build-at24

串口 USART1（115200 8N1）输出示例：

.. code-block:: console

   === N32G45x I2C AT24 EEPROM test ===
   I2C1: SCL=PB6, SDA=PB7, base address=0x50, 512 bytes

   I2C bus "i2c@40005400" ready
   EEPROM "eeprom@50" ready

   1. I2C presence of the 2 chip pages
     bus scan: 0x50 ACKs (word 0x00 -> 0x5a)
     bus scan: 0x51 ACKs (word 0x00 -> 0x6b)
     bus scan: 2 address(es) answered -> 512-byte AT24C04
     [PASS] slave 0x50 (offset 0x000) -> 0x5a
     [PASS] slave 0x51 (offset 0x100) -> 0x6b
     [PASS] all chip pages answer on the bus
   ...
   7. Raw I2C ground truth (driver bypassed)
     offset 0x010 -> slave 0x50, word 0x10
     [PASS] both match the byte that was written
   ...
   8. Byte-wise walk over all 512 offsets
     512 offsets visited in 976 ms (0 write errors, 0 read errors)
     [PASS] every offset reads back what was just written
   === 31 checks, 0 failed ===
   ALL TESTS PASSED

（第 1 项扫描时回读到的字节是**上一次运行**残留在芯片里的内容，所以每次
不同 —— 出厂新片是 0xFF，跑过几轮之后是上轮图案的某个字节。这里的
``0x5a`` / ``0x6b`` 只是某一次的实拍值，不是固定期望值。）

复位板子后重新运行，第 3/9 项的 boot count 应逐次递增，说明数据确实
保存在 EEPROM 里（第 5、8 项会重写整片，所以它们写完要把记录恢复回来）：

.. code-block:: console

   previous boot count: 1        <- 第一次
   previous boot count: 2        <- 复位后
   previous boot count: 3

故障排查
========

* **slave 0x50 就 FAIL、device not ready**：检查 PB6/PB7 接线、
  上拉电阻与 A2 是否接地。
* **只有 0x50 响应，0x51 起 FAIL**：芯片是 AT24C02（256 字节）而不是
  AT24C04/C08，或者 A2 接的不是 GND。
* **0x52/0x53 FAIL、写整片报 ACKFAIL**：芯片只有 512 字节，把板级 dts 的
  ``size`` 改成 ``512``\ （反之如果是 AT24C08 却写成 512，会在偏移 512
  之后读到回绕的数据）。
* **全片读写 FAIL 且错位 256 字节的整数倍**：``address-width`` 或
  ``pagesize`` 填错。
* **读能过、写全 FAIL**：WP 引脚被拉高（写保护），或电源电压不足。
* **写成功了但立刻读回来是旧数据**：EEPROM 的写周期 tWR 最长 5 ms，这期间
  芯片不应答自己的地址。驱动会重发直到应答，但窗口由设备树的
  ``timeout`` 决定 —— 贴着 5 填会在最后一刻放弃，留点余量（样例用 20）。
