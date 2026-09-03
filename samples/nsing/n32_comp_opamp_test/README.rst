.. _n32_comp_opamp_test:

N32G45x COMP / OPAMP J-Link 自动测试
#####################################

基于 Nations N32G45x 的 COMP / OPAMP Zephyr 驱动硬件自动测试。

被测对象（board overlay 使能）
=============================

* ``comp1``: 正端 PA1 vs 负端 VREF1（内部 64 级定标器），NVIC IRQ 82
  - 校验：EN / INPSEL=PA1 / INMSEL=VREF1 / VREF1 定标器使能
  - API：``comparator_get_output`` / ``comparator_set_trigger``(NONE、RISING)
* ``opamp1``: 非反向内部 PGA，输入 PA1；运行期 ``opamp_set_gain(x4)``
  - 校验：EN / MOD=PGA / VMSEL=浮空 / PGAGAN=x4
* ``opamp4``: 内部跟随器，输入 PC3
  - 校验：EN / MOD=FOLLOW / VMSEL=浮空

测试 app 先把驱动在寄存器上写出的配置读回来与期望比对，把每项
PASS/FAIL 写入 RAM 结构 ``autotest_results``，然后进入空闲；由 J-Link
脚本 halt 后读取结果结构与 COMP/OPAMP 寄存器做最终断言。

运行步骤
========

.. code-block:: console

  # 1) 环境（一次性）
  source /home/lee/zephyrproject/.venv/bin/activate
  export ZEPHYR_SDK_INSTALL_DIR=/home/lee/zephyr-sdk-1.0.1

  # 2) 编译（在 west 工作区内）
  cd /home/lee/zephyrproject
  west build -b n32g45xml_stb -d build_n32test \
      zephyr/samples/nsing/n32_comp_opamp_test -p never

  # 3) J-Link 自动测试（烧录→运行→halt→读寄存器→断言）
  python3 zephyr/samples/nsing/n32_comp_opamp_test/run_jlink_test.py

脚本退出码：0 = PASS，非 0 = FAIL。

参考寄存器
==========

+------------------+------------+-------------------------------+
| 外设寄存器       | 地址       | 说明                          |
+==================+============+===============================+
| COMP1_CTRL       | 0x40002410 | COMP1 控制/状态               |
+------------------+------------+-------------------------------+
| COMP_VREFSCL     | 0x40002494 | VREF1/2 定标器使能与微调       |
+------------------+------------+-------------------------------+
| OPAMP1_CS1       | 0x40002000 | OPAMP1 控制/状态               |
+------------------+------------+-------------------------------+
| OPAMP4_CS4       | 0x40002030 | OPAMP4 控制/状态               |
+------------------+------------+-------------------------------+
