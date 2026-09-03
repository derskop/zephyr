#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
N32G45x COMP / OPAMP register auto-test driven by SEGGER J-Link.

Flow:
  1. (optional) build the sample via west
  2. locate the "autotest_results" symbol in the ELF (nm)
  3. J-Link phase 1 : flash zephyr.hex, reset, run
  4. sleep, J-Link phase 2 : halt, read the result structure and the
     COMP / OPAMP registers, assert PASS/FAIL
  5. print a register dump and exit 0 on pass

Prerequisites (env):
  ZEPHYR_SDK_INSTALL_DIR  Zephyr SDK root
  PATH  must include <zephyrproject>/.venv/bin  (west) and J-Link is on PATH
"""
import argparse
import os
import re
import subprocess
import sys
import time

DEVICE = "N32G457ME"
IFACE = "SWD"
SPEED = 1000

COMP_BASE = 0x40002400
OPAMP_BASE = 0x40002000

# Peripheral registers read back from the target as evidence.
EVIDENCE = [
    ("COMP1_CTRL",  COMP_BASE + 0x10),   # comp1 positive=PA1, negative=VREF1
    ("COMP_INTEN",  COMP_BASE + 0x8C),
    ("COMP_INTSTS", COMP_BASE + 0x90),
    ("COMP_VREFSCL", COMP_BASE + 0x94),
    ("OPAMP1_CS1",  OPAMP_BASE + 0x00),  # non-inverting PGA, gain x4 after set_gain
    ("OPAMP4_CS4",  OPAMP_BASE + 0x30),  # follower
]

# Field expectations on the register evidence above.
def check_evidence(regs):
    """regs: dict addr -> value. Returns list of (ok, message)."""
    checks = []

    def get(name, addr):
        if addr not in regs:
            return None
        return regs[addr]

    def add(name, cond):
        checks.append((bool(cond), name))

    ctrl = get("COMP1_CTRL", COMP_BASE + 0x10)
    if ctrl is not None:
        add("COMP1_CTRL.EN set", ctrl & 0x01)
        add("COMP1_CTRL.INPSEL == PA1 (0)", ((ctrl >> 4) & 0x7) == 0)
        add("COMP1_CTRL.INMSEL == VREF1 (3)", ((ctrl >> 1) & 0x7) == 3)

    vref = get("COMP_VREFSCL", COMP_BASE + 0x94)
    if vref is not None:
        add("COMP_VREFSCL.VREF1EN set", vref & 0x01)

    cs1 = get("OPAMP1_CS1", OPAMP_BASE + 0x00)
    if cs1 is not None:
        add("OPAMP1_CS1.EN set", cs1 & 0x01)
        add("OPAMP1_CS1.MOD == PGA (2)", ((cs1 >> 1) & 0x3) == 0x2)
        add("OPAMP1_CS1.VMSEL == float (3)", ((cs1 >> 6) & 0x3) == 0x3)
        add("OPAMP1_CS1.PGAGAN == x4 (1)", ((cs1 >> 3) & 0x7) == 0x1)

    cs4 = get("OPAMP4_CS4", OPAMP_BASE + 0x30)
    if cs4 is not None:
        add("OPAMP4_CS4.EN set", cs4 & 0x01)
        add("OPAMP4_CS4.MOD == FOLLOW (3)", ((cs4 >> 1) & 0x3) == 0x3)
        add("OPAMP4_CS4.VMSEL == float (3)", ((cs4 >> 6) & 0x3) == 0x3)

    return checks


def run_jlink(cmds, timeout=90):
    """Run JLinkExe with the given command lines, return (rc, stdout)."""
    proc = subprocess.run(
        ["JLinkExe", "-device", DEVICE, "-if", IFACE, "-speed", str(SPEED),
         "-AutoConnect", "1", "-ExitOnError", "1"],
        input="\n".join(cmds) + "\n",
        capture_output=True, text=True, timeout=timeout)
    return proc.returncode, proc.stdout


def parse_mem32(stdout):
    """Extract {addr: value} from 'mem32 <addr> 1' outputs.

    J-Link Commander prints lines such as:
        J-Link>40002410 = 00020007
        J-Link>20000228 = 4E333254 00000001 00000001 00000001
    """
    regs = {}
    for m in re.finditer(r'J-Link>\s*([0-9A-Fa-f]{6,8})\s*=\s*([0-9A-Fa-f]{8})', stdout):
        regs[int(m.group(1), 16)] = int(m.group(2), 16)
    return regs


def get_symbol_addr(elf, name, nm):
    out = subprocess.run([nm, elf], capture_output=True, text=True).stdout
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[2] == name:
            return int(parts[0], 16)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", action="store_true", help="build via west first")
    ap.add_argument("--west-dir", default="/home/lee/zephyrproject",
                    help="west workspace root")
    ap.add_argument("--build-dir", default=None,
                    help="build directory (default <west-dir>/build_n32test)")
    ap.add_argument("--sleep", type=float, default=2.0,
                    help="seconds to let the firmware run before phase 2")
    args = ap.parse_args()

    west_dir = os.path.abspath(args.west_dir)
    build_dir = os.path.abspath(args.build_dir or os.path.join(west_dir, "build_n32test"))
    app = os.path.join(west_dir, "zephyr", "samples", "nsing", "n32_comp_opamp_test")
    hexf = os.path.join(build_dir, "zephyr", "zephyr.hex")
    elf = os.path.join(build_dir, "zephyr", "zephyr.elf")

    sdk = os.environ.get("ZEPHYR_SDK_INSTALL_DIR")
    if not sdk or not os.path.isdir(sdk):
        sys.exit("ZEPHYR_SDK_INSTALL_DIR is not set")
    nm = os.path.join(sdk, "gnu", "arm-zephyr-eabi", "bin", "arm-zephyr-eabi-nm")

    if args.build:
        env = dict(os.environ)
        env["PATH"] = os.path.join(west_dir, ".venv", "bin") + os.pathsep + env["PATH"]
        subprocess.run(["west", "build", "-b", "n32g45xml_stb", "-d", build_dir,
                        app, "-p", "never"], check=True, env=env, cwd=west_dir)

    addr = get_symbol_addr(elf, "autotest_results", nm)
    if addr is None:
        sys.exit("symbol autotest_results not found in %s" % elf)
    print("autotest_results @ 0x%08x" % addr)

    # ---- phase 1: flash, reset, run ---------------------------------
    rc, out = run_jlink(["connect", "loadfile " + hexf, "r", "g", "exit"])
    if rc != 0:
        print(out[-3000:])
        sys.exit("phase1 (flash/run) failed, rc=%d" % rc)
    time.sleep(args.sleep)

    # ---- phase 2: halt and read --------------------------------------
    cmds = ["connect", "halt"]
    for _n, a in EVIDENCE:
        cmds.append("mem32 0x%X 1" % a)
    # read the autotest_results structure (8 words)
    for i in range(8):
        cmds.append("mem32 0x%X 1" % (addr + 4 * i))
    cmds.append("exit")
    rc, out = run_jlink(cmds)
    if rc != 0:
        print(out[-3000:])
        sys.exit("phase2 (read) failed, rc=%d" % rc)

    regs = parse_mem32(out)
    res = [regs.get(addr + 4 * i) for i in range(8)]

    print("\n================ results (RAM) ================")
    for i, name in enumerate(["magic", "comp1_cfg_ok", "comp1_out",
                              "comp1_api_ok", "opamp1_cfg_ok",
                              "opamp1_gain_ok", "opamp4_cfg_ok", "all_ok"]):
        print("  %-14s = %s" % (name, "0x%08X" % res[i] if res[i] is not None else "?"))
    print("================ register dump =================")
    for name, a in EVIDENCE:
        print("  %-13s @0x%08X = 0x%08X" % (name, a, regs[a] if a in regs else 0))

    # ---- assertions ------------------------------------------------
    ok = True
    results_fail = []

    # struct indices: 0 magic,1 comp1_cfg_ok,2 comp1_out,3 comp1_api_ok,
    #                 4 opamp1_cfg_ok,5 opamp1_gain_ok,6 opamp4_cfg_ok,7 all_ok
    if res[0] != 0x4E333254:
        ok = False
        results_fail.append("results magic != N32T")
    for idx, name in [(1, "comp1_cfg_ok"), (3, "comp1_api_ok"),
                      (4, "opamp1_cfg_ok"), (5, "opamp1_gain_ok"),
                      (6, "opamp4_cfg_ok"), (7, "all_ok")]:
        if res[idx] != 1:
            ok = False
            results_fail.append("%s != 1" % name)

    print("================ register evidence ==============")
    for okmsg, msg in check_evidence(regs):
        print("  [%s] %s" % ("PASS" if okmsg else "FAIL", msg))
        ok = ok and okmsg

    if results_fail:
        print("  results FAIL: %s" % ", ".join(results_fail))

    print("\nRESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
