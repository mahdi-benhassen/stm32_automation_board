#!/usr/bin/env python3
"""
check_pin_conflicts.py — detect GPIO (port, pin) double-assignments in
inc/board_config.h.

The board header declares pin ownership as pairs of defines:

    #define <NAME>_PORT     GPIOx          (also: <NAME>_GPIO_PORT)
    #define <NAME>_PIN...   GPIO_PIN_n     (also: <NAME>_GPIO_PIN)

This script reconstructs every (port, pin) claim from the define structure
and exits non-zero if any (port, pin) is claimed by two or more functions.

Pure stdlib. Usage:

    python3 scripts/check_pin_conflicts.py [path/to/board_config.h]

Exit code 0 = no conflicts, 1 = conflicts found (or file unreadable).
"""

import re
import sys
from collections import defaultdict

PORT_RE = re.compile(r"^\s*#define\s+([A-Za-z0-9_]+)\s+GPIO([A-Z])\s*(?://.*)?$")
PIN_RE = re.compile(
    r"^\s*#define\s+([A-Za-z0-9_]+)\s+GPIO_PIN_(\d+)\s*(?://.*)?$"
)
MASK_RE = re.compile(
    r"^\s*#define\s+([A-Za-z0-9_]+)_PIN_MASK\s+0x([0-9A-Fa-f]+)U?\s*(?://.*)?$"
)


def port_candidates(pin_name):
    """Yield candidate port-define names for a pin-define name, most
    specific first. The first candidate that exists in the port map wins.

    Handles the naming styles present in board_config.h:
      RS485_TX_PIN   -> RS485_TX_PORT
      AI0_GPIO_PIN   -> AI0_GPIO_PORT
      ETH_TXD0_PIN   -> ETH_TXD0_PORT
      DI_PIN_0       -> DI_PORT        (channel index suffix)
      DO_PIN_7       -> DO_PORT
      RELAY1_PIN     -> RELAY_PORT     (digit inside the prefix)
      RELAY1_LED_PIN -> RELAY_LED_PORT
    """
    # Exact suffix swap: ..._PIN -> ..._PORT
    if pin_name.endswith("_PIN"):
        yield pin_name[: -len("_PIN")] + "_PORT"
    # Channel suffix: ..._PIN_<n> -> <prefix>_PORT
    m = re.match(r"^(.*)_PIN_\d+$", pin_name)
    if m:
        yield m.group(1) + "_PORT"
    # Embedded digits: RELAY1_PIN -> RELAY_PORT, RELAY1_LED_PIN -> RELAY_LED_PORT
    if pin_name.endswith("_PIN"):
        stripped = re.sub(r"\d+", "", pin_name[: -len("_PIN")])
        yield stripped + "_PORT"
    # Last resort: everything before the first _PIN + _PORT
    head = pin_name.split("_PIN", 1)[0]
    yield re.sub(r"\d+", "", head) + "_PORT"


def main(argv):
    path = argv[1] if len(argv) > 1 else "inc/board_config.h"
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except OSError as exc:
        print(f"error: cannot read {path}: {exc}", file=sys.stderr)
        return 1

    ports = {}  # define name -> port letter
    pins = []   # (define name, pin number)
    masks = {}  # prefix -> mask value from <PREFIX>_PIN_MASK

    for line in lines:
        # Function-like macros (e.g. DI_CLK_ENABLE()) cannot match the
        # regexes below because their names contain parentheses.
        m = PORT_RE.match(line)
        if m and m.group(1).endswith("_PORT"):
            ports[m.group(1)] = m.group(2)
            continue
        m = MASK_RE.match(line)
        if m:
            masks[m.group(1)] = int(m.group(2), 16)
            continue
        m = PIN_RE.match(line)
        if m:
            pins.append((m.group(1), int(m.group(2))))

    claims = defaultdict(list)  # (port, pin) -> [define names]
    unresolved = []

    for name, pin in pins:
        port_letter = None
        for cand in port_candidates(name):
            if cand in ports:
                port_letter = ports[cand]
                break
        if port_letter is None:
            unresolved.append(name)
            continue
        claims[(port_letter, pin)].append(name)

    for name in unresolved:
        print(f"warning: no port define found for pin define {name}",
              file=sys.stderr)

    conflicts = {k: v for k, v in claims.items() if len(v) > 1}

    # Consistency check: for every <PREFIX>_PIN_MASK, the mask bits must
    # equal exactly the set of <PREFIX>_PIN_<n> pin numbers. A stale mask
    # (e.g. DO_PIN_MASK missing pins or covering an unrelated pin) silently
    # breaks drivers that init/reset whole ports via the mask.
    mask_errors = []
    for prefix, mask in sorted(masks.items()):
        expected = 0
        member_pins = [
            pin for name, pin in pins
            if re.match(rf"^{re.escape(prefix)}_PIN_\d+$", name)
        ]
        for pin in member_pins:
            expected |= 1 << pin
        if mask != expected:
            mask_errors.append(
                f"  {prefix}_PIN_MASK = 0x{mask:04X} but {prefix}_PIN_0..n "
                f"imply 0x{expected:04X} (pins {sorted(member_pins)})"
            )

    if conflicts or mask_errors:
        if conflicts:
            print("GPIO pin conflicts detected in", path)
            for (port, pin), names in sorted(conflicts.items()):
                print(f"  P{port}{pin}: claimed by {', '.join(sorted(names))}")
        if mask_errors:
            print("Pin-mask inconsistencies detected in", path)
            for err in mask_errors:
                print(err)
        return 1

    print(f"OK: {len(pins)} pin claims checked in {path}, no conflicts.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
