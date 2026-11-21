#!/usr/bin/env python3
"""Automatically generate firmware.uf2 alongside firmware.zip after linking."""

import os

Import("env")


def _generate_uf2(source, target, env):
    firmware_hex = env.subst("${BUILD_DIR}/${PROGNAME}.hex")
    uf2_file = os.environ.get("UF2_FILE_PATH", env.subst("${BUILD_DIR}/${PROGNAME}.uf2"))
    uf2_cmd = " ".join(
        [
            '"$PYTHONEXE"',
            '"$PROJECT_DIR/bin/uf2conv/uf2conv.py"',
            "-f",
            "0xADA52840",
            "-c",
            f'"{firmware_hex}"',
            "-o",
            f'"{uf2_file}"',
        ]
    )
    rc = env.Execute(uf2_cmd)
    if rc != 0:
        raise RuntimeError(f"uf2conv failed with exit code {rc}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", _generate_uf2)
