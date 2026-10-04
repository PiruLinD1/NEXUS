"""Generate IntelliSense commands for every firmware source from the PROS Makefile."""

import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess


def main():
    root = Path(__file__).resolve().parents[1]
    environment = os.environ.copy()
    if os.name == "nt":
        toolchain = (Path(environment["APPDATA"]) / "Code/User/globalStorage/"
                     "sigbots.pros/install/pros-toolchain-windows/usr/bin")
        environment["PATH"] = str(toolchain) + os.pathsep + environment.get("PATH", "")
    make = shutil.which("make", path=environment.get("PATH"))
    if not make:
        raise SystemExit("PROS make not found. Add the PROS toolchain to PATH.")
    result = subprocess.run(
        [make, "-Bn", "quick"], cwd=root, env=environment,
        capture_output=True, text=True, check=True,
    )
    commands = []
    # PROS recipes capture each compiler invocation in output="$(... 2>&1)".
    pattern = r'output="\$\((arm-none-eabi-(?:g\+\+|gcc) -c [^\r\n]*?) 2>&1\)"'
    for command in re.findall(pattern, result.stdout):
        arguments = shlex.split(command)
        source = arguments[-1]
        if Path(source).suffix not in {".c", ".cpp", ".c++", ".cc"}:
            continue
        compiler = shutil.which(arguments[0], path=environment.get("PATH"))
        if not compiler:
            raise SystemExit(f"Compiler not found: {arguments[0]}")
        arguments[0] = Path(compiler).resolve().as_posix()
        commands.append({"directory": root.as_posix(), "file": source,
                         "arguments": arguments})
    expected = {p.resolve() for p in (root / "src").rglob("*")
                if p.suffix in {".c", ".cpp", ".c++", ".cc"}}
    actual = {(root / entry["file"]).resolve() for entry in commands}
    if not commands or actual != expected:
        raise SystemExit("Incomplete compilation database; existing file was not changed.")
    (root / "compile_commands.json").write_text(
        json.dumps(commands, indent=2) + "\n", encoding="utf-8",
    )
    print(f"Updated compile_commands.json: {len(commands)} firmware sources.")


if __name__ == "__main__":
    main()
