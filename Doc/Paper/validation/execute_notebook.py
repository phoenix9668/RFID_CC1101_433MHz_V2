#!/usr/bin/env python3
"""Execute this validation notebook without requiring Jupyter packages.

The workspace runtime does not bundle nbformat/nbclient. This small runner
executes code cells in one shared Python namespace, captures text output, and
writes standard notebook stream/error outputs plus execution counts.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import traceback
from pathlib import Path


def execute_notebook(path: Path) -> None:
    notebook = json.loads(path.read_text(encoding="utf-8"))
    namespace: dict[str, object] = {"__name__": "__notebook__"}
    execution_count = 0

    for cell in notebook.get("cells", []):
        if cell.get("cell_type") != "code":
            continue

        execution_count += 1
        source = "".join(cell.get("source", []))
        stdout = io.StringIO()
        stderr = io.StringIO()
        outputs: list[dict[str, object]] = []

        try:
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                exec(compile(source, f"{path.name}:cell-{execution_count}", "exec"), namespace)
        except Exception as error:
            if stdout.getvalue():
                outputs.append(
                    {"name": "stdout", "output_type": "stream", "text": stdout.getvalue()}
                )
            if stderr.getvalue():
                outputs.append(
                    {"name": "stderr", "output_type": "stream", "text": stderr.getvalue()}
                )
            outputs.append(
                {
                    "ename": type(error).__name__,
                    "evalue": str(error),
                    "output_type": "error",
                    "traceback": traceback.format_exc().splitlines(),
                }
            )
            cell["execution_count"] = execution_count
            cell["outputs"] = outputs
            path.write_text(
                json.dumps(notebook, ensure_ascii=False, indent=1) + "\n",
                encoding="utf-8",
            )
            raise

        if stdout.getvalue():
            outputs.append({"name": "stdout", "output_type": "stream", "text": stdout.getvalue()})
        if stderr.getvalue():
            outputs.append({"name": "stderr", "output_type": "stream", "text": stderr.getvalue()})
        cell["execution_count"] = execution_count
        cell["outputs"] = outputs

    path.write_text(
        json.dumps(notebook, ensure_ascii=False, indent=1) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("notebook", type=Path)
    args = parser.parse_args()
    execute_notebook(args.notebook.resolve())
    print(f"executed={args.notebook.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
