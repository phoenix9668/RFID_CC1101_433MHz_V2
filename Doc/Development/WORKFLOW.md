# Single-worktree development

For cross-computer/session continuation, start with
[the 2026-09-26 handoff](SESSION_HANDOFF_20260926.md) and
[the next-session prompt](NEXT_SESSION_PROMPT.md).

Keep one checkout in the original project directory. Open that directory in
VS Code with STM32CubeIDE for Visual Studio Code. Develop on branches in this
checkout; additional Git worktrees are not required.

The current candidate is `codex/v38-rebuild-from-34d57c2`. Consolidating directories
does not merge this candidate, or the older panting branches, into `main`.
Preserve those branches until their changes are deliberately reviewed and merged.

```text
git status --short
git switch codex/v38-rebuild-from-34d57c2
git switch -c codex/<next-change>
```

Commit the current task's changes before switching branches. Do not use a forced
checkout or clean to resolve untracked-file conflicts; preserve those files first.
Only one development branch can be checked out in this directory at a time.

## Local configuration and artifacts

- `CMakeUserPresets.json` keeps local tool paths; it is not committed.
- `.local/` keeps board backups, captures, machine settings and migration archives.
- `build/` and `tmp/` are generated output, not source files.
- Keep original raw waveforms and unrelated project documents locally. Only the
  reviewed reports and relevant replay evidence belong in this change's commit.
- Archives under `.local/worktree-archive/` retain the removed worktree's build
  outputs for traceability and rollback. Their CMake caches contain old paths and
  must not be reused for development.

Configure new build directories after changing checkout locations:

```text
cmake --preset Debug-local
cmake --build --preset Debug-local
cmake --preset Release-local
cmake --build --preset Release-local
cmake --preset Diagnostics-local
cmake --build --preset Diagnostics-local
```

The `*-local` presets are created by the local setup described in
[V38_REBUILD.md](V38_REBUILD.md). Shared Debug/Release/Diagnostics presets remain
usable when the toolchain is already on PATH. Do not flash automatically when
building; this resampling candidate has not completed hardware qualification.

See [RESAMPLING.md](RESAMPLING.md) for regression commands and acceptance limits.
See [DATA_STORAGE.md](DATA_STORAGE.md) for LFS data and local-only artifact rules.
