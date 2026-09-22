# Data storage policy

## Git LFS

The following measurement and large-data paths use Git LFS:

| Path | Contents |
| --- | --- |
| `Doc/*.dsl` | Original DSLogic captures, including diagnostic/control captures |
| `Doc/Power Consumption Data/*.iotpl` | Original power-consumption recordings |
| `Doc/data/*.mp4` | Original videos |
| `Doc/data/*_result.csv` | Large original/candidate replay exports; retain both distinct versions |
| `Doc/MATLAB/**/*.xlsx` and direct `.xlsx` children | Original collar XYZ workbooks |

This changes newly staged versions only. Existing Git history is not rewritten,
so older workbook blobs remain in earlier commits. Worktree data bytes must stay
unchanged. The index stores LFS pointers with SHA-256 OIDs and sizes; the actual
files live in the local LFS object store until an authorized push uploads them.

LFS is enabled locally for this repository, not globally. Its pre-push hook must
be retained so future pushes upload the objects as well as the pointer commits.
Consumers need Git LFS and the corresponding objects, not just a ZIP of pointer
files, to replay the data:

```text
git lfs install --local
git lfs pull
git lfs ls-files --size
```

## Ordinary Git

- Firmware, tests, analysis scripts and SQL queries stay ordinary text sources.
- Small CSV/JSON validation evidence remains directly reviewable in Git, including
  `Doc/ADXL362_Resampling_Validation` and `Doc/Paper/validation`.
- Final reports, figures and report-generation sources remain tracked. Preview
  HTML files are not a substitute for the final retained PDF.
- Historical panting-analysis scripts/results are archived evidence, not changes
  to the V3.8 classifier. The old script still contains its original session paths;
  merely retaining it does not mean it has been ported or revalidated here.
- Procurement spreadsheets and other unrelated existing documents are not migrated.

## Local-only files

- Build products, temporary dependencies, device backups and machine settings keep
  their existing ignore rules. Never bulk-add `.local/` or EEPROM backups.
- `Doc/data/*_result_precommit.csv` is a duplicate intermediate export. The current
  precommit file was SHA-256-identical to the retained latest-result CSV.
- The panting report's HTML/print-HTML files are generated previews. Its duplicate
  `artifact.json` is ignored; canonical metadata remains in
  `Doc/Paper/validation/output/report_artifact.json`.
- The three explicitly named contract/font helpers in `.gitignore` are one-off
  local tools with machine-specific dependencies, not firmware tooling.

Ignoring a file does not delete it. Do not broadly ignore `*.csv`, `*.json`,
`*.py`, `Doc/`, or `output/`: these paths also contain valuable inputs and evidence.
