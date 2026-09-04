#!/usr/bin/env python3
"""Build the bounded Data Analytics report artifact for the validation results."""

from __future__ import annotations

import json
import math
import sqlite3
from datetime import datetime
from pathlib import Path
from typing import Any

import pandas as pd


VALIDATION_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = VALIDATION_DIR / "output"
QUERY_DIR = VALIDATION_DIR / "queries"
ARTIFACT_PATH = OUTPUT_DIR / "report_artifact.json"


def plain_value(value: Any) -> Any:
    if pd.isna(value):
        return None
    if hasattr(value, "item"):
        return value.item()
    return value


def frame_rows(frame: pd.DataFrame) -> list[dict[str, Any]]:
    return [
        {str(key): plain_value(value) for key, value in row.items()}
        for row in frame.to_dict(orient="records")
    ]


def sql_literal(value: Any) -> str:
    value = plain_value(value)
    if value is None:
        return "NULL"
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, (int, float)):
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError("Non-finite values are not supported in report SQL")
        return repr(value)
    return "'" + str(value).replace("'", "''") + "'"


def values_query(table_name: str, columns: list[str], rows: list[dict[str, Any]]) -> str:
    values = ",\n  ".join(
        "(" + ", ".join(sql_literal(row.get(column)) for column in columns) + ")"
        for row in rows
    )
    quoted_columns = ", ".join(f'"{column}"' for column in columns)
    return (
        f'WITH "{table_name}" ({quoted_columns}) AS (\n'
        f"  VALUES\n  {values}\n"
        ")\n"
        f'SELECT * FROM "{table_name}";\n'
    )


def verify_query(query: str, expected_rows: int) -> None:
    connection = sqlite3.connect(":memory:")
    try:
        result = connection.execute(query).fetchall()
    finally:
        connection.close()
    if len(result) != expected_rows:
        raise RuntimeError(f"SQL returned {len(result)} rows; expected {expected_rows}")


def write_query(file_name: str, query: str, expected_rows: int) -> str:
    QUERY_DIR.mkdir(parents=True, exist_ok=True)
    verify_query(query, expected_rows)
    path = QUERY_DIR / file_name
    path.write_text(query, encoding="utf-8")
    return str(path.relative_to(VALIDATION_DIR.parents[2]))


def main() -> int:
    generated_at = datetime.now().astimezone().isoformat(timespec="seconds")

    recording = pd.read_csv(OUTPUT_DIR / "recording_summary.csv")
    recording["expected_label"] = recording["expected_behavior"].map(
        {"panting": "喘息记录", "rumination": "反刍对照"}
    )
    recording_columns = [
        "recording",
        "file_name",
        "expected_behavior",
        "expected_label",
        "windows",
        "criterion_passes",
        "criterion_pass_rate",
        "median_spectral_ratio",
        "q25_spectral_ratio",
        "q75_spectral_ratio",
        "median_peak_1_2_bpm",
    ]
    recording_rows = frame_rows(recording[recording_columns])

    sensitivity = pd.read_csv(OUTPUT_DIR / "sensitivity_analysis.csv")
    scenario_names = {
        ("primary", "25 Hz, 90 s, raw magnitude, rectangular taper, deduplicated"): "主分析",
        ("taper", "Hann taper"): "Hann 窗",
        ("signal_interpretation", "dominant peak-to-peak axis"): "主导轴",
        ("signal_interpretation", "PCA first component"): "PCA 投影",
        ("signal_interpretation", "magnitude of first differences"): "一阶差分幅值",
        ("window_length", "30 s windows"): "30 秒窗",
        ("window_length", "180 s windows"): "180 秒窗",
        ("sampling_rate", "incorrectly assume paper's 10 Hz for 25 Hz data"): "误按 10 Hz",
    }
    sensitivity["scenario"] = [
        scenario_names.get((check, detail))
        for check, detail in zip(sensitivity["check"], sensitivity["detail"])
    ]
    robustness = sensitivity[sensitivity["scenario"].notna()].copy()
    robustness_columns = [
        "scenario",
        "detail",
        "panting_windows",
        "panting_window_pass_rate",
        "rumination_windows",
        "rumination_window_rejection_rate",
        "balanced_descriptive_rate",
    ]
    robustness_rows = frame_rows(robustness[robustness_columns])

    firmware = pd.read_csv(OUTPUT_DIR / "firmware_comparison.csv")
    firmware["expected_label"] = firmware["expected_behavior"].map(
        {"panting": "喘息记录", "rumination": "反刍对照"}
    )
    firmware_columns = [
        "recording",
        "file_name",
        "expected_behavior",
        "expected_label",
        "processed_samples_after_dedup",
        "breath_samples",
        "breath_sample_rate",
    ]
    firmware_rows = frame_rows(firmware[firmware_columns])

    recording_sql = values_query("recording_results", recording_columns, recording_rows)
    robustness_sql = values_query("robustness_results", robustness_columns, robustness_rows)
    firmware_sql = values_query("firmware_results", firmware_columns, firmware_rows)

    recording_sql_path = write_query(
        "recording_results.sql", recording_sql, len(recording_rows)
    )
    robustness_sql_path = write_query(
        "robustness_results.sql", robustness_sql, len(robustness_rows)
    )
    firmware_sql_path = write_query(
        "firmware_results.sql", firmware_sql, len(firmware_rows)
    )

    sources = [
        {
            "id": "paper_pdf",
            "label": "Detecting Heat Stress in Dairy Cattle Using Neck-Mounted Activity Collars (2020)",
            "path": "Doc/Paper/Detecting Heat Stress in Dairy Cattle Using Neck-Mounted Activity Collars（2020）.pdf",
        },
        {
            "id": "recording_results_source",
            "label": "频谱判据按记录汇总",
            "path": recording_sql_path,
            "query": {
                "engine": "sqlite",
                "language": "sql",
                "executed_at": generated_at,
                "description": (
                    "把独立 Python 复算生成的 recording_summary.csv 物化为报告所用的 "
                    "4 行按记录汇总数据；SQL 已在内存 SQLite 中执行并核对行数。"
                ),
                "sql": recording_sql,
                "tables_used": [],
                "filters": [
                    "移除喘息文件首个全零初始化样本",
                    "从 17:55 记录移除与 16:44 记录相同的 15,536 行前缀",
                    "只保留完整的非重叠 90 秒窗",
                ],
                "metric_definitions": [
                    "判据通过率 = F₁₋₂ > F₂₋₃ 的窗口数 / 该记录完整窗口数",
                    "F₁₋₂ 与 F₂₋₃ 分别为对应频带 FFT 幅值峰值 / 频带均值",
                ],
            },
        },
        {
            "id": "robustness_results_source",
            "label": "频谱判据稳健性分析",
            "path": robustness_sql_path,
            "query": {
                "engine": "sqlite",
                "language": "sql",
                "executed_at": generated_at,
                "description": (
                    "把 sensitivity_analysis.csv 中 8 个代表性处理场景物化为报告图表数据；"
                    "SQL 已在内存 SQLite 中执行并核对行数。"
                ),
                "sql": robustness_sql,
                "tables_used": [],
                "metric_definitions": [
                    "描述性平衡率 = (喘息文件窗口通过率 + 反刍对照窗口排除率) / 2",
                    "该指标不使用逐窗人工真值，不是正式的分类准确率",
                ],
            },
        },
        {
            "id": "firmware_outputs",
            "label": "当前固件 PC runner 对照",
            "path": firmware_sql_path,
            "query": {
                "engine": "sqlite",
                "language": "sql",
                "executed_at": generated_at,
                "description": (
                    "把直接编译当前 C 固件后生成的 firmware_comparison.csv 物化为报告表格；"
                    "SQL 已在内存 SQLite 中执行并核对行数。"
                ),
                "sql": firmware_sql,
                "tables_used": [],
                "metric_definitions": [
                    "breath 样本占比 = behavior=7 样本数 / 处理后样本总数"
                ],
            },
        },
        {
            "id": "validation_code",
            "label": "独立复算代码",
            "path": "Doc/Paper/validation/validate_panting_algorithm.py",
        },
        {
            "id": "sampling_contract",
            "label": "ADXL362 PC runner 数据契约",
            "path": "Tools/adxl362_pc/README.md",
        },
    ]

    manifest = {
        "version": 1,
        "surface": "report",
        "title": "喘息频谱算法验证：有区分信号，但尚未证明正确",
        "description": (
            "使用所提供的三轴加速度 CSV，对论文中 F₁₋₂ > F₂₋₃ "
            "喘息/反刍频谱判据进行独立复算与稳健性检查。"
        ),
        "generatedAt": generated_at,
        "charts": [
            {
                "id": "recording_pass_chart",
                "title": "各记录的 F₁₋₂ > F₂₋₃ 判据通过率",
                "subtitle": "反刍对照为 7.7%，三个喘息记录为 12.5%–85.7%",
                "type": "bar",
                "intent": "comparison",
                "dataset": "recording_results",
                "sourceId": "recording_results_source",
                "valueFormat": "percent",
                "encodings": {
                    "x": {"field": "recording", "type": "nominal", "label": "记录"},
                    "y": {
                        "field": "criterion_pass_rate",
                        "type": "quantitative",
                        "label": "判据通过率",
                        "format": "percent",
                    },
                    "color": {
                        "field": "expected_label",
                        "type": "nominal",
                        "label": "文件级标签",
                    },
                    "tooltip": [
                        {
                            "field": "windows",
                            "type": "quantitative",
                            "label": "窗口数",
                            "format": "number",
                        },
                        {
                            "field": "criterion_passes",
                            "type": "quantitative",
                            "label": "通过窗",
                            "format": "number",
                        },
                        {
                            "field": "median_spectral_ratio",
                            "type": "quantitative",
                            "label": "F₁₋₂/F₂₋₃ 中位数",
                            "format": "number",
                        },
                        {
                            "field": "median_peak_1_2_bpm",
                            "type": "quantitative",
                            "label": "1–2 Hz 峰值中位数",
                            "format": "number",
                            "unit": "次/分",
                        },
                    ],
                },
                "settings": {
                    "orientation": "horizontal",
                    "groupMode": "single",
                    "showValues": True,
                    "sort": "descending",
                },
                "palette": {"kind": "categorical"},
                "legend": {"position": "bottom", "title": "文件级标签"},
                "layout": "full",
            },
            {
                "id": "robustness_chart",
                "title": "不同处理解释下的描述性平衡率",
                "subtitle": "误按 10 Hz 处理或采用差分幅值时，区分能力明显下降",
                "type": "bar",
                "intent": "comparison",
                "dataset": "robustness_results",
                "sourceId": "robustness_results_source",
                "valueFormat": "percent",
                "encodings": {
                    "x": {"field": "scenario", "type": "nominal", "label": "处理场景"},
                    "y": {
                        "field": "balanced_descriptive_rate",
                        "type": "quantitative",
                        "label": "描述性平衡率",
                        "format": "percent",
                    },
                    "tooltip": [
                        {
                            "field": "panting_window_pass_rate",
                            "type": "quantitative",
                            "label": "喘息窗通过率",
                            "format": "percent",
                        },
                        {
                            "field": "rumination_window_rejection_rate",
                            "type": "quantitative",
                            "label": "反刍窗排除率",
                            "format": "percent",
                        },
                        {
                            "field": "panting_windows",
                            "type": "quantitative",
                            "label": "喘息窗数",
                            "format": "number",
                        },
                        {
                            "field": "rumination_windows",
                            "type": "quantitative",
                            "label": "反刍窗数",
                            "format": "number",
                        },
                    ],
                },
                "settings": {
                    "orientation": "horizontal",
                    "groupMode": "single",
                    "showValues": True,
                    "sort": "descending",
                },
                "palette": {"kind": "sequential"},
                "layout": "full",
            },
        ],
        "tables": [
            {
                "id": "recording_results_table",
                "title": "按记录汇总的频谱判据结果",
                "subtitle": (
                    "25 Hz、90 秒非重叠窗、合加速度幅值、矩形窗；"
                    "17:55 记录已去除重复前缀"
                ),
                "dataset": "recording_results",
                "sourceId": "recording_results_source",
                "defaultSort": {"field": "criterion_pass_rate", "direction": "desc"},
                "density": "spacious",
                "columns": [
                    {"field": "recording", "label": "记录", "type": "text"},
                    {"field": "expected_label", "label": "文件级标签", "type": "text"},
                    {"field": "windows", "label": "90 秒窗", "format": "number"},
                    {"field": "criterion_passes", "label": "通过窗", "format": "number"},
                    {
                        "field": "criterion_pass_rate",
                        "label": "判据通过率",
                        "format": "percent",
                    },
                    {
                        "field": "median_spectral_ratio",
                        "label": "频谱比中位数",
                        "format": "number",
                    },
                    {
                        "field": "median_peak_1_2_bpm",
                        "label": "1–2 Hz 峰值中位数（次/分）",
                        "format": "number",
                    },
                ],
            },
            {
                "id": "firmware_results_table",
                "title": "当前固件 PC runner 的样本级 breath 输出",
                "subtitle": (
                    "25 Hz、每次 150 样本（6 秒）；比例基于处理后样本，"
                    "不是逐秒人工真值准确率"
                ),
                "dataset": "firmware_results",
                "sourceId": "firmware_outputs",
                "defaultSort": {"field": "breath_sample_rate", "direction": "desc"},
                "density": "spacious",
                "columns": [
                    {"field": "recording", "label": "记录", "type": "text"},
                    {"field": "expected_label", "label": "文件级标签", "type": "text"},
                    {
                        "field": "processed_samples_after_dedup",
                        "label": "处理样本",
                        "format": "number",
                    },
                    {"field": "breath_samples", "label": "breath 样本", "format": "number"},
                    {
                        "field": "breath_sample_rate",
                        "label": "breath 占比",
                        "format": "percent",
                    },
                ],
            },
        ],
        "sources": sources,
        "blocks": [
            {
                "id": "title",
                "type": "markdown",
                "body": "# 喘息频谱算法验证：有区分信号，但尚未证明正确",
            },
            {
                "id": "technical_summary",
                "type": "markdown",
                "sourceId": "recording_results_source",
                "body": (
                    "## 技术结论\n\n"
                    "- **结论不是“算法正确”，而是“频谱规则得到有限的方向性支持”。** "
                    "主解释下，喘息文件 56 个窗口中 33 个通过（58.9%），反刍对照 "
                    "13 个窗口中 12 个被排除（92.3%）。\n"
                    "- **文件间差异很大。** 三个喘息记录的通过率分别为 12.5%、58.8% "
                    "和 85.7%，说明姿态、行为混合、窗边界或信号解释会显著影响结果。\n"
                    "- **现有数据不能给出生产准确率。** 文件名只是记录级标签；没有逐窗"
                    "人工呼吸率、喘息起止、温湿度或 THI，因此不能把这些比例称为敏感度、"
                    "特异度或热应激诊断准确率。"
                ),
            },
            {
                "id": "recording_findings",
                "type": "markdown",
                "sourceId": "recording_results_source",
                "body": (
                    "## 反刍排除较强，但喘息记录并不稳定\n\n"
                    "下面按记录展示相同判据的通过率。反刍对照只有 1/13 个窗误通过；"
                    "但喘息记录从 1/8 到 12/14，相差超过六倍。**所以，该规则更像一个"
                    "候选特征或二级过滤器，而不是已经由这批数据证明可独立工作的分类器。**"
                ),
            },
            {"id": "recording_chart_block", "type": "chart", "chartId": "recording_pass_chart"},
            {"id": "recording_table_block", "type": "table", "tableId": "recording_results_table"},
            {
                "id": "scope_definitions",
                "type": "markdown",
                "body": (
                    "## 本次验证覆盖的是频谱二级判据，不是完整热应激算法\n\n"
                    "论文流程先用参考文献 [30] 的 `(E, F₂₋₃)` 方法把窗口分为 "
                    "other / rumination / eating，再仅对 rumination 窗应用 "
                    "`F₁₋₂ > F₂₋₃`。PDF 没有给出第 8 步的阈值或模型，因此本次只能"
                    "验证明确写出的频谱不等式。数据也没有牛只标识、姿态真值、逐窗呼吸"
                    "次数、环境温湿度或 THI；因此不能验证从“快速呼吸”到“热应激”的"
                    "因果或诊断链条。"
                ),
            },
            {
                "id": "methodology",
                "type": "markdown",
                "body": (
                    "## 复算采用 25 Hz 实际采样率，并显式固定论文未说明的处理选择\n\n"
                    "项目的 PC runner 声明输入采样率为 25 Hz；论文设备为 10 Hz。主分析"
                    "因此用 25 Hz、90 秒非重叠窗，对 `sqrt(x²+y²+z²)` 去均值后做"
                    "矩形窗 FFT；频带定义为 `[1,2)` 与 `[2,3)`，避免 2 Hz 重复计入。"
                    "16:44 与 17:55 文件存在 15,536 个相同的清洗后样本（约 10.36 分钟），"
                    "已从 17:55 记录删除，避免重复计权。"
                ),
            },
            {
                "id": "robustness",
                "type": "markdown",
                "sourceId": "robustness_results_source",
                "body": (
                    "## 预处理歧义足以改变结论强度\n\n"
                    "主分析的描述性平衡率为 75.6%。Hann 窗为 79.5%，主导轴与 PCA "
                    "解释约为 70%–73%；但把三轴公式解释为“一阶差分幅值”时降至 35.1%，"
                    "把 25 Hz 数据误按论文的 10 Hz 处理时为 49.4%。**这表明论文缺少的"
                    "轴选择、窗函数、频带边界与采样率适配细节不是小问题，而是复现所必需"
                    "的算法规格。**"
                ),
            },
            {"id": "robustness_chart_block", "type": "chart", "chartId": "robustness_chart"},
            {
                "id": "firmware_gap",
                "type": "markdown",
                "sourceId": "firmware_outputs",
                "body": (
                    "## 当前固件有响应，但不是论文算法的实现\n\n"
                    "当前 C 代码在 25 Hz、6 秒块上逐轴做时域正弦检测，频率范围为 "
                    "1.0–2.7 Hz；它没有实现论文的 90 秒 FFT 与 `F₁₋₂/F₂₋₃` 比较。"
                    "PC runner 在反刍对照中输出 0% `breath`，三个喘息记录的样本级占比"
                    "分别为 85.1%、30.8% 和去重后的 74.0%。此外，生产计数逻辑把 "
                    "behavior=7 计入 `other` 计数，而不是独立的 breath 字段；即使检测"
                    "发生，汇总遥测也可能无法单独呈现喘息。"
                ),
            },
            {"id": "firmware_table_block", "type": "table", "tableId": "firmware_results_table"},
            {
                "id": "limitations",
                "type": "markdown",
                "body": (
                    "## 主要不确定性使当前证据不足以用于上线判定\n\n"
                    "- 窗口没有逐窗真值；文件名不能证明每一分钟都在喘息或反刍。\n"
                    "- 只有一个反刍对照文件，没有采食、行走、饮水、甩头、卧立转换等"
                    "难负样本。\n"
                    "- 同一记录内的连续窗口高度相关；名义 Wilson 区间未处理聚类，仅作描述。\n"
                    "- 没有独立牛只、不同项圈佩戴方向、品种或环境的外部验证。\n"
                    "- 论文第 8 步分类器不可由所提供 PDF 完整复现。"
                ),
            },
            {
                "id": "next_steps",
                "type": "markdown",
                "body": (
                    "## 建议下一步：先补真值，再冻结算法规格\n\n"
                    "1. 以 6 秒或 15 秒为最小标注粒度，同步记录人工呼吸率、喘息等级、"
                    "姿态和行为；至少覆盖多个牛只与多个日期。\n"
                    "2. 在数据文件中写入采样率、时间戳、牛只 ID、设备 ID、温度、湿度与 "
                    "THI，并保留原始三轴单位和量程。\n"
                    "3. 明确 FFT 输入信号、去趋势、窗函数、重叠率、2 Hz 边界、初始反刍"
                    "分类器和 15 分钟多数投票规则。\n"
                    "4. 按牛只/日期分组划分训练与测试集，报告逐窗混淆矩阵、事件级召回、"
                    "误报持续时间和置信区间。\n"
                    "5. 若验证当前固件，单独按 6 秒输出做评估，并修正或确认 behavior=7 "
                    "被计入 `other` 的遥测设计。"
                ),
            },
            {
                "id": "further_questions",
                "type": "markdown",
                "body": (
                    "## 仍需确认的问题\n\n"
                    "- 三个“喘息”文件是否全程喘息，还是只包含若干人工观察到的片段？\n"
                    "- 反刍文件是否有同步视频或压力鼻带真值，可定位纯反刍区间？\n"
                    "- 当前目标是复现论文算法，还是评估现有 6 秒固件算法用于量产？\n"
                    "- `other` 计数承载 breath 是否是协议兼容设计，还是需要新增独立喘息字段？"
                ),
            },
        ],
    }

    artifact = {
        "surface": "report",
        "manifest": manifest,
        "snapshot": {
            "version": 1,
            "generatedAt": generated_at,
            "status": "ready",
            "datasets": {
                "recording_results": recording_rows,
                "robustness_results": robustness_rows,
                "firmware_results": firmware_rows,
            },
        },
    }

    ARTIFACT_PATH.write_text(
        json.dumps(artifact, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(f"artifact={ARTIFACT_PATH}")
    print(f"recording_rows={len(recording_rows)}")
    print(f"robustness_rows={len(robustness_rows)}")
    print(f"firmware_rows={len(firmware_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
