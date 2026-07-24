# 喘息算法验证

本目录用于复现并验证论文
`Detecting Heat Stress in Dairy Cattle Using Neck-Mounted Activity Collars（2020）.pdf`
中明确给出的频谱判据。

运行：

```bash
/Users/gally/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  Doc/Paper/validation/validate_panting_algorithm.py
```

主要输出位于 `Doc/Paper/validation/output/`：

- `validation_summary.json`：结论、主方法与限制；
- `window_scores.csv`：每个非重叠 90 秒窗的频谱指标；
- `recording_summary.csv`：每个记录文件的汇总；
- `sensitivity_analysis.csv`：窗函数、轴选择、窗长和采样率敏感性；
- `data_quality_summary.csv`：缺失、重复、重叠与范围检查；
- `firmware_comparison.csv`：当前 C 固件算法的独立对照（不是论文算法）。

重要边界：论文第 8 步依赖未随 PDF 提供的参考文献 [30] 分类器，CSV
只有文件级标签而没有逐窗人工呼吸率真值。因此，本验证只能检验论文明确写出的
`F_1_2 > F_2_3` 频谱规则是否得到数据方向性支持，不能给出完整算法的临床/生产
敏感度、特异度或热应激诊断准确率。
