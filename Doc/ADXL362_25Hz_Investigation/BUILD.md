# 报告文件说明与复建

- `README.md`：报告正文，PDF 的唯一文本来源。
- `assets/`：11 幅原理图局部、实测波形重绘和数据图。
- `source_manifest.json`：原始归档哈希、采样参数、图表数据来源。
- `reviewed_measurements.json`：从原始归档重新计算并校验的关键数据。
- `build_report.py`：生成插图、解析 Markdown、排版 PDF 并渲染检查页。
- 最终 PDF：项目根目录 `output/pdf/ADXL362_25Hz_Supplier_Report.pdf`。

## 依赖

Python 3.12；`numpy`、`matplotlib`、`markdown-it-py`、`reportlab`、`pypdfium2`。
当前脚本使用 Windows 的等线/等线粗体及 Consolas 字体并嵌入 PDF。
可在独立虚拟环境安装上述依赖，不需要板卡、串口或调试器。

只重新转换已有 Markdown 和插图：

```powershell
python -B Doc/ADXL362_25Hz_Investigation/build_report.py --pdf-only
```

从原始数据重绘再转换：

```powershell
python -B Doc/ADXL362_25Hz_Investigation/build_report.py --source-root <diagnostic-repository>
```

`<diagnostic-repository>` 须包含诊断提交 `3c02631` 的 `Tools` 解码脚本、
V3.8 原理图以及当日 `.local/captures` 中的七份真实 DSL 归档。
程序先校验原始文件 SHA-256；不匹配则终止，不接受静默替换数据。
重建会更新本报告目录内的派生图表/JSON 和最终 PDF，不改原始波形。

## 对外提供

PDF 可独立阅读。提供 Markdown 时需同时保留 `assets` 相对路径。
原始 DSL 未嵌入报告；如供货商需要，应按正文第 13 节索引另行提供，
并核对哈希。不要随报告发送 EEPROM、探针信息、采购合同或完整工程备份。

报告没有自动发送或上传给任何第三方。
