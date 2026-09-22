"""Build supplier evidence figures and PDF from the authored Markdown.

Requires the diagnostic worktree's original captures and decoder scripts.
No probe access, board operations, invented waveform data or network calls.
"""
import argparse
import gc
import hashlib
import html
import json
import os
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
VENDOR = ROOT / "tmp/pdfs/vendor"
if VENDOR.exists():
    sys.path.insert(0, str(VENDOR))
os.environ.setdefault("MPLCONFIGDIR", str(ROOT / "tmp/pdfs/mpl"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle
import pypdfium2 as pdfium
from markdown_it import MarkdownIt
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, Table, TableStyle, PageBreak, KeepTogether

FONT = Path("C:/Windows/Fonts/Deng.ttf")
BOLD = Path("C:/Windows/Fonts/Dengb.ttf")
font_manager.fontManager.addfont(str(FONT))
plt.rcParams.update({"font.family": font_manager.FontProperties(fname=str(FONT)).get_name(),
                     "font.size": 10, "axes.unicode_minus": False, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.labelcolor": "#30383c",
                     "xtick.color": "#535d62", "ytick.color": "#535d62",
                     "axes.edgecolor": "#acb5ba", "savefig.facecolor": "white"})
TEAL, RED, BLUE, GRAY = "#087f81", "#b44b46", "#356ab7", "#919da5"
ASSETS = HERE / "assets"
TMP = ROOT / "tmp/pdfs"

CAPTURES = {
    "A": ("20260921-152326-adxl362-timing", "74DD03753694FD1B7238F5C1B7CC0012988146958D13A14C85A4861E0F15253A"),
    "B": ("20260921-153828-adxl362-spi", "616603F5C0E21C350A7571041C0A48655C9B0BF1AC295A121E2708DE1D461A44"),
    "C": ("20260921-155625-adxl362-startup", "E6969BA8AF2D52CCC37D53B351E049F880D862E7AC6E59B7D8E2D586D466843B"),
    "D3": ("20260921-163735-adxl362-drdy", "00676EF8ADC037EA569122703581E7238B3F898D588D759DFC202FA13F2F6D30"),
    "E": ("20260921-165326-adxl362-sweep", "B13F1FF0D4E1D0CC344A821D38B0F2773C5681E3A42328E22BCD96DD30D9F4AB"),
    "F": ("20260921-215505-adxl362-extclk", "E858D5503F13A74213C20709B6AD224AC1EEC50072D6499561749862634FDAC4"),
    "G": ("20260921-221426-adxl362-extclk-high", "799ED3E3CF3E533D4D07350AD2915690A82740ADADD4DBE0D9980A0689B2A031"),
}


def save(fig, name):
    fig.savefig(ASSETS / (name + ".png"), dpi=220, bbox_inches="tight", pad_inches=0.1)
    plt.close(fig)


def digital(ax, signal, rate, start, end, base, label, color, unit=1):
    """Draw every transition in the stated window, without resampling."""
    lo, hi = int(start * rate), min(len(signal), int(end * rate) + 1)
    part = signal[lo:hi]
    changes = np.flatnonzero(part[1:] != part[:-1]) + 1
    ix = np.r_[0, changes, len(part) - 1]
    ax.step((lo + ix) / rate * unit, base + 0.64 * part[ix], where="post", lw=1, color=color)
    ax.text(-0.018, base + 0.32, label, ha="right", va="center", transform=ax.get_yaxis_transform(), color=color)


def wave_axis(ax, start, end, unit=1):
    ax.set_xlim(start * unit, end * unit)
    ax.set_yticks([])
    ax.spines["left"].set_visible(False)
    ax.grid(axis="x", alpha=0.16)


def build_figures(source):
    sys.path.insert(0, str(source / "Tools"))
    from analyze_adxl362_dsl import load_capture, edges, analyze
    from analyze_adxl362_timing import segment_timings, summarize
    from analyze_adxl362_extclock import analyze_signals
    private = source / ".local/captures"
    manifest = {"report": "ADXL362-ODR-20260921", "date": "2026-09-21", "diagnostic_commit": "3c02631",
                "captures": [], "figure_policy": "Actual sampled transitions or explicitly identified schematic/summary; no smoothing.",
                "excluded_personal_data": ["device identity", "probe serial", "EEPROM contents", "purchase contracts"]}
    results = {}

    def load(key, six=False):
        stem, expected = CAPTURES[key]
        p = private / (stem + ".dsl")
        actual = hashlib.sha256(p.read_bytes()).hexdigest().upper()
        if actual != expected:
            raise ValueError("Capture hash mismatch: " + key)
        h, session, rate, signals = load_capture(p, timing_only=True, channel_count=6 if six else 5)
        parts = stem.split("-")
        manifest["captures"].append({"id": key, "original_file": f"DSLogic U3Pro16-la-{parts[0][2:]}-{parts[1]}.dsl",
            "sha256": actual, "bytes": p.stat().st_size, "sample_rate_hz": rate,
            "samples": len(signals[0]), "duration_seconds": len(signals[0]) / rate,
            "threshold_v": float(session["Threshold Level"]), "trigger_unix_ms": h.getint("header", "trigger time")})
        return rate, signals

    rate, s = load("A")
    ar, ac = edges(s[4], 0, 1) / rate, edges(s[0], 1, 0) / rate
    fig, ax = plt.subplots(figsize=(7.5, 2.65), layout="constrained")
    ax.vlines(ar, 1.04, 1.68, color=TEAL, lw=1.5)
    ax.vlines(ac, 0.04, 0.68, color=BLUE, lw=0.8)
    ax.set_yticks([0.36, 1.36], ["CS 事务开始", "INT2 上升沿"])
    ax.set_xlim(0, 50); ax.set_ylim(-0.1, 2.3); ax.set_xlabel("记录内时间（s）")
    ax.axvspan(10.65, 11.12, color=RED, alpha=0.12)
    ax.annotate("检查点主动读取\n没有水位 IRQ", xy=(10.876, 0.7), xytext=(10.876, 1.88),
                ha="center", color=RED, arrowprops={"arrowstyle": "->", "color": RED})
    ax.annotate("连续水位间隔约 7.64 s", xy=(29.9, 1.72), xytext=(29.9, 2.08), ha="center", color=TEAL)
    ax.grid(axis="x", alpha=0.2)
    save(fig, "04_fifo_timing")
    del s; gc.collect()

    rate, s = load("B")
    b = analyze(private / (CAPTURES["B"][0] + ".dsl"))
    results["B"] = b
    fig, ax = plt.subplots(figsize=(7.5, 2.6), layout="constrained")
    digital(ax, s[4], rate, .0095, .0188, 2, "INT2", TEAL, 1000)
    digital(ax, s[0], rate, .0095, .0188, 0.9, "CS", BLUE, 1000)
    for tr, label in zip(b["transactions"], ["STATUS", "FIFO 数量", "510 bytes", "396 bytes"]):
        x, end = tr["start_ms"], tr["end_ms"]
        ax.plot([x, end], [0.24, 0.24], lw=5, color=GRAY, solid_capstyle="butt")
        ax.annotate(label, xy=((x + end) / 2, .26), xytext=((x + end) / 2 + (-.32 if label == "STATUS" else .35 if label == "FIFO 数量" else 0), -.23 if label == "STATUS" else -.6),
                    ha="center", va="center", fontsize=9, arrowprops={"arrowstyle": "-", "color": GRAY})
    wave_axis(ax, .0095, .0188, 1000); ax.set_ylim(-.85, 3)
    ax.set_xlabel("记录内时间（ms）")
    save(fig, "05_spi_fifo")
    del s; gc.collect()

    rate, s = load("C")
    c = analyze(private / (CAPTURES["C"][0] + ".dsl"))
    results["C"] = c
    writes = [t for t in c["transactions"] if t["command"] == "write" and t["address"] >= 0x20]
    fig, ax = plt.subplots(figsize=(7.5, 2.85), layout="constrained")
    for y, w in enumerate(writes):
        r = next(t for t in c["transactions"] if t["command"] == "read" and t["address"] == w["address"] and t["start_ms"] > w["end_ms"])
        assert w["data_hex"] == r["data_hex"]
        ax.plot([w["start_ms"], r["start_ms"]], [y, y], color="#c3cbd0", lw=1)
        ax.scatter(w["start_ms"], y, marker="s", s=24, color=BLUE)
        ax.scatter(r["start_ms"], y, marker="o", s=22, color=TEAL)
    ax.set_yticks(range(len(writes)), [f'{w["address"]:02X} = {w["data_hex"].upper()}' for w in writes], fontsize=8)
    ax.invert_yaxis(); ax.set_xlabel("记录内时间（ms）")
    ax.plot([], [], "s", color=BLUE, label="W 写入"); ax.plot([], [], "o", color=TEAL, label="R 读回一致")
    ax.legend(loc="upper right", frameon=False); ax.grid(axis="x", alpha=.2)
    save(fig, "06_startup")
    del s; gc.collect()

    rate, s = load("D3")
    dr = edges(s[4], 0, 1)
    ds = summarize(s[4], rate, True)
    results["D3"] = {k: v for k, v in ds.items() if k != "start_samples"}
    assert len(dr) == 988 and abs(ds["frequency_hz"] - 19.750382) < 1e-6
    fig, axs = plt.subplots(2, 1, figsize=(7.5, 3.25), layout="constrained", gridspec_kw={"height_ratios": [1, 1.05]})
    digital(axs[0], s[4], rate, 0, .28, 1, "INT2", TEAL, 1000)
    digital(axs[0], s[0], rate, 0, .28, 0, "CS", BLUE, 1000)
    wave_axis(axs[0], 0, .28, 1000); axs[0].set_ylim(-.1, 1.9); axs[0].set_xlabel("局部记录时间（ms）")
    axs[1].plot(dr[1:] / rate, np.diff(dr) / rate * 1000, color=TEAL, lw=.75)
    axs[1].axhline(40, color=RED, linestyle="--", lw=1, label="25 Hz 对应 40 ms")
    axs[1].set_ylim(38, 54); axs[1].set_xlim(0, 50)
    axs[1].set_ylabel("相邻 IRQ 周期（ms）"); axs[1].set_xlabel("完整记录内时间（s）")
    axs[1].legend(loc="lower right", frameon=False, fontsize=9); axs[1].grid(alpha=.16)
    save(fig, "07_data_ready")
    del s; gc.collect()

    rate, s = load("E")
    seg = segment_timings(s[4], rate)
    complete = [p for p in seg if p["steady"]["pulse_starts"] > 100 and p["leading_gap_observed"] and p["trailing_gap_observed"]]
    assert len(complete) == 4
    measured = np.array([p["steady"]["frequency_hz"] for p in complete])
    nominal = np.array([100, 25, 50, 100])
    assert np.allclose(measured, [79.269014, 19.750956, 39.502107, 79.233530], atol=1e-6)
    results["E"] = complete
    fig, axs = plt.subplots(1, 2, figsize=(7.5, 2.95), layout="constrained", gridspec_kw={"width_ratios": [1.1, 1]})
    axs[0].plot([0, 105], [0, 105], "--", color=GRAY, label="标称一致线")
    axs[0].scatter(nominal, measured, color=TEAL, s=45, zorder=3)
    for n, m in zip(nominal[:3], measured[:3]): axs[0].annotate(f"{m:.2f}", (n, m), xytext=(-8, -16), textcoords="offset points", fontsize=9)
    axs[0].set(xlim=(0, 110), ylim=(0, 110), xlabel="选择 ODR（Hz）", ylabel="实测 ODR（Hz）")
    axs[0].legend(loc="upper left", fontsize=9, frameon=False); axs[0].grid(alpha=.15)
    ratios = measured / nominal
    axs[1].bar(range(4), ratios, color=[TEAL, BLUE, BLUE, TEAL], width=.58)
    axs[1].set_xticks(range(4), ["100\n阶段2", "25\n阶段3", "50\n阶段4", "100\n阶段5"])
    axs[1].set_ylim(0, 1.12); axs[1].set_ylabel("实测 / 标称")
    axs[1].axhline(1, color=GRAY, ls="--", lw=1)
    for i, v in enumerate(ratios): axs[1].text(i, v + .035, f"{v:.4f}", ha="center", fontsize=9)
    save(fig, "08_sweep")
    del s; gc.collect()

    rate, s = load("F", six=True)
    f = analyze_signals(s, rate)
    phases = [p for p in f["irq_phases"] if p["leading_gap_observed"] and p["trailing_gap_observed"] and p["steady"]["pulse_starts"] > 100]
    assert len(phases) == 3
    assert np.allclose([p["steady"]["frequency_hz"] for p in phases], [15.6252289904, 19.7468349026, 15.6254394655])
    results["F"] = f
    fig, axs = plt.subplots(2, 1, figsize=(7.5, 3.05), layout="constrained", gridspec_kw={"height_ratios": [.7, 1.4]})
    for p, label, color in zip(phases, ["外部 / 阶段3", "内部 / 阶段4", "外部 / 阶段5"], [TEAL, RED, TEAL]):
        axs[0].text((p["start_s"] + p["end_s"]) / 2, 1.22, label, color=color, ha="center", fontsize=9)
    for burst in f["reference_bursts"]:
        if burst["clock_candidate"]:
            axs[0].axvspan(burst["first_sample"] / rate, burst["last_sample"] / rate, ymin=.15, ymax=.65, color=TEAL, alpha=.5)
    axs[0].set_ylim(0, 1.65); axs[0].set_yticks([]); axs[0].set_xlim(0, 50); axs[0].set_ylabel("D5 参考\n活动区间")
    rises = edges(s[4], 0, 1); intervals = np.diff(rises) / rate * 1000
    valid = intervals < 500
    axs[1].scatter(rises[1:][valid] / rate, intervals[valid], s=5, color=BLUE, linewidths=0)
    axs[1].axhline(64, color=TEAL, ls="--", lw=.8)
    axs[1].axhline(40, color=RED, ls="--", lw=.8)
    axs[1].set(xlim=(0, 50), ylim=(0, 72), xlabel="记录内时间（s）", ylabel="相邻 IRQ 间隔（ms）")
    axs[1].annotate("64.217 ms 间隔保留", xy=(7.128998, 64.217), xytext=(10, 69), fontsize=9, color=TEAL,
                    arrowprops={"arrowstyle": "->", "lw": .7, "color": TEAL})
    axs[1].grid(alpha=.16)
    save(fig, "10_ab_long")
    del s; gc.collect()

    rate, s = load("G", six=True)
    g = analyze_signals(s, rate)
    gr, cr = edges(s[4], 0, 1), edges(s[5], 0, 1)
    cycles = np.diff(np.searchsorted(cr, gr)).tolist()
    assert cycles == [2048] and len(cr) == 3200
    results["G"] = {"clock": g, "cycles_between_irq": cycles, "spi": analyze(private / (CAPTURES["G"][0] + ".dsl"))}
    fig, axs = plt.subplots(2, 1, figsize=(7.5, 3.4), layout="constrained", gridspec_kw={"height_ratios": [1.2, 1]})
    digital(axs[0], s[4], rate, 0, .1000038, 1, "INT2", TEAL, 1000)
    digital(axs[0], s[0], rate, 0, .1000038, 0, "CS", BLUE, 1000)
    wave_axis(axs[0], 0, .1000038, 1000); axs[0].set_ylim(-.1, 2.38)
    a, b_ = gr / rate * 1000
    axs[0].annotate("", xy=(a, 1.92), xytext=(b_, 1.92), arrowprops={"arrowstyle": "<->", "color": TEAL})
    axs[0].text((a + b_) / 2, 2.07, "63.99814 ms = 2048 个参考周期", ha="center", fontsize=9, color=TEAL)
    axs[0].set_xlabel("记录内时间（ms）")
    digital(axs[1], s[5], rate, .036, .0362, 0, "D5 / INT1", TEAL, 1e6)
    wave_axis(axs[1], .036, .0362, 1e6); axs[1].set_ylim(-.1, 1.2)
    axs[1].text(.5, .91, "周期 31.20～31.30 μs；高电平 15.60～15.68 μs", transform=axs[1].transAxes, ha="center", fontsize=9)
    axs[1].set_xlabel("记录内时间（μs），36.0～36.2 ms 局部放大")
    save(fig, "11_ab_high")
    del s; gc.collect()

    fig, ax = plt.subplots(figsize=(7.5, 2.45), layout="constrained")
    y = np.array([1, 0]); expected = [25, phases[2]["expected_odr_hz_at_25_setting"]]
    actual = [phases[1]["steady"]["frequency_hz"], phases[2]["steady"]["frequency_hz"]]
    ax.barh(y + .15, expected, height=.27, color="#c7cfd2", label="理论 / 标称")
    ax.barh(y - .15, actual, height=.27, color=[RED, TEAL], label="实测")
    ax.set_yticks(y, ["内部时钟", "外部约 32 kHz"]); ax.set_xlim(0, 30); ax.set_xlabel("输出速率 ODR（Hz）")
    for yy, e, a in zip(y, expected, actual):
        ax.text(e + .4, yy + .15, f"{e:.5f}", va="center", fontsize=9)
        ax.text(a + .4, yy - .15, f"{a:.5f}", va="center", fontsize=9)
    ax.legend(loc="lower right", frameon=False, fontsize=9); ax.grid(axis="x", alpha=.13)
    save(fig, "01_summary")

    fig, ax = plt.subplots(figsize=(7.5, 2.5), layout="constrained")
    ax.axis("off"); ax.set_xlim(0, 3); ax.set_ylim(0, 2)
    labels = ["1  正常 FIFO\n7.64 s 而非约 6 s", "2  SPI / 配置核验\n字节、轴标签、档位正确", "3  独立 DATA_READY\n移除业务仍约 19.75 Hz",
              "4  多档 ODR\n25/50/100 均约 0.79 倍", "5  替换参考时钟\n外部 ODR 符合预期", "6  高速确认\n2048 参考周期 / IRQ"]
    positions = [(0, 1), (1, 1), (2, 1), (0, 0), (1, 0), (2, 0)]
    for (x, y), label in zip(positions, labels):
        ax.add_patch(Rectangle((x + .04, y + .15), .9, .7, fill=False, edgecolor="#aabbbb", lw=1))
        ax.text(x + .49, y + .5, label, ha="center", va="center", fontsize=9.5, linespacing=1.8, color=TEAL)
    for y in [0, 1]:
        for x in [.95, 1.95]: ax.annotate("", (x + .08, y + .5), (x, y + .5), arrowprops={"arrowstyle": "->", "color": GRAY})
    save(fig, "03_evidence_chain")

    fig, ax = plt.subplots(figsize=(7.5, 2.6), layout="constrained")
    ax.axis("off"); ax.set_xlim(0, 10); ax.set_ylim(0, 4)
    boxes = [(0.2, 1, 2.8, 2, "STM32L051\nHSE / PLL → 32 MHz\nTIM2 每 500 tick 翻转 PB0"),
             (4, 1, 2.6, 2, "ADXL362\nINT1 输入参考\n25 Hz 档位 / 2048"),
             (7.4, 1, 2.35, 2, "DSLogic U3Pro16\nD5：输入参考\nD4：DATA_READY")]
    for x, y, w, h, txt in boxes:
        ax.add_patch(Rectangle((x, y), w, h, fill=False, edgecolor=TEAL, lw=1.2))
        ax.text(x + w / 2, y + h / 2, txt, ha="center", va="center", linespacing=1.9, fontsize=9.5)
    ax.annotate("", (4, 2), (3, 2), arrowprops={"arrowstyle": "->", "color": TEAL})
    ax.text(3.5, 2.35, "PB0\n32 kHz", ha="center", fontsize=9)
    ax.annotate("", (7.4, 2), (6.6, 2), arrowprops={"arrowstyle": "->", "color": BLUE})
    ax.text(7, 2.4, "INT2", ha="center", fontsize=9)
    ax.plot([3.5, 3.5, 8.5, 8.5], [2, .55, .55, 1], color=TEAL, lw=.9)
    ax.text(5.6, .1, "共地；INT1 输出映射先关闭并读回；无其他时钟源", ha="center", fontsize=9, color=TEAL)
    save(fig, "09_clock_setup")

    sch = next((source / "PCB/RFID_CC1101_433MHz_V3.8").glob("SCH_*.pdf"))
    doc = pdfium.PdfDocument(sch)
    doc[1].render(scale=6, crop=(105, 370, 505, 100)).to_pil().save(ASSETS / "02_schematic.png")
    manifest["schematic"] = {"file": sch.name, "sha256": hashlib.sha256(sch.read_bytes()).hexdigest().upper(), "page": 2,
                             "crop_pdf_points_left_bottom_right_top": [105, 370, 505, 100]}
    manifest["figure_sources"] = {"01": "F, phases 4/5", "02": "schematic page 2, native crop", "03": "explanatory process diagram",
        "04": "A full 50 s, event starts", "05": "B, 9.5-18.8 ms, all transitions", "06": "C decoded writes/readbacks",
        "07": "D3 0-280 ms all transitions; all 987 intervals", "08": "E full-phase steady windows",
        "09": "explanatory diagram, hardware + diagnostic code", "10": "F whole record, standby intervals >=500 ms omitted from lower plot only",
        "11": "G whole 100 ms IRQ/CS; D5 at 36.0-36.2 ms, all transitions"}
    (HERE / "source_manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (HERE / "reviewed_measurements.json").write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def inline(children):
    out = []
    for t in children or []:
        if t.type == "text": out.append(html.escape(t.content))
        elif t.type == "code_inline":
            font = "Mono" if t.content.isascii() else "Body"
            out.append(f'<font name="{font}" size="8.5">{html.escape(t.content)}</font>')
        elif t.type == "strong_open": out.append('<font name="Bold">')
        elif t.type == "strong_close": out.append('</font>')
        elif t.type == "em_open": out.append('<i>')
        elif t.type == "em_close": out.append('</i>')
        elif t.type == "link_open": out.append(f'<link href="{html.escape(t.attrGet("href"), quote=True)}" color="#176b89">')
        elif t.type == "link_close": out.append('</link>')
        elif t.type == "hardbreak": out.append('<br/>')
        elif t.type == "softbreak": out.append(' ')
        elif t.type != "image": raise ValueError("Unsupported inline: " + t.type)
    return "".join(out)


class ReportDoc(SimpleDocTemplate):
    def afterFlowable(self, flowable):
        if isinstance(flowable, Paragraph) and flowable.style.name == "H2":
            title = flowable.getPlainText()
            key = "section-" + str(self.page)
            self.canv.bookmarkPage(key)
            self.canv.addOutlineEntry(title, key, 0)


def render_pdf():
    pdfmetrics.registerFont(TTFont("Body", str(FONT)))
    pdfmetrics.registerFont(TTFont("Bold", str(BOLD)))
    pdfmetrics.registerFont(TTFont("Mono", "C:/Windows/Fonts/consola.ttf"))
    pdfmetrics.registerFontFamily("Body", normal="Body", bold="Bold", italic="Body", boldItalic="Bold")
    styles = {
        "P": ParagraphStyle("P", fontName="Body", fontSize=9.6, leading=15, spaceAfter=7, wordWrap="CJK", textColor=colors.HexColor("#27363c")),
        "H1": ParagraphStyle("H1", fontName="Bold", fontSize=22, leading=29, spaceAfter=10, textColor=colors.HexColor(TEAL)),
        "H2": ParagraphStyle("H2", fontName="Bold", fontSize=16, leading=22, spaceAfter=10, keepWithNext=True, textColor=colors.HexColor(TEAL)),
        "H3": ParagraphStyle("H3", fontName="Bold", fontSize=11, leading=16, spaceBefore=5, spaceAfter=5, keepWithNext=True),
        "T": ParagraphStyle("T", fontName="Body", fontSize=8.6, leading=12.5, wordWrap="CJK"),
        "TH": ParagraphStyle("TH", fontName="Bold", fontSize=8.6, leading=12.5, textColor=colors.white, wordWrap="CJK"),
        "Code": ParagraphStyle("Code", fontName="Mono", fontSize=8, leading=12, spaceAfter=8, backColor=colors.HexColor("#f1f5f5"), borderPadding=7),
        "Bullet": ParagraphStyle("Bullet", fontName="Body", fontSize=9.6, leading=15, spaceAfter=5, leftIndent=12, firstLineIndent=-10, wordWrap="CJK"),
    }
    parser = MarkdownIt("commonmark").enable("table")
    tokens = parser.parse((HERE / "README.md").read_text(encoding="utf-8"))
    story, i, list_kind, number, pending = [], 0, None, 0, False
    content_w = A4[0] - 84
    while i < len(tokens):
        t = tokens[i]
        if t.type == "heading_open":
            st = {"h1": "H1", "h2": "H2", "h3": "H3"}[t.tag]
            story.append(Paragraph(inline(tokens[i + 1].children), styles[st])); i += 3
        elif t.type == "paragraph_open":
            child = tokens[i + 1].children
            if len(child) == 1 and child[0].type == "image":
                im = Image(str(HERE / child[0].attrGet("src")))
                width = 365 if child[0].attrGet("src").endswith("02_schematic.png") else content_w
                im.drawHeight *= width / im.drawWidth; im.drawWidth = width
                story.extend([im, Spacer(1, 7)])
            else:
                text = inline(child)
                style = "P"
                if pending:
                    number += 1; text = (f"{number}. " if list_kind == "ordered" else "• ") + text
                    style = "Bullet"; pending = False
                story.append(Paragraph(text, styles[style]))
            i += 3
        elif t.type == "table_open":
            rows, header, row, th = [], False, [], False
            i += 1
            while tokens[i].type != "table_close":
                q = tokens[i]
                if q.type == "tr_open": row = []
                elif q.type == "thead_open": header = True
                elif q.type == "thead_close": header = False
                elif q.type in ("th_open", "td_open"): th = header
                elif q.type == "inline": row.append(Paragraph(inline(q.children), styles["TH" if th else "T"]))
                elif q.type == "tr_close": rows.append(row)
                i += 1
            cols = len(rows[0]); weights = {2: [0.4, .6], 3: [.25, .37, .38], 4: [.23, .27, .23, .27], 5: [.1, .27, .19, .22, .22]}[cols]
            if rows[0][0].getPlainText() == "证据":
                weights = [.1, .18, .36, .36]
            table = Table(rows, colWidths=[content_w * w for w in weights], repeatRows=1, hAlign="LEFT")
            table.setStyle(TableStyle([("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#166d71")),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.HexColor("#f0f5f5"), colors.white]),
                ("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 7),
                ("RIGHTPADDING", (0, 0), (-1, -1), 7), ("TOPPADDING", (0, 0), (-1, -1), 5),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
                ("LINEBELOW", (0, 0), (-1, 0), .5, colors.HexColor("#166d71"))]))
            story.extend([table, Spacer(1, 9)]); i += 1
        elif t.type == "html_block" and "pagebreak" in t.content:
            story.append(PageBreak()); i += 1
        elif t.type in ("bullet_list_open", "ordered_list_open"):
            list_kind = "ordered" if t.type.startswith("ordered") else "bullet"; number = 0; i += 1
        elif t.type == "list_item_open": pending = True; i += 1
        elif t.type in ("list_item_close", "bullet_list_close", "ordered_list_close"): i += 1
        elif t.type == "fence":
            story.append(Paragraph(html.escape(t.content).replace("\n", "<br/>"), styles["Code"])); i += 1
        else: raise ValueError("Unhandled Markdown block " + t.type)

    output = ROOT / "output/pdf/ADXL362_25Hz_Supplier_Report.pdf"
    output.parent.mkdir(parents=True, exist_ok=True)
    def chrome(canvas, doc):
        canvas.saveState()
        canvas.setStrokeColor(colors.HexColor("#c7d6d8")); canvas.setLineWidth(.5)
        canvas.line(42, A4[1] - 33, A4[0] - 42, A4[1] - 33)
        canvas.setFillColor(colors.HexColor("#627279")); canvas.setFont("Body", 8)
        canvas.drawString(42, A4[1] - 25, "ADXL362  |  25 Hz 采样率偏差技术复核")
        canvas.drawRightString(A4[0] - 42, A4[1] - 25, "V1.0 · 2026-09-21")
        canvas.drawString(42, 23, "实测事实与工程推断分别列示；非原厂最终失效判定")
        canvas.drawRightString(A4[0] - 42, 23, f"第 {doc.page} 页")
        canvas.restoreState()
    doc = ReportDoc(str(output), pagesize=A4, rightMargin=42, leftMargin=42, topMargin=47, bottomMargin=39,
                    title="ADXL362 采样率偏低问题分析报告", author="RFID V3.8 项目技术记录", subject="Supplier technical review: internal/external clock evidence")
    doc.build(story, onFirstPage=chrome, onLaterPages=chrome)
    rendered = pdfium.PdfDocument(output)
    for idx, page in enumerate(rendered):
        page.render(scale=1.4).to_pil().save(TMP / f"report-page-{idx + 1:02d}.png")
    print(json.dumps({"pdf": str(output), "pages": len(rendered), "figures": len(list(ASSETS.glob('*.png')))}, ensure_ascii=False))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--source-root", type=Path)
    ap.add_argument("--pdf-only", action="store_true")
    args = ap.parse_args()
    ASSETS.mkdir(parents=True, exist_ok=True); TMP.mkdir(parents=True, exist_ok=True)
    if not args.pdf_only:
        if not args.source_root: ap.error("--source-root is required when rebuilding evidence figures")
        build_figures(args.source_root)
    render_pdf()
