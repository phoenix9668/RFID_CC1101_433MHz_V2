WITH "robustness_results" ("scenario", "detail", "panting_windows", "panting_window_pass_rate", "rumination_windows", "rumination_window_rejection_rate", "balanced_descriptive_rate") AS (
  VALUES
  ('主分析', '25 Hz, 90 s, raw magnitude, rectangular taper, deduplicated', 56, 0.5892857142857143, 13, 0.9230769230769232, 0.7561813186813187),
  ('Hann 窗', 'Hann taper', 56, 0.5892857142857143, 13, 1.0, 0.7946428571428572),
  ('主导轴', 'dominant peak-to-peak axis', 56, 0.6071428571428571, 13, 0.8461538461538461, 0.7266483516483516),
  ('PCA 投影', 'PCA first component', 56, 0.5714285714285714, 13, 0.8461538461538461, 0.7087912087912087),
  ('一阶差分幅值', 'magnitude of first differences', 56, 0.625, 13, 0.0769230769230769, 0.3509615384615384),
  ('30 秒窗', '30 s windows', 171, 0.543859649122807, 40, 0.825, 0.6844298245614036),
  ('180 秒窗', '180 s windows', 28, 0.6428571428571429, 6, 1.0, 0.8214285714285714),
  ('误按 10 Hz', 'incorrectly assume paper''s 10 Hz for 25 Hz data', 142, 0.6549295774647887, 33, 0.3333333333333333, 0.494131455399061)
)
SELECT * FROM "robustness_results";
