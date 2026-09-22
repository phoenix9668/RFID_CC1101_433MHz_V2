WITH "firmware_results" ("recording", "file_name", "expected_behavior", "expected_label", "processed_samples_after_dedup", "breath_samples", "breath_sample_rate") AS (
  VALUES
  ('反刍对照', '05项圈 5826牛 0302 1549  反刍.csv', 'rumination', '反刍对照', 30450, 0, 0.0),
  ('喘息 2025-08-19 16:22', '喘息20250819 1622.csv', 'panting', '喘息记录', 19650, 16725, 0.851145038167939),
  ('喘息 2025-08-20 16:44', '喘息20250820 1644.csv', 'panting', '喘息记录', 77700, 23950, 0.3082368082368082),
  ('喘息 2025-08-20 17:55', '喘息20250820 1755.csv', 'panting', '喘息记录', 31863, 23563, 0.7395097762294824)
)
SELECT * FROM "firmware_results";
