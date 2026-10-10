import learning_demo
ct_result = learning_demo.build_ct()
mri_result = learning_demo.build_mri()
_result = {"CT": ct_result, "MRI": mri_result}
(ROOT / "work" / "scene_build_results.json").write_text(__import__('json').dumps(_result, ensure_ascii=False, indent=2), encoding="utf-8")
