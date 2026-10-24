# LARK Registration-Error Results: Data Dictionary

Two CSV files accompany the LARK multi-view video dataset. They contain the per-measurement registration-error results computed from the videos: every fiducial, target, and trajectory error, for both LARK tracking pipelines and the gold-standard reference tracker, across all trials, camera configurations, and filtering variants. These are the data underlying Table 1, Table C.1, and Figures 8 to 11 and C.1 of the paper.

- `Dataset_1__Grid_.csv`: machined-grid experiment (99,760 rows)
- `Dataset_2__Head_Phantom_.csv`: head-phantom experiment (34,440 rows)

Each recording in the video dataset comprises the five synchronized camera videos plus an `IGTPos.txt` file containing the simultaneous fusionTrack 500 measurement stream: one row per sample, three whitespace-separated values giving the calibrated pointer tip position (x, y, z) in millimeters, expressed in the frame of the fusionTrack's static reference pattern. This frame is distinct from the LARK static-reference frame; the paper compares the two systems after landmark registration (Appendix B). Point recordings hold the pointer stationary at a landmark; trajectory recordings slide it along a groove.

Each row of the CSV files is one registration-error measurement. All errors are in millimeters.

| Column | Values | Meaning |
|---|---|---|
| `Trial` | 1 to 20 | Experimental trial |
| `Device` | `Ring`, `Atracsys` | `Ring` is the LARK multi-camera system; `Atracsys` is the fusionTrack 500 gold-standard reference. `Atracsys` rows have empty `Config`, `Tracking`, `Fusion`, and `Cams` |
| `Config` | Grid: `C0` to `C30`; Head: `top`, `right`, `left` | Grid: camera-subset ID, one per non-empty subset of the five cameras (1 five-camera, 5 four-camera, 10 three-camera, 10 two-camera, 5 one-camera). Head: occlusion scenario relative to the operating table (Fig. 7), with severity given by `Cams` |
| `Point` | Grid: `01` to `25`, `RT1` to `RT4`; Head: `R1` to `R9`, `01` to `10`, `RF`, `RB` | Grid: numbered grid points, where `01`, `03`, `05`, `11`, `13`, `15`, `21`, `23`, `25` are the nine fiducials and the remaining sixteen are targets; `RT1` to `RT4` are the four S-groove trajectory placements. Head: `R1` to `R9` are the anatomical fiducials of Fig. 6b, `01` to `10` the cranial target points, and `RF` and `RB` the frontal and parietal trajectory grooves |
| `Tracking` | `Mono`, `Stereo` | `Mono` is multi-view monocular pose fusion (Sec. 3.4.1); `Stereo` is multi-view triangulation (Sec. 3.4.2). Triangulation requires at least two cameras, so no `Stereo` rows exist at `Cams` = 1 |
| `Fusion` | `Basic`, `Kalman`, `Kalman Adaptive` | Filtering variant: no filter, static-covariance Kalman filter (Appendix A), or adaptive Kalman filter (Sec. 3.4.1) |
| `Cams` | 1 to 5 | Number of active cameras |
| `Metric` | `FRE`, `TRE`, `Traj_RMSE` | Fiducial registration error, point target registration error, or trajectory RMSE computed by nearest-neighbour matching (Appendix B) |
| `Error` | float | Error value in mm |

Notes:

- Every video recording contains all five camera views. Camera subsets and occlusion scenarios were applied during analysis by excluding views from the full five-camera recordings; there are no separately recorded occluded trials.

- In the head-phantom file, the five-camera (no occlusion) condition appears under each of the three scenario labels.
- These files contain computed errors. The raw data are per recording: the five camera videos and the fusionTrack measurement stream (`IGTPos.txt`). The landmark coordinates and reference trajectory point sets used for registration-based FRE and TRE are distributed with the evaluation code (see the software link below).
- Reference: Sideris et al., "LARK: A Low-Cost, Accurate, Occlusion-Resilient, Kalman Filter-Assisted Tracking System for Image-Guided Surgery." Hardware designs and software: https://nist.mni.mcgill.ca/software/
