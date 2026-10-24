"""Explicit statistics for point-localization data; no conformity claim.

Rows marked invalid are excluded and counted, not turned into zero error.
Reported precision and standard deviation use population denominators N.
Repetitions/independent units must be identified by the caller's protocol.
"""
import numpy as np


def point_statistics(measured, truth, valid=None):
    measured = np.asarray(measured, dtype=float)
    truth = np.asarray(truth, dtype=float)
    if measured.ndim != 2 or measured.shape[1] != 3:
        raise ValueError("Measured points must be N x 3")
    if truth.shape == (3,):
        truth = np.broadcast_to(truth, measured.shape)
    if truth.shape != measured.shape or not np.all(np.isfinite(truth)):
        raise ValueError("True point coordinates must be finite and correspond to each row")
    if valid is None:
        valid = np.ones(len(measured), dtype=bool)
    valid = np.asarray(valid, dtype=bool)
    if valid.shape != (len(measured),):
        raise ValueError("One validity flag is required per measurement")
    valid = valid & np.all(np.isfinite(measured), axis=1)
    count = int(np.sum(valid))
    result = {"n_total": len(measured), "n_valid": count, "n_invalid": len(measured)-count,
              "valid_fraction": count/len(measured) if len(measured) else None,
              "bias_vector_mm": None, "bias_norm_mm": None, "mean_distance_mm": None,
              "rms_distance_mm": None, "max_distance_mm": None, "p95_distance_mm": None,
              "precision_rms_mm": None, "error_vector_dispersion_rms_mm": None,
              "span_mm": None,
              "constant_truth_point": bool(len(truth) and np.allclose(truth, truth[0], atol=1e-9, rtol=0)),
              "distance_sd_population_mm": None}
    if not count:
        return result
    errors = measured[valid] - truth[valid]
    mean_vector = errors.mean(axis=0)
    distances = np.linalg.norm(errors, axis=1)
    result.update({"bias_vector_mm": mean_vector.tolist(), "bias_norm_mm": float(np.linalg.norm(mean_vector)),
        "mean_distance_mm": float(distances.mean()),
        "rms_distance_mm": float(np.sqrt(np.mean(distances**2))),
        "max_distance_mm": float(distances.max()), "p95_distance_mm": float(np.percentile(distances, 95)),
        "error_vector_dispersion_rms_mm": float(np.sqrt(np.mean(np.sum((errors-mean_vector)**2, axis=1)))),
        "distance_sd_population_mm": float(distances.std(ddof=0))})
    if result["constant_truth_point"]:
        result["precision_rms_mm"] = result["error_vector_dispersion_rms_mm"]
        result["span_mm"] = float(np.max(np.linalg.norm(errors[:, None, :]-errors[None, :, :], axis=2)))
    return result


def pairwise_distance_statistics(measured, truth):
    measured, truth = np.asarray(measured, dtype=float), np.asarray(truth, dtype=float)
    if measured.shape != truth.shape or measured.ndim != 2 or measured.shape[1] != 3:
        raise ValueError("Corresponding N x 3 point sets required")
    if not np.all(np.isfinite(measured)) or not np.all(np.isfinite(truth)):
        raise ValueError("Pairwise input must be finite; do not mix missing points")
    errors = [abs(np.linalg.norm(measured[i]-measured[j])-np.linalg.norm(truth[i]-truth[j]))
              for i in range(len(measured)) for j in range(i+1, len(measured))]
    return {"point_count": len(measured), "pair_count": len(errors),
            "mean_abs_distance_error_mm": float(np.mean(errors)) if errors else None,
            "rms_distance_error_mm": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            "max_abs_distance_error_mm": float(np.max(errors)) if errors else None}
