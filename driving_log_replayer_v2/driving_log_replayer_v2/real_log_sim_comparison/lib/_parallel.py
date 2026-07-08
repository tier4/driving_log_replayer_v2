"""Shared parallelism helpers for real_log_sim_comparison.

All scripts should resolve their worker count through this module so CLI defaults,
environment overrides, pool sizing, and chunk sizing stay consistent.
"""

from __future__ import annotations

import os

PARALLEL_JOBS_ENV = "REAL_LOG_SIM_COMPARISON_JOBS"


def _cpu_count() -> int:
    return max(1, os.cpu_count() or 1)


def default_parallel_jobs(*, cap: int | None = None) -> int:
    """Return the shared default worker count.

    `REAL_LOG_SIM_COMPARISON_JOBS` overrides CPU count when set to a positive
    integer. `cap` can keep lightweight commands conservative.
    """
    raw = os.environ.get(PARALLEL_JOBS_ENV)
    if raw:
        try:
            value = int(raw)
        except ValueError:
            value = _cpu_count()
    else:
        value = _cpu_count()
    value = max(1, value)
    if cap is not None:
        value = min(value, max(1, int(cap)))
    return value


def normalize_parallel_jobs(
    n_jobs: int | None,
    *,
    n_tasks: int | None = None,
    default: int | None = None,
) -> int:
    """Normalize and optionally clamp worker count to task count."""
    value = int(n_jobs) if n_jobs and int(n_jobs) > 0 else (default or default_parallel_jobs())
    value = max(1, value)
    if n_tasks is not None:
        value = min(value, max(1, int(n_tasks)))
    return value


def pool_chunksize(n_items: int, n_jobs: int, *, factor: int = 4) -> int:
    """Chunk size used by multiprocessing pool map/imap calls."""
    jobs = normalize_parallel_jobs(n_jobs)
    return max(1, int(n_items) // (jobs * max(1, int(factor))))


def set_worker_thread_env_defaults(num_threads: int = 1) -> None:
    """Limit BLAS/OpenMP thread fan-out inside process pools unless caller set it."""
    value = str(max(1, int(num_threads)))
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ.setdefault(name, value)
