"""Process-pool sizing helpers for reidentify."""

from __future__ import annotations

import os

def normalize_parallel_jobs(
    n_jobs: int,
    *,
    n_tasks: int | None = None,
) -> int:
    """Validate and optionally clamp worker count to task count."""
    value = int(n_jobs)
    if value < 1:
        raise ValueError("n_jobs must be at least 1")
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
