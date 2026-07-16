"""Process-pool sizing helpers for reidentify."""

from __future__ import annotations

import multiprocessing.pool
import os
import sys
from typing import Callable, TypeVar

_T = TypeVar("_T")
_R = TypeVar("_R")

# imap_with_watchdog の既定タイムアウト: 通常の単一バッチ評価は数秒以内に完結する
# ワークロードなので、この値は十分に余裕を持たせつつ、fork 直後の worker デッドロック
# (jemalloc 背景スレッド等が fork 時にロックを保持していた場合など) を検出したときに
# 数時間ではなく数分でパイプラインを失敗させることを目的としている。
DEFAULT_IMAP_WATCHDOG_TIMEOUT_S = 300.0


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
    _force_pyarrow_system_pool()


def _force_pyarrow_system_pool() -> None:
    """pandas が連鎖 import する pyarrow の既定アロケータを system に強制する。

    fit_plateau の無限ハング事例の調査で最有力の疑いとして浮上した仮説への対策:
    jemalloc (Arrow がバンドル) の背景スレッドが内部ロックを保持した瞬間に
    multiprocessing.get_context("fork") で子プロセスを fork すると、ロックを解放する
    はずのスレッドが子に存在せず、子の最初の malloc でデッドロックしうるという既知の
    危険パターン。ただし再現テストでは再現せず未確定。副作用のない対策のため
    Makefile 側の ARROW_DEFAULT_MEMORY_POOL=system と二重に先んじて入れている。
    """
    if "pyarrow" not in sys.modules:
        return
    import pyarrow

    pyarrow.set_memory_pool(pyarrow.system_memory_pool())


def imap_with_watchdog(
    pool: multiprocessing.pool.Pool,
    func: Callable[[_T], _R],
    tasks: list[_T],
    *,
    chunksize: int = 1,
    timeout_s: float = DEFAULT_IMAP_WATCHDOG_TIMEOUT_S,
) -> list[_R]:
    """pool.imap 相当 (投入順の全結果) を回収する。fork 直後にデッドロックした worker
    (jemalloc 背景スレッド等が fork 時に内部ロックを保持していた場合など) が1個でも
    いると、他の worker が完了していてもタスク全体が永久に返らない。timeout_s 秒以内
    に全件完了しない場合は無限待ちせず明確に失敗させる。

    chunksize > 1 のとき pool.imap() が返すのは IMapIterator ではなく素の generator
    (`.next(timeout=)` を持たない) になるため、代わりに map_async().get(timeout=) で
    一括タイムアウトを掛ける。

    タイムアウト時は pool.terminate() を呼んでから例外を投げる: デッドロックした worker
    は close()+join() では回収できず (worker が自発的に終了しないため join() が再度
    無限に待ち続ける)、呼び出し元の `finally: pool.close(); pool.join()` を再ハング
    させてしまう。terminate() で全 worker に SIGTERM を送ってから返せば、その後の
    close()/join() は即座に完了する (すでに terminate 済みの pool に対して呼んでも
    安全)。
    """
    async_result = pool.map_async(func, tasks, chunksize=chunksize)
    try:
        return async_result.get(timeout=timeout_s)
    except multiprocessing.TimeoutError as exc:
        alive_pids = [p.pid for p in getattr(pool, "_pool", []) if p.is_alive()]
        pool.terminate()
        raise RuntimeError(
            f"pool worker から {timeout_s:.0f}秒経過しても {len(tasks)} 件の結果が"
            " 返りません。fork 直後の worker デッドロック (jemalloc 背景スレッド等が"
            f" fork 時にロックを保持していた場合など) の可能性があります。生存中の"
            f" worker PID: {alive_pids} を terminate しました。make を再実行してください。"
        ) from exc
