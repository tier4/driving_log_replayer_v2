"""T4 dataset パス解決の単一リゾルバ (ローカル/クラウド共通契約).

クラウド (Web.Auto evaluator) とローカル (`make local_cloud_run`) は、いずれも
launch に `t4_dataset_path` を**直接**渡す。両者が満たす共通契約は
「`input_bag/` と `map/` を直下に持つディレクトリ」であること。

- クラウド: Web.Auto が対象 dataset を固定マウント点 (例 `…/data/t4_dataset`) に
  事前ステージし、その path を注入する。UUID / frame 階層は無い (フラット)。
- ローカル: webauto CLI `annotation-dataset pull` の出力
  `~/.webauto/data/data/annotation_dataset/<UUID>/<frame>/` を解決して渡す。

本モジュールは**ローカル側の解決**を一本化したもので、Web.Auto ステージングの
ローカル版に相当する。`Makefile` の shell `find` ロジックを置き換える SSOT。
標準ライブラリのみに依存し、`install/setup.bash` を source する前 (= ROS 環境
無し) でもファイルパス直接実行で動く。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

# input_bag 内に存在を要求する rosbag ファイルの拡張子 (Makefile の検証と一致)。
_BAG_GLOBS = ("*.mcap", "*.db3")


class DatasetResolutionError(RuntimeError):
    """T4 dataset パスを解決できなかったときに送出する例外。"""


def _has_valid_input_bag(dataset_dir: Path) -> bool:
    """`dataset_dir/input_bag/` が存在し、*.mcap / *.db3 を 1 つ以上含むか。"""
    input_bag = dataset_dir / "input_bag"
    if not input_bag.is_dir():
        return False
    return any(next(iter(input_bag.glob(g)), None) is not None for g in _BAG_GLOBS)


def _find_uuid_dir(root: Path, uuid: str) -> Path | None:
    """`root` 配下から名前が `uuid` のディレクトリを探す。

    まず `root/<uuid>` を直接見て (通常ケース)、無ければ root を再帰探索する
    (UUID dir が root 直下とは限らない前提)。`.lock` 配下は除外。
    """
    direct = root / uuid
    if direct.is_dir():
        return direct
    for cand in sorted(root.rglob(uuid)):
        if cand.is_dir() and ".lock" not in cand.parts:
            return cand
    return None


def resolve_t4_dataset_path(root: Path | str, uuid: str) -> Path:
    """webauto キャッシュ `root` と dataset `uuid` から t4_dataset_path を解決する.

    解決規則:
      1. `root` 配下の `<uuid>` ディレクトリを探す (直下優先、無ければ再帰探索)。
      2. その dir 直下に有効な `input_bag/` があればそれを返す
         (フラット / クラウド相当レイアウト)。
      3. 無ければ frame サブディレクトリ群のうち **最新 (名前 sort の末尾)** で
         有効な `input_bag/` を含むものを返す
         (webauto CLI pull の `<UUID>/<frame>` レイアウト)。
      4. いずれも見つからなければ `DatasetResolutionError` を送出する。

    Returns:
        `input_bag/` と `map/` を直下に持つ T4 dataset ディレクトリの絶対パス。

    Raises:
        DatasetResolutionError: UUID dir が無い、または有効な input_bag が無い場合。
    """
    root = Path(root)
    uuid_dir = _find_uuid_dir(root, uuid)
    if uuid_dir is None:
        raise DatasetResolutionError(
            f"dataset {uuid} が {root} に見つかりません。\n"
            "次のコマンドで取得してください:\n"
            "  webauto data annotation-dataset pull \\\n"
            "    --project-id x2_dev \\\n"
            f"    --annotation-dataset-id {uuid} \\\n"
            "    --include-intermediate-artifacts"
        )

    # 2. フラット (クラウド相当): input_bag/ が UUID dir 直下にある。
    if _has_valid_input_bag(uuid_dir):
        return uuid_dir.resolve()

    # 3. frame レイアウト (ローカル webauto CLI pull): 最新 frame を採用。
    frame_dirs = sorted(
        (d for d in uuid_dir.iterdir() if d.is_dir()),
        key=lambda p: p.name,
    )
    for frame_dir in reversed(frame_dirs):
        if _has_valid_input_bag(frame_dir):
            return frame_dir.resolve()

    # 4. UUID dir はあるが有効な input_bag が無い → intermediate artifacts 不足。
    raise DatasetResolutionError(
        f"{uuid_dir} 配下に input_bag/*.{{mcap,db3}} が見つかりません。\n"
        "--include-intermediate-artifacts を付けて再 pull してください:\n"
        "  webauto data annotation-dataset pull \\\n"
        "    --project-id x2_dev \\\n"
        f"    --annotation-dataset-id {uuid} \\\n"
        "    --include-intermediate-artifacts"
    )


def main(argv: list[str] | None = None) -> int:
    """CLI エントリポイント。解決パスを stdout に出力、失敗時は stderr + exit 1。"""
    parser = argparse.ArgumentParser(
        description="T4 dataset の t4_dataset_path を webauto キャッシュ root と UUID から解決する。",
    )
    parser.add_argument(
        "--root",
        required=True,
        help="webauto キャッシュルート (例: ~/.webauto/data/data/annotation_dataset)",
    )
    parser.add_argument("--uuid", required=True, help="annotation-dataset の UUID")
    args = parser.parse_args(argv)

    try:
        print(resolve_t4_dataset_path(args.root, args.uuid))
    except DatasetResolutionError as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
