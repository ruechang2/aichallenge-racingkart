"""LiDAR スキャンの扇形から距離の代表値を取り出す共通処理。

スタック復帰（左右どちらが空いているかを見る）と衝突検知（前方が詰まっているかを
見る）で同じ計算を使うため、ここに切り出している。
"""

import math
from typing import Tuple

import numpy as np


def sector_clearance(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    angle_range_deg: Tuple[float, float],
    percentile: float = 50.0,
) -> float:
    """指定した角度範囲の距離を代表値 1 つに畳む。

    Args:
        ranges: LiDAR の距離配列（前処理前の生値）。
        angle_min: LiDAR の最小角 [rad]。
        angle_increment: LiDAR の角度分解能 [rad]。
        angle_range_deg: 扇形の角度範囲 [deg]。正面を 0 度、左を正とする。
        percentile: 扇形内の距離のうち何パーセンタイルを返すか。
            50 なら中央値で、1 点の外れ値に振られずに「その側の空き具合」を測れる。
            小さい値にすると最短距離寄りになり、接触判定のように「一番近い物体まで
            何 m か」を知りたい場合に使う。

    Returns:
        扇形内の距離の代表値 [m]。有効な点が 1 つもなければ ``inf``。
        LiDAR が測り損ねた方向を「詰まっている」と誤解しないための既定値。
    """
    if ranges.size == 0 or angle_increment == 0.0:
        return math.inf
    angles = angle_min + np.arange(ranges.size) * angle_increment
    lo, hi = np.deg2rad(angle_range_deg)
    sector = ranges[(angles >= lo) & (angles <= hi)]
    # 非有限値は「遠すぎて返らなかった」、0 は「測れなかった」点。どちらも
    # 距離 0 として扱うと接触と区別がつかなくなるので、母集団から外す。
    sector = sector[np.isfinite(sector) & (sector > 0.0)]
    if sector.size == 0:
        return math.inf
    return float(np.percentile(sector, percentile))
