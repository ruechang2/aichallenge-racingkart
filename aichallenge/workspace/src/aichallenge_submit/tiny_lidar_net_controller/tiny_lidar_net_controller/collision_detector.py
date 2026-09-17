"""壁・カートへの衝突検知。

入力は 2 系統あり、優先度が違う。

1. 車両ダメージ ``/aichallenge/pitstop/condition``（``std_msgs/Int32``）— 最優先
    シミュレータが判定した結果そのものなので、来ているならこれだけを見る。
    Condition は「たまったダメージ量」で、衝突すると増え、修理で減る
    （AWSIM のアイテム説明文 "How much damage (Condition) is removed when
    collected." より）。したがって増加方向のジャンプだけを衝突とみなす。
    閾値 30 は ``multi_purpose_mpc_ros`` のサンプルに合わせた。

2. 車輪速と LiDAR からの推定 — 上が来ないときのフォールバック
    **手元の AWSIM（AIChallenge2026）は 1 の topic を publish しない。**
    アセンブリ内に ``/aichallenge/`` で始まる topic 名の文字列が 1 つも無く、
    レース中に ``ros2 topic info`` で見ても Publisher count は 0 だった
    （list に出るのは自分の購読分）。MPC サンプルの購読は 2025 年の
    SW 部門向けビルドの名残とみられる。実車にもこの topic は無い。
    そのため、実際に働くのは通常こちらになる。

    a. 急減速 (``impact``)
       車輪速が制動では説明できない速さで落ちた。加速度指令の下限は -1.6 m/s^2、
       転がり抵抗 0.37 m/s^2 を足しても 2.0 m/s^2 程度なので、それを超える減速は
       接触以外に理由がない。正面でも側面の擦りでも速度は落ちるため、どちらも拾える。
    b. 前方接触 (``contact``)
       LiDAR の前方扇形が車体のすぐ前まで詰まったまま、速度が出ていない。
       LiDAR は base_link から x=1.65 m、車体前端は 1.554 m
       （wheel_base 1.087 + front_overhang 0.467）なので前端のほぼ真上にあり、
       前方距離をそのまま「前端からの隙間」とみなせる。緩やかに押し付けて
       止まった（a を取り逃がす）場合の受け皿。

壁とカートは区別しない。復帰の手順はどちらでも同じ（空いている側へ切って脱出する）
であり、区別に必要な物体認識は E2E の入力からは得られないため。
"""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import numpy as np

from lidar_sector import sector_clearance


@dataclass
class CollisionEvent:
    """検知した衝突の内容。ログとチューニングのための記録。

    Attributes:
        kind: ``"damage"``（Condition 由来）、``"impact"``（急減速）、
            ``"contact"``（前方が詰まったまま停止）のいずれか。
        detail: 判定の根拠を人が読める形にしたもの。ログにそのまま出す。
    """

    kind: str
    detail: str


class CollisionDetector:
    """衝突イベントを検知する。

    ダメージ topic が来ているなら :meth:`update_condition`、来ていないなら
    :meth:`update`（車輪速と LiDAR）で判定する。どちらも衝突した瞬間だけ
    :class:`CollisionEvent` を返し、それ以外は ``None`` を返す。
    検知するだけで制御には介入しない。実際に抜け出す動きは
    :class:`stuck_recovery.StuckRecovery` の担当。
    """

    def __init__(
        self,
        damage_threshold: int = 30,
        damage_window: float = 1.0,
        decel_threshold: float = 4.0,
        min_impact_speed: float = 1.0,
        contact_range: float = 0.8,
        contact_speed: float = 0.5,
        contact_duration: float = 0.5,
        front_angle_deg: Tuple[float, float] = (-20.0, 20.0),
        arm_speed: float = 1.0,
        refractory_duration: float = 1.5,
        max_dt: float = 0.5,
    ):
        """衝突検知を初期化する。

        Args:
            damage_threshold: この量だけダメージが増えたら衝突とみなす。
            damage_window: ダメージの増加量を見る時間窓 [s]。単発の衝突はこの窓に
                関係なくすぐ閾値を超える。窓が効くのは擦り続けている場合。
            decel_threshold: これを超える減速度を衝突とみなす [m/s^2]。
                制動で出せる 2.0 m/s^2 程度より十分大きくとる。
            min_impact_speed: 急減速判定に必要な衝突直前の速度 [m/s]。
                停止寸前のふらつきを衝突と誤認しないためのガード。
            contact_range: 前方がこの距離以下なら接触とみなす [m]。
            contact_speed: 接触判定に必要な速度の上限 [m/s]。
                前走車に近づいているだけの場面を除くためのガード。
            contact_duration: 接触とみなすまでの継続時間 [s]。
            front_angle_deg: 前方接触を見る扇形の角度範囲 [deg]。
            arm_speed: 一度この速度を超えるまでセンサ由来の検知を作動させない [m/s]。
                スタート前のグリッドは前走車が目の前にいるため、これがないと
                並んで待っているだけで接触と判定されてしまう。
            refractory_duration: 一度検知してから次を検知するまでの不感時間 [s]。
                1 回の衝突で何度も発火させないためのもの。
            max_dt: 減速度の計算に使うスキャン間隔の上限 [s]。これを超えたら
                取りこぼしとみなして減速度を計算しない。
        """
        self.damage_threshold = damage_threshold
        self.damage_window = damage_window
        self.decel_threshold = decel_threshold
        self.min_impact_speed = min_impact_speed
        self.contact_range = contact_range
        self.contact_speed = contact_speed
        self.contact_duration = contact_duration
        self.front_angle_deg = front_angle_deg
        self.arm_speed = arm_speed
        self.refractory_duration = refractory_duration
        self.max_dt = max_dt

        # ダメージ topic を 1 度でも受け取ったか。受け取ったらセンサ推定は止める。
        self.condition_active = False
        self._damage_history: Deque[Tuple[float, int]] = deque()
        self._armed = False
        self._prev: Optional[Tuple[float, float]] = None
        self._contact_since: Optional[float] = None
        self._last_event_at: Optional[float] = None
        self.event_count = 0
        self.last_event: Optional[CollisionEvent] = None

    def reset(self) -> None:
        """状態を初期化する。走行のやり直し時に呼ぶ。"""
        self.condition_active = False
        self._damage_history.clear()
        self._armed = False
        self._prev = None
        self._contact_since = None
        self._last_event_at = None
        self.event_count = 0
        self.last_event = None

    def update_condition(
        self, now: float, condition: int, suppress_events: bool = False
    ) -> Optional[CollisionEvent]:
        """車両ダメージを 1 件受け取り、衝突した瞬間だけイベントを返す。

        1 件でも受け取ったら :attr:`condition_active` が立ち、以後 :meth:`update`
        のセンサ推定は止まる。同じ衝突を 2 系統で二重に拾わないため。

        Args:
            now: 単調増加する現在時刻 [s]。
            condition: ``/aichallenge/pitstop/condition`` の値。
            suppress_events: True の間はイベントを返さない。

        Returns:
            衝突を検知したら :class:`CollisionEvent`、それ以外は ``None``。
        """
        self.condition_active = True
        self._damage_history.append((now, condition))
        self._trim_damage_history(now)

        if suppress_events or self._in_refractory(now):
            return None

        damage = condition - self._damage_history[0][1]
        if damage < self.damage_threshold:
            return None

        # 同じダメージを次の窓でも数えないよう、履歴を今の値だけにする。
        self._damage_history.clear()
        self._damage_history.append((now, condition))
        return self._fire("damage", f"+{damage} (condition={condition})", now)

    def update(
        self,
        now: float,
        speed: float,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
        suppress_events: bool = False,
    ) -> Optional[CollisionEvent]:
        """1 スキャン分の状態を更新し、衝突した瞬間だけイベントを返す。

        ダメージ topic が来ている間は何もしない（そちらが正しいため）。

        Args:
            now: 単調増加する現在時刻 [s]。
            speed: 現在の車速 [m/s]（Wheel Odometry 由来）。
            ranges: LiDAR の距離配列（前処理前の生値）。
            angle_min: LiDAR の最小角 [rad]。
            angle_increment: LiDAR の角度分解能 [rad]。
            suppress_events: True の間はイベントを返さない。既に復帰シーケンスが
                動いているときに使う。壁に押し付けたまま脱出を試している最中は
                接触が続くのが当たり前で、通知しても復帰の動きは変わらないため。
                減速度や接触継続時間の計算自体は続けるので、抑止が解けた直後から
                通常どおり判定できる。

        Returns:
            衝突を検知したら :class:`CollisionEvent`、それ以外は ``None``。
        """
        if self.condition_active:
            return None

        prev = self._prev
        self._prev = (now, speed)

        # スタート前のグリッド停止で誤発火しないよう、一度走り出すまで作動させない。
        if not self._armed:
            if abs(speed) >= self.arm_speed:
                self._armed = True
            return None

        front = sector_clearance(
            ranges, angle_min, angle_increment, self.front_angle_deg, percentile=10.0)

        decel = self._decel(prev, now, speed)
        contact_held = self._update_contact(now, speed, front)

        if suppress_events or self._in_refractory(now):
            return None

        if (prev is not None and abs(prev[1]) >= self.min_impact_speed
                and decel is not None and decel <= -self.decel_threshold):
            return self._fire(
                "impact",
                f"decel={decel:.1f} m/s^2, speed={speed:.2f} m/s, front={front:.2f} m",
                now)

        if contact_held:
            return self._fire(
                "contact", f"front={front:.2f} m, speed={speed:.2f} m/s", now)

        return None

    def _trim_damage_history(self, now: float) -> None:
        """時間窓の外に出たダメージ履歴を捨てる。

        窓の左端をまたぐ 1 件は残す。それが「窓の始まりのダメージ量」であり、
        差分の基準になるため。
        """
        horizon = now - self.damage_window
        while len(self._damage_history) >= 2 and self._damage_history[1][0] <= horizon:
            self._damage_history.popleft()

    def _decel(
        self, prev: Optional[Tuple[float, float]], now: float, speed: float
    ) -> Optional[float]:
        """前回スキャンとの差から減速度を求める [m/s^2]。負が減速。

        スキャンを取りこぼして間隔が開いたときは、平均化されて衝突の鋭さが
        失われるため計算しない。
        """
        if prev is None:
            return None
        dt = now - prev[0]
        if dt <= 0.0 or dt > self.max_dt:
            return None
        return (abs(speed) - abs(prev[1])) / dt

    def _update_contact(self, now: float, speed: float, front: float) -> bool:
        """前方接触の継続状態を更新し、継続時間を満たしたら True を返す。"""
        if front <= self.contact_range and abs(speed) <= self.contact_speed:
            if self._contact_since is None:
                self._contact_since = now
            return (now - self._contact_since) >= self.contact_duration
        self._contact_since = None
        return False

    def _in_refractory(self, now: float) -> bool:
        """直前の検知からの不感時間内かどうか。"""
        if self._last_event_at is None:
            return False
        return (now - self._last_event_at) < self.refractory_duration

    def _fire(self, kind: str, detail: str, now: float) -> CollisionEvent:
        """イベントを確定させ、不感時間を開始する。"""
        self._last_event_at = now
        self._contact_since = None
        self.event_count += 1
        self.last_event = CollisionEvent(kind=kind, detail=detail)
        return self.last_event
