"""スタック・衝突からの復帰ロジック。

決勝では車両がスタックしても運営による復帰は行われない（AWSIM は
``--wall-recovery off`` で起動する）ため、自力で抜け出せないとそこでレースが終わる。
一方で E2E 部門は単一 AI モデルが期待されているので、この層は
「NN が明らかに詰んだときだけ動く最小限のフェイルセーフ」に留める。通常走行中は
一切介入しない。

発火の 2 経路:
    1. 低速の継続
       「動けと指令しているのに動いていない」状態が ``stuck_duration`` 続いたとき。
       じわじわ詰まった場合の受け皿。
    2. 衝突の通知
       :class:`collision_detector.CollisionDetector` が壁・カートへの衝突を
       検知したとき。:meth:`notify_collision` で通知を受け、その直後は
       ``collision_stuck_duration``（低速継続よりずっと短い）で発火する。
       衝突後に NN が減速側の指令を出していても発火する点も 1 と違う。
       ぶつかって止まったのに「アクセルを踏んでいないから」で待たされると、
       壁に張り付いたままレースが終わってしまうため。

後退の扱いについて:
    この車両の LiDAR は前方 180 度（-90 度 〜 +90 度）しかカバーしておらず、E2E 部門では
    V2X も使えないため、**後方は原理的に観測できない**。ルール上、後退中に後方車両へ
    衝突すると後退した側にも CRASH ペナルティ（10 秒間 5km/h）が科される。
    そこで、
      1. まず前進での脱出を試す（前方 LiDAR だけで完結し、観測できる範囲で判断できる）
      2. それでも駄目なときだけ、一定時間待って（後続が通り過ぎるのを期待して）
         ゆっくり短く後退する
    という順序にしている。後退は「見えないので賭けになる」操作であり、最後の手段。
    ただし前方が車体のすぐ前まで詰まっている（壁に正面から刺さった）ときだけは
    前進を繰り返しても無駄なので、試行回数を減らして早めに後退へ移る。

後退時の加速度の符号について:
    REVERSE ギアを入れたときに加速度指令のどちらの符号が「後退」になるかは、
    シミュレータ側の解釈次第で決まる。
      - 「シフトした向きへの加速」と解釈するなら **正**
      - 「車体前方を正とする符号付き加速度」（Autoware の慣習）なら **負**
    どちらかを決め打ちすると、外したときはギアだけ REVERSE になって車は
    壁を押し続ける（＝復帰できない）。仕様を仮定せず、動かなければ符号を反転して
    試し、動いた符号をその走行中は覚えておく方式にしている。
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import numpy as np

from lidar_sector import sector_clearance


class RecoveryState(Enum):
    """復帰シーケンスの状態。"""

    IDLE = "idle"
    ESCAPE_FORWARD = "escape_forward"
    WAIT_REAR = "wait_rear"
    ESCAPE_REVERSE = "escape_reverse"
    COOLDOWN = "cooldown"


@dataclass
class RecoveryCommand:
    """復帰中に出力する制御指令。

    Attributes:
        accel: 加速度指令 [m/s^2]。
        steer: 操舵角指令 [rad]。
        reverse: True なら REVERSE ギアを要求する。
    """

    accel: float
    steer: float
    reverse: bool


class StuckRecovery:
    """スタック判定と復帰シーケンスを持つ状態機械。

    毎周期 :meth:`update` を呼び、戻り値が ``None`` なら NN の出力をそのまま使う。
    ``RecoveryCommand`` が返ったときだけ、その指令で NN の出力を上書きする。
    衝突を検知した側は :meth:`notify_collision` で知らせる。
    """

    def __init__(
        self,
        stuck_speed: float = 0.5,
        stuck_duration: float = 2.0,
        collision_stuck_duration: float = 0.5,
        collision_window: float = 3.0,
        arm_speed: float = 1.0,
        release_speed: float = 0.8,
        forward_duration: float = 1.5,
        forward_attempts: int = 2,
        forward_attempts_when_blocked: int = 1,
        front_blocked_range: float = 1.2,
        rear_wait_duration: float = 2.0,
        reverse_duration: float = 1.2,
        reverse_probe_duration: float = 1.0,
        reverse_move_speed: float = 0.15,
        max_escape_cycles: int = 3,
        cooldown_duration: float = 2.0,
        sidestep_duration: float = 0.0,
        sidestep_steer_gain: float = 0.5,
        forward_accel: float = 1.2,
        reverse_accel: float = 1.2,
        max_steer: float = 0.45,
        side_angle_deg: Tuple[float, float] = (25.0, 90.0),
        front_angle_deg: Tuple[float, float] = (-20.0, 20.0),
    ):
        """復帰ロジックを初期化する。

        Args:
            stuck_speed: これを下回る速度が続いたらスタック候補とみなす [m/s]。
            stuck_duration: スタックと判定するまでの継続時間 [s]。
            collision_stuck_duration: 衝突直後にスタックと判定するまでの継続時間 [s]。
                ぶつかって止まったことは分かっているので、短くしてよい。
            collision_window: 衝突の通知を「直後」とみなす時間 [s]。
                これを過ぎたら通常の低速継続判定に戻す。
            arm_speed: 一度この速度を超えるまで復帰を作動させない [m/s]。
                スタート前のグリッド停止で誤発火させないためのガード。
            release_speed: 復帰成功とみなす速度 [m/s]。実走では脱出後の到達速度が
                0.84〜0.96 m/s だったため、1.0 では成功と判定できず
                max_escape_cycles の 3 往復を使い切っていた。判定はそれより
                下、かつスタック判定の stuck_speed より十分上に置く。
            forward_duration: 前進脱出を試す時間 [s]。
            forward_attempts: 後退に移る前に前進脱出を試す回数。
            forward_attempts_when_blocked: 前方が詰まっているときの前進試行回数。
                壁に正面から刺さった状態では前進を繰り返しても抜けないため、
                通常より少ない回数で後退に移る。
            front_blocked_range: 前方がこの距離以下なら「詰まっている」とみなす [m]。
            rear_wait_duration: 後退前に待つ時間 [s]。後方が見えないため、
                後続車が通り過ぎるのを時間で代替している。
            reverse_duration: 後退する時間 [s]。
            reverse_probe_duration: 後退指令を出してから「動いたか」を見るまでの
                時間 [s]。この時間で動かなければ加速度の符号を反転して試す。
                正味 0.83 m/s^2 で加速するので、1.0 秒あれば
                reverse_move_speed には十分届く。短すぎると、符号は合っているのに
                「まだ加速しきっていないだけ」を反転の根拠にしてしまう。
            reverse_move_speed: 後退できたとみなす速度 [m/s]。
            max_escape_cycles: 前進と後退を往復する回数の上限。使い切ったら
                一度 NN に制御を返す。まだ詰まっていれば衝突検知が拾い直す。
            cooldown_duration: 復帰後、再発火を抑える時間 [s]。
            forward_accel: 前進脱出時の加速度 [m/s^2]。
            reverse_accel: 後退時の加速度 [m/s^2]。
                いずれも車両の転がり抵抗 0.37 m/s^2 を差し引いた残りでしか加速しない。
                レース中の上限である 0.7 では正味 0.33 m/s^2 しか残らず、
                forward_duration 1.5 秒では 0.5 m/s 止まりで release_speed に
                届かない。脱出は速度を出すのが目的ではなく
                「動き出すこと」が目的なので、走行時の上限より大きくとる。
            max_steer: 復帰中の操舵角の絶対値 [rad]。
            side_angle_deg: 左右の空きを測る扇形の角度範囲 [deg]（絶対値で指定）。
            front_angle_deg: 前方の詰まりを測る扇形の角度範囲 [deg]。
        """
        self.stuck_speed = stuck_speed
        self.stuck_duration = stuck_duration
        self.collision_stuck_duration = collision_stuck_duration
        self.collision_window = collision_window
        self.arm_speed = arm_speed
        self.release_speed = release_speed
        self.forward_duration = forward_duration
        self.forward_attempts = forward_attempts
        self.forward_attempts_when_blocked = forward_attempts_when_blocked
        self.front_blocked_range = front_blocked_range
        self.rear_wait_duration = rear_wait_duration
        self.reverse_duration = reverse_duration
        self.reverse_probe_duration = reverse_probe_duration
        self.reverse_move_speed = reverse_move_speed
        self.max_escape_cycles = max_escape_cycles
        self.cooldown_duration = cooldown_duration
        # 脱出直後、NN に返す前に空いている側へ寄せて進む時間 [s]。0 なら従来どおり。
        # 脱出して即 NN に返すと、止まっている相手のいる同じラインへ戻って
        # また刺さる（実測: 同じ地点で 19 回の復帰を繰り返した）。
        self.sidestep_duration = sidestep_duration
        self.sidestep_steer_gain = sidestep_steer_gain
        self.forward_accel = forward_accel
        self.reverse_accel = reverse_accel
        self.max_steer = max_steer
        self.side_angle_deg = side_angle_deg
        self.front_angle_deg = front_angle_deg

        self.state = RecoveryState.IDLE
        self._armed = False
        self._slow_since: Optional[float] = None
        self._state_since = 0.0
        self._attempt = 0
        self._attempt_budget = forward_attempts
        self._escape_steer = 0.0
        self._collision_at: Optional[float] = None
        self._reverse_flipped = False
        self._reverse_sign_confirmed = False
        self._phase_peak_speed = 0.0
        self.trigger_count = 0
        # 前進 → 後退 の往復回数。max_escape_cycles で頭打ちにする。
        self.escape_cycle = 0
        # 後退時に出す加速度の符号。実測で決まる（上のモジュール docstring 参照）。
        self.reverse_sign = 1.0
        # 直近の復帰中に観測した速度の絶対値の最大。後から「動いたのか」を判断する材料。
        self.peak_speed_in_recovery = 0.0
        # 復帰に入るきっかけになった経路。"collision" か "slow"。ログ用。
        self.trigger_reason = ""

    @property
    def is_active(self) -> bool:
        """復帰シーケンスの実行中かどうか。"""
        return self.state is not RecoveryState.IDLE

    def reset(self) -> None:
        """状態を初期化する。走行のやり直し時に呼ぶ。"""
        self.state = RecoveryState.IDLE
        self._armed = False
        self._slow_since = None
        self._attempt = 0
        self._collision_at = None
        self._reverse_flipped = False
        self.escape_cycle = 0
        self.trigger_count = 0
        self.trigger_reason = ""
        # reverse_sign は「このシミュレータではどちらが後退か」という
        # 走行をまたいで変わらない情報なので、意図的に持ち越す。

    def notify_collision(self, now: float) -> None:
        """壁・カートへの衝突を通知する。

        通知そのものでは復帰を始めない。ぶつかっても走り続けられているなら
        介入しないほうが速いので、「衝突後に止まったままか」を
        :meth:`update` 側で確かめてから発火する。

        Args:
            now: 単調増加する現在時刻 [s]。
        """
        self._collision_at = now
        # ここで arm 判定を通したことにはしない。スタート前のグリッドで
        # 他車に小突かれただけでも衝突は検知されるため、通してしまうと
        # スタート前に脱出動作を始めてしまう。

    def update(
        self,
        now: float,
        speed: float,
        commanded_accel: float,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float,
    ) -> Optional[RecoveryCommand]:
        """1 周期分の状態を更新し、必要なら復帰指令を返す。

        Args:
            now: 単調増加する現在時刻 [s]。
            speed: 現在の車速 [m/s]（Wheel Odometry 由来）。
            commanded_accel: NN が出した加速度指令 [m/s^2]。
            ranges: LiDAR の距離配列（前処理前の生値）。
            angle_min: LiDAR の最小角 [rad]。
            angle_increment: LiDAR の角度分解能 [rad]。

        Returns:
            復帰中なら :class:`RecoveryCommand`、通常走行中なら ``None``。
        """
        # スタート前のグリッド停止で誤発火しないよう、一度走り出すまで作動させない。
        if not self._armed:
            if abs(speed) >= self.arm_speed:
                self._armed = True
            return None

        if self.state is not RecoveryState.IDLE:
            self.peak_speed_in_recovery = max(
                self.peak_speed_in_recovery, abs(speed))
            self._phase_peak_speed = max(self._phase_peak_speed, abs(speed))

        if self.state is RecoveryState.IDLE:
            if self._is_stuck(now, speed, commanded_accel):
                self.trigger_reason = (
                    "collision" if self._recent_collision(now) else "slow")
                self._enter(RecoveryState.ESCAPE_FORWARD, now)
                self._attempt = 1
                self.trigger_count += 1
                self.peak_speed_in_recovery = 0.0
                self.escape_cycle = 0
                self._escape_steer = self._pick_escape_steer(
                    ranges, angle_min, angle_increment)
                self._attempt_budget = self._pick_attempt_budget(
                    ranges, angle_min, angle_increment)
                # 発火したその周期から動かす。1 周期でも NN の指令（衝突直後は
                # 制動であることが多い）を通すと、壁に押し付ける向きに働く。
                return self._forward_command()
            return None

        elapsed = now - self._state_since

        if self.state is RecoveryState.ESCAPE_FORWARD:
            # 空いている側へ切りながら前に出る。壁に沿って車体が振れて抜けることが多い。
            if abs(speed) >= self.release_speed:
                self._enter(RecoveryState.COOLDOWN, now)
                return None
            if elapsed >= self.forward_duration:
                if self._attempt < self._attempt_budget:
                    self._attempt += 1
                    self._enter(RecoveryState.ESCAPE_FORWARD, now)
                    # 同じ側に振り直しても結果は変わらないので、逆側を試す。
                    self._escape_steer = -self._escape_steer
                elif self.escape_cycle >= self.max_escape_cycles:
                    # 前進と後退を往復しても抜けられなかった。一度 NN に返す。
                    # まだ詰まっていれば衝突検知がすぐ拾い直す。
                    self._enter(RecoveryState.COOLDOWN, now)
                    return None
                else:
                    self._enter(RecoveryState.WAIT_REAR, now)
                return self._forward_command()
            return self._forward_command()

        if self.state is RecoveryState.WAIT_REAR:
            # 後方が見えないので、後続が通過するのを時間で待つ。動かずに待機する。
            if elapsed >= self.rear_wait_duration:
                self.escape_cycle += 1
                self._reverse_flipped = False
                self._enter(RecoveryState.ESCAPE_REVERSE, now)
                return self._reverse_command()
            return RecoveryCommand(accel=0.0, steer=0.0, reverse=False)

        if self.state is RecoveryState.ESCAPE_REVERSE:
            # 一度でも後退できた符号は正解と分かっているので、以後は試さない。
            # 動かない理由は符号ではなく引っ掛かりのほうなので、反転を繰り返すと
            # 正解の符号を捨ててしまう。
            if self._phase_peak_speed >= self.reverse_move_speed:
                self._reverse_sign_confirmed = True
            # 指令を出しても動いていないなら、加速度の符号の解釈を外している。
            # 反転して同じ時間だけやり直す（この後退につき 1 回だけ）。
            if (not self._reverse_sign_confirmed
                    and not self._reverse_flipped
                    and elapsed >= self.reverse_probe_duration
                    and self._phase_peak_speed < self.reverse_move_speed):
                self.reverse_sign = -self.reverse_sign
                self._reverse_flipped = True
                self._enter(RecoveryState.ESCAPE_REVERSE, now)
                return self._reverse_command()
            if elapsed >= self.reverse_duration:
                # 下がれたとしても、壁の目の前で NN に返すとまた突っ込む。
                # 壁に刺さった状態は学習データにないため。空いている側へ
                # 前進して、走り出せてから返す。
                self._attempt = 1
                self._attempt_budget = 1
                self._enter(RecoveryState.ESCAPE_FORWARD, now)
                return self._forward_command()
            return self._reverse_command()

        if self.state is RecoveryState.COOLDOWN:
            if elapsed >= max(self.cooldown_duration, self.sidestep_duration):
                self._enter(RecoveryState.IDLE, now)
                self._slow_since = None
                self._attempt = 0
                self.escape_cycle = 0
                # 復帰を一巡したので、同じ衝突で即座に再発火させない。
                self._collision_at = None
                return None
            if elapsed < self.sidestep_duration:
                # 脱出で向いた「空いている側」へそのまま寄せて進み、相手の横を抜ける。
                return RecoveryCommand(
                    accel=self.forward_accel * 0.6,
                    steer=self._escape_steer * self.sidestep_steer_gain,
                    reverse=False)
            return None

        return None

    def _enter(self, state: RecoveryState, now: float) -> None:
        """状態を遷移させ、滞在時間とそのフェーズの最高速度を初期化する。"""
        self.state = state
        self._state_since = now
        self._phase_peak_speed = 0.0

    def _forward_command(self) -> RecoveryCommand:
        """前進脱出時の指令を作る。"""
        return RecoveryCommand(
            accel=self.forward_accel, steer=self._escape_steer, reverse=False)

    def _reverse_command(self) -> RecoveryCommand:
        """後退脱出時の指令を作る。

        前進脱出とは逆に切ることで、車首が壁から離れる向きに振れる。
        加速度の符号は実測で決めた :attr:`reverse_sign` を掛ける。
        """
        return RecoveryCommand(
            accel=self.reverse_accel * self.reverse_sign,
            steer=-self._escape_steer, reverse=True)

    def _recent_collision(self, now: float) -> bool:
        """直近に衝突の通知を受けているかどうか。"""
        if self._collision_at is None:
            return False
        return (now - self._collision_at) <= self.collision_window

    def _is_stuck(self, now: float, speed: float, commanded_accel: float) -> bool:
        """スタック状態かどうかを判定する。

        通常は「動けと指令しているのに動いていない」状態が一定時間続いたときだけ
        真とする。モデルが自ら減速している場面（コーナー手前など）では発火しない。

        衝突直後だけは例外で、加速指令の有無を問わず、止まったままなら短い時間で
        真とする。ぶつかった直後は NN が減速側の指令を出していることがあり、
        それを待っていると壁に張り付いたまま動けなくなるため。
        """
        recent_collision = self._recent_collision(now)
        trying_to_move = commanded_accel > 0.0
        if abs(speed) >= self.stuck_speed or not (trying_to_move or recent_collision):
            self._slow_since = None
            return False
        if self._slow_since is None:
            self._slow_since = now
            return False
        threshold = (self.collision_stuck_duration if recent_collision
                     else self.stuck_duration)
        return (now - self._slow_since) >= threshold

    def _pick_attempt_budget(
        self, ranges: np.ndarray, angle_min: float, angle_increment: float
    ) -> int:
        """前進脱出を何回試すかを決める。

        前方が車体のすぐ前まで詰まっているなら、前進を繰り返しても壁を押すだけなので
        試行回数を減らし、早めに後退へ移る。
        """
        front = sector_clearance(
            ranges, angle_min, angle_increment, self.front_angle_deg, percentile=10.0)
        if front <= self.front_blocked_range:
            return self.forward_attempts_when_blocked
        return self.forward_attempts

    def _pick_escape_steer(
        self, ranges: np.ndarray, angle_min: float, angle_increment: float
    ) -> float:
        """左右で空いている側を選び、その向きの操舵角を返す。

        前方 180 度しか見えないので、判断材料は左右の扇形の距離だけ。
        両側とも同程度なら左に振る（どちらでも良い場面で迷わないようにするため）。
        """
        lo, hi = self.side_angle_deg
        # 中央値を使うのは、1 点の外れ値で左右の判断が反転しないようにするため。
        left = sector_clearance(
            ranges, angle_min, angle_increment, (lo, hi), percentile=50.0)
        right = sector_clearance(
            ranges, angle_min, angle_increment, (-hi, -lo), percentile=50.0)
        return self.max_steer if left >= right else -self.max_steer
