import logging
from collections import deque
from typing import Optional, Tuple

import numpy as np

from collision_detector import CollisionDetector, CollisionEvent
from model.tinylidarnet import TinyLidarNetNp, TinyLidarNetSmallNp
from stuck_recovery import RecoveryCommand, StuckRecovery


class TinyLidarNetCore:
    """Core logic for the TinyLidarNet autonomous driving controller.

    This class manages the neural network model lifecycle, including initialization,
    weight loading, input preprocessing (cleaning, resizing, normalizing), and
    inference execution. It is designed to be framework-agnostic.

    Attributes:
        input_dim (int): Dimension of the input vector expected by the model.
        output_dim (int): Dimension of the output vector (acceleration, steering).
        architecture (str): Model architecture type ('large' or 'small').
        acceleration (float): Fixed acceleration value used in 'fixed' control mode.
        control_mode (str): Control strategy ('ai' or 'fixed').
        max_range (float): Maximum LiDAR range used for normalization and clipping.
        model (object): The instantiated neural network model.
        logger (logging.Logger): Logger instance.
    """

    def __init__(
        self,
        input_dim: int = 1080,
        output_dim: int = 2,
        architecture: str = 'large',
        ckpt_path: str = '',
        acceleration: float = 0.1,
        control_mode: str = 'ai',
        max_range: float = 30.0,
        n_frames: int = 1,
        max_speed: float = 8.34,
        speed_kp: float = 1.0,
        accel_limits: Tuple[float, float] = (-1.6, 0.7),
        recovery: Optional[StuckRecovery] = None,
        collision_detector: Optional[CollisionDetector] = None
    ):
        """Initializes the TinyLidarNetCore with specified parameters.

        Args:
            input_dim (int, optional): The number of LiDAR points expected by the model.
                Defaults to 1080.
            output_dim (int, optional): The number of output control values.
                Defaults to 2.
            architecture (str, optional): The model architecture to use ('large' or 'small').
                Defaults to 'large'.
            ckpt_path (str, optional): Path to the numpy weight file (.npy or .npz).
                Defaults to ''.
            acceleration (float, optional): The constant acceleration value to apply
                when control_mode is set to 'fixed'. Defaults to 0.1.
            control_mode (str, optional): The control mode to determine output behavior.
                'ai' uses model output for both acceleration and steering.
                'fixed' uses the fixed acceleration value and model output for steering.
                Defaults to 'ai'.
            max_range (float, optional): The maximum range value for normalization.
                Values exceeding this will be clipped, and infinity will be replaced
                by this value. Defaults to 30.0.
            n_frames (int, optional): Number of consecutive scans stacked along the
                channel axis. Must match the value used at training time. Defaults to 1.
            max_speed (float, optional): Normalization constant [m/s] used when the
                model was trained with target_mode="speed". Defaults to 8.34 (30 km/h).
            speed_kp (float, optional): Proportional gain converting the speed error
                into an acceleration command. Defaults to 1.0.
            accel_limits (tuple, optional): (min, max) acceleration command. Defaults
                to the range the MPC uses, (-1.6, 0.7).
            recovery (StuckRecovery, optional): スタック復帰ロジック。None なら復帰なし。
                通常走行では介入せず、詰んだときだけ NN の出力を上書きする。
            collision_detector (CollisionDetector, optional): 壁・カートへの衝突検知。
                車両ダメージの topic を :meth:`update_condition` に流し込んで使う。
                検知結果は recovery に通知され、ぶつかって止まったときの復帰を
                低速継続の判定より早く始めるために使う。None なら衝突検知なし。
        """
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.architecture = architecture
        self.acceleration = acceleration
        self.control_mode = control_mode.lower()
        self.max_range = max_range
        self.n_frames = n_frames
        self.max_speed = max_speed
        self.speed_kp = speed_kp
        self.accel_limits = accel_limits
        self.current_speed = 0.0
        self.recovery = recovery
        self.collision_detector = collision_detector
        self.reverse_requested = False
        # 直近に検知した衝突。ノード側がログに出すための記録。
        self.last_collision: Optional[CollisionEvent] = None
        self.logger = logging.getLogger(__name__)

        # 直近 n_frames 分のスキャンを保持する。1 フレームでは「近づいてくる他車」と
        # 「静止した壁」が区別できないため、時間方向の情報をモデルに渡す。
        self.scan_history = deque(maxlen=n_frames)

        if self.architecture == 'small':
            self.model = TinyLidarNetSmallNp(
                input_dim=self.input_dim, output_dim=self.output_dim, in_channels=n_frames
            )
        else:
            self.model = TinyLidarNetNp(
                input_dim=self.input_dim, output_dim=self.output_dim, in_channels=n_frames
            )

        if ckpt_path:
            self._load_weights(ckpt_path)
        else:
            self.logger.warning("No weight file provided. Using randomly initialized weights.")

    def set_current_speed(self, speed: float) -> None:
        """Records the latest measured vehicle speed.

        Only used by control_mode "speed", where the acceleration command is derived
        from the error between the predicted target speed and this value.

        Args:
            speed (float): Current longitudinal velocity [m/s].
        """
        self.current_speed = float(speed)

    def update_condition(self, now: float, condition: int) -> Optional[CollisionEvent]:
        """車両ダメージ量を 1 件受け取り、衝突なら復帰ロジックに通知する。

        ``/aichallenge/pitstop/condition`` は LiDAR とは別の周期で届くため、
        :meth:`process` とは独立した入口にしている。

        Args:
            now: 単調増加する現在時刻 [s]。:meth:`process` に渡すものと同じ時計。
            condition: ``/aichallenge/pitstop/condition`` の値。

        Returns:
            衝突を検知したら :class:`collision_detector.CollisionEvent`、
            それ以外は ``None``。
        """
        if self.collision_detector is None:
            return None
        return self._handle_collision(self.collision_detector.update_condition(
            now=now,
            condition=condition,
            # 復帰中は壁際で擦ってダメージが増えるのが当たり前なので通知しない。
            suppress_events=self._recovery_active(),
        ), now)

    def _recovery_active(self) -> bool:
        """復帰シーケンスの実行中かどうか。"""
        return self.recovery is not None and self.recovery.is_active

    def _handle_collision(
        self, event: Optional[CollisionEvent], now: float
    ) -> Optional[CollisionEvent]:
        """検知した衝突を記録し、復帰ロジックに通知する。"""
        if event is None:
            return None
        self.last_collision = event
        if self.recovery is not None:
            self.recovery.notify_collision(now)
        return event

    def process(
        self,
        ranges: np.ndarray,
        now: Optional[float] = None,
        angle_min: float = 0.0,
        angle_increment: float = 0.0,
    ) -> Tuple[float, float]:
        """Runs the complete inference pipeline on raw LiDAR data.

        This method handles data cleaning (NaN/Inf removal), resizing, normalization,
        and model inference.

        Args:
            ranges (np.ndarray): A 1D numpy array containing raw LiDAR range data.
            now (float, optional): Monotonic timestamp [s]. Required for stuck recovery.
            angle_min (float, optional): LiDAR minimum angle [rad], for stuck recovery.
            angle_increment (float, optional): LiDAR angular resolution [rad].

        Returns:
            Tuple[float, float]: A tuple containing (acceleration, steering_angle).
                Values are clipped between -1.0 and 1.0.
        """
        # 1. Preprocess (Clean -> Resize -> Normalize)
        processed_ranges = self._preprocess_ranges(ranges)

        # 起動直後は履歴が空なので、最初のスキャンで埋めてチャネル数をそろえる。
        if not self.scan_history:
            for _ in range(self.n_frames):
                self.scan_history.append(processed_ranges)
        else:
            self.scan_history.append(processed_ranges)

        # Prepare input tensor: (1, n_frames, input_dim) — 古い順に並べる
        x = np.stack(self.scan_history, axis=0)[np.newaxis, ...]

        # 2. Inference
        outputs = self.model(x)[0]

        # 3. Post-process
        if self.control_mode == "ai":
            accel = float(np.clip(outputs[0], -1.0, 1.0))
        elif self.control_mode == "speed":
            # モデルは正規化した目標速度を出す。加速度への変換は比例制御に任せる。
            # 加速度を直接回帰させると最大加速と急制動の二値に張り付いて収束しないため。
            target_speed = float(np.clip(outputs[0], 0.0, 1.0)) * self.max_speed
            accel = float(np.clip(
                self.speed_kp * (target_speed - self.current_speed),
                self.accel_limits[0], self.accel_limits[1]))
        else:
            accel = self.acceleration

        steer = float(np.clip(outputs[1], -1.0, 1.0))

        # 4. 衝突検知。検知しても直接は介入せず、復帰ロジックに通知するだけ。
        #    ぶつかっても走り続けられているなら、止めないほうが速い。
        #    ダメージ topic が来ている環境では、この呼び出しは何もしない。
        if self.collision_detector is not None and now is not None:
            self._handle_collision(self.collision_detector.update(
                now=now,
                speed=self.current_speed,
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
                suppress_events=self._recovery_active(),
            ), now)

        # 5. スタック復帰。通常走行では None が返り、NN の出力がそのまま通る。
        self.reverse_requested = False
        if self.recovery is not None and now is not None:
            command: Optional[RecoveryCommand] = self.recovery.update(
                now=now,
                speed=self.current_speed,
                commanded_accel=accel,
                ranges=ranges,
                angle_min=angle_min,
                angle_increment=angle_increment,
            )
            if command is not None:
                self.reverse_requested = command.reverse
                return command.accel, command.steer

        return accel, steer

    def _load_weights(self, path: str) -> None:
        """Loads model weights from a file into the model parameters.

        Args:
            path (str): Path to the .npy or .npz weight file.

        Raises:
            ValueError: If the weight file format is unsupported.
            IOError: If the file cannot be read.
        """
        try:
            weights = np.load(path, allow_pickle=True)

            if isinstance(weights, np.lib.npyio.NpzFile):
                weight_dict = dict(weights.items())
            elif isinstance(weights, np.ndarray) and weights.dtype == object:
                weight_dict = weights.item()
            elif isinstance(weights, dict):
                weight_dict = weights
            else:
                raise ValueError(f"Unsupported weight format type: {type(weights)}")

            loaded_count = 0
            for key, value in weight_dict.items():
                key_norm = key.replace('.', '_')

                if key_norm in self.model.params:
                    self.model.params[key_norm] = value
                    loaded_count += 1

            self.logger.info(f"Successfully loaded {loaded_count} parameters from {path}")

        except Exception as e:
            self.logger.error(f"Failed to load weights from {path}: {e}")
            raise e

    def _preprocess_ranges(self, ranges: np.ndarray) -> np.ndarray:
        """Cleans, resizes, and normalizes LiDAR ranges.

        This method performs the following operations:
        1. Replaces NaNs with 0.0.
        2. Replaces infinite values with `self.max_range`.
        3. Clips all values to the range [0.0, `self.max_range`].
        4. Resizes the array to match `self.input_dim` via interpolation or padding.
        5. Normalizes the data by dividing by `self.max_range`.

        Args:
            ranges (np.ndarray): Source LiDAR range data.

        Returns:
            np.ndarray: Processed data array of shape (self.input_dim,).
        """
        # Work on a copy to avoid side effects on the input array
        ranges = ranges.copy()
        
        # Handle invalid values
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = self.max_range
        
        # Clip to ensure data is within the expected range
        ranges = np.clip(ranges, 0.0, self.max_range)

        # Resize input if necessary
        current_len = len(ranges)
        if current_len > self.input_dim:
            idx = np.linspace(0, current_len - 1, self.input_dim, dtype=int)
            ranges = ranges[idx]
        elif current_len < self.input_dim:
            ranges = np.pad(ranges, (0, self.input_dim - current_len), 'constant')

        # Normalize
        return ranges / self.max_range 
