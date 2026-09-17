import logging
from pathlib import Path
from typing import List, Optional, Tuple, Union

import numpy as np
from torch.utils.data import Dataset, ConcatDataset

logger = logging.getLogger(__name__)


class ScanControlSequenceDataset(Dataset):
    """
    A PyTorch Dataset for a single sequence of LiDAR scans and control commands.

    Loads synchronized .npy files (scans, steers, accelerations) from a specific
    directory. The LiDAR scans are normalized by the specified maximum range.

    Attributes:
        seq_dir (Path): Path to the sequence directory.
        max_range (float): Maximum range for LiDAR normalization.
        scans (np.ndarray): Normalized scan data array (N, num_points).
        steers (np.ndarray): Steering angle array (N,).
        accels (np.ndarray): Acceleration array (N,).
    """

    def __init__(self, seq_dir: Union[str, Path], max_range: float = 30.0, n_frames: int = 1,
                 target_mode: str = "accel", max_speed: float = 8.34):
        """
        Initializes the dataset from a sequence directory.

        Args:
            seq_dir: Path to the directory containing .npy files.
            max_range: Maximum range value to normalize LiDAR data (0.0 to 1.0).
            n_frames: Number of consecutive scans to stack along the channel axis.
                1 keeps the original single-frame behaviour.
            target_mode: "accel" trains on the raw acceleration command; "speed"
                trains on the commanded target speed normalized by max_speed.
                加速度は最大加速と急制動の二値に張り付くため回帰が収束しにくい。
                目標速度はコーナー手前でなめらかに落ちるので学習しやすい。
            max_speed: Normalization constant for target_mode="speed" [m/s].

        Raises:
            ValueError: If data lengths do not match or files are missing.
        """
        self.seq_dir = Path(seq_dir)
        self.max_range = max_range
        self.n_frames = n_frames
        self.target_mode = target_mode
        self.max_speed = max_speed

        try:
            # Load raw data
            self.scans = np.load(self.seq_dir / "scans.npy")         # Shape: (N, num_points)
            self.steers = np.load(self.seq_dir / "steers.npy")       # Shape: (N,)
            self.accels = np.load(self.seq_dir / "accelerations.npy") # Shape: (N,)
            if self.target_mode == "speed":
                self.speeds = np.load(self.seq_dir / "speeds.npy")   # Shape: (N,)
        except FileNotFoundError as e:
            raise FileNotFoundError(f"Missing required .npy files in {self.seq_dir}: {e}")

        # Validate data consistency
        n_samples = len(self.scans)
        if not (len(self.steers) == n_samples and len(self.accels) == n_samples):
            raise ValueError(
                f"Data length mismatch in {self.seq_dir}: "
                f"Scans={len(self.scans)}, Steers={len(self.steers)}, Accels={len(self.accels)}"
            )

        # Preprocessing: Clip and Normalize
        # Values are clipped to [0, max_range] and then scaled to [0, 1]
        self.scans = np.clip(self.scans, 0.0, self.max_range) / self.max_range

        # 学習に使うサンプルの選択。extract 側が valid.npy（停止区間を False）を
        # 出していればそれに従う。行は残っているので、過去フレームのスタックは
        # 止まっている間のスキャンも含めて時間どおりに参照できる。
        valid_path = self.seq_dir / "valid.npy"
        if valid_path.exists():
            valid = np.load(valid_path).astype(bool)
            if len(valid) != n_samples:
                raise ValueError(f"valid.npy length mismatch in {self.seq_dir}: {len(valid)} vs {n_samples}")
            self.index = np.flatnonzero(valid)
            dropped = n_samples - len(self.index)
            if dropped:
                logger.info(f"{self.seq_dir.name}: using {len(self.index)}/{n_samples} samples ({dropped} stopped)")
        else:
            self.index = np.arange(n_samples)

    def __len__(self) -> int:
        return len(self.index)

    def __getitem__(self, idx: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Retrieves a sample from the dataset.

        Args:
            idx: Index of the sample to retrieve.

        Returns:
            scan: Normalized LiDAR scans of shape (n_frames, num_points), oldest
                first. Near the start of a sequence the oldest frame is repeated,
                so the model always sees a fixed number of channels.
            target: Control command vector [acceleration, steering] (float32).
        """
        idx = int(self.index[idx])
        # 過去 n_frames 分を古い順に並べる。系列の先頭では過去が足りないので
        # 先頭フレームを繰り返して埋める（系列をまたいで参照しない）。
        frame_ids = np.clip(np.arange(idx - self.n_frames + 1, idx + 1), 0, None)
        scan = self.scans[frame_ids].astype(np.float32)
        
        steer = np.float32(self.steers[idx])

        # Target vector construction: [longitudinal, Steering]
        # longitudinal は target_mode で中身が変わる（加速度 or 正規化した目標速度）。
        if self.target_mode == "speed":
            longitudinal = np.float32(self.speeds[idx] / self.max_speed)
        else:
            longitudinal = np.float32(self.accels[idx])
        target = np.array([longitudinal, steer], dtype=np.float32)
        
        return scan, target


class MultiSeqConcatDataset(ConcatDataset):
    """
    A PyTorch ConcatDataset that aggregates multiple SequenceDatasets.

    Automatically discovers valid sequence directories within a root directory.
    Supports filtering sequences using inclusion and exclusion keywords.
    """

    def __init__(
        self, 
        dataset_root: Union[str, Path], 
        max_range: float = 30.0, 
        include: Optional[List[str]] = None, 
        exclude: Optional[List[str]] = None,
        n_frames: int = 1,
        target_mode: str = "accel",
        max_speed: float = 8.34
    ):
        """
        Initializes the concatenated dataset.

        Args:
            dataset_root: Root directory containing sequence folders.
            max_range: Maximum range for LiDAR normalization.
            include: List of substrings; if provided, only directories containing
                     at least one of these substrings will be loaded.
            exclude: List of substrings; directories containing any of these
                     substrings will be skipped.
            n_frames: Number of consecutive scans to stack along the channel axis.

        Raises:
            RuntimeError: If no valid sequences are found after filtering.
        """
        dataset_root = Path(dataset_root)
        
        # Discover all subdirectories
        all_seq_dirs = sorted([p for p in dataset_root.iterdir() if p.is_dir()])
        target_seq_dirs = []

        # Apply filters
        for p in all_seq_dirs:
            name = p.name
            
            # Check inclusion criteria (OR logic)
            if include and not any(inc in name for inc in include):
                continue
            
            # Check exclusion criteria (OR logic)
            if exclude and any(exc in name for exc in exclude):
                continue
            
            target_seq_dirs.append(p)

        # Instantiate datasets
        datasets = []
        for seq_dir in target_seq_dirs:
            # Quick check for file existence before initialization
            required_files = ["scans.npy", "steers.npy", "accelerations.npy"]
            if target_mode == "speed":
                required_files = required_files + ["speeds.npy"]
            if all((seq_dir / f).exists() for f in required_files):
                try:
                    ds = ScanControlSequenceDataset(
                        seq_dir, max_range=max_range, n_frames=n_frames,
                        target_mode=target_mode, max_speed=max_speed)
                    datasets.append(ds)
                except Exception as e:
                    logger.warning(f"Failed to load sequence {seq_dir}: {e}")
            else:
                logger.warning(f"Skipping {seq_dir.name}: Missing .npy files.")

        if not datasets:
            raise RuntimeError(f"No valid sequences found in {dataset_root} with provided filters.")

        super().__init__(datasets)
        logger.info(f"Loaded {len(datasets)} sequences from {dataset_root}. Total samples: {len(self)}")
