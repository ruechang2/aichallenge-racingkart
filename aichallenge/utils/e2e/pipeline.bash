#!/bin/bash
# 教師 bag → 抽出 → 学習 → 変換 → 等価性確認 → デプロイ（autoware-command コンテナ内で実行）
#   pipeline.bash <タグ> <train bag 名...> -- <val bag 名...>
# bag は /aichallenge/ml_workspace/train/<名> と val/<名> に置いてあること。
set -eo pipefail
TAG="$1"; shift
TRAIN=(); VAL=(); mode=train
for a in "$@"; do
  if [ "$a" = "--" ]; then mode=val; continue; fi
  if [ $mode = train ]; then TRAIN+=("/aichallenge/ml_workspace/train/$a"); else VAL+=("/aichallenge/ml_workspace/val/$a"); fi
done
cd /aichallenge/ml_workspace/tiny_lidar_net
echo "== extract"
[ ${#TRAIN[@]} -gt 0 ] && python3 extract_data_from_bag.py --seq-dirs "${TRAIN[@]}" --outdir ./dataset/train/
[ ${#VAL[@]} -gt 0 ]   && python3 extract_data_from_bag.py --seq-dirs "${VAL[@]}"   --outdir ./dataset/val/
echo "== dataset"
python3 - <<'PY'
import numpy as np, pathlib
for split in ("train","val"):
    tot=0; use=0
    for d in sorted(pathlib.Path(f"dataset/{split}").iterdir()):
        if not (d/"scans.npy").exists(): continue
        n=len(np.load(d/"steers.npy")); v=np.load(d/"valid.npy").sum() if (d/"valid.npy").exists() else n
        tot+=n; use+=v; print(f"  {split}/{d.name}: {v}/{n}")
    print(f"  {split}: {use}/{tot} samples used")
PY
echo "== train"
python3 train.py 2>&1 | grep -E "^Epoch|SAVE|EarlyStop|Error|Traceback|finished" | tail -n 6
cp checkpoints/best_model.pth "checkpoints/best_model.${TAG}.pth"
echo "== convert"
python3 convert_weight.py --n-frames 4 --ckpt checkpoints/best_model.pth --output "weights/converted_weights_${TAG}.npy"
echo "== parity"
python3 ../check_numpy_parity.py --ckpt checkpoints/best_model.pth --weights "weights/converted_weights_${TAG}.npy" --n-frames 4
echo "== deploy"
CK=/aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/ckpt
cp "weights/converted_weights_${TAG}.npy" "$CK/tinylidarnet_weights.${TAG}.npy"
cp "weights/converted_weights_${TAG}.npy" "$CK/tinylidarnet_weights.npy"
echo "deployed ${TAG} -> $CK/tinylidarnet_weights.npy"
