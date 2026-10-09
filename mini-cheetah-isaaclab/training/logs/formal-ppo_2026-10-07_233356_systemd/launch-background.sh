#!/usr/bin/env bash
set -euo pipefail
STAMP=$(date +%Y-%m-%d_%H%M%S)
PROJECT=/run/media/pma213x/0C6A8CC86A8CAFCE/Ubuntu2/CodeU/Dog1/mini-cheetah-isaaclab
PREV_RUN_NAME='2026-10-07_23-08-19_repair-v1-resume_2026-10-07_230815'
source /home/pma213x/.venvs/minicheetah-isaaclab/bin/activate
export PYTHONPATH="$PROJECT"
export TERM=xterm-256color
export OMP_NUM_THREADS=4
WRAPPER=$(cat <<'PY'
import builtins, os, runpy, sys
train_path = "/home/pma213x/IsaacLab/scripts/reinforcement_learning/rsl_rl/train.py"
sys.argv = [train_path] + sys.argv[1:]
sys.path.insert(0, os.path.dirname(train_path))
_original_import = builtins.__import__
_registration_done = False
def _delayed_import(name, globals=None, locals=None, fromlist=(), level=0):
    global _registration_done
    module = _original_import(name, globals, locals, fromlist, level)
    if name == "isaaclab_tasks" and not _registration_done:
        _registration_done = True
        _original_import("training.envs", globals, locals, (), 0)
    return module
builtins.__import__ = _delayed_import
runpy.run_path(train_path, run_name="__main__")
PY
)
cd "$PROJECT/training"
exec /home/pma213x/IsaacLab/isaaclab.sh -p -c "$WRAPPER" \
  --task YoboGo-Velocity-Flat-v0 --headless --num_envs 4 --seed 0 \
  --max_iterations 3000 --logger tensorboard --resume True \
  --load_run "$PREV_RUN_NAME" --checkpoint model_1800.pt \
  --experiment_name yobogo_velocity_flat \
  --run_name "repair-v1-systemd_${STAMP}"
