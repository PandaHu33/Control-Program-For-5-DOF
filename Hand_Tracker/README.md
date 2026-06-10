# Hand Tracker Arm Control

This folder provides the local hand-recognition service used by the H5 control
UI. The service only outputs normalized XYZ incremental input axes and a
skeleton camera stream. Arm IK is still handled by the same UI position-control
path used by keyboard and gamepad.

## Environment

Create the isolated conda environment:

```powershell
conda env create -f Hand_Tracker\environment.yml
conda activate hand-tracker-arm
```

If the environment already exists:

```powershell
conda env update -f Hand_Tracker\environment.yml --prune
```

The one-click launcher checks this environment, creates it if missing, and
repairs missing Python packages with `requirements.txt` before starting the
service.

## Run

Start the service manually:

```powershell
conda activate hand-tracker-arm
python Hand_Tracker\hand_arm_control.py
```

It exposes:

- `http://127.0.0.1:8091/stream.mjpg` for the camera and hand skeleton
- `http://127.0.0.1:8091/events` for realtime XYZ axis data
- `http://127.0.0.1:8091/api/status` for the latest status snapshot

The default unlock gesture is `victory`: index and middle fingers extended,
ring and pinky folded. Hold it for `0.35s` to unlock. After unlocking, the
state remains unlocked even if the hand is lost or the gesture changes. The
same gesture is ignored for `5s` after a state change; after that, hold
`victory` again to lock. The hand position at the unlock moment becomes the
neutral anchor. While unlocked:

- hand closer/farther controls arm X forward/back
- hand left/right controls arm Y left/right
- hand up/down controls arm Z up/down

The mapping coefficients are intentionally small at first. Adjust them in
`hand_control_config.yaml`:

- `axis_gain_x`
- `axis_gain_y`
- `axis_gain_z`
- `position_step_m`
- `deadzone_xy`
- `deadzone_depth`
- `state_toggle_cooldown_sec`

If an axis moves opposite to expectation, flip the corresponding value under
`axis_sign`.
