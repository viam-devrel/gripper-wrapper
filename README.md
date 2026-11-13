# gripper-wrapper

A Gripper proxy for a uFactory gripper (`viam:ufactory:gripper`) that puts its position on the 0–1 scale, with 0 = open, that [`viam-labs:vla:controller`](https://app.viam.com/module/viam-labs/vla) uses for a `do_command` gripper. Its DoCommand `set` also returns at once, which a 10 Hz control loop needs.

The uFactory driver reports `{"pos": 0–840}` (840 = fully open). Its `{"set": n}` waits until the jaws reach the goal, stop moving for 1 s, or 10 s pass. Use this module to record demonstrations and run VLA policies, so that the training data and the controller see the same gripper values.

## Model devrel:gripper-wrapper:gripper

### Configuration

```json
{
  "name": "gripper-wrapper",
  "api": "rdk:component:gripper",
  "model": "devrel:gripper-wrapper:gripper",
  "attributes": { "gripper": "gripper" }
}
```

| Attribute | Type | Required | Default | Description |
|---|---|---|---|---|
| `gripper` | string | yes | | Name of the `viam:ufactory:gripper` to wrap. |
| `min_position` | number | no | `0` | Raw position mapped to 1 (closed). Raise it to stop `set 1` from closing the jaws all the way. |
| `max_position` | number | no | `840` | Raw position mapped to 0 (open). Set it to what the raw gripper's `{"get": true}` reports when open, if yours differs, or lower it to limit how far the jaws open. `full_open` is accepted as an older name for it. |
| `open_value` | number | no | `0` | DoCommand value for fully open (`max_position`). |
| `closed_value` | number | no | `1` | DoCommand value for fully closed (`min_position`). Either order works, e.g. `open_value: 100, closed_value: 0`. |
| `deadband_steps` | number | no | `10` | Skip a `set` closer than this (in raw steps) to the last one sent. |

### DoCommand

| Command | Response | Behavior |
|---|---|---|
| `{"get": true}` | `{"position": v}` | Raw `pos` mapped linearly from `max_position..min_position` to `open_value..closed_value` (default 0 = open, 1 = closed), clamped. |
| `{"set": v}` | `{}` | Returns immediately. `v` is mapped back from `open_value..closed_value` to a raw position (clamped) and the gripper moves there in the background; while a move runs, newer commands replace older ones and only the newest is sent. A failed move is retried on the next `set`. |
| anything else | the raw gripper's response | Forwarded unchanged, e.g. `set_gripper_speed`, `grab_with_torque`. |

### Gripper API

`Open`, `Grab`, `Stop`, `IsMoving`, `IsHoldingSomething`, `GetCurrentInputs`, `GoToInputs`, `GetKinematics` and `GetGeometries` are forwarded to the wrapped gripper. `Open`, `Grab`, `Stop` and `GoToInputs` first drop any queued DoCommand `set`.

### With viam-labs:vla:controller

```json
"gripper": { "type": "do_command", "name": "gripper-wrapper", "open_value": 0, "closed_value": 1 }
```

Keep the controller's `open_value` and `closed_value` equal to this module's.

To record demonstrations, capture `gripper-wrapper` with method `DoCommand` and input `{"get": true}`, and train with the gripper key `position`.

### Requirements

Linux with Python 3.10+. On first start the module creates a virtual environment. On Debian or Ubuntu it installs `python3-venv` with apt if it's missing; viam-server normally runs as root, which this needs.
