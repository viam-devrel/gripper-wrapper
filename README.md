# gripper-wrapper

A transparent proxy for a uFactory gripper (`viam:ufactory:gripper`). It speaks the same DoCommand protocol and units as the gripper it wraps, so anything configured against the raw gripper, such as [`viam-labs:vla:controller`](https://app.viam.com/module/viam-labs/vla), works unchanged against the wrapper. Only the resource name differs.

What it adds:

- **Non-blocking `set`.** The uFactory driver's `{"set": n}` waits until the jaws reach the goal, stop moving for 1 s, or 10 s pass. The wrapper's returns at once, which a 10 Hz control loop needs.
- **Position limits.** `set` goals are clamped to `min_position..max_position`.
- **Deadband.** Goals within `deadband_steps` of the last one sent are skipped.

## Model devrel:gripper-wrapper:gripper

### Configuration

```json
{
  "name": "gripper-wrapper",
  "api": "rdk:component:gripper",
  "model": "devrel:gripper-wrapper:gripper",
  "attributes": { "gripper": "gripper", "min_position": 100, "max_position": 700 }
}
```

| Attribute | Type | Required | Default | Description |
|---|---|---|---|---|
| `gripper` | string | yes | | Name of the `viam:ufactory:gripper` to wrap. |
| `min_position` | number | no | no limit | Lowest raw position a `set` may target (raw units; lower = more closed on the G2). |
| `max_position` | number | no | no limit | Highest raw position a `set` may target (840 = fully open on the G2). |
| `deadband_steps` | number | no | `10` | Skip a `set` closer than this (in raw steps) to the last one sent. |

### DoCommand

| Command | Response | Behavior |
|---|---|---|
| `{"set": n, ...}` | `{}` | Returns immediately. `n` is clamped to `min_position..max_position` and the command, with any other keys, is sent to the raw gripper in the background. While a move runs, newer commands replace older ones and only the newest is sent. A failed move is retried on the next `set`. |
| anything else | the raw gripper's response | Forwarded unchanged, including `{"get": true}` → `{"pos": n}`, `set_gripper_speed` and `grab_with_torque`. |

### Gripper API

`Open`, `Grab`, `Stop`, `IsMoving`, `IsHoldingSomething`, `GetCurrentInputs`, `GoToInputs`, `GetKinematics` and `GetGeometries` are forwarded to the wrapped gripper. `Open`, `Grab`, `Stop` and `GoToInputs` first drop any queued DoCommand `set`.

### With viam-labs:vla:controller

Use the same `gripper` block you would for the raw uFactory gripper, pointed at the wrapper:

```json
"gripper": { "type": "do_command", "name": "gripper-wrapper", "open_value": 840, "closed_value": 0, "read_key": "pos" }
```

`0` is where the uFactory driver's `Grab` closes a G2; it closes a G1 to `2`.

To record demonstrations, capture `gripper-wrapper` with method `DoCommand` and input `{"get": true}`.

### Requirements

Linux with Python 3.10+. On first start the module creates a virtual environment. On Debian or Ubuntu it installs `python3-venv` with apt if it's missing; viam-server normally runs as root, which this needs.
