import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from google.protobuf.struct_pb2 import Struct
from multidict import MultiDict
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName

from viam.components.gripper import Gripper

from gripper_wrapper import GripperWrapper, LatestWins, to_raw, to_unit


class FakeRaw:
    """Stands in for viam:ufactory:gripper: 'set' blocks like the real one."""

    def __init__(self, pos=840, set_delay=0.2):
        self.pos, self.set_delay, self.sets, self.calls = pos, set_delay, [], []

    def _record(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))
        return f"{name}-result"

    async def open(self, **kw):
        return self._record("open", **kw)

    async def grab(self, **kw):
        self._record("grab", **kw)
        return True

    async def stop(self, **kw):
        return self._record("stop", **kw)

    async def is_moving(self):
        self._record("is_moving")
        return True

    async def is_holding_something(self, **kw):
        self._record("is_holding_something", **kw)
        return Gripper.HoldingStatus(is_holding_something=True)

    async def get_current_inputs(self, **kw):
        self._record("get_current_inputs", **kw)
        return [42.0]

    async def go_to_inputs(self, values, **kw):
        return self._record("go_to_inputs", values, **kw)

    async def get_kinematics(self, **kw):
        self._record("get_kinematics", **kw)
        return ("fmt", b"kin")

    async def get_geometries(self, **kw):
        return self._record("get_geometries", **kw)

    async def do_command(self, cmd, **kw):
        if cmd.get("get"):
            return {"pos": self.pos}
        if "set" not in cmd:
            self.calls.append(("do_command", (cmd,), kw))
            return {"echo": dict(cmd)}
        self.sets.append(cmd["set"])
        await asyncio.sleep(self.set_delay)
        self.pos = int(cmd["set"])
        return {"position": self.pos}


def make(raw, **attrs):
    s = Struct()
    s.update({"gripper": "gripper-raw", **attrs})
    cfg = ComponentConfig(name="gripper", attributes=s)
    return GripperWrapper.new(cfg, {ResourceName(name="gripper-raw"): raw})


def test_scale_round_trip():
    assert to_unit(850, 850) == 0.0  # fully open
    assert to_unit(0, 850) == 1.0  # fully closed
    assert to_raw(0.0, 850) == 850
    assert to_raw(1.0, 850) == 0
    assert to_raw(to_unit(425, 850), 850) == 425
    assert to_unit(900, 850) == 0.0 and to_raw(1.3, 850) == 0  # clamped


def test_validate_requires_gripper():
    try:
        GripperWrapper.validate_config(ComponentConfig(name="gripper", attributes=Struct()))
    except ValueError:
        return
    raise AssertionError("missing 'gripper' attribute was accepted")


def test_get_reports_unit_scale():
    async def run():
        w = make(FakeRaw(pos=0))
        assert await w.do_command({"get": True}) == {"position": 1.0}

    asyncio.run(run())


def test_set_returns_at_once_and_newest_wins():
    async def run():
        raw = FakeRaw(set_delay=0.2)
        w = make(raw)
        loop = asyncio.get_running_loop()
        for v in (0.2, 0.5, 0.8, 1.0):  # 4 ticks, 10 ms apart, while the first set is still moving
            t0 = loop.time()
            assert await w.do_command({"set": v}) == {}
            assert loop.time() - t0 < 0.1, "set blocked"
            await asyncio.sleep(0.01)
        await asyncio.sleep(0.5)
        assert raw.sets == [672.0, 0.0], raw.sets  # first, then only the newest

    asyncio.run(run())


def test_deadband_skips_tiny_moves():
    async def run():
        raw = FakeRaw(set_delay=0)
        w = make(raw, deadband_steps=10)
        await w.do_command({"set": 0.5})
        await w.do_command({"set": 0.505})  # ~4 steps away
        await asyncio.sleep(0.05)
        assert raw.sets == [420.0]

    asyncio.run(run())


def test_other_commands_forwarded():
    async def run():
        raw = FakeRaw()
        w = make(raw)
        assert await w.do_command({"set_gripper_speed": 2000}, metadata=MultiDict()) == {"echo": {"set_gripper_speed": 2000}}
        assert raw.calls == [("do_command", ({"set_gripper_speed": 2000},), {"timeout": None})]

    asyncio.run(run())


def test_failed_set_is_retried():
    class FlakyRaw(FakeRaw):
        async def do_command(self, cmd):
            if "set" in cmd and not self.sets:
                self.sets.append(cmd["set"])
                raise RuntimeError("boom")
            return await super().do_command(cmd)

    async def run():
        raw = FlakyRaw(set_delay=0)
        w = make(raw)
        await w.do_command({"set": 1.0})
        await asyncio.sleep(0.05)
        await w.do_command({"set": 1.0})  # same goal again: must not be swallowed by the deadband
        await asyncio.sleep(0.05)
        assert raw.sets == [0.0, 0.0], raw.sets

    asyncio.run(run())


def test_close_drops_queued_set():
    async def run():
        raw = FakeRaw(set_delay=0.2)
        w = make(raw)
        await w.do_command({"set": 0.0})
        await asyncio.sleep(0.01)  # first set is in flight
        await w.do_command({"set": 1.0})  # queued behind it
        await w.close()
        await asyncio.sleep(0.5)
        assert raw.sets == [840.0], raw.sets

    asyncio.run(run())


def test_get_accepts_truthy():
    async def run():
        w = make(FakeRaw(pos=0))
        assert await w.do_command({"get": 1}) == {"position": 1.0}

    asyncio.run(run())


def test_missing_dependency_is_readable():
    s = Struct()
    s.update({"gripper": "gripper-raw"})
    try:
        GripperWrapper.new(ComponentConfig(name="gripper", attributes=s), {})
    except ValueError as e:
        assert "gripper-raw" in str(e)
        return
    raise AssertionError("missing dependency was accepted")


def test_model_and_abstracts():
    assert str(GripperWrapper.MODEL) == "devrel:gripper-wrapper:gripper"
    assert issubclass(GripperWrapper, Gripper)
    assert GripperWrapper.__abstractmethods__ == frozenset()


def test_gripper_methods_forward():
    async def run():
        raw = FakeRaw()
        w = make(raw)
        ex = {"k": 1}
        md = MultiDict()  # grpclib metadata the server passes in kwargs; must not reach raw
        assert await w.open(extra=ex, timeout=3.0, metadata=md) == "open-result"
        assert await w.grab(extra=ex, timeout=3.0, metadata=md) is True
        assert await w.stop(extra=ex, timeout=3.0, metadata=md) == "stop-result"
        assert await w.is_moving() is True
        assert (await w.is_holding_something(extra=ex, timeout=3.0, metadata=md)).is_holding_something is True
        assert await w.get_current_inputs(extra=ex, timeout=3.0, metadata=md) == [42.0]
        assert await w.go_to_inputs([1.0], extra=ex, timeout=3.0, metadata=md) == "go_to_inputs-result"
        assert await w.get_kinematics(include_collision_meshes=True, extra=ex, timeout=3.0, metadata=md) == ("fmt", b"kin")
        assert await w.get_geometries(extra=ex, timeout=3.0, metadata=md) == "get_geometries-result"
        kw = {"extra": ex, "timeout": 3.0}
        assert raw.calls == [
            ("open", (), kw),
            ("grab", (), kw),
            ("stop", (), kw),
            ("is_moving", (), {}),
            ("is_holding_something", (), kw),
            ("get_current_inputs", (), kw),
            ("go_to_inputs", ([1.0],), kw),
            ("get_kinematics", (), {"include_collision_meshes": True, "include_visual_meshes": False, **kw}),
            ("get_geometries", (), kw),
        ], raw.calls

    asyncio.run(run())


def test_typed_move_resets_deadband():
    async def run():
        raw = FakeRaw(set_delay=0)
        w = make(raw)
        await w.do_command({"set": 0.5})
        await asyncio.sleep(0.05)
        await w.open()
        await w.do_command({"set": 0.5})  # same goal as before the typed move: must be sent again
        await asyncio.sleep(0.05)
        assert raw.sets == [420.0, 420.0], raw.sets

    asyncio.run(run())


def test_stop_drops_queued_set():
    async def run():
        raw = FakeRaw(set_delay=0.2)
        w = make(raw)
        await w.do_command({"set": 0.0})
        await asyncio.sleep(0.01)  # first set is in flight
        await w.do_command({"set": 1.0})  # queued behind it
        await w.stop()
        await asyncio.sleep(0.5)
        assert raw.sets == [840.0], raw.sets

    asyncio.run(run())
