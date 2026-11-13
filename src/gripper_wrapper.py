"""Transparent proxy for viam:ufactory:gripper that makes DoCommand "set" non-blocking.

Speaks the wrapped gripper's own protocol and units, so a client (e.g. the vla controller's
do_command adapter) is configured identically whether it targets this or the raw gripper.
{"set": v} is clamped to [min_position, max_position], returns at once, and moves the gripper
in the background, newest command wins; viam:ufactory:gripper's own "set" blocks until the
jaws settle. Everything else, including {"get": true}, is forwarded unchanged.
"""

import asyncio
import math
from typing import Any, Awaitable, Callable, ClassVar, Dict, List, Mapping, Optional, Sequence, Tuple

from viam.components.gripper import Gripper
from viam.logging import getLogger
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

LOGGER = getLogger(__name__)


def position_range(attrs: Mapping[str, Any]) -> Tuple[float, float]:
    """(min_position, max_position) in the wrapped gripper's raw units; unset ends don't limit."""
    lo = float(attrs.get("min_position", -math.inf))
    hi = float(attrs.get("max_position", math.inf))
    if lo >= hi:
        raise ValueError(f"'min_position' ({lo:g}) must be less than 'max_position' ({hi:g})")
    return lo, hi


class LatestWins:
    """Runs send(value) in the background; values submitted while one runs collapse to the newest."""

    def __init__(self, send: Callable[[Any], Awaitable[Any]]):
        self._send = send
        self._next: Any = None
        self._task: Optional[asyncio.Task] = None

    def submit(self, value: Any) -> None:
        self._next = value
        if self._task is None or self._task.done():
            self._task = asyncio.get_running_loop().create_task(self._drain())

    async def _drain(self) -> None:
        while self._next is not None:
            value, self._next = self._next, None
            try:
                await self._send(value)
            except Exception:
                LOGGER.exception("gripper set %s failed", value)


class GripperWrapper(Gripper, EasyResource):
    MODEL: ClassVar[Model] = Model(ModelFamily("devrel", "gripper-wrapper"), "gripper")

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        # EasyResource.new only sets the name; it does not call reconfigure.
        self = super().new(config, dependencies)
        self.reconfigure(config, dependencies)
        return self

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        attrs = struct_to_dict(config.attributes)
        raw = attrs.get("gripper")
        if not isinstance(raw, str) or not raw:
            raise ValueError("'gripper' (name of the viam:ufactory:gripper to wrap) is required")
        position_range(attrs)
        return [raw], []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        attrs = struct_to_dict(config.attributes)
        self.min_position, self.max_position = position_range(attrs)
        # Skip writes closer than this to the last one; the controller writes every tick.
        # The G2's set only returns early within +/-6 of the goal, so tiny moves cost ~1 s each.
        self.deadband_steps = float(attrs.get("deadband_steps", 10))
        self.raw = next((v for k, v in dependencies.items() if k.name == attrs["gripper"]), None)
        if self.raw is None:
            raise ValueError(f"dependency {attrs['gripper']!r} not found")
        self.last_sent: Optional[float] = None
        self.sender = LatestWins(self._set)

    async def _set(self, command: Dict[str, Any]) -> None:
        try:
            await self.raw.do_command(command)
        except Exception:
            self.last_sent = None  # failed: let the next command through the deadband
            raise

    def _cancel_queued(self) -> None:
        # Drop the queued goal and forget the last one so the next DoCommand set isn't deadbanded.
        self.sender._next = None
        self.last_sent = None

    async def close(self) -> None:
        # viam-server closes this instance and builds a new one on any config change;
        # drop the queued goal so it can't land after the new instance's commands.
        self._cancel_queued()

    # Typed Gripper API: forwarded to the wrapped gripper.
    # **kwargs carries server-side metadata (a MultiDict) the raw client can't take; don't forward it.
    async def open(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        self._cancel_queued()
        return await self.raw.open(extra=extra, timeout=timeout)

    async def grab(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs) -> bool:
        self._cancel_queued()
        return await self.raw.grab(extra=extra, timeout=timeout)

    async def stop(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        self._cancel_queued()
        return await self.raw.stop(extra=extra, timeout=timeout)

    async def is_moving(self) -> bool:
        return await self.raw.is_moving()

    async def is_holding_something(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> Gripper.HoldingStatus:
        return await self.raw.is_holding_something(extra=extra, timeout=timeout)

    async def get_current_inputs(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ) -> List[float]:
        return await self.raw.get_current_inputs(extra=extra, timeout=timeout)

    async def go_to_inputs(
        self, values: List[float], *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs
    ):
        self._cancel_queued()
        return await self.raw.go_to_inputs(values, extra=extra, timeout=timeout)

    async def get_kinematics(
        self,
        *,
        include_collision_meshes: bool = False,
        include_visual_meshes: bool = False,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ):
        return await self.raw.get_kinematics(
            include_collision_meshes=include_collision_meshes,
            include_visual_meshes=include_visual_meshes,
            extra=extra,
            timeout=timeout,
        )

    async def get_geometries(self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None, **kwargs):
        return await self.raw.get_geometries(extra=extra, timeout=timeout)

    async def do_command(
        self, command: Mapping[str, ValueTypes], *, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, ValueTypes]:
        if "set" in command:
            goal = min(self.max_position, max(self.min_position, float(command["set"])))
            if self.last_sent is None or abs(goal - self.last_sent) >= self.deadband_steps:
                self.last_sent = goal
                # Keep any other keys the caller sent alongside "set" (e.g. controller write_args).
                self.sender.submit({**command, "set": goal})
            return {}
        return await self.raw.do_command(command, timeout=timeout)
