import asyncio

from viam.module.module import Module

import gripper_wrapper  # noqa: F401  registers GripperWrapper with the registry

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
