# extra_script.py
#
# The uStepperS32 library defines its own TIM4_IRQHandler in HAL/timer.c.
# The STM32 Arduino framework (HardwareTimer.cpp) also defines TIM4_IRQHandler,
# causing a linker "multiple definition" error.
#
# The uStepper library's own source (timer.c line 51) documents this conflict
# and requires the framework's TIM4_IRQHandler to be suppressed.
#
# This script makes the framework's TIM4_IRQHandler a LOCAL symbol after
# compilation, so the linker ignores it and uses the uStepper's handler.
# No framework files are modified on disk.

import subprocess
import os
Import('env')


def localize_tim4_irqhandler(source, target, env):
    obj = str(target[0])
    if not os.path.exists(obj):
        return
    # Derive objcopy from the PlatformIO toolchain package directory
    toolchain_dir = env.PioPlatform().get_package_dir("toolchain-gccarmnoneeabi")
    objcopy = os.path.join(toolchain_dir, "bin", "arm-none-eabi-objcopy.exe")
    if not os.path.exists(objcopy):
        # Fallback: try without .exe (Linux/macOS)
        objcopy = os.path.join(toolchain_dir, "bin", "arm-none-eabi-objcopy")
    if not os.path.exists(objcopy):
        print(f"extra_script: objcopy not found at {objcopy}, skipping TIM4 patch")
        return
    result = subprocess.run(
        [objcopy, "--localize-symbol=TIM4_IRQHandler", obj, obj],
        capture_output=True, text=True
    )
    if result.returncode == 0:
        print("extra_script: TIM4_IRQHandler localised in HardwareTimer.cpp.o")
    else:
        print(f"extra_script: objcopy failed: {result.stderr}")


env.AddPostAction(
    "$BUILD_DIR/SrcWrapper/src/HardwareTimer.cpp.o",
    localize_tim4_irqhandler
)
