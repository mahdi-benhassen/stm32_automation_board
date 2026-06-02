#!/bin/bash
set -e

HAL_VERSION="v1.28.1"
HAL_DIR="lib/stm32f4_hal"

if [ -f "$HAL_DIR/Src/stm32f4xx_hal.c" ]; then
    echo "HAL already present, skipping download."
    exit 0
fi

echo "Cloning STM32CubeF4 ${HAL_VERSION} with submodules..."

TMPDIR=$(mktemp -d)

git clone --depth 1 --branch "${HAL_VERSION}" \
    https://github.com/STMicroelectronics/STM32CubeF4.git "$TMPDIR/STM32CubeF4"

git -C "$TMPDIR/STM32CubeF4" submodule update --init --depth 1 --recommend-shallow \
    Drivers/STM32F4xx_HAL_Driver \
    Drivers/CMSIS

mkdir -p "$HAL_DIR/Src" "$HAL_DIR/Inc" "$HAL_DIR/Inc/CMSIS_Core"

cp "$TMPDIR/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Src/"*.c "$HAL_DIR/Src/"
cp -r "$TMPDIR/STM32CubeF4/Drivers/STM32F4xx_HAL_Driver/Inc/"* "$HAL_DIR/Inc/"
cp "$TMPDIR/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c" "$HAL_DIR/Src/"
cp "$TMPDIR/STM32CubeF4/Drivers/CMSIS/Device/ST/STM32F4xx/Include/"*.h "$HAL_DIR/Inc/"
cp "$TMPDIR/STM32CubeF4/Drivers/CMSIS/Include/"*.h "$HAL_DIR/Inc/CMSIS_Core/"

rm -rf "$TMPDIR"
echo "HAL library downloaded successfully ($(find "$HAL_DIR" -name "*.c" | wc -l) source files)."
