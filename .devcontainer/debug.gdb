# Remote setup
set architecture armv7
target remote :2331
monitor reset
monitor halt

# Load debug symbols and firmware
file build/CubeOrangePlus/bin/ardurover
load build/CubeOrangePlus/bin/ardurover

# Set early breakpoints
b Reset_Handler
b main

# Enable graphical debug
tui enable

# Run
continue