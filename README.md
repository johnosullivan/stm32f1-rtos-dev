# stm32f1-rtos-dev
Using stm32f103c8t6 and FreeRTOS Kernel for real time embedded solutions.

### Required Dependencies

- [stlink](https://github.com/stlink-org/stlink)
- [FreeRTOS](https://www.freertos.org/)
- [libopencm3](https://github.com/libopencm3/libopencm3)

Once you have successfully installed all the dependencies, plug in your stm32 programmer with the stm32f1 wired up. You should be able to run `st-info --probe` and see it pick up a "F1 Medium-density" which is the stm32f103c8t6 (Blue Pill) we will be flashing with our firmware bin.  
 
```bash
$ st-info --probe
Found 1 stlink programmers
 serial: 543f6d06497156532042223f
openocd: "\x54\x3f\x6d\x06\x49\x71\x56\x53\x20\x42\x22\x3f"
  flash: 131072 (pagesize: 1024)
   sram: 20480
 chipid: 0x0410
  descr: F1 Medium-density device
```
