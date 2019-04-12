# Infantry-RTOS (Sentry Version)
This code is modified for sentry.
  
### For debugging IR sensor output:   
1. Please make sure yellow wire is connected to **C** (PH11), white wire to **E** (PD15), and orange wire to **G** (PD13).  
2. Open J-Scope and select the axf file, set device to Cortex-M4.  
3. Tick `GPIO_left_debug_js` and `GPIO_right_debug_js` in `gimbal_task.c`.  
4. The correct pattern should be: When it detects nothing, output is **5000**; when it detects something, output is **1000**; when it is offline, output is **1000**.  
