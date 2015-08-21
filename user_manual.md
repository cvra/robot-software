# Master Board User Manual

## Connector & Board Modifications

### Timing Connector
```
PD3     ACTIVE_TASK_0
PD4     ACTIVE_TASK_1
PD5     ACTIVE_TASK_2
PD6     ACTIVE_TASK_3
PD7     ACTIVE_TASK_4
PD8     ISR
PD9     USER_DEBUG
PD10    PANIC
```

### Interface Panel
```
PF0     LED_READY
PF1     LED_DEBUG
PF2     LED_ERROR
PF3     LED_POWER_ERROR
PF4     LED_PC_ERROR
PF5     LED_BUS_ERROR
PF6     LED_YELLOW_1
PF7     LED_YELLOW_2
PF8     LED_GREEN_1
PF9     LED_GREEN_2
PF10    BTN_YELLOW
PF11    (USB_HS_FAULT)
PF12    START
PF13    BTN_GREEN
PF14    (unused)
PF15    (unused)
```

### Debug Uart Interface (Boot connector on olimex board):
```
PB10    UART3 TX
PB11    UART3 RX
```

### CAN Port
```
PD0     CAN1 RX
PD1     CAN1 TX
```