# CANopen on STM32 — Setup & Troubleshooting

## The Silent Drop Problem

The STM32 FDCAN peripheral has a global filter whose default setting
(`FDCAN_FILTER_REJECT`) silently discards all frames that don't match
a configured filter. The Pylontech BMS uses **29-bit extended frames**.
If your filters only cover 11-bit standard frames, BMS messages are
silently dropped with no error.

## Correct Filter Configuration

```c
/* Step 1: Configure individual extended ID filter */
FDCAN_FilterTypeDef ext_filter = {
    .IdType       = FDCAN_EXTENDED_ID,
    .FilterIndex  = 0,
    .FilterType   = FDCAN_FILTER_RANGE,
    .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
    .FilterID1    = 0x00000000,   /* Accept all extended IDs */
    .FilterID2    = 0x1FFFFFFF,
};
HAL_FDCAN_ConfigFilter(&hfdcan1, &ext_filter);

/* Step 2: Set global filter — do NOT leave as default */
FDCAN_GlobalFilterTypeDef global = {
    .NonMatchingStd = FDCAN_FILTER_REJECT,      /* Reject unknown standard */
    .NonMatchingExt = FDCAN_FILTER_TO_RXFIFO0,  /* Accept all extended */
    .RejectRemoteStd = DISABLE,
    .RejectRemoteExt = DISABLE,
};
HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, &global);
```

## devibot CAN ID Map

| CAN ID | Frame | Device | Message |
|--------|-------|--------|---------|
| 0x601 | Standard | Motor 1 | SDO Request |
| 0x581 | Standard | Motor 1 | SDO Response |
| 0x181 | Standard | Motor 1 | PDO1 (velocity/position) |
| 0x281 | Standard | Motor 1 | PDO2 (current/temp) |
| 0x701 | Standard | Motor 1 | NMT Heartbeat |
| 0x602 | Standard | Motor 2 | SDO Request |
| 0x351 | Extended | BMS | Pack voltage & current |
| 0x355 | Extended | BMS | SoC & SoH |
| 0x356 | Extended | BMS | Cell temperatures |

## Bitrate

devibot CAN bus: **500 kbit/s**

```c
hfdcan1.Init.NominalPrescaler = 4;
hfdcan1.Init.NominalSyncJumpWidth = 1;
hfdcan1.Init.NominalTimeSeg1 = 13;
hfdcan1.Init.NominalTimeSeg2 = 2;
/* Results in 500 kbit/s at 80 MHz APB clock */
```

## Diagnostics

```bash
# Check bus activity
./scripts/check_can_bus.sh can0

# Live dump — verify BMS frames (29-bit IDs are 8 hex chars)
candump can0 | grep -E "[0-9A-F]{8}"

# Decode Pylontech BMS frames
candump can0 #351 #355 #356 #359
```

## ISR Safety Rule

Never call blocking FreeRTOS APIs from a CAN RX callback.
Use queue `FromISR` variants only:

```c
/* WRONG */
void HAL_FDCAN_RxFifo0Callback(...) {
    xSemaphoreTake(mutex, portMAX_DELAY); /* CRASH */
}

/* CORRECT */
void HAL_FDCAN_RxFifo0Callback(...) {
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(can_queue, &frame, &woken);
    portYIELD_FROM_ISR(woken);
}
```

See [freertos-isr-mutex article](https://www.jagnani.com/articles/freertos-isr-mutex.html).

---
*Peribott Dynamic LLP · [peribott.com](https://peribott.com)*
