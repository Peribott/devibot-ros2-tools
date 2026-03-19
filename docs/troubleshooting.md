# Troubleshooting Guide — devibot ROS2 Tools

## Packet Decoder Issues

### No packets decoded — decoder stats show 0

**Check 1: Baud rate mismatch**
```bash
# STM32 firmware uses 460800. Verify:
stty -F /dev/ttyS0
# Should show: speed 460800 baud
```

**Check 2: Wrong serial port**
```bash
ls /dev/ttyS* /dev/ttyUSB* /dev/ttyACM*
dmesg | grep tty   # Shows recent USB-serial connections
```

**Check 3: CRC errors climbing (decoder.stats.crc_errors)**
Indicates baud rate mismatch or signal noise. Check cable length
and shielding. devibot uses RS232 level-shifted UART — ensure
your level shifter matches.

---

### High framing_errors in decoder stats

Framing errors mean the decoder is finding SOF bytes but the frame
doesn't validate. Common causes:

1. **Wrong baud rate** — partial frames look like garbage
2. **UART flow control enabled** — disable RTS/CTS if not used
3. **Buffer overflow** — check `buffer_overflows` stat

---

## ROS2 Node Issues

### Node not appearing in `ros2 node list`

```bash
# Check ROS_DOMAIN_ID matches across all terminals
echo $ROS_DOMAIN_ID

# Check if node process is running
ps aux | grep devibot

# Check systemd service
systemctl status robot-dashboard

# Check logs
./scripts/log_viewer.sh --filter ERROR
```

### Topic exists but no messages

```bash
# Check publisher QoS
ros2 topic info /your_topic --verbose

# Check if publisher and subscriber QoS are compatible
# TRANSIENT_LOCAL publisher + VOLATILE subscriber = no stored messages
```

---

## CANopen / FDCAN Issues

### BMS data frozen / never updates

Most common cause: FDCAN global filter rejecting 29-bit extended frames.
See [can_bus_setup.md](can_bus_setup.md) for the fix.

Verify with:
```bash
./scripts/check_can_bus.sh can0
# Look for BMS frame IDs: 351, 355, 356
```

### Motor controllers not responding to SDO

1. Check CANopen node ID matches drive configuration
2. Verify NMT state — drives must be in OPERATIONAL state
3. Check for CAN bus errors: `ip -s link show can0`

---

## Health Check Failures

### `ros2_health_check.sh` reports nav2_bt_navigator missing

Navigation stack not fully started. Check boot phase:

```bash
cat /var/log/robot-dashboard/boot.log | tail -20
```

If Phase 3 failed, Nav2 did not start. Common causes:
- Map file missing for configured map name
- `robot_config.yaml` has invalid map path
- Nav2 parameter file has a syntax error

### Topics exist but show "no messages"

Some topics are only published when the robot is active:
- `/amcl_pose` — only published during active localisation
- `/nav2_bt_navigator/feedback` — only during active navigation

These are expected. The health check notes this as a warning, not a failure.

---

## Performance

### LiDAR scan rate low (<8 Hz on LD19)

```bash
# Check actual rate
ros2 topic hz /scan

# LD19 nominal: 10 Hz. If <8 Hz:
# 1. Check USB connection quality
# 2. Check CPU load: htop
# 3. Check UART errors in driver node logs
```

### Navigation replanning too slow

Increase Nav2 controller frequency in `nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Default 10 Hz
```

---

*For further support: amit@peribott.com · [peribott.com](https://peribott.com)*
