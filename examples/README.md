# ARX-D-CAN Examples

按编号顺序运行，越靠前风险越低。

```bash
sudo chmod 666 /dev/ttyACM0
cd arx_d_can
pip install -e .
```

| # | Script | 说明 |
|---|--------|------|
| 01 | `example_01_scan_ids.py` | 扫描电机 ID，不使能 |
| 02 | `example_02_read_state.py` | 读取关节状态 |
| 03 | `example_03_zero_current_position.py` | 写零点（需 `--yes`） |
| 04 | `example_04_send_position.py` | 发送关节位置 |
| 05 | `example_05_gripper_open_close.py` | 夹爪开合（0=闭，1000=开） |
| 06 | `example_06_benchmark_read_rate.py` | 读取频率 benchmark |
| 07 | `example_07_change_damiao_id.py` | 修改达妙电机 ID（需 `--yes`） |

所有示例支持 `--port` 和 `--baud`。
