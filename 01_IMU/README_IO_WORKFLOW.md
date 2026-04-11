# 01_IMU I/O Workflow (for collaboration)

This page defines a stable collaboration workflow:
- see **input** shape
- see **output** shape
- record real output data
- replay recorded data without hardware

## 1) Input observer (hardware side)
IMU hardware input enters this module from serial in:
- `01_IMU/imu_bridge.py` (`SerialReader._read_loop`)

The parser boundary is:
- `IMUPipeline.process(raw_line)`

## 2) Output observer (consumer side)
Use this script to inspect bridge output in real time:

```bash
cd 01_IMU
python listen_imu_websocket.py --url ws://localhost:8766
```

It prints decoded fields and records every frame to:
- `01_IMU/data_log/imu_raw_YYYYMMDD_HHMMSS.jsonl`

## 3) Record real field data
Field run procedure:

1. Start real bridge (`imu_bridge.py`) connected to hardware.
2. Run `listen_imu_websocket.py` in parallel.
3. Drive/collect in real environment.
4. Stop listener and keep generated `.jsonl` as canonical sample.

## 4) Replay real data to collaborators (no hardware)
Use the replay server:

```bash
cd 01_IMU
python replay_imu_websocket.py \
  --input data_log/imu_raw_v1.jsonl \
  --port 8766 --hz 50 --loop
```

Now downstream modules can connect to `ws://localhost:8766` as if IMU hardware is live.

## 5) Contract recommendation
For long-term compatibility, publish a short contract document per module:
- required keys: e.g. `rot`, `euler`, `heading`, `hz`
- optional keys
- field units and coordinate convention
- sample JSONL files for each contract version

This makes modules replaceable as black boxes while keeping integration stable.
