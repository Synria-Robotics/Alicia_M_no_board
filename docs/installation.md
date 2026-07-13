# Installation

```bash
cd arx_d_can
pip install -e .
```

## Teleop / IK (geom_arx)

```bash
pip install -e ".[kinematics]"
```

## Development

```bash
pip install -e ".[dev]"
```

Default hardware config: `arx_d_can/config/arx_d_can_dm.yaml`

After package layout changes:

```bash
pip install -e . --no-build-isolation
```
