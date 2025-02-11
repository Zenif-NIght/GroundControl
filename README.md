## Installing IsaacLab
https://isaac-sim.github.io/IsaacLab/source/setup/installation/index.html

## Installing GroundControl

```bash
# GroundControl sits on top of IsaacLab, and is a spearate set of packages.
# Activate the conda environment that was created via the IsaacLab setup.
conda activate IsaacLab

git clone git@github.com:UWRobotLearning/GroundControl.git
cd source/extensions
pip install -e groundcontrol
pip install -e groundcontrol_assets
pip install -e groundcontrol_tasks
```

### Run Teleop Example
```bash
# Assuming this python is tied to isaac-sim, otherwise see Isaac-Sim / IsaacLab docs:
python source/standalone/environments/teleoperation/teleop_se2_agent.py --task Isaac-Navigation-Flat-Spot-Play-v0 --num_envs 1 --teleop_device keyboard
```
