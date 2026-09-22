
# Reference Case 
---

### Step 1 — Grant display privileges

```bash
sudo xhost +
```

### Step 2 — Start the Docker container from the onboard computer

```bash
cd ~/refcase_ws/Docker
./run.sh
```

### Step 3 — Enter the Docker container

```bash
docker exec -it refcase_melodic bash
cd ..
```

### Step 4 - Launch following nodes in orders:

```bash
bash 0_can_active.sh
bash 1_launch_scout.sh
bash 2_launch_lidar.sh
bash 3_launch.sh
bash 4_launch_discrepancy_monitor.sh
bash 5_launch_smach.sh
bash 6_java_agent.sh
```