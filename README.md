# RTAB-Map GPU Support Guide

This guide documents my successful attempts and knowledge gained while building RTAB-Map with GPU support. The process involved significant debugging, and I benefited greatly from the help of the RTAB-Map development team. For reference, see their support here:  
[RTAB-Map ROS Issue #1238, Comment](https://github.com/introlab/rtabmap_ros/issues/1238#issuecomment-2851189592)

## Table of Contents
1. [Disclaimer](#disclaimer)
2. [Build Sets Context](#build-sets-context)
3. [Build & Run](#build--run)
4. [Launch File & Testing](#launch-file--testing)
5. [Hardware Tested](#hardware-tested)

---

## Disclaimer

If you encounter errors while running the `.launch` file, some packages may be missing. Most missing dependencies can be resolved by installing them via `apt install`.

---

## Build Sets Context

There are two build attempts described. "Set 1" is more complete than the second, with the main difference being in the final step involving the building of `rtabmap-ros` in the ROS Noetic `catkin_ws`.

### **Build Set 1: rtabmap-auto**
- **Type:** Semi-automatic build process for `rtabmap-ros`
- **Notes:** Feels more complete; compiles without errors

### **Build Set 2 (Auxiliary): rtabmap**
- **Type:** Full manual installation of most required packages for `rtabmap-ros` targeting specific versions
- **Notes:** First successful build, but later found not strictly necessary to manually install all. Some packages, such as `aruco-detect`, were excluded due to incompatibilities with other libraries.

---

## Build & Run

- Execute the appropriate `script` file to build.
- **Important:** Change the image name in `run.sh` as needed. The flags used in the run script are important and are based on the official `rtabmap-ros` docker-image.
- Remember to source both `ros` and `catkin_ws` if the `entryfile` does not work properly.

---

## Launch File & Testing

- The provided `.launch` file includes all flags known to me to use `Gpu`.
- Testing was performed using publicly available `rosbags`. https://docs.google.com/uc?export=download&confirm=5Ptp&id=0B46akLGdg-uaVEs2SGFzT3ZVSU0
- Both `odometry` and `slam` nodes were observed running on the GPU. However, GPU activity increased by only 3–5%.
- **Tip:** Suppress terminal output, as it can register as GPU utilization. Stop any other GPU-related tasks (e.g., video playback) to accurately observe actual GPU usage. [menioned in the end of docker file]
- Commented lines in scripts may include extra information, packages I thought were needed, or configurations I couldn’t set up properly.

---

## Hardware Tested

- **OS:** Linux Mint 22.1
- **CPU:** Intel i7-6700HQ
- **GPU:** GTX 970m
- **RAM:** 16GB 
- **Storage:** Samsung SSD 970 EVO 250GB ( 80GB Ext4 Partition, 16GB Swap ) 
