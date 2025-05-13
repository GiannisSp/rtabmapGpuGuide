# rtabmapGpuGuide
Here is my successfull attempts and the knowledge I gathered in trying to build rtabmap with gpu suppport. It took me a lot of debugging and I couldn't have done it without the support I got from the dev-team, which you can find here : https://github.com/introlab/rtabmap_ros/issues/1238#issuecomment-2851189592
Below there are two build attemps. "Set 2" probably is more complete than the first set. Sets only differ in the last step of building rtabmap-ros in ros-noetic catkwin_ws.

**Disclaimer** - If any errors appear while running the .launch file , there are may be some packages missing that can be resolved by `apt install` them. 


**A. Build sets context** 

**Build Set 1:** rtabmap-auto
This is a semi-automatic process of rtabmap-ros. Feels more complete and compiles without errors 

**Build Set 2 (auxillery):** rtabmap 
First successfull build I did, that ended up not being needed, after it worked and I tried to follow different route. This is full manual installation of most of the packages needed for rtabmap-ros. Some packages may be missing e.g. aruco-detect that I coudn't include because of incopatibilities with other libraries

**B. Build & Run**
Run the appropriate `script` file to build. Remind to change the appropriate image name in `run.sh`, flags used in run script are important. Remember to source `ros` & `catkin_ws` if `entryfile` does not work well 

**C. Launch file & Test**
In theory I included all the known flags to me in the .launch file. I tested the build using publicly availiable `rosbags`. It is obvious that `odometry` & `slam` nodes are running on GPU but the GPU activity only went up by 3-5%. Remember to supress the output of the terminals because that also register as gpu utilization and stop any other gpu related task e.g videos, to observe the actual utilization of the gpu. 

The commented lines are either extra information, packages that I thought they were needed and/or I didn't manage to setup them properly or how I thought I had to

Hardware tested : 
Linux Mint <version>
i7-6700HQ
GTX 970m 
16GB RAM
<ssd specs> 
