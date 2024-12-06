![Project Hermes Full](/assets/DemoAll.gif "Project Hermes")
![Project Hermes with Pivot](/assets/pivot.gif "Project Hermes")

# Starting the Project

**Disclaimer: This work was accomplished with a PC running Ubuntu 24.04, With a 24 core AMD CPU. You must also have Docker installed**

```
xhost +local:docker
docker compose up -d
docker exec -it rigor bash
make all
source setup_env.bash
ros2 launch hermes_robot all.launch.py 
```

# Overview of the Project

More of the specifics on timelines and goals can be found [here](/LIVING_DOC.md)

# High Level Focus
To keep this document understandable, each goal represents a milestone I'd like to achieve. Once all the milestones are complete, the research is complete. The high-level goal is to be able to run a simulated environment on my PC that a Raspberry Pi can interact with. I will vary the environment and the ROS params that the Pi consumes to simulate how it behaves/achieves autonomy in a simulated space. The hypothesis is that, for low risk scenarios, you can reduce the frequency of data aquisition. A "low risk" scenario is one with few other dynamic actors. "High Risk" would be the opposite: greater sampling of the area. 

While this idea isn't novel on the face, I haven't found research into to what degree of risk requires what degree of sampling. Most companies right now just use high same rates as a rule, which can decrease the lifespan of their products. I want to know if there is a more dynamic way of doing it.

Note: Each goal section will have notes, thoughts, and conclusions. The idea is that anyone could restart this research by simply following the conclusion blocks.