MSDE f1tenth Obstacle Passing
---

Seoultech MSDE autonomouse driving (f1tenth) obstacle passing ros package   

#### tested environment
Ubuntu 18.04   
ROS Melodic   

#### Qaulifying

robot will start driving after (around) 5 seconds

1. run command - roslaunch (single driving)
```bash
$ roslaunch msde_fgm sim_msde_driving.launch
```

---

#### Grand Prix - Head to Head

1. unique team id : ```/msde_race```   

* scan : ```/msde_race/scan```
* odom : ```/msde_race/odom```
* nav : ```/msde_race/drive```
   
   
2. run command (without gym_bridge_host.launch)
```bash
$ roslaunch msde_fgm sim_grand_prix_driving.launch
```
