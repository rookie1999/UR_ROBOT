如果调整附着物体与末端坐标系的三维坐标关系，当该物体嵌入进末端，会直接显示碰撞产生


注意：plan的api的返回值 目前已经改为如下所示
```
plan_success, traj, planning_time, error_code = arm.plan()
```