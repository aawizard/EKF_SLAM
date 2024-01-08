# Nuturtle  Description
URDF files for Nuturtle SLAMbot
* `ros2 launch nuturtle_description load_one.launch.py color:=red` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.
![](images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![](images/rqt_graph.svg)
# Launch File Details
* `ros2 launch nuturtle_description load_one.launch.py --show-args`
  ```
    Arguments (pass arguments as '<name>:=<value>'):

    'use_jsp':
        true (default): use joint_state_publisher,false: no joint states published
        (default: 'true')

    'use_rviz':
        true (default): start rviz, otherwise don't start rviz
        (default: 'true')
```
* `ros2 launch nuturtle_description load_one.launch.py --show-args`
  ```
    Arguments (pass arguments as '<name>:=<value>'):

    'use_jsp':
        true (default): use joint_state_publisher,false: no joint states published
        (default: 'true')

    'use_rviz':
        true (default): start rviz, otherwise don't start rviz
        (default: 'true')
```