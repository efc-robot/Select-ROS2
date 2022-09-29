# Select-ROS2
## Abstract
ROS (Robot Operating System) and its successor ROS2 are widely used middlewares, providing diverse libraries and a Data-Centric Publish/Subscriber (DCPS) communication framework, which separates computing and communication.
However, present computing-communication-separated middleware does not adapt to the dynamicity of heterogeneous computing systems and the diversity of artificial intelligence (AI) algorithms.
To fully leverage the heterogeneous computing platforms and AI algorithms for more autonomous robots, we improve ROS2 to Select-ROS2.
Select-ROS2 allows different subscribers in ROS2 to perform the same function, and let the same message only be sent to one of the subscribers.
Furthermore, Select-ROS2 enables automatic runtime algorithm and hardware selection instead of handcraft offline selection, to perform the computing workload balance on the heterogeneous computing system.
Select-ROS2 is open-sourced at https://github.com/efc-robot/Select-ROS2.

<img src="https://s1.328888.xyz/2022/09/29/Mlalw.png" align="center" style="width: 100%" />
