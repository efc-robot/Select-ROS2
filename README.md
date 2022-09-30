# Select-ROS2
## Abstract
ROS (Robot Operating System) and its successor ROS2 are widely used middlewares, providing diverse libraries and a Data-Centric Publish/Subscriber (DCPS) communication framework, which separates computing and communication.
However, present computing-communication-separated middleware does not adapt to the dynamicity of heterogeneous computing systems and the diversity of artificial intelligence (AI) algorithms.
To fully leverage the heterogeneous computing platforms and AI algorithms for more autonomous robots, we improve ROS2 to Select-ROS2.
Select-ROS2 allows different subscribers in ROS2 to perform the same function, and let the same message only be sent to one of the subscribers.
Furthermore, Select-ROS2 enables automatic runtime algorithm and hardware selection instead of handcraft offline selection, to perform the computing workload balance on the heterogeneous computing system.
Select-ROS2 is open-sourced at https://github.com/efc-robot/Select-ROS2.

At present, the selection of algorithms and computing platforms depends on manual and offline.
However, computing resources are dynamic.
For example, CPU resources can be used in smart cars to assist the driver or provide cockpit services, such as playing media.
When the CPU plays media, other resources in the heterogeneous platforms must provide corresponding driving assistance functions. 
Thus, the middleware must perform the workload balance on the heterogeneous computing system.

<img src="https://s1.328888.xyz/2022/09/29/Mlalw.png" align="center" style="width: 50%" />


Fig.1a gives an example of online algorithm selection and hardware scheduling in robots.
The image sensor sends RGBD images at regular intervals, and the images are fed to Visual Odometer (VO) and Object Detection (OD) modules. 
Each module has different implementations, with different resource consumption and calculation accuracy. 
VO is based on either SIFT, ORB , or SuperPoint  feature point extraction algorithms.
OD is based on either YOLOv3 or YOLOtiny algorithms.

Fig.1b shows the candidate computation resources.
The heterogeneous platform contains GPUs and CPUs, whose resources are dynamic.
We hope to automatically select the algorithm and hardware for VO and OD modules, according to the hardware resources and the sensor data frequency.

Therefore, we propose a self-organizing computing scheduling framework based on ROS2, with the following contributions:
- We propose a method to label algorithms for the same functional module (each algorithm is a **node** in ROS2). The input data is sent to only one node for processing, reducing the communication traffic and computing requirements.
- We propose a method to dynamically select each functional module's processing algorithm and hardware, increasing the processing speed by 19\%-55\%.
- We integrate the methods into ROS2 middleware and open-source the integrated ROS2 at https://github.com/efc-robot/Select-ROS2. So that developers do not need to care about dynamic resource scheduling on heterogeneous platforms.

## EXTENDED PROTOCOLS
We extend the DDS protocols to label the subscribers which perform the same algorithm, and to provide more information for algorithm and hardware automatic selection.
Mainly, there are three types of data in our extended protocols. 1) The subscriber label with hardware resources information. 2) The frequency and latency requirement of the published topic. 3) The run-time computation capability of each hardware.

<img src="https://s1.328888.xyz/2022/09/29/MlZ2k.png" align="center" style="width: 50%" />

## ALGORITHM & HARDWARE SELECTION
The overall algorithm is shown in the figure below：
The publisher records the static subscriber information like computing resources cost and monitors the runtime computing capability as well as the computing requirement.

In this section, we detail the algorithm and hardware selection method based on the above information.

The algorithm and hardware selection method are processed at the publisher node.

Algorithm 1 shows that we could choose the algorithm and hardware according to the computing requirement and computing capability .

<img src="https://s1.328888.xyz/2022/09/29/MmC4U.png" align="center" style="width: 50%" />

Algorithm 2 shows the code to create a subscriber with call back function in the original ROS2 middleware.

At the creation stage, users need to assign the extended subscriber label (SubLabel) if they want the middleware automatically select the algorithms and computing hardware.

Users can leave the SubLabel default, and our middleware will send the message to the subscriber in the original manner according to the topic name and QoS.

After the call-back function is complete, the subscriber needs to inform the publisher that the algorithm is finished.

<img src="https://s1.328888.xyz/2022/09/29/Mmwnd.png" align="center" style="width: 50%" />

## Demo video
<iframe src="//player.bilibili.com/player.html?aid=96500031&bvid=BV1ve4y1r7fu&cid=165000000&page=1" scrolling="no"  frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>
