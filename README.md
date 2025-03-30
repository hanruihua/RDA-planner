<!-- | **[`PDF_IEEE`](https://ieeexplore.ieee.org/document/10036019)** | **[`PDF_Arxiv`](https://arxiv.org/pdf/2210.00192.pdf)** | **[`Video_Youtube`](https://www.youtube.com/watch?v=qUNMQQRhNFo)** | **[`Video_Bilibili`](https://www.bilibili.com/video/BV1zT411d7aL/?vd_source=cf6ba629063343717a192a5be9fe8985)** |  -->
<div align="center">

 # RDA Planner

<a href="https://ieeexplore.ieee.org/document/10036019"><img src='https://img.shields.io/badge/PDF-IEEE-brightgreen' alt='PDF'></a>
<a href="https://arxiv.org/pdf/2210.00192.pdf"><img src='https://img.shields.io/badge/ArXiv-2210.00192-brightgreen' alt='ArXiv'></a>
<a href="https://github.com/hanruihua/RDA_planner/releases"><img src='https://img.shields.io/github/v/release/hanruihua/RDA_planner?color=orange' alt='version'></a>
<a href="https://www.youtube.com/watch?v=qUNMQQRhNFo"><img src='https://img.shields.io/badge/Video-Youtube-blue' alt='youtube'></a>
<a href="https://www.bilibili.com/video/BV1zT411d7aL/?vd_source=cf6ba629063343717a192a5be9fe8985"><img src='https://img.shields.io/badge/Video-Bilibili-blue' alt='bilibili'></a>
<a href="#citation"><img src='https://img.shields.io/badge/BibTex-RDA_planner-lightgreen' alt='Paper BibTex'></a>
<a href="https://github.com/hanruihua/RDA_planner/blob/main/LICENSE"><img src='https://img.shields.io/badge/License-MIT-yellow' alt='Paper BibTex'></a>
<a href="https://github.com/hanruihua/rda_ros"><img src='https://img.shields.io/badge/Wrapper-ROS-red' alt='ROS'></a>


</div>

RDA Planner is a high-performance, optimization-based, Model Predictive Control (MPC) motion planner designed for autonomous navigation in complex and cluttered environments. Utilizing the Alternating Direction Method of Multipliers (ADMM), RDA decomposes complex optimization problems into several simple subproblems. This decomposition enables parallel computation of collision avoidance constraints for each obstacle, significantly enhancing computation speed.

**Key Features:**

- **Shape-Aware Planning:** Handles robots and obstacles with arbitrary convex shapes, ensuring versatility across diverse scenarios.
- **High-Precision Control:** Achieves accurate control trajectories through advanced optimization solvers, enhancing navigation reliability.
- **Dynamic Obstacle Handling:** Supports both static and dynamic obstacles, enabling robust performance in ever-changing environments.
- **Real-Time Performance:** Offers fast computation times suitable for real-time applications, ensuring timely decision-making and responsiveness.
- **Versatile Kinematic Support:** Compatible with various types of robot kinematics, including differential drive, Ackermann steering, and omnidirectional systems, providing flexibility for different robotic platforms.

## News
**10 December 2024:** We have uploaded the code implementation of the linearized OBCA algorithm in the [dev_lobca] branch to facilitate the comparison with the RDA planner. Run [path_track_lobca.py](https://github.com/hanruihua/RDA-planner/blob/dev_lobca/example/path_track/path_track_lobca.py) to see the performance. (Note: The linearized OBCA algorithm is not integrated with the DPP framework, resulting in additional problem formulation time for each iteration. Concentrating on problem-solving time is more appropriate for comparison purposes.)

**25 September 2024:** The ROS wrapper of RDA planner is available at [rda_ros](https://github.com/hanruihua/rda_ros)

## Prerequisite
- Python >= 3.9

## Installation 

```
git clone https://github.com/hanruihua/RDA_planner
cd RDA_planner
pip install -e .  
```

## Run examples

### Demonstrations on [ir-sim](https://github.com/hanruihua/ir-sim) 

Please install ir-sim by:

```
pip install ir-sim
```

**Path Track (example/path_track.py)**                 |  <img src="https://github.com/user-attachments/assets/6a1304e0-85cd-4bb8-a281-86c1cefe3adc" width="400" /> 
|:-------------------------:|:-------------------------:|
**Cross Corridor (example/corridor.py)** | <img src="https://github.com/user-attachments/assets/6965bef3-f79d-4732-8103-6b92b92ce37a" width="500" />
**Reverse Parking (example/reverse.py)** | <img src="https://github.com/user-attachments/assets/dcfe8603-3a4d-433f-a66a-5eee86ccc3ec" width="400" />
**Input Lidar points (example/lidar_path_track.py)** | <img src="https://github.com/user-attachments/assets/22d69e89-f40f-42cc-ae34-8fc68ecd8111" width="400" />
**Dynamic obstacles avoidance (example/dynamic_obs.py)**| <img src="https://github.com/user-attachments/assets/a899d9ef-c36a-43e2-bfc9-303243eb589a" width="400" />

**Note:** You can customize the scenario by modifying the parameters in the corresponding yaml file as introduced in [ir-sim](https://github.com/hanruihua/ir-sim). For the polygon obstacles, please make sure the obstacles are convex (CCW order is not necessary now).


### Demonstrations of dynamic collision avoidance and autonomous driving

Please see [rda_ros](https://github.com/hanruihua/rda_ros) for detail

https://github.com/user-attachments/assets/94c40338-9e77-43c4-ad01-59b303f188c1

## Citation

If you find this code or paper is helpful, you can **star** this repository and cite our paper by the following **BibTeX** entry:

```
  @ARTICLE{10036019,
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Zhang, Qianru and Eldar, Yonina C. and Hao, Qi and Pan, Jia},
  journal={IEEE Robotics and Automation Letters}, 
  title={RDA: An Accelerated Collision Free Motion Planner for Autonomous Navigation in Cluttered Environments}, 
  year={2023},
  volume={8},
  number={3},
  pages={1715-1722},
  doi={10.1109/LRA.2023.3242138}}

```
