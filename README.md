 # RDA_planner

The source code of the paper "RDA: An Accelerated Collision Free Motion Planner for Autonomous Navigation in Cluttered Environments" [**RA-Letter**] 

| **[`PDF_IEEE`](https://ieeexplore.ieee.org/document/10036019)** | **[`PDF_Arxiv`](https://arxiv.org/pdf/2210.00192.pdf)** | **[`Video_Youtube`](https://www.youtube.com/watch?v=qUNMQQRhNFo)** | **[`Video_Bilibili`](https://www.bilibili.com/video/BV1zT411d7aL/?vd_source=cf6ba629063343717a192a5be9fe8985)** | 

## Prerequisite
- Python >= 3.8
- numpy
- cvxpy
- [ir_sim==1.1.9](https://github.com/hanruihua/ir_sim): A python based 2d robot simulator for robotics navigation algorithm. 
- [GenerateCurveTool](https://github.com/hanruihua/GenerateCurveTool): A tool of generating the common curves from way points for the robot path planning, including dubins path, reeds shepp, etc.

## Installation 

```
git clone https://github.com/hanruihua/RDA_planner
cd RDA_planner
pip install -e .  
```

## Run examples

**Path Track (example/path_track.py)**                 |  <img src="example\path_track\animation\path_track.gif" width="400" />  
|:-------------------------:|:-------------------------:|
**Cross Corridor (example/corridor.py)** | <img src="example\corridor\animation\corridor.gif" width="500" />|
**Reverse Parking (example/reverse.py)** | <img src="example\reverse\animation\reverse_park.gif" width="400" />


**Dynamic obstacles avoidance (example/dynamic_obs.py)** | <img src="example\dynamic_obs\animation\dynamic_obs1.gif" width="300" /> |  <img src="example\dynamic_obs\animation\dynamic_obs2.gif" width="300" />
|:-------------------------:|:-------------------------:|:-------------------------:|

## Contact

Han Ruihua (hanrh@connect.hku.hk)


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
