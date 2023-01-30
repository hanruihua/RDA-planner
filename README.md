 # RDA_planner

The source code of the paper "RDA: An Accelerated Collision Free Motion Planner for Autonomous Navigation in Cluttered Environments" [**RA-Letter**] 

## Prerequisite
- Python >= 3.8
- numpy
- cvxpy
- [ir_sim](https://github.com/hanruihua/ir_sim): A python based 2d robot simulator for robotics navigation algorithm. 
- [GenerateCurveTool](https://github.com/hanruihua/GenerateCurveTool): A tool of generating the common curves from way points for the robot path planning, including dubins path, reeds shepp, etc.

## Installation 

```
git clone https://github.com/hanruihua/RDA_planner
cd RDA_planner
pip install -e .  
```

## Run examples

- **Path Track (example/path_track.py)**

<div align=center>
<img src="example\path_track\animation\path_track.gif" width="300" />
</div>

- **Cross Corridor (example/corridor.py)**

<div align=center>
<img src="example\corridor\animation\corridor.gif" width="500" />
</div>

- **Reverse Parking (example/reverse.py)**

<div align=center>
<img src="example\reverse\animation\reverse_park.gif" width="300" />
</div>


## Citation