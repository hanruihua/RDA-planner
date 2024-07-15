from setuptools import setup
import sys

setup(
    name='RDA_planner',
    py_modules=['RDA_planner'],
    version= '1.3',
    install_requires=[
        'cvxpy==1.5.2',
        'numpy',
        'pathos',
        'ir_sim==1.1.11',
        'matplotlib',
        'gctl==1.1',
        'opencv-python',
        'imageio',
        'scikit-learn',
        'scikit-image',
    ],
    description="The source code of optimization based RDA motion planner",
    author="Han Ruihua"
)