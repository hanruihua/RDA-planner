from setuptools import setup
import sys

setup(
    name='RDA_planner',
    py_modules=['RDA_planner'],
    version= '1.2',
    install_requires=[
        'cvxpy',
        'numpy',
        'pathos',
        'ir_sim==1.1.9',
        'matplotlib'
    ],
    description="The source code of optimization based RDA motion planner",
    author="Han Ruihua"
)