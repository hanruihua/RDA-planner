from setuptools import setup
import sys

setup(
    name='RDA_planner',
    py_modules=['RDA_planner'],
    version= '1.0',
    install_requires=[
        'cvxpy',
        'numpy',
        'pathos',
    ],
    description="The source code of optimization based RDA motion planner",
    author="Han Ruihua"
)