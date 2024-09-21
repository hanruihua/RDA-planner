from setuptools import setup

setup(
    name='RDA_planner',
    py_modules=['RDA_planner'],
    version= '2.3',
    install_requires=[
        'cvxpy==1.5.2',
        'numpy',
        'pathos',
<<<<<<< HEAD
        'ir-sim==2.2.3',
=======
        'ir_sim==2.1.4',
>>>>>>> change for irsim 2.2.4
        'matplotlib',
        'gctl==1.1',
        'opencv-python',
        'imageio',
        'scikit-learn',
        'scikit-image',
    ],
    description="The source code of the accelerated optimization based RDA motion planner",
    author="Han Ruihua"
)