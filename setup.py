from setuptools import setup

setup(
    name='RDA_planner',
    py_modules=['RDA_planner'],
    version= '2.4',
    python_requires='>=3.8',
    install_requires=[
        'cvxpy==1.5.2',
        'numpy',
        'pathos',
        'ir_sim==2.2.4',
        'matplotlib',
        'gctl==1.2',
        'opencv-python',
        'imageio',
        'scikit-learn',
        'scikit-image',
    ],
    description="The source code of the accelerated optimization based RDA motion planner",
    author="Han Ruihua"
)