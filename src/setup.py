from setuptools import Extension, setup
from Cython.Build import cythonize


ext_modules = [
    Extension(
        "measurePrediction",
        ["measurePrediction.pyx"],
        extra_compile_args=['-O3', '-fopenmp'],
        extra_link_args=['-fopenmp'],
    )
]

setup(
    ext_modules = cythonize(ext_modules),
)
