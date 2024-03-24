from distutils.core import setup
from Cython.Build import cythonize

setup(
    ext_modules=cythonize([
        
        "cython1.pyx",  # Añade tus archivos .pyx aquí
        "cython2.pyx",
        "cython3.pyx",
        "cython4.pyx",
    ]),
)
