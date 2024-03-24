import numpy as np
import numexpr as ne
import timeit


a = np.random.rand(10000)
b = np.random.rand(10000)
c = np.random.rand(10000)
d = ne.evaluate('a + b * c')
import numpy as np

def distance_matrix_numpy(r):
    r_i = r[:, np.newaxis]
    r_j = r[np.newaxis, :]
    d_ij = r_j - r_i
    d_ij = np.sqrt((d_ij ** 2).sum(axis=2))
    return d_ij

# Generamos las posiciones aleatorias de las partículas
r = np.random.rand(10000, 2)

# Calculamos la matriz de distancias usando NumPy
d_numpy = distance_matrix_numpy(r)



def distance_matrix_numexpr(r):
    r_i = r[:, np.newaxis]
    r_j = r[np.newaxis, :]
    
    # Numexpr no soporta slicing directamente, por lo tanto, operamos sobre los arrays completos
    d_ij2 = ne.evaluate('sum((r_j - r_i)**2, axis=2)')
    d_ij = ne.evaluate('sqrt(d_ij2)')
    
    return d_ij

# Calculamos la matriz de distancias usando numexpr
d_numexpr = distance_matrix_numexpr(r)


def benchmark():
    r = np.random.rand(10000, 2)
    result = timeit.timeit('distance_matrix_numpy(r)',
                           setup = 'from __main__ import distance_matrix_numpy, r', 
                           number = 1000)
    print("NumPy: {}".format(result/1000))
    
    result = timeit.timeit('distance_matrix_numexpr(r)',
                           setup = 'from __main__ import distance_matrix_numexpr, r', 
                           number = 1000)
    print("Numexpr: {}".format(result/1000))
    
if __name__ == "__main__":
    
    benchmark()