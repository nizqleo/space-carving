import numpy as np 
from sklearn.preprocessing import normalize

a = np.arange(0,4)
a = a.reshape([4,1])
b = normalize(a)
print(a,b)