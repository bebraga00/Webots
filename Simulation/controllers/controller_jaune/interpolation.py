import matplotlib.pyplot as plt
import numpy as np
import time

def interpolation(L):
    
    n = len(L)
    ref = 0
    k = 0
    
    while L[k] == 0:
        
        k += 1
        
    ref = L[k]
    
    for i in range(0,k):
        
        L[i] = ref
        
    for i in range(k+1,n):
        
        if L[i] == 0:
            
            k = i
            
            while L[k] == 0:
                
                k -= 1
                
            L[i] = L[k]       
            
    return(L)       