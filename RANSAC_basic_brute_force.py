import random
import numpy as np
import matplotlib.pyplot as plt

# Generate random points
x_gen1 = [random.randint(1, 20) for _ in range(20)]
y_gen1 = [random.randint(1, 20) for _ in range(20)]

# medium number of points
x_gen2 = [random.randint(15, 40) for _ in range(30)]
y_gen2 = [random.randint(15, 40) for _ in range(30)]

# more number of points 
x_gen3 = [random.randint(35, 70) for _ in range(40)]
y_gen3 = [random.randint(35, 70) for _ in range(40)]


inlier_id_array = []
outlier_id_array = []
best_inlier_id_array = []
best_outlier_id_array = []

# Combine all sections

x_gen = x_gen1 + x_gen2 + x_gen3
y_gen = y_gen1 + y_gen2 + y_gen3

tolerance = 3.5 
max_inliers = 0  
best_slope = 0
best_intercept = 0

for _ in range(100):  # 100 iterations 
 # Randomly select two points
  idx1, idx2= random.sample(range(90), 2)
  x1, y1 = x_gen[idx1], y_gen[idx1]
  x2, y2 = x_gen[idx2], y_gen[idx2]
                                               # this section is for lines 
  if x2 - x1 == 0:
    continue 
  slope = (y2 - y1) / (x2 - x1)
  intercept = y1 - slope * x1
    
  inlier_counter = 0
  for i in range(90):
    distance = abs(slope * x_gen[i] - y_gen[i] + intercept) / np.sqrt(slope**2 + 1)  # here its calculating distance
    if distance < tolerance:
     inlier_counter += 1
     inlier_id_array.append(i)
    else:
     outlier_id_array.append(i)

    
    # finding best model 
  if inlier_counter > max_inliers:
    max_inliers = inlier_counter
    best_slope = slope
    best_intercept = intercept
    # finding the best array of inliers and outliers
    best_inlier_id_array = inlier_id_array.copy()
    best_outlier_id_array = outlier_id_array.copy()

  inlier_id_array = []   # resetting these arrays after calculating corresponding to the line fitted 
  outlier_id_array = []
    

x_line = [max(x_gen), min(x_gen)]
y_line = [best_slope * x_line[0] + best_intercept, best_slope * x_line[1] + best_intercept]


for i in range(90):                                 #     -|     
  if i in best_inlier_id_array:                     #      |
    plt.scatter(x_gen[i], y_gen[i], color="green")  #      | Painting the outliers RED and inliers GREEN
  else:                                             #      |
    plt.scatter(x_gen[i], y_gen[i], color="red")    #     -|


plt.plot(x_line, y_line, color="navy", linewidth=2)  
plt.grid(True)
print("No. of safe points:", len(best_inlier_id_array))
plt.show()