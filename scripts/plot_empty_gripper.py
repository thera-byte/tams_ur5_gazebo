import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv("/home/aura/Schreibtisch/Bags/end_differences_empty_gripper.csv", header=None, sep=';')

print(data)
finger_1_joint_1 = data[0].tolist()
print(finger_1_joint_1)
finger_1_joint_2 = data[1].tolist()
print(finger_1_joint_2)
finger_1_joint_3 = data[2].tolist()
print(finger_1_joint_3)

finger_2_joint_1 = data[3].tolist()
print(finger_2_joint_1)
finger_2_joint_2 = data[4].tolist()
print(finger_2_joint_2)
finger_2_joint_3 = data[5].tolist()
print(finger_2_joint_3)

finger_middle_joint_1 = data[6].tolist()
print(finger_middle_joint_1)
finger_middle_joint_2 = data[7].tolist()
print(finger_middle_joint_2)
finger_middle_joint_3 = data[8].tolist()
print(finger_middle_joint_3)


# create a sample DataFrame
data = {'Finger Middle Joint 1': finger_middle_joint_1  ,'Finger Middle Joint 2': finger_middle_joint_2, 'Finger Middle Joint 3': finger_middle_joint_3}
df = pd.DataFrame(data)

# plot boxplot
df.boxplot()
for i, d in enumerate(df):
   y = df[d]
   x = np.random.normal(i + 1, 0.04, len(y))-0.3
   plt.scatter(x, y)

# show plot
plt.show()
