import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-Object/end_differences_empty_gripper.csv", header=None, sep=',')

#data = pd.read_csv("/home/aura/Schreibtisch/Bags/All-Sensors/Wooden-Block/end_differences_wooden_block.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/All-Sensors/Beer/end_differences_beer.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/All-Sensors/Coke/end_differences_coke.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/All-Sensors/Plastic/end_differences_plastic.csv", header=None, sep=',')

#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-3rd-Sensor/FC-no-3rd-sensor-beer/end_differences_no-3rd-sensor-beer.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-3rd-Sensor/FC-no-3rd-plastic/end_differences_no_3rd_sensor_plastic.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-3rd-Sensor/FC-no-3rd-sensor-coke/end_differences_no_3rd_sensor_coke.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-3rd-Sensor/FC-no-3rd-sensor-wooden-block/end_differences_no_3rd_sensor_wooden_block.csv", header=None, sep=',')

#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-2nd-Sensor/No-2nd-Sensor-Wooden-Block/end_differences_no_2nd_sensor_wooden_block.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-2nd-Sensor/No-2nd-Sensor-Beer/end_differences_no_2nd_sensor_beer.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-2nd-Sensor/No-2nd-Sensor-Coke/end_differences_no_2nd_sensor_coke.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-2nd-Sensor/No-2nd-Sensor-Plastic/end_differences_no_2nd_sensor_plastic.csv", header=None, sep=',')

#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-1st-Sensor/No-1st-Sensor-Wooden-Block/end_differences_no_1st_sensor_wooden_block.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-1st-Sensor/No-1st-Sensor-Beer/end_differences_no_1st_sensor_beer.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-1st-Sensor/No-1st-Sensor-Coke/end_differences_no_1st_sensor_coke.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-1st-Sensor/No-1st-Sensor-Plastic/end_differences_no_1st_sensor_plastic.csv", header=None, sep=',')

#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-3rd-Sensor/end_differences_no_3rd_sensor_ALL.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-2nd-Sensor/end_differences_no_2nd_sensor_ALL.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-1st-Sensor/end_differences_no_1st_sensor_ALL.csv", header=None, sep=',')

#data = pd.read_csv("/home/aura/Schreibtisch/Bags/All-Sensors/end_differences_ALL.csv", header=None, sep=',')

#data = pd.read_csv("/home/aura/Schreibtisch/Bags/Tube/Tube_Only_3/end_differences_only_3_tube_3.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/Tube/Tube_None/end_differences_none_tube.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/Tube/Tube_All/end_differences_all_sensors_tube.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/Tube/Empty/end_differences_empty_tube.csv", header=None, sep=',')

#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-Sensors/Block/end_differences_no_sensors_wooden_block.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-Sensors/Beer/end_differences_no_sensors_beer.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-Sensors/Coke/end_differences_no_sensors_coke.csv", header=None, sep=',')
data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-Sensors/Plastik/end_differences_no_sensors_plastic.csv", header=None, sep=',')
#data = pd.read_csv("/home/aura/Schreibtisch/Bags/No-Sensors/end_differences_no_sensors_ALL.csv", header=None, sep=',')


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




PLOT_MIN_Y = -1.2
PLOT_MAX_Y = 1.2

plt.figure().set_figwidth(15)
#plt.title("All Fingers at g=254 - Sensor Group 1 Deactivated - Plastic Cup")
#plt.title("All Fingers at g=254 - Sensor Group 2 Deactivated - All Objects")
#plt.title("All Fingers at g=254 - Sensor Group 3 Deactivated - All Objects")
#plt.title("All Fingers at g=254 - All Sensors Activated - All Objects")
#plt.title("All Fingers at g=254 - All Sensors Deactivated - All Objects")
#plt.title("All Fingers at g=254 - All Sensors Activated - Empty Pinch Grasp")
#plt.title("All Fingers at g=254 - All Sensors Activated - Pinch Grasp - Tube")
#plt.title("All Fingers at g=254 - All Sensors Deactivated - Pinch Grasp - Tube")
plt.title("All Fingers at g=254 - All Sensors Deactivated - Basic Grasp - Wooden Cube")
#plt.title("All Fingers at g=254 - Group 3 Sensors Activated - Pinch Grasp - Tube")
plt.ylabel("Difference in Joint Position")

# create a sample DataFrame
data = {'FM J1': finger_middle_joint_1  ,'FM J2': finger_middle_joint_2, 'FM J3': finger_middle_joint_3, 'F1 J1': finger_1_joint_1  ,'F1 J2': finger_1_joint_2, 'F1 J3': finger_1_joint_3, 'F2 J1': finger_2_joint_1  ,'F2 J2': finger_2_joint_2, 'F2 J3': finger_2_joint_3}
df = pd.DataFrame(data)


# plot boxplot
plot = df.boxplot()
plot.set_ylim(PLOT_MIN_Y, PLOT_MAX_Y)


for i, d in enumerate(df):
   y = df[d]
   x = np.random.normal(i + 1, 0.04, len(y))
   plt.scatter(x, y)
   
"""
plt.title("Finger Middle at g=254")
plt.ylabel("Difference in Joint Position")
# create a sample DataFrame
data = {'Finger Middle Joint 1': finger_middle_joint_1  ,'Finger Middle Joint 2': finger_middle_joint_2, 'Finger Middle Joint 3': finger_middle_joint_3}
df = pd.DataFrame(data)

# plot boxplot
plot = df.boxplot()
plot.set_ylim(PLOT_MIN_Y, PLOT_MAX_Y)
for i, d in enumerate(df):
   y = df[d]
   x = np.random.normal(i + 1, 0.04, len(y))-0.3
   plt.scatter(x, y)

# next plot starts here
plt.figure()
plt.title("Finger 1 at g=254")
plt.ylabel("Difference in Joint Position")

# create a sample DataFrame
data = {'Finger 1 Joint 1': finger_1_joint_1  ,'Finger 1 Joint 2': finger_1_joint_2, 'Finger 1 Joint 3': finger_1_joint_3}
df = pd.DataFrame(data)

# plot boxplot
plot1 = df.boxplot()
plot1.set_ylim(PLOT_MIN_Y, PLOT_MAX_Y)
for i, d in enumerate(df):
   y = df[d]
   x = np.random.normal(i + 1, 0.04, len(y))-0.3
   plt.scatter(x, y)

# next plot starts here
plt.figure()
plt.title("Finger 2 at g=254")
plt.ylabel("Difference in Joint Position")
data = {'Joint 1': finger_2_joint_1  ,'Joint 2': finger_2_joint_2, 'Joint 3': finger_2_joint_3}
df = pd.DataFrame(data)

# plot boxplot
plot2 = df.boxplot()
plot2.set_ylim(PLOT_MIN_Y, PLOT_MAX_Y)
for i, d in enumerate(df):
   y = df[d]
   x = np.random.normal(i + 1, 0.04, len(y))-0.3
   plt.scatter(x, y)   

   """



# show plot
plt.show()
