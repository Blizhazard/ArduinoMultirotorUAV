import pandas as pd
import matplotlib.pyplot as plt
df = pd.read_csv('FlightData6D.tsv',sep='\t',skiprows=11)
gimbal_pos = df[['Gimbal1 X','Y','Z']].to_numpy()
gimbal_rpy = df[['Roll','Pitch','Yaw']].to_numpy()
drone_pos = df[['M2 X','Y.2','Z.2']].to_numpy()
drone_rpy = df[['Roll.2','Pitch.2','Yaw.2']].to_numpy()




labels = ['roll', 'pitch', 'yaw']
for i in range(3):
    plt.subplot(1,3,i+1)
    plt.plot(drone_rpy[:,i], label=f'drone-{labels[i]}')
    plt.legend()

plt.show()