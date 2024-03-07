import pandas as pd 

df = pd.read_csv('/home/rouf20/Documents/2022RoboCup/2022-07-12_FieldA_B-Human_Bembelbots_7vs7_DerPinguin.csv')

req_data = []
sens_data = []
print(df.shape[0])
for i in range(int(df.shape[0])):
    requests = df.iloc[i][1:12].tolist()
    requests.pop(5)
    req_data.append(requests)
    sensors = df.iloc[i][12:].tolist()
    sensors.pop(5)
    sens_data.append(sensors)



feature_name = [col.split(".")[2].capitalize() for col in df.columns.tolist()[1:12]]
print(feature_name)

print(req_data)
print(sens_data)
  

# final_features = ['LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']

# source_features = ["HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch",
#                    "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch",
#                    "RAnklePitch", "RAnkleRoll", "LShoulderPitch", "LShoulderRoll", "LElbowYaw",
#                    "LElbowRoll", "LWristYaw", "LHand", "RShoulderPitch", "RShoulderRoll", "RElbowYaw",
#                    "RElbowRoll", "RWristYaw", "RHand", "RFinger23", "RFinger13", "RFinger12", "LFinger21",
#                    "LFinger13", "LFinger11", "RFinger22", "LFinger22", "RFinger21", "LFinger12", "RFinger11",
#                    "LFinger23", "LThumb1", "RThumb1", "RThumb2", "LThumb2"]

# source_values = [0.0, -9.093359999989836e-05, 0.004986279199999943, -3.8051500000024774e-05, -0.00018283900000004571,
#                          -0.00016400378000000493, -0.00016097489999999937, -1.490230000000814e-05, 0.004986279199999943,
#                          -7.893800000002837e-05, -0.00018283900000004571, -0.00016400378000000493, -4.8639999999711137e-05,
#                          -0.0001017730000000272, 0.0, -0.00014643740000003236, 0.0, -0.7897633000000001, 0.0, 0.0, 0.0,
#                          -1.7623500000008008e-05, 0.0, 0.7897633000000001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# final_values = []

# for feature in final_features:
#     index = source_features.index(feature)
#     final_values.append(source_values[index])

# print("Final Values:", final_values)