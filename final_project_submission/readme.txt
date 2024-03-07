== JointAngles dataset ==

This folder contains the dataset accompanying the paper "Neural Network-based Joint Angle Prediction for the NAO Robot" by Fiedler and Laue [1].

The dataset consists of 4 ZIP archives of CSV files.
Each ZIP archive contains the requested and measured joint angles of the leg joints of all B-Human robots during walking, recorded on multiple different fields during four full RoboCup tournaments in 2019 and 2022.
The data were extracted and processed from the logs recorded by the B-Human system.

Each CSV file contains the data of one robot from one game.
The structure of the filename is the following: <date-of-the-game>_[<field>_]<team1>_<team2>_<robot>.csv
The first column is the index, which will be explained in more detail below.
This is followed by the requested and then the measured joint angles, in the order in which they are used in the B-Human system.
The hipYawPitch is given only once, because the robot provides it only once.
The angles are given in radian.

The index divides the data into segments of continuous walking with a fixed interval of 12ms between 2 time steps.
Within a segment, the index increases by 1 with each time step. If the index increases more than 1 between two time steps, a new segment begins.

An example loading of the data as `tf.Dataset` is provided in `load_as_tf_dataset.py`.

In total, 12112153 time steps are included, which corresponds to 2432.16 minutes.


[1] Fiedler, J., Laue, T.: Neural network-based joint angle prediction for the NAO robot. In: RoboCup 2023: Robot World Cup XXVI. To appear.
