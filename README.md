# Unscented-Kalman-Filter

This is a prototype of Unscented Kalman Filter software. 

UKF library has been created. Eigen3 has been used to ease matrix/vector calculations. 5 Headers are created. Their functions are described below

| Header |  Function |
|--------|-----------|
|UserInput.h	|               Contains all commonly used constants and constant expressions, will be included in all cpp files. |
|UnscentedKF.h	|            Construct a “parent” struct UnscentedKalmanFilter, which declares all standard UKF variables and some getter                              method. |
|UKFStatePredictor.h	|       Construct a struct UKF state predictor, which contains all methods used in system time update. |
|UKFMeasurementPredictor.h	| Construct a struct UKF measurement predictor, which contains all methods used in measurement update. |
|UKFStateUpdator.h	|         Construct a struct UKF state updater, which performs filtering data process.|

Then 4 corresponding cpp files are created to implement each struct. I have not made the whole process recursive yet. It is easier to make it happen when we have a simulator such as Matlab, Gazebo, Stage. Or we have collected sensor data at each deltaT and record it in a file. Assuming we have a simulator, we can push back all vectors/matrix into a std::vector. If we have sensor data in a file, we can read it from the file and process it.
