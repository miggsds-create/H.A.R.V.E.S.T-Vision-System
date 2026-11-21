# H.A.R.V.E.S.T-Vision-System
Strawberry detection, ripeness classification and 3D co-ordinate and yaw labeling 
The strawberry part in H.A.R.V.E.S.T. handles finding the berries and checking ripeness along with camera calibration and getting 3D positions plus yaw. That way the system can pick them without damage. I broke the code into three modules called Camera_Calibration, ESP32-N16R8_Firmware, and Strawberry_detection_model. It links up the ESP32-S3 using BLE and ESP-NOW with the YOLOv8 model that reached 83.1 percent mAP50 and the camera numbers to turn detections into 3D space. 

Building all that gave a real sense of how vision and firmware and hardware limits mix together in a robotics project. Some parts felt straightforward but others got messy when trying to keep everything stable. I think the accuracy works okay but it might change under different field conditions.
