
# Computer Vision Challenge

It is the repository of the final challenge project for Computer Vision (SS23) at Technical University Munich.

## Introductoin
Lecturer: Prof. Dr.-Ing. Klaus Diepold

Assistant: Luca Sacchetto, Sven Gronauer

Team Member:
- Feng Xu
- Haoqing Huang
- Lei Zhang
- Long Zhao
- Yuyang Zhang

This project develops an application that allows customers to check the interior of a 3D scene of their own apartment. The customer is able to easily upload their own captured images of the room into the application and receive a 3D model of the interior space.
## GUI User's Guide
> :computer: : work for **Windows**
>
> :rocket: : MATLAB Version: at least **MATLAB R2023a** and Toolbox **Computer Vision Toolbox** must be installed
> 
> :keyboard: : GUI develop kit: **APP designer**


### Step1
First, open the `CV_challenge.mlapp` in the current folder.
![step1](https://github.com/Huang15/Team-project/assets/128314731/694cb85c-9cf2-4ab5-8dfd-fae9ee6e1d25)

### Step2
Next, click the `Run` button. Alternatively, you can click `Code View` to see the code of GUI.
![step2 (2)](https://github.com/Huang15/Team-project/assets/128314731/c73d9f23-ffbe-4bcb-9625-1b602e9821ec)

The GUI looks like this.
![gui (2)](https://github.com/Huang15/Team-project/assets/128314731/cd102ecc-eabd-4911-92d9-3c958dbdc66a)


### Step3
Select the folder of the images you want to upload. Also select the folder containing the camera paramters. Camera parameters will be displayed automatically. Then click `Confirm`. The first two lights should be green and then you can click `Run` to start the computation.
![屏幕截图(192)](https://github.com/Huang15/Team-project/assets/128314731/cd79f590-a2a5-48df-8437-722b707cbefa)

After running, the point cloud and 3D model will be displayed. Figure 1 is the point cloud without clustering and figure 2 with clustering. 
![屏幕截图(8)](https://github.com/Huang15/Team-project/assets/128314731/59807b09-4ac8-40ca-997f-bfffb1639be3)

![屏幕截图(9)](https://github.com/Huang15/Team-project/assets/128314731/a8d6fd88-7024-4067-8d43-55e4f8ab7f30)


3D model is shown in the center of the interface. You can drag to view it from different positions. On the right side of the 3D model, volume of each object and the distance between their center and original point are shown. If you want to try a new image, please use the `Reset` button.
![屏幕截图(11)](https://github.com/Huang15/Team-project/assets/128314731/6f6121e8-cd96-4587-9a00-b23402339ea2)


