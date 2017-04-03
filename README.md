## Emotion classification and Baxter drawing
#### Winter project for MSR
#### Yuchen Rao


## Introduction

This project has two parts. The first part is about multi-emotion classification(happy, sad, surprise and disgust) based on Opencv and machine learning. The second part is about Baxter drawing control based on joint trajectory action server, Baxter can draw a specific face based on the detected emotion.

Video 'winter_project.mp4' is the result of this project. Also you can see this [video](https://youtu.be/6yXKkfkGYj4) on YouTube.

## Part 1: Emotion classification based on Opencv and machine learning

#### 1. Dataset

[The Cohn-Kanade AU-Coded Facial Expression Database](http://www.pitt.edu/~emotion/ck-spread.htm) is for research in automatic facial image analysis and synthesis and for perceptual studies.

I choose and relabeled data for four emotions: happy (302), sad (238), surprise (256) and disgust (218).

#### 2. Data pre-processing

1). Use Opencv Haar cascade to detect human face, and trainsform it to gray;

2). Extract a new face based on offset coefficients, and reshape it to a 64*64 image;

3). Extraxt face features based on dense SIFT.

#### 3. Machine learning

Use [SVM](http://scikit-learn.org/stable/modules/svm.html#svm) machine learning algorithm for training and prediction. The kernel is "linear", and multiclass classification mathod is One-Vs.-One.

In this project, I choose training data: testing data = 7:3.

#### 4. Result

Use 5 folds corss-validation to calculate mean score. And then calculate accuracy for training and testing set(98%), and shows the classification report and confusion matrix.

![Image of 4_0.01](https://github.com/yuchenrao/emotion-detection-and-baxter-drawing/blob/master/picture/classify_result.png)
<!-- 4_0.01 -->

#### 5. Live detection based on webcam

Use Opencv to capture each frame of the video and do the same pre-processing for the frame. It can show the emotion it detects.

During live detection, 'Disgust' is hard to detect for some people, and other emotions are easy to be classified for everyone.

![Image of 4_1.01](https://github.com/yuchenrao/emotion-detection-and-baxter-drawing/blob/master/picture/live_det2.png)
<!-- 4_1.01 -->

![Image of 4_1.02](https://github.com/yuchenrao/emotion-detection-and-baxter-drawing/blob/master/picture/live_det1.png)
<!-- 4_1.02 -->

## Part 2: Baxter drawing control based on joint trajectory action server

#### 1. Introduction

I design four simple faces to represent happy, sad, surprise and disgust. Then I use joint trajectory action server for the Baxter drawing control.
When Baxter get the drawing command, it will use inverse kinematics to calculate a list of reference angles for each joint for baxter arm moving, which based on the trajectory and time setting. There is a code walkthrough for [joint trajectory client](http://sdk.rethinkrobotics.com/wiki/Joint_Trajectory_Client_-_Code_Walkthrough).

'Control.py' is based on move_to_joint_positions and 'trajectory.py' is based on joint trajectory action server.

#### 2. Baxter hand

In this project, Baxter uses a new designed hand to capture the pen. The new hand has springs, which makes the pen more flexible on the vertical direction.

![Image of 4_0.07](https://github.com/yuchenrao/emotion-detection-and-baxter-drawing/blob/master/picture/Baxter_hand.png)
<!-- 4_0.07 -->

#### 3. Result

Here is the drawing results for Baxter drawing.

![Image of 4_0.03](https://github.com/yuchenrao/emotion-detection-and-baxter-drawing/blob/master/picture/draw_res1.png)
<!-- 4_0.03 -->

![Image of 4_0.04](https://github.com/yuchenrao/emotion-detection-and-baxter-drawing/blob/master/picture/draw_res2.png)
<!-- 4_0.04 -->

## Part 3: Future work

#### 1. Emotion detection

1). Find new method to improve the live detection for 'Disgust' emotion;

2). Use more data(people with glasses, Asian people, different directions and light situations);

3). Detect more emotions like fear and anger.

#### 2. Baxter control

Use velocity controller for Baxter movement.

## Part 4: Reference

[Emotion detection reference](http://flothesof.github.io/smile-recognition.html)

[Baxter drawing control reference](https://github.com/MingheJiang/baxter_drawing)

