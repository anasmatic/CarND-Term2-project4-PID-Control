# CarND : Controls PID Project
Self-Driving Car Engineer Nanodegree Program

In this project, I decide the stearing angle of simulator's car stearing wheel , using PID controller and Twiddle (coordinate ascent)

---
#### default project installation :
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid 

#### my own Build Instructions
I was using Windows 10 and VisualStudio17

-to build this project using **Bash for window** :

    navigate to projet
    write cmd : mkdir build
    then navigate to build
    write cmd : cmake .. -G "Unix Makefiles" && make
    write cmd : ./pid 

#### PID initial code, manual tuning, & Twiddle

1. in class PID we fill the initial parameters Kp, Ki, Kd.
   * in every step we update p & i & d Errors.
   * and then calculate Total Error, it is the sum of paramters and their errors multiplied.
2. now we tune the Kp, Ki, Kd parameters using [these instructions](https://udacity-reviews-uploads.s3.amazonaws.com/_attachments/41330/1493863065/pid_control_document.pdf#%5B%7B%22num%22%3A37%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C56.7%2C785.3%2C0%5D)
3. now we build the twiddle algorithm 
   * I'll use the same implementation in lesson
   * using boolean flags, I'll decide which part will be active during the current loop
   * using p array as the [kp, ki, kd] , and pd array as a helper to tune p array
   * initialize kp ki kd, to the manually tuned parameters [0.12,0,3]
   * initialize pd array to [1,0,15] , I decided these values after trial and error.
   * I choose to twiddle every 5 steps, to auto-tune on fly without restarting simulator.

#### Results and comments

1. while twiddle is running, the car will swing, but eventually it will stop after twiddle is done.
   ![twiddle00](https://github.com/anasmatic/CarND-Term2-project4-PID-Control/tree/master/res/t00.gif)
    [alternative vid](https://github.com/anasmatic/CarND-Term2-project4-PID-Control/tree/master/res/t00.mp4)
2. but after twiddle is done car can take curves successfuly :
   ![twiddle01](https://github.com/anasmatic/CarND-Term2-project4-PID-Control/tree/master/res/t01.gif)
    [alternative vid](https://github.com/anasmatic/CarND-Term2-project4-PID-Control/tree/master/res/t01.mp4)
   * these images are taken from diffrent testing sessions.
   ![twiddle02](https://github.com/anasmatic/CarND-Term2-project4-PID-Control/tree/master/res/t02.gif)
    [alternative vid](https://github.com/anasmatic/CarND-Term2-project4-PID-Control/tree/master/res/t02.mp4)


3. I got only few comment , that if I can improve the PID controller will be perfect
   * start with p=[0,0,0] & pd=[1,1,1]
   * drive succesfuly with throttle more than 0.4
   * use more steps between twiddle steps -now using 5 steps-

