# SimpleStereoVO
A very simple stereo visual odometer implemented in OpenCV.<br>

## Some results: 
* For 'VO+IMU', I simply use the orientation provided by the IMU and the translation results from VO. <br>
* Video: http://v.youku.com/v_show/id_XMTgzMzczODI4MA==.html
![image](https://github.com/meyiao/SimpleStereoVO/blob/master/Results/kitti00.png)<br>
![image](https://github.com/meyiao/SimpleStereoVO/blob/master/Results/kitti01.png)<br>
![image](https://github.com/meyiao/SimpleStereoVO/blob/master/Results/kitti02.png)<br>
![image](https://github.com/meyiao/SimpleStereoVO/blob/master/Results/kitti03.png)<br>
![image](https://github.com/meyiao/SimpleStereoVO/blob/master/Results/kitti04.png)<br>
![image](https://github.com/meyiao/SimpleStereoVO/blob/master/Results/kitti05.png)<br>


## Usage
Specifying your dataset parameters (marked as !!!CHANGE!!!) in main.cpp<br>
Note that the stereo images must be rectified.<br>

## Dataset
It runs properly on the KITTI benchmark dataset : http://www.cvlibs.net/datasets/kitti/eval_odometry.php <br>


