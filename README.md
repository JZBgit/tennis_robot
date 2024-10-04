# 智能网球回收装置的设计与实现

基于嘉楠K210芯片，使用TensorFlow搭建YOLO v3目标检测神经网络模型，采集网球图像训练网络模型，在K210芯片上部署该模型实时检测网球位置，通过计算网球在图像中的位置和大小推断网球位置，通过PID算法控制电机，使得机器人回收网球。

yolov3主要参考了[这个博客](https://blog.csdn.net/weixin_44791964/article/details/119614577?fromshare=blogdetail&sharetype=blogdetail&sharerId=119614577&sharerefer=PC&sharesource=weixin_44333448&sharefrom=from_link)，在此感谢博主Bubbliiiing