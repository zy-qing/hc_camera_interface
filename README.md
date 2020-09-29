# 海康摄像头驱动
## 1. 海康SDK安装

海康SDK采用了动态链接库，动态链接库在HCNetSDKCom文件夹，因此需要将该文件夹加入系统动态链接库配置中。
```
sudo vi /etc/ld.so.conf
```
打开文件，在文件末尾加入了/XXX/HCNetSDKCom/（项目附带了SDK在/hc_sdk/lib/HCNetSDKCom路径下）

然后执行
```
sudo ldconfig 
```