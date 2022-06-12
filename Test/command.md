# Android连接

usb连接选择传输文件

> adb devices
>
> //此处应该是一个，然后让android手机和电脑处于一个wifi下即可
>
> adb tcpip 5555
>
> //此处连接ip地址
>
> adb connect 192.168.86.196
>
> adb devices
>
> //此时应该是两个
>
> adb disconnect 192.168.86.196
>
> //断开连接后，将usb线拔离电脑，然后将手机端设置可以进行usb调试，usb配置选择MTP（多媒体传输），然后再次连接
>
> adb connect 192.168.86.196
>
> //此时就可以通过wifi相连了