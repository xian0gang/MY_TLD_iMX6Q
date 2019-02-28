#ifndef _VPU_CAMERA_H
#define _VPU_CAMERA_H

//resolution : 输入图像分辨率的宽度512、576、1080。
//VPU_Format : <format> 0 - MPEG4, 1 - H.263, 2 - H.264, 7 - MJPG
int VPU_camera(unsigned short resolution, unsigned char VPU_Format);


#endif
