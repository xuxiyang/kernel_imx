/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @file ipu.c
 *
 * @brief IPU device lib test implementation
 *
 * @ingroup IPU
 */

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/mxcfb.h>
#include "../include/tq_ipu.h"


#define TFAIL -1
#define TPASS 0

#define PAGE_ALIGN(x) (((x) + 4095) & ~4095)

ipu_devinfo_st *ipu_dinfo;
int ipu_dbg_level = 1;
FILE *file_out;
FILE *file_in;

static unsigned int fmt_to_bpp(unsigned int pixelformat)
{
        unsigned int bpp;

        switch (pixelformat)
        {
                case IPU_PIX_FMT_RGB565:
                /*interleaved 422*/
                case IPU_PIX_FMT_YUYV:
                case IPU_PIX_FMT_UYVY:
                /*non-interleaved 422*/
                case IPU_PIX_FMT_YUV422P:
                case IPU_PIX_FMT_YVU422P:
                        bpp = 16;
                        break;
                case IPU_PIX_FMT_BGR24:
                case IPU_PIX_FMT_RGB24:
                case IPU_PIX_FMT_YUV444:
                case IPU_PIX_FMT_YUV444P:
                        bpp = 24;
                        break;
                case IPU_PIX_FMT_BGR32:
                case IPU_PIX_FMT_BGRA32:
                case IPU_PIX_FMT_RGB32:
                case IPU_PIX_FMT_RGBA32:
                case IPU_PIX_FMT_ABGR32:
                        bpp = 32;
                        break;
                /*non-interleaved 420*/
                case IPU_PIX_FMT_YUV420P:
                case IPU_PIX_FMT_YVU420P:
                case IPU_PIX_FMT_YUV420P2:
                case IPU_PIX_FMT_NV12:
                case IPU_PIX_FMT_TILED_NV12:
                        bpp = 12;
                        break;
                default:
                        bpp = 8;
                        break;
        }
        return bpp;
}
static void dump_ipu_task(struct ipu_task *t)
{
    printf("====== ipu task ======\n");
    printf("input:\n");
    printf("\tforamt: 0x%x\n", t->input.format);
    printf("\twidth: %d\n", t->input.width);
    printf("\theight: %d\n", t->input.height);
    printf("\tcrop.w = %d\n", t->input.crop.w);
    printf("\tcrop.h = %d\n", t->input.crop.h);
    printf("\tcrop.pos.x = %d\n", t->input.crop.pos.x);
    printf("\tcrop.pos.y = %d\n", t->input.crop.pos.y);
    if (t->input.deinterlace.enable) {
        printf("deinterlace enabled with:\n");
        if (t->input.deinterlace.motion != HIGH_MOTION)
            printf("\tlow/medium motion\n");
        else
            printf("\thigh motion\n");
    }
    printf("output:\n");
    printf("\tforamt: 0x%x\n", t->output.format);
    printf("\twidth: %d\n", t->output.width);
    printf("\theight: %d\n", t->output.height);
    printf("\troate: %d\n", t->output.rotate);
    printf("\tcrop.w = %d\n", t->output.crop.w);
    printf("\tcrop.h = %d\n", t->output.crop.h);
    printf("\tcrop.pos.x = %d\n", t->output.crop.pos.x);
    printf("\tcrop.pos.y = %d\n", t->output.crop.pos.y);
    if (t->overlay_en) {
        printf("overlay:\n");
        printf("\tforamt: 0x%x\n", t->overlay.format);
        printf("\twidth: %d\n", t->overlay.width);
        printf("\theight: %d\n", t->overlay.height);
        printf("\tcrop.w = %d\n", t->overlay.crop.w);
        printf("\tcrop.h = %d\n", t->overlay.crop.h);
        printf("\tcrop.pos.x = %d\n", t->overlay.crop.pos.x);
        printf("\tcrop.pos.y = %d\n", t->overlay.crop.pos.y);
        if (t->overlay.alpha.mode == IPU_ALPHA_MODE_LOCAL)
            printf("combine with local alpha\n");
        else
            printf("combine with global alpha %d\n", t->overlay.alpha.gvalue);
        if (t->overlay.colorkey.enable)
            printf("colorkey enabled with 0x%x\n", t->overlay.colorkey.value);
    }
}
/* **************************************************************************************
 * funcation: start_ipu 开始ipu图像信息处理，数据以一帧为处理单元
 * 函数内部自动识别ipu_handle->show_to_fb：
 *    为：1，帧数据处理后在LCD上显示屏；
 *    为：0，且file_out !=NULL 帧数据处理后写入到 file_out
 *          若file_out ==NULL ipu_info->outbuf 将是处理后的帧数据，ipu_info->osize为帧数据大小
 * input: u8 *image_target 指向需要被处理的目标缓存，可以是摄像头数据、文件内容，格式详见：ipu.h
 *        struct ipu_devinfo *ipu_info ipu信息的结构体
 * output: FILE *file_out ipu处理帧数据后输出到的文件目标
 * return: 0 成功
 * other:关于该库的API使用，如果已经初始化了当次的输入输出信息，若想更改为其信息的，必须先关闭ipu后再次重新操作
 * 操作流程：开始->ipu_handle_init->init_ipu->start_ipu[可不断循环]->ipu_close(结束)
 */
int start_ipu(unsigned char *image_target,FILE * file_out,struct ipu_devinfo *ipu_info)
{
    int ret = 0;
    ipu_test_handle_t *ipu_handle = &ipu_info->ipu_handle;
    struct ipu_task *i_task = &ipu_handle->task;
    if (i_task->input.deinterlace.enable &&
            (i_task->input.deinterlace.motion != HIGH_MOTION))
        memcpy(ipu_info->vdibuf,image_target,ipu_info->isize);

    memcpy(ipu_info->inbuf,image_target,ipu_info->isize);

    ret = ioctl(ipu_info->fd_ipu, IPU_QUEUE_TASK, ipu_handle);
    if (ret < 0) {
        printf("ioct IPU_QUEUE_TASK fail\n");
    }
    if (ipu_handle->show_to_fb) {
        ret = ioctl(ipu_info->fd_fb, FBIOPAN_DISPLAY, &ipu_info->fb_var);
        if (ret < 0) {
            printf("fb ioct FBIOPAN_DISPLAY fail\n");
        }
    }else {
        if(file_out != NULL){
            ret = fwrite(ipu_info->outbuf, 1, ipu_info->osize, file_out);
            if (ret < ipu_info->osize) {
                ret = -1;
                printf("Can not write enough data into output file\n");
            }
        }
    }
    return ret;
}

/* **************************************************************************************
 * funcation: ipu_close 关闭ipu，释放所有申请的存储区及映射
 * input: ipu_info 被释放的ipu设备信息
 * output:
 * return: 0 成功
 * other:
 */
int close_ipu(struct ipu_devinfo *ipu_info)
{
    int blank;
    if (ipu_info->fd_fb) {
            blank = FB_BLANK_POWERDOWN;
            ioctl(ipu_info->fd_fb, FBIOBLANK, blank);
    }
    if (file_out)
            fclose(file_out);
    if (ipu_info->outbuf)
            munmap(ipu_info->outbuf, ipu_info->osize);
    if (ipu_info->fd_fb)
            close(ipu_info->fd_fb);
    if (ipu_info->ipu_handle.task.output.paddr)
            ioctl(ipu_info->fd_ipu, IPU_FREE, &ipu_info->ipu_handle.task.output.paddr);
    if (ipu_info->alpbuf)
            munmap(ipu_info->alpbuf, ipu_info->alpsize);
    if (ipu_info->ipu_handle.task.overlay.alpha.loc_alp_paddr)
            ioctl(ipu_info->fd_ipu, IPU_FREE, &ipu_info->ipu_handle.task.overlay.alpha.loc_alp_paddr);
    if (ipu_info->ovbuf)
            munmap(ipu_info->ovbuf, ipu_info->ovsize);
    if (ipu_info->ipu_handle.task.overlay.paddr)
            ioctl(ipu_info->fd_ipu, IPU_FREE, &ipu_info->ipu_handle.task.overlay.paddr);
    if (ipu_info->vdibuf)
            munmap(ipu_info->vdibuf, ipu_info->isize);
    if (ipu_info->ipu_handle.task.input.paddr_n)
            ioctl(ipu_info->fd_ipu, IPU_FREE, &ipu_info->ipu_handle.task.input.paddr_n);
    if (ipu_info->inbuf)
            munmap(ipu_info->inbuf, ipu_info->isize);
    if (ipu_info->ipu_handle.task.input.paddr)
            ioctl(ipu_info->fd_ipu, IPU_FREE, &ipu_info->ipu_handle.task.input.paddr);
    if (ipu_info->fd_ipu)
            close(ipu_info->fd_ipu);
    if (file_in)
            fclose(file_in);
    return 0;
}
/* **************************************************************************************
 * funcation: init_ipu 初始化ipu，内部打开ipu设备节点：/dev/mxc_ipu，显示屏节点/dev/fbx[x:0~4]
 * 使用前必须先初始化ipu_info结构体，其中包含输入格式、输入像素、去交织等，输出像素、
 * 输出到文件或者显示屏（二选一），图像旋转和其它的一些图像处理的参数，具体参见：ipu_handle_init函数
 * input: struct ipu_devinfo *ipu_info ipu设备信息
 * output: FILE *file_ipuout ipu处理图像后输出到的目标文件
 * return: 0 成功
 * other:关于该库的API使用，如果已经初始化了当次的输入输出信息，若想更改为其信息的，必须先关闭ipu后再次重新操作
 * 操作流程：开始->ipu_handle_init->init_ipu->start_ipu[可不断循环]->ipu_close(结束)
 */
int init_ipu(struct ipu_devinfo *ipu_info,FILE *file_ipuout)
{
    int ret = 0;
    int ipu_close_fb = 0;
    int blank;
    dma_addr_t outpaddr;
    file_out = file_ipuout;
    ipu_test_handle_t *ipu_handle = &ipu_info->ipu_handle;
    struct ipu_task *i_task = &ipu_handle->task;

    ipu_handle->task.overlay_en = 0;/*overlay 功能暂时不支持*/
    ipu_handle->task.overlay.width = ipu_handle->task.output.width;
    ipu_handle->task.overlay.height = ipu_handle->task.output.height;
    ipu_handle->task.overlay.format = ipu_handle->task.output.format;
    ipu_handle->task.overlay.crop.pos.x = 0;
    ipu_handle->task.overlay.crop.pos.y = 0;
    ipu_handle->task.overlay.crop.w = 0;
    ipu_handle->task.overlay.crop.h = 0;
    ipu_handle->task.overlay.alpha.mode = IPU_ALPHA_MODE_LOCAL;
    ipu_handle->task.overlay.alpha.gvalue = 0;
    ipu_handle->task.overlay.colorkey.enable = 0;
    ipu_handle->task.overlay.colorkey.value = 0x55555;


    ipu_info->fd_ipu = open("/dev/mxc_ipu",O_RDWR, 0);
    if(ipu_info->fd_ipu < 0)
    {
        IPU_ERR("open ipu dev is failure\r\n");
        ret = -1;
    }
    if (IPU_PIX_FMT_TILED_NV12F == i_task->input.format) {
        IPU_INFO("IPU_PIX_FMT nv12f \r\n");
        ipu_info->isize = PAGE_ALIGN(i_task->input.width * i_task->input.height/2) +
                PAGE_ALIGN(i_task->input.width * i_task->input.height/4);
        ipu_info->isize = i_task->input.paddr = ipu_info->isize * 2;
    } else
    {
        IPU_INFO("IPU_PIX_FMT : %d \r\n",i_task->input.format);
        ipu_info->isize = i_task->input.paddr =
                i_task->input.width * i_task->input.height
                * fmt_to_bpp(i_task->input.format)/8;
    }
    ret = ioctl(ipu_info->fd_ipu, IPU_ALLOC, &i_task->input.paddr);
    if (ret < 0) {
        IPU_ERR("ioctl IPU_ALLOC fail\n");
        goto err2;
    }
    ipu_info->inbuf = mmap(0, ipu_info->isize, PROT_READ | PROT_WRITE,
            MAP_SHARED, ipu_info->fd_ipu, i_task->input.paddr);
    if (!ipu_info->inbuf) {
        IPU_ERR("mmap input.paddr is failure\r\n");
        ret = -1;
        goto err3;
    }

    if (i_task->input.deinterlace.enable &&
            (i_task->input.deinterlace.motion != HIGH_MOTION)) {
            i_task->input.paddr_n = ipu_info->isize;
            ret = ioctl(ipu_info->fd_ipu, IPU_ALLOC, &i_task->input.paddr_n);
            if (ret < 0) {
                    IPU_ERR("ioctl IPU_ALLOC fail\n");
                    goto err4;
            }
            ipu_info->vdibuf = mmap(0, ipu_info->isize, PROT_READ | PROT_WRITE,
                            MAP_SHARED, ipu_info->fd_ipu, i_task->input.paddr_n);
            if (!ipu_info->vdibuf) {
                    printf("mmap fail\n");
                    ret = -1;
                    goto err5;
            }
    }
    if (i_task->overlay_en){
        ipu_info->ovsize = i_task->overlay.paddr =
                i_task->overlay.width * i_task->overlay.height
                * fmt_to_bpp(i_task->overlay.format)/8;
        ret = ioctl(ipu_info->fd_ipu, IPU_ALLOC, &i_task->overlay.paddr);
        if (ret < 0) {
            IPU_ERR("ioctl IPU_ALLOC fail\n");
            goto err6;
        }
        ipu_info->ovbuf = mmap(0, ipu_info->ovsize, PROT_READ | PROT_WRITE,
                     MAP_SHARED, ipu_info->fd_ipu, i_task->overlay.paddr);
        if (!ipu_info->ovbuf) {
            printf("mmap fail\n");
            ret = -1;
            goto err7;
        }

        /*fill overlay buffer with dedicated data*/
        memset(ipu_info->ovbuf, 0x00, ipu_info->ovsize/4);
        memset(ipu_info->ovbuf+ipu_info->ovsize/4, 0x55, ipu_info->ovsize/4);
        memset(ipu_info->ovbuf+ipu_info->ovsize/2, 0xaa, ipu_info->ovsize/4);
        memset(ipu_info->ovbuf+ipu_info->ovsize*3/4, 0xff, ipu_info->ovsize/4);

        if (i_task->overlay.alpha.mode == IPU_ALPHA_MODE_LOCAL) {
            ipu_info->alpsize = i_task->overlay.alpha.loc_alp_paddr =
                    i_task->overlay.width * i_task->overlay.height;
            ret = ioctl(ipu_info->fd_ipu, IPU_ALLOC, &i_task->overlay.alpha.loc_alp_paddr);
            if (ret < 0) {
                IPU_ERR("ioctl IPU_ALLOC fail\n");
                goto err8;
            }
            ipu_info->alpbuf = mmap(0, ipu_info->alpsize, PROT_READ | PROT_WRITE,
                          MAP_SHARED, ipu_info->fd_ipu, i_task->overlay.alpha.loc_alp_paddr);
            if (!ipu_info->alpbuf) {
                printf("mmap fail\n");
                ret = -1;
                goto err9;
            }

            /*fill loc alpha buffer with dedicated data*/
            memset(ipu_info->alpbuf, 0x00, ipu_info->alpsize/4);
            memset(ipu_info->alpbuf+ipu_info->alpsize/4, 0x55, ipu_info->alpsize/4);
            memset(ipu_info->alpbuf+ipu_info->alpsize/2, 0xaa, ipu_info->alpsize/4);
            memset(ipu_info->alpbuf+ipu_info->alpsize*3/4, 0xff, ipu_info->alpsize/4);
        }
    }
    if (ipu_handle->show_to_fb) {
        int found = 0, i;
        char fb_dev[] = "/dev/fb0";
        IPU_INFO("ipu is pass the image data to fb device：%s\r\n",ipu_handle->fb_name);
        for (i=0; i<5; i++) {
            fb_dev[7] = '0';
            fb_dev[7] += i;
            ipu_info->fd_fb = open(fb_dev, O_RDWR, 0);
            if (ipu_info->fd_fb > 0) {
                ioctl(ipu_info->fd_fb, FBIOGET_FSCREENINFO, &ipu_info->fb_fix);
                IPU_INFO("get the fb fix id[%d] is:%s\r\n",i,ipu_info->fb_fix.id);
                if (!strcmp(ipu_info->fb_fix.id, ipu_handle->fb_name)) {
                    IPU_INFO("found fb dev %s\n", fb_dev);
                    found = 1;
                    break;
                } else
                    close(ipu_info->fd_fb);
            }
        }

        if (!found) {
            IPU_ERR("can not find fb dev %s\n", ipu_handle->fb_name);
            ret = -1;
            goto err10;
        }

        ioctl(ipu_info->fd_fb, FBIOGET_VSCREENINFO, &ipu_info->fb_var);
        ipu_info->fb_var.xres = i_task->output.width;
        ipu_info->fb_var.xres_virtual = ipu_info->fb_var.xres;
        ipu_info->fb_var.yres = i_task->output.height;
        ipu_info->fb_var.yres_virtual = ipu_info->fb_var.yres;
        ipu_info->fb_var.activate |= FB_ACTIVATE_FORCE;
        ipu_info->fb_var.vmode |= FB_VMODE_YWRAP;
        ipu_info->fb_var.nonstd = i_task->output.format;
        ipu_info->fb_var.bits_per_pixel = fmt_to_bpp(i_task->output.format);

        ret = ioctl(ipu_info->fd_fb, FBIOPUT_VSCREENINFO, &ipu_info->fb_var);
        if (ret < 0) {
            IPU_ERR("fb ioctl FBIOPUT_VSCREENINFO fail\n");
            goto err11;
        }
        ioctl(ipu_info->fd_fb, FBIOGET_VSCREENINFO, &ipu_info->fb_var);
        ioctl(ipu_info->fd_fb, FBIOGET_FSCREENINFO, &ipu_info->fb_fix);

        outpaddr = ipu_info->fb_fix.smem_start;
        i_task->output.paddr = outpaddr;
        blank = FB_BLANK_UNBLANK;
        ioctl(ipu_info->fd_fb, FBIOBLANK, blank);
    }else {
        IPU_INFO("\t Ipu processing data will be saved in the ‘file_ipuout’\r\n");
        ipu_info->osize = i_task->output.paddr =
                i_task->output.width * i_task->output.height
                * fmt_to_bpp(i_task->output.format)/8;
        ret = ioctl(ipu_info->fd_ipu, IPU_ALLOC, &i_task->output.paddr);
        if (ret < 0) {
            IPU_ERR("ioctl IPU_ALLOC fail\n");
            goto err10;
        }
        ipu_info->outbuf = mmap(0, ipu_info->osize, PROT_READ | PROT_WRITE,
                                MAP_SHARED, ipu_info->fd_ipu, i_task->output.paddr);
        if (!ipu_info->outbuf) {
            IPU_ERR("mmap fail\n");
            ret = -1;
            goto err11;
        }
    }
    int again_cnt = 0;
again:
    again_cnt++;
    ret = ioctl(ipu_info->fd_ipu, IPU_CHECK_TASK, i_task);
    if (ret != IPU_CHECK_OK) {
        if (ret > IPU_CHECK_ERR_MIN) {
            if (ret == IPU_CHECK_ERR_SPLIT_INPUTW_OVER) {
                i_task->input.crop.w -= 8;
                goto again;
            }
            if (ret == IPU_CHECK_ERR_SPLIT_INPUTH_OVER) {
                i_task->input.crop.h -= 8;
                goto again;
            }
            if (ret == IPU_CHECK_ERR_SPLIT_OUTPUTW_OVER) {
                i_task->output.crop.w -= 8;
                goto again;
            }
            if (ret == IPU_CHECK_ERR_SPLIT_OUTPUTH_OVER) {
                i_task->output.crop.h -= 8;
                goto again;
            }
            ret = 0;
            printf("ipu task check fail\n");
            goto err13;
        }
    }
    memset(ipu_info->inbuf,0xff,ipu_info->isize);
    IPU_INFO("again_cnt: %d\r\n",again_cnt);
    if(ipu_info->dump_printf)
    {
        dump_ipu_task(i_task);
    }
    IPU_INFO("ipu init start is ok\r\n");


if(ipu_close_fb)
{
err13:
        if (ipu_info->fd_fb) {
                blank = FB_BLANK_POWERDOWN;
                ioctl(ipu_info->fd_fb, FBIOBLANK, blank);
        }
        if (file_out)
                fclose(file_out);
//err12:
        if (ipu_info->outbuf)
                munmap(ipu_info->outbuf, ipu_info->osize);
err11:
        if (ipu_info->fd_fb)
                close(ipu_info->fd_fb);
        if (i_task->output.paddr)
                ioctl(ipu_info->fd_ipu, IPU_FREE, &i_task->output.paddr);
err10:
        if (ipu_info->alpbuf)
                munmap(ipu_info->alpbuf, ipu_info->alpsize);
err9:
        if (i_task->overlay.alpha.loc_alp_paddr)
                ioctl(ipu_info->fd_ipu, IPU_FREE, &i_task->overlay.alpha.loc_alp_paddr);
err8:
        if (ipu_info->ovbuf)
                munmap(ipu_info->ovbuf, ipu_info->ovsize);
err7:
        if (i_task->overlay.paddr)
                ioctl(ipu_info->fd_ipu, IPU_FREE, &i_task->overlay.paddr);
err6:
        if (ipu_info->vdibuf)
                munmap(ipu_info->vdibuf, ipu_info->isize);
err5:
        if (i_task->input.paddr_n)
                ioctl(ipu_info->fd_ipu, IPU_FREE, &i_task->input.paddr_n);
err4:
        if (ipu_info->inbuf)
                munmap(ipu_info->inbuf, ipu_info->isize);
err3:
        if (i_task->input.paddr)
                ioctl(ipu_info->fd_ipu, IPU_FREE, &i_task->input.paddr);
err2:
        if (ipu_info->fd_ipu)
                close(ipu_info->fd_ipu);
//err1:
        if (file_in)
                fclose(file_in);
}
        return ret;
}
