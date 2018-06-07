/*
 * Copyright 2004-2014 Freescale Semiconductor, Inc.
 *
 * Copyright (c) 2006, Chips & Media.  All rights reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../include/tqvpu_lib.h"

extern int quitflag;
#ifdef _FSL_VTS_
#include "../include/dut_probes_vts.h"
extern FuncProbeDut g_pfnVTSProbe;
#endif

int vpu_v4l_performance_test;

static FILE *fpFrmStatusLogfile = NULL;
static FILE *fpErrMapLogfile = NULL;
static FILE *fpQpLogfile = NULL;
static FILE *fpSliceBndLogfile = NULL;
static FILE *fpMvLogfile = NULL;
static FILE *fpUserDataLogfile = NULL;

static int isInterlacedMPEG4 = 0;

#define FN_FRAME_BUFFER_STATUS "dec_frame_buf_status.log"
#define FN_ERR_MAP_DATA "dec_error_map.log"
#define FN_QP_DATA "dec_qp.log"
#define FN_SB_DATA "dec_slice_bnd.log"
#define FN_MV_DATA "dec_mv.log"
#define FN_USER_DATA "dec_user_data.log"

#define JPG_HEADER_SIZE	     0x200

#ifdef COMBINED_VIDEO_SUPPORT
#define MAX_FRAME_WIDTH 720
#define MAX_FRAME_HEIGHT 576
#endif

//ccq
static struct cmd_line *cmdl = NULL;
static vpu_mem_desc mem_desc = {0};
static vpu_mem_desc ps_mem_desc = {0};
static vpu_mem_desc slice_mem_desc = {0};
static vpu_mem_desc vp8_mbparam_mem_desc = {0};
static struct decode *dec;

//ccq-en


/*
 * Seek SOI
 * Return SOI position (+2) if found, -1 if not found
 */
static int mjpg_seek_soi(struct decode *dec, u8 *buf, int size)
{
    u8 *ptr = buf;
    u8 byte;
    while (ptr < buf + size) {
        byte = *ptr++;
        switch (dec->mjpg_sc_state)
        {
        case 0:
            if (byte == 0xFF)
                dec->mjpg_sc_state = 1;
            break;
        case 1:
            if (byte == 0xD8) {
                dec->mjpg_sc_state = 2;
                goto end;
            }
            else if (byte != 0xFF)
                dec->mjpg_sc_state = 0;
            break;
        case 2:
            if (byte == 0xFF)
                dec->mjpg_sc_state = 1;
            else
                dec->mjpg_sc_state = 0;
            break;
        default:
            break;
        }
    }

end:
    if (dec->mjpg_sc_state == 2) {
        dprintf(4, "found %x%x\n", *(ptr - 2), *(ptr - 1));
        return (int)(ptr - buf);
    } else
        return -1;

}

/*
 * Read a chunk to bs buffer, stopped before next SOI
 * Return chunk size + 2 if success, -1 if error, 0 if eof
 * For jpeg file having thumbnail, it's not suitable to simply pick
 * bitstream between 2 SOIs, as the picture has SOI-SOI-EOI-EOI.
 * Please read the complete jpeg file one time.
 */
static int mjpg_read_chunk_till_soi(struct decode *dec)
{
    int nread;
    int pos;
    u8 *buf;

    buf = dec->mjpg_cached_bsbuf;

    if (dec->mjpg_rd_ptr > 2){
        memmove((void *)buf, (void *)(buf + dec->mjpg_rd_ptr - 2),
                dec->mjpg_wr_ptr - (dec->mjpg_rd_ptr - 2));
        dec->mjpg_wr_ptr -= dec->mjpg_rd_ptr - 2;
        dec->mjpg_rd_ptr = 2;
    }

    dprintf(4, "mjpg_rd_ptr 0x%lx mjpg_wr_ptr 0x%lx\n", dec->mjpg_rd_ptr, dec->mjpg_wr_ptr);

    while (1)
    {
        if (dec->mjpg_rd_ptr == dec->mjpg_wr_ptr) {
            nread = vpu_read(dec->cmdl, (void *)buf + dec->mjpg_wr_ptr,
                             MJPG_FILL_SIZE);
            if (nread < 0) {
                /* error */
                err_msg("nread %d < 0\n", nread);
                return -1;
            } else if (nread == 0) {
                if (dec->mjpg_eof) {
                    dprintf(4, "eof\n");
                    return 0;
                }
                *(buf + dec->mjpg_wr_ptr) = 0xFF;
                dec->mjpg_wr_ptr++;
                *(buf + dec->mjpg_wr_ptr) = 0xD8;
                dec->mjpg_wr_ptr++;
                dprintf(4, "pad soi\n");
                dec->mjpg_eof = 1;
            } else if (nread < MJPG_FILL_SIZE) {
                dec->mjpg_wr_ptr += nread;
                dprintf(4, "last piece of bs\n");
            } else
                dec->mjpg_wr_ptr += nread;
        }
        pos = mjpg_seek_soi(dec, buf + dec->mjpg_rd_ptr,
                            dec->mjpg_wr_ptr - dec->mjpg_rd_ptr);
        if (pos < 0)
        {
            dprintf(4, "soi not found\n");
            dec->mjpg_rd_ptr = dec->mjpg_wr_ptr;
        } else {
            dec->mjpg_rd_ptr += pos;
            dprintf(4, "chunk size %lu\n", dec->mjpg_rd_ptr - 2);
            memcpy((void *)dec->virt_bsbuf_addr, buf, dec->mjpg_rd_ptr - 2);
            return dec->mjpg_rd_ptr;
        }
    }
}

/*
 * Read a chunk to bs buffer
 * Return chunk size + 2 if success, -1 if error, 0 if eof
 */
static int mjpg_read_chunk(struct decode *dec)
{
    int ret;
#ifndef READ_WHOLE_FILE
    /* read a chunk between 2 SOIs for MJPEG clip */
    if (dec->mjpg_rd_ptr == 0) {
        ret = mjpg_read_chunk_till_soi(dec);
        if (ret <= 0)
            return ret;
    }
    ret = mjpg_read_chunk_till_soi(dec);
    return ret;
#else
    /* read whole file for JPEG file */
    ret = vpu_read(dec->cmdl, (void *)dec->virt_bsbuf_addr,
                   STREAM_BUF_SIZE);
    if (ret <= 0)
        return ret;
    else {
        dec->mjpg_rd_ptr = ret + 2;
        dprintf(4, "chunk size %lu\n", dec->mjpg_rd_ptr - 2);
        return dec->mjpg_rd_ptr;
    }
#endif
}

void SaveFrameBufStat(u8 *frmStatusBuf, int size, int DecNum)
{

    int i;

    if (fpFrmStatusLogfile == NULL) {
        fpFrmStatusLogfile = fopen(FN_FRAME_BUFFER_STATUS, "w+");
    }

    fprintf(fpFrmStatusLogfile, "FRAME [%1d]\n", DecNum);

    for (i = 0; i < size; i++) {
        fprintf(fpFrmStatusLogfile, "[%d] %d ", i*2, ((frmStatusBuf[i]>>4)&0x0F));
        fprintf(fpFrmStatusLogfile, "[%d] %d ", (i*2)+1, (frmStatusBuf[i]&0x0F));
    }
    fprintf(fpFrmStatusLogfile, "\n");
    fflush(fpFrmStatusLogfile);
}


void SaveMB_Para(u8 *mbParaBuf, int size, int DecNum)
{

    int i;

    if (DecNum == 1)
        DecNum = DecNum;

    if (fpErrMapLogfile == NULL)
        fpErrMapLogfile = fopen(FN_ERR_MAP_DATA, "w+");
    if (fpQpLogfile == NULL)
        fpQpLogfile = fopen(FN_QP_DATA, "w+");
    if (fpSliceBndLogfile == NULL)
        fpSliceBndLogfile = fopen(FN_SB_DATA, "w+");

    fprintf(fpQpLogfile, "FRAME [%1d]\n", DecNum);
    fprintf(fpSliceBndLogfile, "FRAME [%1d]\n", DecNum);
    fprintf(fpErrMapLogfile, "FRAME [%1d]\n", DecNum);


    for (i = 0; i < size; i++) {
        fprintf(fpQpLogfile, "MbAddr[%4d]: MbQs[%2d]\n", i, mbParaBuf[i]&0x3F);
        fprintf(fpSliceBndLogfile, "MbAddr[%4d]: Slice Boundary Flag[%1d]\n", i, (mbParaBuf[i]>>6)&1);
        fprintf(fpErrMapLogfile, "MbAddr[%4d]: ErrMap[%1d]\n", i, (mbParaBuf[i]>>7)&1);
    }

    fflush(fpQpLogfile);
    fflush(fpSliceBndLogfile);
    fflush(fpErrMapLogfile);
}

void SaveMvPara(u8 *mvParaBuf, int size, int mvNumPerMb, int mbNumX, int DecNum)
{

    int i, j;
    short mvX, mvY;
    u8 *mvDatabuf;

    if (fpMvLogfile == 0) {
        fpMvLogfile = fopen(FN_MV_DATA, "w+");
    }

    fprintf(fpMvLogfile, "FRAME [%1d]\n", DecNum);

    for (i = 0; i < size; i++) {
        for (j = 0; j < mvNumPerMb; j++) {
            mvDatabuf = (mvParaBuf + (i*mvNumPerMb + j)*4);
            mvX = (short)((mvDatabuf[0]<<8) | (mvDatabuf[1]<<0));
            mvY = (short)((mvDatabuf[2]<<8) | (mvDatabuf[3]<<0));

            if (!(mvX & 0x8000)){
                /* Intra MB */
                mvX = 0;
                mvY = 0;

                if (j == 0 )
                    fprintf(fpMvLogfile, "MbAddr[%4d:For ]: Avail[0] Mv[%5d:%5d]\n", i, mvX, mvY);
                if (j == 1)
                    fprintf(fpMvLogfile, "MbAddr[%4d:Back]: Avail[0] Mv[%5d:%5d]\n", i, mvX, mvY);

            } else {

                if(mvX & 0x2000) {/* Signed extension */
                    mvX = mvX | 0xC000;
                } else {
                    mvX = mvX & 0x1FFF;
                }

                if(mvY & 0x2000) {
                    mvY = mvY | 0xC000;
                } else {
                    mvY = mvY & 0x1FFF;
                }

                if (j == 0 )
                    fprintf(fpMvLogfile, "MbAddr[%4d:For ]: Avail[1] Mv[%5d:%5d]\n", i, mvX, mvY);
                if (j == 1)
                    fprintf(fpMvLogfile, "MbAddr[%4d:Back]: Avail[1] Mv[%5d:%5d]\n", i, mvX, mvY);
            }
        }
    }
    fflush(fpMvLogfile);
}

void SaveUserData(u8 *userDataBuf) {
    int i, UserDataType, UserDataSize, userDataNum, TotalSize;
    u8 *tmpBuf;

    if(fpUserDataLogfile == 0) {
        fpUserDataLogfile = fopen(FN_USER_DATA, "w+");
    }

    tmpBuf = userDataBuf;
    userDataNum = (short)((tmpBuf[0]<<8) | (tmpBuf[1]<<0));
    TotalSize = (short)((tmpBuf[2]<<8) | (tmpBuf[3]<<0));

    tmpBuf = userDataBuf + 8;

    for(i=0; i<userDataNum; i++) {
        UserDataType = (short)((tmpBuf[0]<<8) | (tmpBuf[1]<<0));
        UserDataSize = (short)((tmpBuf[2]<<8) | (tmpBuf[3]<<0));
        fprintf(fpUserDataLogfile, "\n[Idx Type Size] : [%4d %4d %4d]",i, UserDataType, UserDataSize);

        tmpBuf += 8;
    }
    fprintf(fpUserDataLogfile, "\n");

    tmpBuf = userDataBuf + USER_DATA_INFO_OFFSET;
    for(i=0; i<TotalSize; i++) {
        fprintf(fpUserDataLogfile, "%02x", tmpBuf[i]);
        if ((i&7) == 7)
            fprintf(fpUserDataLogfile, "\n");
    }
    fprintf(fpUserDataLogfile, "\n");
    fflush(fpUserDataLogfile);

}

/*
 * Fill the bitstream ring buffer
 */
int dec_fill_bsbuffer(DecHandle handle, struct cmd_line *cmd,
                u32 bs_va_startaddr, u32 bs_va_endaddr,
                u32 bs_pa_startaddr, int defaultsize,
                int *eos, int *fill_end_bs)
{
        RetCode ret;
        PhysicalAddress pa_read_ptr, pa_write_ptr;
        u32 target_addr, space;
        int size;
        int nread, room;
        *eos = 0;

        ret = vpu_DecGetBitstreamBuffer(handle, &pa_read_ptr, &pa_write_ptr,
                                        &space);
        if (ret != RETCODE_SUCCESS) {
                err_msg("vpu_DecGetBitstreamBuffer failed\n");
                return -1;
        }

        /* Decoder bitstream buffer is full */
        if (space <= 0) {
                warn_msg("space %lu <= 0\n", space);
                return 0;
        }

        if (defaultsize > 0) {
                if (space < defaultsize)
                        return 0;

                size = defaultsize;
        } else {
                size = ((space >> 9) << 9);
        }

        if (size == 0) {
                warn_msg("size == 0, space %lu\n", space);
                return 0;
        }

        /* Fill the bitstream buffer */
        target_addr = bs_va_startaddr + (pa_write_ptr - bs_pa_startaddr);
        if ( (target_addr + size) > bs_va_endaddr) {//环形读取，处理读指针倒追问题
                room = bs_va_endaddr - target_addr;
                nread = vpu_read(cmd, (void *)target_addr, room);
                if (nread <= 0) {
                        /* EOF or error */
                        if (nread < 0) {
                                if (nread == -EAGAIN)
                                        return 0;

                                err_msg("nread %d < 0\n", nread);
                                return -1;
                        }

                        *eos = 1;
                } else {
                        /* unable to fill the requested size, so back off! */
                        if (nread != room)
                                goto update;

                        /* read the remaining */
                        space = nread;
                        nread = vpu_read(cmd, (void *)bs_va_startaddr,
                                        (size - room));
                        if (nread <= 0) {
                                /* EOF or error */
                                if (nread < 0) {
                                        if (nread == -EAGAIN)
                                                return 0;

                                        err_msg("nread %d < 0\n", nread);
                                        return -1;
                                }

                                *eos = 1;
                        }

                        nread += space;
                }
        } else {
                nread = vpu_read(cmd, (void *)target_addr, size);
                if (nread <= 0) {
                        /* EOF or error */
                        if (nread < 0) {
                                if (nread == -EAGAIN)
                                        return 0;

                                err_msg("nread %d < 0\n", nread);
                                return -1;
                        }

                        *eos = 1;
                }
        }

update:
        if (*eos == 0) {
                ret = vpu_DecUpdateBitstreamBuffer(handle, nread);
                if (ret != RETCODE_SUCCESS) {
                        err_msg("vpu_DecUpdateBitstreamBuffer failed\n");
                        return -1;
                }
                *fill_end_bs = 0;
        } else {
                if (!*fill_end_bs) {
                        ret = vpu_DecUpdateBitstreamBuffer(handle,
                                        STREAM_END_SIZE);
                        if (ret != RETCODE_SUCCESS) {
                                err_msg("vpu_DecUpdateBitstreamBuffer failed"
                                                                "\n");
                                return -1;
                        }
                        *fill_end_bs = 1;
                }

        }

        return nread;
}

/*
 * This function is to convert framebuffer from interleaved Cb/Cr mode
 * to non-interleaved Cb/Cr mode.
 *
 * Note: This function does _NOT_ really store this framebuffer into file.
 */
static void
saveNV12ImageHelper(u8 *pYuv, struct decode *dec, u8 *buf)
{
        int Y, Cb;
        u8 *Y1, *Cb1, *Cr1;
        int img_size;
        int y, x;
        u8 *tmp;
        int height = dec->picheight;
        int stride = dec->stride;

        if (!pYuv || !buf) {
                err_msg("pYuv or buf should not be NULL.\n");
                return;
        }

        img_size = stride * height;

        Y = (int)buf;
        Cb = Y + img_size;

        Y1 = pYuv;
        Cb1 = Y1 + img_size;
        Cr1 = Cb1 + (img_size >> 2);

        memcpy(Y1, (u8 *)Y, img_size);

        for (y = 0; y < (dec->picheight / 2); y++) {
                tmp = (u8*)(Cb + dec->picwidth * y);
                for (x = 0; x < dec->picwidth; x += 2) {
                        *Cb1++ = tmp[x];
                        *Cr1++ = tmp[x + 1];
                }
        }
}

/*
 * This function is to store the framebuffer with Cropping size.
 *
 * Note: The output of picture width or height from VPU is always
 * 4-bit aligned. For example, the Cropping information in one
 * bit stream is crop.left/crop.top/crop.right/crop.bottom = 0/0/320/136
 * VPU output is picWidth = 320, picHeight = (136 + 15) & ~15 = 144;
 * whereas, this function will use crop information to save the framebuffer
 * with picWidth = 320, picHeight = 136. Kindly remind that all this calculation
 * is under the case without Rotation or Mirror.
 */
static void
saveCropYuvImageHelper(struct decode *dec, u8 *buf, Rect cropRect)
{
    u8 *pCropYuv;
    int cropWidth, cropHeight;
    int i;

    int width = dec->picwidth;
    int height = dec->picheight;
    int rot_en = dec->cmdl->rot_en;
    int rot_angle = dec->cmdl->rot_angle;
#ifdef _FSL_VTS_
    FRAME_COPY_INFO sFrmCpInfo;
#endif

    if (!buf) {
        err_msg("buffer point should not be NULL.\n");
        return;
    }

    if (rot_en && (rot_angle == 90 || rot_angle == 270)) {
        i = width;
        width = height;
        height = i;
    }

    dprintf(3, "left/top/right/bottom = %lu/%lu/%lu/%lu\n", cropRect.left,
            cropRect.top, cropRect.right, cropRect.bottom);

    pCropYuv = buf;
    cropWidth = cropRect.right - cropRect.left;
    cropHeight = cropRect.bottom - cropRect.top;

    pCropYuv += width * cropRect.top;
    pCropYuv += cropRect.left;

#ifdef _FSL_VTS_
    sFrmCpInfo.puchLumY = pCropYuv;
    sFrmCpInfo.iFrmWidth = cropWidth;
    sFrmCpInfo.iFrmHeight = cropHeight;
    sFrmCpInfo.iBufStrideY = width;
#else
    for (i = 0; i < cropHeight; i++) {
        decoder_complete_1frame_data_outputCallBack(dec->cmdl->dst_fd, pCropYuv, cropWidth);
        pCropYuv += width;
    }
#endif
    if (dec->cmdl->format == STD_MJPG && dec->mjpg_fmt == MODE400)
        return;

    cropWidth /= 2;
    cropHeight /= 2;
    pCropYuv = buf + (width * height);
    pCropYuv += (width / 2) * (cropRect.top / 2);
    pCropYuv += cropRect.left / 2;

#ifdef _FSL_VTS_
    sFrmCpInfo.puchChrU = pCropYuv;
    sFrmCpInfo.iBufStrideUV = width >> 1;
#else
    for (i = 0; i < cropHeight; i++) {
        decoder_complete_1frame_data_outputCallBack(dec->cmdl->dst_fd, pCropYuv, cropWidth);
        pCropYuv += width / 2;
    }
#endif

    pCropYuv = buf + (width * height) * 5 / 4;
    pCropYuv += (width / 2) * (cropRect.top / 2);
    pCropYuv += cropRect.left / 2;

#ifdef _FSL_VTS_
    sFrmCpInfo.puchChrV = pCropYuv;
#else
    for (i = 0; i < cropHeight; i++) {
        decoder_complete_1frame_data_outputCallBack(dec->cmdl->dst_fd, pCropYuv, cropWidth);
        pCropYuv += width / 2;
    }
#endif
#ifdef _FSL_VTS_
    g_pfnVTSProbe( E_OUTPUT_FRAME, &sFrmCpInfo );
#endif
}

/*
 *YUV image copy from on-board memory of tiled YUV to host buffer
 */
int SaveTiledYuvImageHelper(struct decode *dec, int yuvFp,
                              int picWidth, int picHeight, int index)
{
    int frameSize, pix_addr, offset;
    int y, x, nY, nCb, j;
    Uint8 *puc, *pYuv = 0, *dstAddrCb, *dstAddrCr;
    Uint32 addrY, addrCb, addrCr;
    Uint8 temp_buf[8];
    struct frame_buf *pfb = NULL;

    frameSize = picWidth * picHeight * 3 / 2;
    pfb = dec->pfbpool[index];

    pYuv = malloc(frameSize);
    if (!pYuv) {
        err_msg("Fail to allocate memory\n");
        return -1;
    }

    if (dec->cmdl->rot_en && (dec->cmdl->rot_angle == 90 || dec->cmdl->rot_angle == 270)) {
        j = picWidth;
        picWidth = picHeight;
        picHeight = j;
    }

    puc = pYuv;
    nY = picHeight;
    nCb = picHeight / 2;
    addrY = pfb->addrY;
    addrCb = pfb->addrCb;
    addrCr = pfb->addrCr;
    offset = pfb->desc.virt_uaddr - pfb->desc.phy_addr;

    for (y = 0; y < nY; y++) {
        for (x = 0; x < picWidth; x += 8) {
            pix_addr = vpu_GetXY2AXIAddr(dec->handle, 0, y, x, picWidth,
                                         addrY, addrCb, addrCr);
            pix_addr += offset;
            memcpy(puc + y * picWidth + x, (Uint8 *)pix_addr, 8);
        }
    }

    dstAddrCb = (Uint8 *)(puc + picWidth * nY);
    dstAddrCr = (Uint8 *)(puc + picWidth * nY * 5 / 4);

    for (y = 0; y < nCb ; y++) {
        for (x = 0; x < picWidth; x += 8) {
            pix_addr = vpu_GetXY2AXIAddr(dec->handle, 2, y, x, picWidth,
                                         addrY, addrCb, addrCr);
            pix_addr += offset;
            memcpy(temp_buf, (Uint8 *)pix_addr, 8);
            for (j = 0; j < 4; j++) {
                *dstAddrCb++ = *(temp_buf + j * 2);
                *dstAddrCr++ = *(temp_buf + j * 2 + 1);
            }
        }
    }

    decoder_complete_1frame_data_outputCallBack(yuvFp, (u8 *)pYuv, frameSize);
    free(pYuv);

    return 0;
}
/*
 * This function is to swap the cropping left/top/right/bottom
 * when there's cropping information, under rotation case.
 *
 * Note: If there's no cropping information in bit stream, just
 *	set rotCrop as no cropping info. And hence, the calling
 *	function after this will handle this case.
 */
static void
swapCropRect(struct decode *dec, Rect *rotCrop)
{
    int mode = 0;
    int rot_en = dec->cmdl->rot_en;
    int rotAngle = dec->cmdl->rot_angle;
    int framebufWidth = dec->picwidth;
    int framebufHeight = dec->picheight;
    int mirDir = dec->cmdl->mirror;
    int crop;

    if (!rot_en)
        err_msg("VPU Rotation disabled. No need to call this func.\n");

    Rect *norCrop = &(dec->picCropRect);
    dprintf(3, "left/top/right/bottom = %lu/%lu/%lu/%lu\n", norCrop->left,
            norCrop->top, norCrop->right, norCrop->bottom);

    /*
         * If no cropping info in bitstream, we just set rotCrop as it is.
         */
    crop = norCrop->left | norCrop->top | norCrop->right | norCrop->bottom;
    if (!crop) {
        memcpy(rotCrop, norCrop, sizeof(*norCrop));
        return;
    }

    switch (rotAngle) {
    case 0:
        switch (mirDir) {
        case MIRDIR_NONE:
            mode = 0;
            break;
        case MIRDIR_VER:
            mode = 5;
            break;
        case MIRDIR_HOR:
            mode = 4;
            break;
        case MIRDIR_HOR_VER:
            mode = 2;
            break;
        }
        break;
        /*
                 * Remember? VPU sets the rotation angle by counter-clockwise.
                 * We convert it to clockwise, in order to make it consistent
                 * with V4L2 rotation angle strategy. (refer to decoder_start)
                 *
                 * Note: if you wanna leave VPU rotation angle as it originally
                 *	is, bear in mind you need exchange the codes for
                 *	"case 90" and "case 270".
                 */
    case 90:
        switch (mirDir) {
        case MIRDIR_NONE:
            mode = 3;
            break;
        case MIRDIR_VER:
            mode = 7;
            break;
        case MIRDIR_HOR:
            mode = 6;
            break;
        case MIRDIR_HOR_VER:
            mode = 1;
            break;
        }
        break;
    case 180:
        switch (mirDir) {
        case MIRDIR_NONE:
            mode = 2;
            break;
        case MIRDIR_VER:
            mode = 4;
            break;
        case MIRDIR_HOR:
            mode = 5;
            break;
        case MIRDIR_HOR_VER:
            mode = 0;
            break;
        }
        break;
        /*
                 * Ditto. As the rot angle 90.
                 */
    case 270:
        switch (mirDir) {
        case MIRDIR_NONE:
            mode = 1;
            break;
        case MIRDIR_VER:
            mode = 6;
            break;
        case MIRDIR_HOR:
            mode = 7;
            break;
        case MIRDIR_HOR_VER:
            mode = 3;
            break;
        }
        break;
    }

    switch (mode) {
    case 0:
        rotCrop->bottom = norCrop->bottom;
        rotCrop->left = norCrop->left;
        rotCrop->right = norCrop->right;
        rotCrop->top = norCrop->top;
        break;
    case 1:
        rotCrop->bottom = framebufWidth - norCrop->left;
        rotCrop->left = norCrop->top;
        rotCrop->right = norCrop->bottom;
        rotCrop->top = framebufWidth - norCrop->right;
        break;
    case 2:
        rotCrop->bottom = framebufHeight - norCrop->top;
        rotCrop->left = framebufWidth - norCrop->right;
        rotCrop->right = framebufWidth - norCrop->left;
        rotCrop->top = framebufHeight - norCrop->bottom;
        break;
    case 3:
        rotCrop->bottom = norCrop->right;
        rotCrop->left = framebufHeight - norCrop->bottom;
        rotCrop->right = framebufHeight - norCrop->top;
        rotCrop->top = norCrop->left;
        break;
    case 4:
        rotCrop->bottom = norCrop->bottom;
        rotCrop->left = framebufWidth - norCrop->right;
        rotCrop->right = framebufWidth - norCrop->left;
        rotCrop->top = norCrop->top;
        break;
    case 5:
        rotCrop->bottom = framebufHeight - norCrop->top;
        rotCrop->left = norCrop->left;
        rotCrop->right = norCrop->right;
        rotCrop->top = framebufHeight - norCrop->bottom;
        break;
    case 6:
        rotCrop->bottom = norCrop->right;
        rotCrop->left = norCrop->top;
        rotCrop->right = norCrop->bottom;
        rotCrop->top = norCrop->left;
        break;
    case 7:
        rotCrop->bottom = framebufWidth - norCrop->left;
        rotCrop->left = framebufHeight - norCrop->bottom;
        rotCrop->right = framebufHeight - norCrop->top;
        rotCrop->top = framebufWidth - norCrop->right;
        break;
    }

    return;
}

/*
 * This function is to store the framebuffer into file.
 * It will handle the cases of chromaInterleave, or cropping,
 * or both.
 */
static void
write_to_file(struct decode *dec, Rect cropRect, int index)
{
    int height = (dec->picheight + 15) & ~15 ;
    int stride = dec->stride;
    int chromaInterleave = dec->cmdl->chromaInterleave;
    int img_size;
    u8 *pYuv = NULL, *pYuv0 = NULL, *buf;
    struct frame_buf *pfb = NULL;
    int cropping;

    dprintf(3, "left/top/right/bottom = %lu/%lu/%lu/%lu\n", cropRect.left,
            cropRect.top, cropRect.right, cropRect.bottom);
    cropping = cropRect.left | cropRect.top | cropRect.bottom | cropRect.right;

    if (cpu_is_mx6x() && (dec->cmdl->mapType != LINEAR_FRAME_MAP) &&
            !dec->tiled2LinearEnable) {
        SaveTiledYuvImageHelper(dec, dec->cmdl->dst_fd, stride, height, index);
        goto out;
    }

    pfb = dec->pfbpool[index];
    buf = (u8 *)(pfb->addrY + pfb->desc.virt_uaddr - pfb->desc.phy_addr);

    img_size = stride * height * 3 / 2;
    if (dec->cmdl->format == STD_MJPG) {
        if (dec->mjpg_fmt == MODE422 || dec->mjpg_fmt == MODE224)
            img_size = stride * height * 2;
        else if (dec->mjpg_fmt == MODE400)
            img_size = stride * height;
        else if (dec->mjpg_fmt == MODE444)
            img_size = stride * height * 3;
    }

    if ((chromaInterleave == 0 && cropping == 0) || dec->cmdl->format == STD_MJPG) {
#ifdef _FSL_VTS_
        FRAME_COPY_INFO sFrmCpInfo;
        int iOffsetY = stride * height;
        int iOffsetUV = iOffsetY >> 2;

        sFrmCpInfo.puchLumY = buf;
        sFrmCpInfo.puchChrU = buf + iOffsetY;
        sFrmCpInfo.puchChrV = buf + iOffsetY + iOffsetUV;
        sFrmCpInfo.iFrmWidth = stride;
        sFrmCpInfo.iFrmHeight = height;
        sFrmCpInfo.iBufStrideY = stride;
        sFrmCpInfo.iBufStrideUV = stride >> 1;
        g_pfnVTSProbe( E_OUTPUT_FRAME, &sFrmCpInfo );
#else
        decoder_complete_1frame_data_outputCallBack(dec->cmdl->dst_fd, buf, img_size);
#endif
        goto out;
    }

    /*
         * There could be these three cases:
         * interleave == 0, cropping == 1
         * interleave == 1, cropping == 0
         * interleave == 1, cropping == 1
         */
    if (chromaInterleave) {
        if (!pYuv) {
            pYuv0 = pYuv = malloc(img_size);
            if (!pYuv)
                err_msg("malloc error\n");
        }

        saveNV12ImageHelper(pYuv, dec, buf);
    }

    if (cropping) {
        if (!pYuv0) {
            pYuv0 = pYuv = malloc(img_size);
            if (!pYuv)
                err_msg("malloc error\n");

            saveCropYuvImageHelper(dec, buf, cropRect);
        } else {/* process the buf beginning at pYuv0 again for Crop. */
            pYuv = pYuv0;
            saveCropYuvImageHelper(dec, pYuv, cropRect);
        }

    } else {
#ifdef _FSL_VTS_
        FRAME_COPY_INFO sFrmCpInfo;
        int iOffsetY = stride * height;
        int iOffsetUV = iOffsetY >> 2;

        sFrmCpInfo.puchLumY = pYuv0;
        sFrmCpInfo.puchChrU = pYuv0 + iOffsetY;
        sFrmCpInfo.puchChrV = pYuv0 + iOffsetUV;
        sFrmCpInfo.iFrmWidth = stride;
        sFrmCpInfo.iFrmHeight = height;
        sFrmCpInfo.iBufStrideY = stride;
        sFrmCpInfo.iBufStrideUV = stride >> 1;
        g_pfnVTSProbe( E_OUTPUT_FRAME, &sFrmCpInfo );
#else
        decoder_complete_1frame_data_outputCallBack(dec->cmdl->dst_fd, (u8 *)pYuv0, img_size);
#endif
    }

out:
    if (pYuv0) {
        free(pYuv0);
        pYuv0 = NULL;
        pYuv = NULL;
    }

    return;
}
struct dec_argst{
    DecHandle handle;
    DecOutputInfo outinfo;
    DecParam decparam;
    int rot_en;
    int rot_stride;
    int fwidth;
    int fheight;
    int rot_angle;
    int deblock_en;
    int dering_en;
    FrameBuffer *deblock_fb;
    FrameBuffer *fb;
    struct frame_buf **pfbpool;
    struct frame_buf *pfb;
    struct vpu_display *disp;
    struct v4l_specific_data *v4l_rsd;
    int err;
    int eos;
    int fill_end_bs;
    int decodefinish;
    struct timeval tdec_begin;
    struct timeval tdec_end;
    struct timeval total_start;
    struct timeval total_end;//
    RetCode ret;
    int sec;
    int usec;
    int loop_id;
    u32 img_size;
    double tdec_time;
    double frame_id;
    double total_time;
    int decIndex;
    int rotid;
    int dblkid;
    int mirror;
    int count;
    int totalNumofErrMbs;
    int disp_clr_index;
    int actual_display_index ;
    int is_waited_int;//
    int tiled2LinearEnable;
    char *delay_ms;
    char *endptr;
    int mjpgReadChunk;
    int index;
};
struct dec_argst dec_st;
typedef enum vpu_process_states{
    DEC_DEFAULT = 0,
    DEC_INIT,
    DEC_LOOP,
    DEC_COMPLETE,
    DEC_CLOSE,
}vpu_process_states_en;
static vpu_process_states_en dec_beacon = DEC_DEFAULT;
void vpu_argur_init(struct dec_argst *args){
    struct dec_argst igars={
        .handle = dec->handle,
        .outinfo = {0},
        .decparam = {0},
        .rot_en = dec->cmdl->rot_en,
        .rot_stride = 0,
        .fwidth = 0,
        .fheight=0,
        .rot_angle = dec->cmdl->rot_angle,
        .deblock_en = dec->cmdl->deblock_en,
        .dering_en = dec->cmdl->dering_en,
        .deblock_fb = NULL,
        .fb = dec->fb,
        .pfbpool = dec->pfbpool,
        .pfb = NULL,
        .disp = dec->disp,
        .v4l_rsd=NULL,
        .err = 0,
                .eos = 0,
                .fill_end_bs = 0,
                .decodefinish = 0,
        .tdec_begin={0},
                .tdec_end={0},
                .total_start={0},
                .total_end={0},
        .ret = RETCODE_FAILURE,
        .sec=0,
                .usec=0,
                .loop_id=0,
        .img_size=0,
        .tdec_time = 0,
                .frame_id = 0,
                .total_time=0,
        .decIndex = 0,
        .rotid = 0,
                .dblkid = 0,
                .mirror = 0,
        .count = dec->cmdl->count,
        .totalNumofErrMbs = 0,
        .disp_clr_index = -1,
                .actual_display_index = -1,
        .is_waited_int = 0,
        .tiled2LinearEnable = dec->tiled2LinearEnable,
        .delay_ms=NULL,
                .endptr=NULL,
        .mjpgReadChunk = 0,
        .index = -1,
    };
    memcpy((struct dec_argst *)args,(struct dec_argst *)&igars,sizeof(igars));
}
int decoder_process(void)
{
    while(1){
        if (dec_st.rot_en || dec_st.dering_en || dec_st.tiled2LinearEnable || (dec->cmdl->format == STD_MJPG)) {
            vpu_DecGiveCommand(dec_st.handle, SET_ROTATOR_OUTPUT,
                               (void *)&dec_st.fb[dec_st.rotid]);
            if (dec_st.frame_id == 0) {
                if (dec_st.rot_en) {
                    vpu_DecGiveCommand(dec_st.handle,
                                       ENABLE_ROTATION, 0);
                    vpu_DecGiveCommand(dec_st.handle,
                                       ENABLE_MIRRORING,0);
                }
                if (dec_st.dering_en) {
                    vpu_DecGiveCommand(dec_st.handle,
                                       ENABLE_DERING, 0);
                }
            }
        }
        if (dec->userData.enable == 1) {
            dec_st.ret = vpu_DecGiveCommand(dec_st.handle,DEC_SET_REPORT_USERDATA, &dec->userData);
            if (dec_st.ret != RETCODE_SUCCESS) {
                err_msg("Failed to set user data report, dec_st.ret %d\n", dec_st.ret);
                return -1;
            }
        }

        /*
         * For mx6x MJPG decoding with streaming mode
         * bitstream buffer filling cannot be done when JPU is in decoding,
         * there are three places can do this:
         * 1. before vpu_DecStartOneFrame;
         * 2. in the case of RETCODE_JPEG_BIT_EMPTY returned in DecStartOneFrame() func;
         * 3. after vpu_DecGetOutputInfo.
         */
        if (cpu_is_mx6x() && (dec->cmdl->format == STD_MJPG)) {
            do{
                dprintf(1, "dec->mjpgLineBufferMode is ok frame_id: %d\n",(int)dec_st.frame_id);
            }while(1);
            if (dec->mjpgLineBufferMode) {
                /* First chunk is already read before vpu_DecGetInitialInfo */
                if (dec_st.mjpgReadChunk) {
                    dec_st.ret = mjpg_read_chunk(dec);
                    if (dec_st.ret < 0)
                        return dec_st.ret;
                    else if (dec_st.ret == 0)
                        break;
                } else
                    dec_st.mjpgReadChunk = 1;
                dec_st.decparam.chunkSize = dec->mjpg_rd_ptr - 2;
                dec_st.decparam.phyJpgChunkBase = dec->phy_bsbuf_addr;
                dec_st.decparam.virtJpgChunkBase = (unsigned char *)dec->virt_bsbuf_addr;
            } else {
                dec_st.err = dec_fill_bsbuffer(dec_st.handle, dec->cmdl,
                                               dec->virt_bsbuf_addr,
                                               (dec->virt_bsbuf_addr + STREAM_BUF_SIZE),
                                               dec->phy_bsbuf_addr, STREAM_FILL_SIZE,
                                               &dec_st.eos, &dec_st.fill_end_bs);
                if (dec_st.err < 0) {
                    err_msg("dec_fill_bsbuffer failed\n");
                    return -1;
                }
            }
            dec_st.decparam.mjpegScaleDownRatioWidth = dec->mjpegScaleDownRatioWidth;
            dec_st.decparam.mjpegScaleDownRatioHeight = dec->mjpegScaleDownRatioHeight;
        }

        gettimeofday(&dec_st.tdec_begin, NULL);
        dec_st.ret = vpu_DecStartOneFrame(dec_st.handle, &dec_st.decparam);
        if (dec_st.ret == RETCODE_JPEG_EOS) {
            info_msg(" JPEG bitstream is end\n");
            break;
        } else if (dec_st.ret == RETCODE_JPEG_BIT_EMPTY) {
            if (cpu_is_mx6x() && (dec->cmdl->format == STD_MJPG)
                    && !dec->mjpgLineBufferMode) {
                dprintf(1, "dec_st.ret == RETCODE_JPEG_BIT_EMPTY   frame_id: %d\n",(int)dec_st.frame_id);
                dec_st.err = dec_fill_bsbuffer(dec_st.handle, dec->cmdl,
                                               dec->virt_bsbuf_addr,
                                               (dec->virt_bsbuf_addr + STREAM_BUF_SIZE),
                                               dec->phy_bsbuf_addr, STREAM_FILL_SIZE,
                                               &dec_st.eos, &dec_st.fill_end_bs);
                if (dec_st.err < 0) {
                    err_msg("dec_fill_bsbuffer failed\n");
                    return -1;
                }
            }
            continue;
        }

        if (dec_st.ret != RETCODE_SUCCESS) {
            err_msg("DecStartOneFrame failed, dec_st.ret=%d\n", dec_st.ret);
            return -1;
        }

        dec_st.is_waited_int = 0;
        dec_st.loop_id = 0;
        while (vpu_IsBusy()) {
            if (!(cpu_is_mx6x() && (dec->cmdl->format == STD_MJPG))) {
                //                            dprintf(1, "vpu_IsBusy | dec->cmdl->format =STD_MJPG   frame_id: %d\n",(int)frame_id);
                dec_st.err = dec_fill_bsbuffer(dec_st.handle, dec->cmdl,
                                               dec->virt_bsbuf_addr,
                                               (dec->virt_bsbuf_addr + STREAM_BUF_SIZE),
                                               dec->phy_bsbuf_addr, STREAM_FILL_SIZE,
                                               &dec_st.eos, &dec_st.fill_end_bs);
                if (dec_st.err < 0) {
                    err_msg("dec_fill_bsbuffer failed\n");
                    return -1;
                }
            }
            /*
                                 * Suppose vpu is hang if one frame cannot be decoded in 5s,
                                 * then do vpu software reset.
                                 * Please take care of this for network case since vpu
                                 * interrupt also cannot be received if no enough data.
                                 */
            if (dec_st.loop_id == 50) {
                dec_st.err = vpu_SWReset(dec_st.handle, 0);
                return -1;
            }

            if (vpu_WaitForInt(100) == 0)
                dec_st.is_waited_int = 1;
            dec_st.loop_id ++;
        }

        if (!dec_st.is_waited_int)
            vpu_WaitForInt(100);

        gettimeofday(&dec_st.tdec_end, NULL);
        dec_st.sec = dec_st.tdec_end.tv_sec - dec_st.tdec_begin.tv_sec;
        dec_st.usec = dec_st.tdec_end.tv_usec - dec_st.tdec_begin.tv_usec;

        if (dec_st.usec < 0) {
            dec_st.sec--;
            dec_st.usec = dec_st.usec + 1000000;
        }

        dec_st.tdec_time += (dec_st.sec * 1000000) + dec_st.usec;

        dec_st.ret = vpu_DecGetOutputInfo(dec_st.handle, &dec_st.outinfo);

        /* In 8 instances test, we found some instance(s) may not get a chance to be scheduled
                         * until timeout, so we yield schedule each frame explicitly.
                         * This may be kernel dependant and may be removed on customer platform */
        usleep(0);

        if ((dec->cmdl->format == STD_MJPG) &&
                (dec_st.outinfo.indexFrameDisplay == 0)) {
            dec_st.outinfo.indexFrameDisplay = dec_st.rotid;
        }

        dprintf(3, "frame_id %d, decidx %d, disidx %d, rotid %d, decodingSuccess 0x%x\n",
                (int)dec_st.frame_id, dec_st.outinfo.indexFrameDecoded, dec_st.outinfo.indexFrameDisplay,
                dec_st.rotid, dec_st.outinfo.decodingSuccess);
        if (dec_st.ret != RETCODE_SUCCESS) {
            err_msg("vpu_DecGetOutputInfo failed Err code is %d\n"
                    "\tframe_id = %d\n", dec_st.ret, (int)dec_st.frame_id);
            return -1;
        }

        if (dec_st.outinfo.decodingSuccess == 0) {
            warn_msg("Incomplete finish of decoding process.\n"
                     "\tframe_id = %d\n", (int)dec_st.frame_id);
            if ((dec_st.outinfo.indexFrameDecoded >= 0) && (dec_st.outinfo.numOfErrMBs)) {
                if (cpu_is_mx6x() && dec->cmdl->format == STD_MJPG)
                    info_msg("Error restart idx:%d, MCU x:%d, y:%d, in Frame : %d\n",
                             (dec_st.outinfo.numOfErrMBs & 0x0F000000) >> 24,
                             (dec_st.outinfo.numOfErrMBs & 0x00FFF000) >> 12,
                             (dec_st.outinfo.numOfErrMBs & 0x00000FFF),
                             dec_st.decIndex);
            }
            if (quitflag)
                break;
            else
                continue;
        }

        if (cpu_is_mx6x() && (dec_st.outinfo.decodingSuccess & 0x10)) {
            warn_msg("vpu needs more bitstream in rollback mode\n"
                     "\tframe_id = %d\n", (int)dec_st.frame_id);

            dec_st.err = dec_fill_bsbuffer(dec_st.handle,  dec->cmdl, dec->virt_bsbuf_addr,
                                           (dec->virt_bsbuf_addr + STREAM_BUF_SIZE),
                                           dec->phy_bsbuf_addr, 0, &dec_st.eos, &dec_st.fill_end_bs);
            if (dec_st.err < 0) {
                err_msg("dec_fill_bsbuffer failed\n");
                return -1;
            }

            if (quitflag)
                break;
            else
                continue;
        }

        if (cpu_is_mx6x() && (dec_st.outinfo.decodingSuccess & 0x100000))
            warn_msg("sequence parameters have been changed\n");

        if (dec_st.outinfo.notSufficientPsBuffer) {
            err_msg("PS Buffer overflow\n");
            return -1;
        }

        if (dec_st.outinfo.notSufficientSliceBuffer) {
            err_msg("Slice Buffer overflow\n");
            return -1;
        }

        /* User Data */
        if (dec_st.outinfo.indexFrameDecoded >= 0 &&
                dec_st.outinfo.userData.enable && dec_st.outinfo.userData.size) {
            if (dec_st.outinfo.userData.userDataBufFull)
                warn_msg("User Data Buffer is Full\n");
            SaveUserData(dec->userData.addr);
        }

        if (dec_st.outinfo.indexFrameDisplay == -1)
            dec_st.decodefinish = 1;
        else if ((dec_st.outinfo.indexFrameDisplay > dec->regfbcount) &&
                 (dec_st.outinfo.prescanresult != 0) && !cpu_is_mx6x())
            dec_st.decodefinish = 1;

        if (dec_st.decodefinish && (!(dec_st.rot_en || dec_st.dering_en || dec_st.tiled2LinearEnable)))
            break;

        if (!cpu_is_mx6x() && (dec_st.outinfo.prescanresult == 0) &&
                (dec_st.decparam.prescanEnable == 1)) {
            dprintf(1, "prescanresult  prescanEnable  frame_id: %d\n",(int)dec_st.frame_id);
            if (dec_st.eos) {
                break;
            } else {
                int fillsize = 0;

                if (dec->cmdl->src_scheme == PATH_NET)
                    fillsize = 1000;
                else
                    warn_msg("Prescan: not enough bs data"
                             "\n");
                dec->cmdl->complete = 1;
                dec_st.err = dec_fill_bsbuffer(dec_st.handle,
                                               dec->cmdl,
                                               dec->virt_bsbuf_addr,
                                               (dec->virt_bsbuf_addr +
                                                STREAM_BUF_SIZE),
                                               dec->phy_bsbuf_addr, fillsize,
                                               &dec_st.eos, &dec_st.fill_end_bs);
                dec->cmdl->complete = 0;
                if (dec_st.err < 0) {
                    err_msg("dec_fill_bsbuffer failed\n");
                    return -1;
                }

                if (dec_st.eos)
                    break;
                else
                    continue;
            }
        }

        if (quitflag)
            break;

        if(dec_st.outinfo.indexFrameDecoded >= 0) {
            /* We MUST be careful of sequence param change (resolution change, etc)
                                 * Different frame buffer number or resolution may require vpu_DecClose
                                 * and vpu_DecOpen again to reallocate sufficient resources.
                                 * If you already allocate enough frame buffers of max resolution
                                 * in the beginning, you may not need vpu_DecClose, etc. But sequence
                                 * headers must be ahead of their pictures to signal param change.
                                 */
            if ((dec_st.outinfo.decPicWidth != dec->lastPicWidth)
                    ||(dec_st.outinfo.decPicHeight != dec->lastPicHeight)) {
                warn_msg("resolution changed from %dx%d to %dx%d\n",
                         dec->lastPicWidth, dec->lastPicHeight,
                         dec_st.outinfo.decPicWidth, dec_st.outinfo.decPicHeight);
                dec->lastPicWidth = dec_st.outinfo.decPicWidth;
                dec->lastPicHeight = dec_st.outinfo.decPicHeight;
            }

            if (dec_st.outinfo.numOfErrMBs) {
                dec_st.totalNumofErrMbs += dec_st.outinfo.numOfErrMBs;
                info_msg("Num of Error Mbs : %d, in Frame : %d \n",
                         dec_st.outinfo.numOfErrMBs, dec_st.decIndex);
            }
        }

        if(dec_st.outinfo.indexFrameDecoded >= 0)
            dec_st.decIndex++;

        /* BIT don't have picture to be displayed */
        if ((dec_st.outinfo.indexFrameDisplay == -3) ||
                (dec_st.outinfo.indexFrameDisplay == -2)) {
            do{
                dprintf(3, "VPU doesn't have picture to be displayed.\n"
                           "\tdec_st.outinfo.indexFrameDisplay = %d\n",
                        dec_st.outinfo.indexFrameDisplay);
                sleep(1);
            }while(0);

            if (!vpu_v4l_performance_test && (dec->cmdl->dst_scheme != PATH_IPU)) {
                if (dec->cmdl->format != STD_MJPG && dec_st.disp_clr_index >= 0) {
                    dec_st.err = vpu_DecClrDispFlag(dec_st.handle, dec_st.disp_clr_index);
                    if (dec_st.err)
                        err_msg("vpu_DecClrDispFlag failed Error code"
                                " %d\n", dec_st.err);
                }
                dec_st.disp_clr_index = dec_st.outinfo.indexFrameDisplay;
            }
            continue;
        }

        if (dec_st.rot_en || dec_st.dering_en || dec_st.tiled2LinearEnable || (dec->cmdl->format == STD_MJPG)) {
            /* delay one more frame for PP */
            if ((dec->cmdl->format != STD_MJPG) && (dec_st.disp_clr_index < 0)) {
                dec_st.disp_clr_index = dec_st.outinfo.indexFrameDisplay;
                continue;
            }
            dec_st.actual_display_index = dec_st.rotid;
        }
        else
            dec_st.actual_display_index = dec_st.outinfo.indexFrameDisplay;


        //quan
        if (dec_st.rot_en) {
            Rect rotCrop;
            swapCropRect(dec, &rotCrop);
            write_to_file(dec, rotCrop, dec_st.actual_display_index);
        } else {
            write_to_file(dec, dec->picCropRect, dec_st.actual_display_index);
        }

        if (dec->cmdl->format != STD_MJPG && dec_st.disp_clr_index >= 0) {
            dec_st.err = vpu_DecClrDispFlag(dec_st.handle,dec_st.disp_clr_index);
            if (dec_st.err)
                err_msg("vpu_DecClrDispFlag failed Error code"
                        " %d\n", dec_st.err);
        }
        dec_st.disp_clr_index = dec_st.outinfo.indexFrameDisplay;
        //quan e

        dec_st.frame_id++;
        if ((dec_st.count != 0) && (dec_st.frame_id >= dec_st.count))
            break;

        if (dec->cmdl->src_scheme == PATH_NET) {
            dec_st.err = dec_fill_bsbuffer(dec_st.handle,	dec->cmdl,
                                           dec->virt_bsbuf_addr,
                                           (dec->virt_bsbuf_addr + STREAM_BUF_SIZE),
                                           dec->phy_bsbuf_addr, STREAM_FILL_SIZE,
                                           &dec_st.eos, &dec_st.fill_end_bs);
            if (dec_st.err < 0) {
                err_msg("dec_fill_bsbuffer failed\n");
                return -1;
            }
        }

        dec_st.delay_ms = getenv("VPU_DECODER_DELAY_MS");
        if (dec_st.delay_ms && strtol(dec_st.delay_ms, &dec_st.endptr, 10))
            usleep(strtol(dec_st.delay_ms,&dec_st.endptr, 10) * 1000);

        if (dec_st.decodefinish)
            break;
    }
}
int
decoder_start(void)
{
    switch((int)dec_beacon)
    {
        case DEC_DEFAULT:{

        }break;
        case DEC_INIT:{
            vpu_argur_init(&dec_st);

            /* deblock_en is zero on none mx27 since it is cleared in decode_open() function. */
            if (dec_st.rot_en || dec_st.dering_en || dec_st.tiled2LinearEnable) {
                    dec_st.rotid = dec->regfbcount;
                    if (dec_st.deblock_en) {
                            dec_st.dblkid = dec->regfbcount + dec->rot_buf_count;
                    }
            } else if (dec_st.deblock_en) {
                    dec_st.dblkid = dec->regfbcount;
            }
            if (dec->cmdl->format == STD_MJPG)
                    dec_st.rotid = 0;

            dec_st.decparam.dispReorderBuf = 0;

            if (!cpu_is_mx6x()) {
                    dec_st.decparam.prescanEnable = dec->cmdl->prescan;
                    dec_st.decparam.prescanMode = 0;
            }

            dec_st.decparam.skipframeMode = 0;
            dec_st.decparam.skipframeNum = 0;
            /*
             * once iframeSearchEnable is enabled, prescanEnable, prescanMode
             * and skipframeMode options are ignored.
             */
            dec_st.decparam.iframeSearchEnable = 0;

            dec_st.fwidth = ((dec->picwidth + 15) & ~15);
            dec_st.fheight = ((dec->picheight + 15) & ~15);

            if (dec_st.rot_en || dec_st.dering_en || dec_st.tiled2LinearEnable || (dec->cmdl->format == STD_MJPG)) {
                    /*
                     * VPU is setting the rotation angle by counter-clockwise.
                     * We convert it to clockwise, which is consistent with V4L2
                     * rotation angle strategy.
                     */
                    if (dec_st.rot_en) {
                            if (dec_st.rot_angle == 90 || dec_st.rot_angle == 270)
                                    dec_st.rot_angle = 360 - dec_st.rot_angle;
                    } else
                            dec_st.rot_angle = 0;

                    vpu_DecGiveCommand(dec_st.handle, SET_ROTATION_ANGLE,
                                            &dec_st.rot_angle);

                    dec_st.mirror = dec->cmdl->mirror;
                    vpu_DecGiveCommand(dec_st.handle, SET_MIRROR_DIRECTION,
                                            &dec_st.mirror);

                    if (dec_st.rot_en)
                            dec_st.rot_stride = (dec_st.rot_angle == 90 || dec_st.rot_angle == 270) ?
                                            dec_st.fheight : dec_st.fwidth;
                    else
                            dec_st.rot_stride = dec_st.fwidth;
                    vpu_DecGiveCommand(dec_st.handle, SET_ROTATOR_STRIDE, &dec_st.rot_stride);
            }

            if (dec_st.deblock_en) {
                    dec_st.deblock_fb = &dec_st.fb[dec_st.dblkid];
            }

            gettimeofday(&dec_st.total_start, NULL);
            dec_beacon = DEC_LOOP;
        };
        case DEC_LOOP:{
            decoder_process();
            dec_beacon = DEC_COMPLETE;
        };
        case DEC_COMPLETE:{
             if (dec_st.totalNumofErrMbs) {
                 info_msg("Total Num of Error MBs : %d\n", dec_st.totalNumofErrMbs);
             }

             gettimeofday(&dec_st.total_end, NULL);
             dec_st.sec = dec_st.total_end.tv_sec - dec_st.total_start.tv_sec;
             dec_st.usec = dec_st.total_end.tv_usec - dec_st.total_start.tv_usec;
             if (dec_st.usec < 0) {
                 dec_st.sec--;
                 dec_st.usec = dec_st.usec + 1000000;
             }
             dec_st.total_time = (dec_st.sec * 1000000) + dec_st.usec;

             info_msg("%d frames took %d microseconds\n", (int)dec_st.frame_id,
                 (int)dec_st.total_time);
             info_msg("dec fps = %.2f\n", (dec_st.frame_id / (dec_st.tdec_time / 1000000)));
             info_msg("total fps= %.2f \n",(dec_st.frame_id / (dec_st.total_time / 1000000)));
             dec_beacon = DEC_INIT;
        }break;
        case DEC_CLOSE:{

        }break;
    }
    return 0;
}

void
decoder_free_framebuffer(struct decode *dec)
{
    int i, totalfb;
    vpu_mem_desc *mvcol_md = dec->mvcol_memdesc;
    int deblock_en = dec->cmdl->deblock_en;

    totalfb = dec->regfbcount + dec->extrafb;

    /* deblock_en is zero on none mx27 since it is cleared in decode_open() function. */
    if (((dec->cmdl->dst_scheme != PATH_V4L2) && (dec->cmdl->dst_scheme != PATH_IPU)) ||
            (((dec->cmdl->dst_scheme == PATH_V4L2) || (dec->cmdl->dst_scheme == PATH_IPU))
             && deblock_en)) {
        for (i = 0; i < totalfb; i++) {
            if (dec->pfbpool)
                framebuf_free(dec->pfbpool[i]);
        }
    }

    if (fpFrmStatusLogfile) {
        fclose(fpFrmStatusLogfile);
        fpFrmStatusLogfile = NULL;
    }
    if (fpErrMapLogfile) {
        fclose(fpErrMapLogfile);
        fpErrMapLogfile = NULL;
    }
    if (fpQpLogfile) {
        fclose(fpQpLogfile);
        fpQpLogfile = NULL;
    }
    if (fpSliceBndLogfile) {
        fclose(fpSliceBndLogfile);
        fpSliceBndLogfile = NULL;
    }
    if (fpMvLogfile) {
        fclose(fpMvLogfile);
        fpMvLogfile = NULL;
    }
    if (fpUserDataLogfile) {
        fclose(fpUserDataLogfile);
        fpUserDataLogfile = NULL;
    }

    if (dec->fb) {
        free(dec->fb);
        dec->fb = NULL;
    }
    if (dec->pfbpool) {
        free(dec->pfbpool);
        dec->pfbpool = NULL;
    }

    if (dec->frameBufStat.addr) {
        free(dec->frameBufStat.addr);
    }

    if (dec->mbInfo.addr) {
        free(dec->mbInfo.addr);
    }

    if (dec->mvInfo.addr) {
        free(dec->mvInfo.addr);
    }

    if (dec->userData.addr) {
        free(dec->userData.addr);
    }

    return;
}

int
decoder_allocate_framebuffer(struct decode *dec)
{
    DecBufInfo bufinfo;
    int i, regfbcount = dec->regfbcount, totalfb, img_size, mvCol;
    int dst_scheme = dec->cmdl->dst_scheme, rot_en = dec->cmdl->rot_en;
    int deblock_en = dec->cmdl->deblock_en;
    int dering_en = dec->cmdl->dering_en;
    int tiled2LinearEnable =  dec->tiled2LinearEnable;
    struct rot rotation = {0};
    RetCode ret;
    DecHandle handle = dec->handle;
    FrameBuffer *fb;
    struct frame_buf **pfbpool;
    struct vpu_display *disp = NULL;
    struct v4l_specific_data *v4l_rsd = NULL;
#ifdef BUILD_FOR_ANDROID
    struct g2d_specific_data *g2d_rsd = NULL;
#endif
    int stride, divX, divY;
    vpu_mem_desc *mvcol_md = NULL;
    Rect rotCrop;
    int delay = -1;

    if (rot_en || dering_en || tiled2LinearEnable) {
        /*
                 * At least 1 extra fb for rotation(or dering) is needed, two extrafb
                 * are allocated for rotation if path is V4L,then we can delay 1 frame
                 * de-queue from v4l queue to improve performance.
                 */
        dec->rot_buf_count = ((dec->cmdl->dst_scheme == PATH_V4L2) ||
                              (dec->cmdl->dst_scheme == PATH_IPU)) ? 2 : 1;
        /*
                 * Need more buffers for V4L2 performance mode
                 * otherwise, buffer will be queued twice
                 */
        dec->rot_buf_count = dec->regfbcount - dec->minfbcount + 1;
        dec->extrafb += dec->rot_buf_count;
        dec->post_processing = 1;
    }

    /*
         * 1 extra fb for deblocking on MX32, no need extrafb for blocking on MX37 and MX51
         * dec->cmdl->deblock_en has been cleared to zero after set it to oparam.mp4DeblkEnable
         * in decoder_open() function on MX37 and MX51.
         */
    if (deblock_en) {
        dec->extrafb++;
    }

    totalfb = regfbcount + dec->extrafb;
    dprintf(4, "regfb %d, extrafb %d\n", regfbcount, dec->extrafb);

    fb = dec->fb = calloc(totalfb, sizeof(FrameBuffer));
    if (fb == NULL) {
        err_msg("Failed to allocate fb\n");
        return -1;
    }

    pfbpool = dec->pfbpool = calloc(totalfb, sizeof(struct frame_buf *));
    if (pfbpool == NULL) {
        err_msg("Failed to allocate pfbpool\n");
        free(dec->fb);
        dec->fb = NULL;
        return -1;
    }

    if (dec->cmdl->format == STD_MJPG)
        mvCol = 0;
    else
        mvCol = 1;

    if (((dst_scheme != PATH_V4L2) && (dst_scheme != PATH_IPU)) ||
            (((dst_scheme == PATH_V4L2) || (dst_scheme == PATH_IPU)) && deblock_en)) {
        if (dec->cmdl->mapType == LINEAR_FRAME_MAP) {
            /* All buffers are linear */
            for (i = 0; i < totalfb; i++) {
                pfbpool[i] = framebuf_alloc(&dec->fbpool[i], dec->cmdl->format, dec->mjpg_fmt,
                                            dec->stride, dec->picheight, mvCol);
                if (pfbpool[i] == NULL)
                    goto err;
            }
        } else {
            /* decoded buffers are tiled */
            for (i = 0; i < regfbcount; i++) {
                pfbpool[i] = tiled_framebuf_alloc(&dec->fbpool[i], dec->cmdl->format, dec->mjpg_fmt,
                                                  dec->stride, dec->picheight, mvCol, dec->cmdl->mapType);
                if (pfbpool[i] == NULL)
                    goto err;
            }

            for (i = regfbcount; i < totalfb; i++) {
                /* if tiled2LinearEnable == 1, post processing buffer is linear,
                                 * otherwise, the buffer is tiled */
                if (dec->tiled2LinearEnable)
                    pfbpool[i] = framebuf_alloc(&dec->fbpool[i], dec->cmdl->format, dec->mjpg_fmt,
                                                dec->stride, dec->picheight, mvCol);
                else
                    pfbpool[i] = tiled_framebuf_alloc(&dec->fbpool[i], dec->cmdl->format, dec->mjpg_fmt,
                                                      dec->stride, dec->picheight, mvCol, dec->cmdl->mapType);
                if (pfbpool[i] == NULL)
                    goto err1;
            }
        }

        for (i = 0; i < totalfb; i++) {
            fb[i].myIndex = i;
            fb[i].bufY = pfbpool[i]->addrY;
            fb[i].bufCb = pfbpool[i]->addrCb;
            fb[i].bufCr = pfbpool[i]->addrCr;
            if (!cpu_is_mx27()) {
                fb[i].bufMvCol = pfbpool[i]->mvColBuf;
            }
        }
    }

    stride = ((dec->stride + 15) & ~15);

    if (dec->cmdl->format == STD_AVC) {
        bufinfo.avcSliceBufInfo.bufferBase = dec->phy_slice_buf;
        bufinfo.avcSliceBufInfo.bufferSize = dec->phy_slicebuf_size;
    } else if (dec->cmdl->format == STD_VP8) {
        bufinfo.vp8MbDataBufInfo.bufferBase = dec->phy_vp8_mbparam_buf;
        bufinfo.vp8MbDataBufInfo.bufferSize = dec->phy_vp8_mbparam_size;
    }

    /* User needs to fill max suported macro block value of frame as following*/
    bufinfo.maxDecFrmInfo.maxMbX = dec->stride / 16;
    bufinfo.maxDecFrmInfo.maxMbY = dec->picheight / 16;
    bufinfo.maxDecFrmInfo.maxMbNum = dec->stride * dec->picheight / 256;

    /* For H.264, we can overwrite initial delay calculated from syntax.
         * delay can be 0,1,... (in unit of frames)
         * Set to -1 or do not call this command if you don't want to overwrite it.
         * Take care not to set initial delay lower than reorder depth of the clip,
         * otherwise, display will be out of order. */
    vpu_DecGiveCommand(handle, DEC_SET_FRAME_DELAY, &delay);

    ret = vpu_DecRegisterFrameBuffer(handle, fb, dec->regfbcount, stride, &bufinfo);
    if (ret != RETCODE_SUCCESS) {
        err_msg("Register frame buffer failed, ret=%d\n", ret);
        goto err1;
    }

    dec->disp = disp;
    return 0;

err1:
err:
    if (((dst_scheme != PATH_V4L2) && (dst_scheme != PATH_IPU))||
            ((dst_scheme == PATH_V4L2) && deblock_en )) {
        for (i = 0; i < totalfb; i++) {
            framebuf_free(pfbpool[i]);
        }
    }

    if (fpFrmStatusLogfile) {
        fclose(fpFrmStatusLogfile);
        fpFrmStatusLogfile = NULL;
    }
    if (fpErrMapLogfile) {
        fclose(fpErrMapLogfile);
        fpErrMapLogfile = NULL;
    }
    if (fpQpLogfile) {
        fclose(fpQpLogfile);
        fpQpLogfile = NULL;
    }
    if (fpSliceBndLogfile) {
        fclose(fpSliceBndLogfile);
        fpSliceBndLogfile = NULL;
    }
    if (fpMvLogfile) {
        fclose(fpMvLogfile);
        fpMvLogfile = NULL;
    }
    if (fpUserDataLogfile) {
        fclose(fpUserDataLogfile);
        fpUserDataLogfile = NULL;
    }

    free(dec->fb);
    free(dec->pfbpool);
    dec->fb = NULL;
    dec->pfbpool = NULL;
    return -1;
}

int
decoder_parse(struct decode *dec)
{
    DecInitialInfo initinfo = {0};
    DecHandle handle = dec->handle;
    int align, profile, level, extended_fbcount;
    RetCode ret;
    char *count;
    int origPicWidth, origPicHeight;

    /*
         * If userData report is enabled, buffer and comamnd need to be set
         * before DecGetInitialInfo for MJPG.
         */
    if (dec->userData.enable) {
        dec->userData.size = SIZE_USER_BUF;
        dec->userData.addr = malloc(SIZE_USER_BUF);
        if (!dec->userData.addr)
            err_msg("malloc_error\n");
    }

    if(!cpu_is_mx6x() && dec->cmdl->format == STD_MJPG) {
        ret = vpu_DecGiveCommand(handle,DEC_SET_REPORT_USERDATA, &dec->userData);
        if (ret != RETCODE_SUCCESS) {
            err_msg("Failed to set user data report, ret %d\n", ret);
            return -1;
        }
    }

    /* Parse bitstream and get width/height/framerate etc */
    vpu_DecSetEscSeqInit(handle, 1);
    ret = vpu_DecGetInitialInfo(handle, &initinfo);
    vpu_DecSetEscSeqInit(handle, 0);
    if (ret != RETCODE_SUCCESS) {
        err_msg("vpu_DecGetInitialInfo failed, ret:%d, errorcode:%ld\n",
                ret, initinfo.errorcode);
        return -1;
    }

    if (initinfo.streamInfoObtained) {
        switch (dec->cmdl->format) {
        case STD_AVC:
            info_msg("H.264 Profile: %d Level: %d Interlace: %d\n",
                     initinfo.profile, initinfo.level, initinfo.interlace);

            if (initinfo.aspectRateInfo) {
                int aspect_ratio_idc;
                int sar_width, sar_height;

                if ((initinfo.aspectRateInfo >> 16) == 0) {
                    aspect_ratio_idc = (initinfo.aspectRateInfo & 0xFF);
                    info_msg("aspect_ratio_idc: %d\n", aspect_ratio_idc);
                } else {
                    sar_width = (initinfo.aspectRateInfo >> 16) & 0xFFFF;
                    sar_height = (initinfo.aspectRateInfo & 0xFFFF);
                    info_msg("sar_width: %d, sar_height: %d\n",
                             sar_width, sar_height);
                }
            } else {
                info_msg("Aspect Ratio is not present.\n");
            }

            break;
        case STD_VC1:
            if (initinfo.profile == 0)
                info_msg("VC1 Profile: Simple\n");
            else if (initinfo.profile == 1)
                info_msg("VC1 Profile: Main\n");
            else if (initinfo.profile == 2)
                info_msg("VC1 Profile: Advanced\n");

            info_msg("Level: %d Interlace: %d Progressive Segmented Frame: %d\n",
                     initinfo.level, initinfo.interlace, initinfo.vc1_psf);

            if (initinfo.aspectRateInfo)
                info_msg("Aspect Ratio [X, Y]:[%3d, %3d]\n",
                         (initinfo.aspectRateInfo >> 8) & 0xff,
                         (initinfo.aspectRateInfo) & 0xff);
            else
                info_msg("Aspect Ratio is not present.\n");

            break;
        case STD_MPEG2:
            info_msg("Mpeg2 Profile: %d Level: %d Progressive Sequence Flag: %d\n",
                     initinfo.profile, initinfo.level, !initinfo.interlace);
            /*
                         * Profile: 3'b101: Simple, 3'b100: Main, 3'b011: SNR Scalable,
                         * 3'b10: Spatially Scalable, 3'b001: High
                         * Level: 4'b1010: Low, 4'b1000: Main, 4'b0110: High 1440, 4'b0100: High
                         */
            if (initinfo.aspectRateInfo)
                info_msg("Aspect Ratio Table index: %d\n", initinfo.aspectRateInfo);
            else
                info_msg("Aspect Ratio is not present.\n");
            break;

        case STD_MPEG4:
            if (initinfo.level & 0x80) { /* VOS Header */
                initinfo.level &= 0x7F;
                if (initinfo.level == 8 && initinfo.profile == 0) {
                    level = 0; profile = 0;	 /* Simple, Level_L0 */
                } else {
                    switch (initinfo.profile) {
                    case 0xB:
                        profile = 1; /* advanced coding efficiency object */
                        break;
                    case 0xF:
                        if ((initinfo.level & 8) == 0)
                            profile = 2; /* advanced simple object */
                        else
                            profile = 5; /* reserved */
                        break;
                    case 0x0:
                        profile = 0;
                        break;    /* Simple Profile */
                    default:
                        profile = 5;
                        break;
                    }
                    level = initinfo.level;
                }
            } else { /* VOL Header only */
                level = 7;  /* reserved */
                switch (initinfo.profile) {
                case 0x1:
                    profile = 0;  /* simple object */
                    break;
                case 0xC:
                    profile = 1;  /* advanced coding efficiency object */
                    break;
                case 0x11:
                    profile = 2;  /* advanced simple object */
                    break;
                default:
                    profile = 5;  /* reserved */
                    break;
                }
            }
            isInterlacedMPEG4 = initinfo.interlace;

            info_msg("Mpeg4 Profile: %d Level: %d Interlaced: %d\n",
                     profile, level, initinfo.interlace);
            /*
                         * Profile: 8'b00000000: SP, 8'b00010001: ASP
                         * Level: 4'b0000: L0, 4'b0001: L1, 4'b0010: L2, 4'b0011: L3, ...
                         * SP: 1/2/3/4a/5/6, ASP: 0/1/2/3/4/5
                         */
            if (initinfo.aspectRateInfo)
                info_msg("Aspect Ratio Table index: %d\n", initinfo.aspectRateInfo);
            else
                info_msg("Aspect Ratio is not present.\n");

            break;

        case STD_RV:
            info_msg("RV Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
            break;

        case STD_H263:
            info_msg("H263 Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
            break;

        case STD_DIV3:
            info_msg("DIV3 Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
            break;

        case STD_MJPG:
            dec->mjpg_fmt = initinfo.mjpg_sourceFormat;
            info_msg("MJPG SourceFormat: %d\n", initinfo.mjpg_sourceFormat);
            break;

        case STD_AVS:
            info_msg("AVS Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
            break;

        case STD_VP8:
            info_msg("VP8 Profile: %d Level: %d\n", initinfo.profile, initinfo.level);
            info_msg("hScaleFactor: %d vScaleFactor:%d\n",
                     initinfo.vp8ScaleInfo.hScaleFactor,
                     initinfo.vp8ScaleInfo.vScaleFactor);
            break;

        default:
            break;
        }
    }

    dec->lastPicWidth = initinfo.picWidth;
    dec->lastPicHeight = initinfo.picHeight;

    if (cpu_is_mx6x())
        info_msg("Decoder: width = %d, height = %d, frameRateRes = %lu, frameRateDiv = %lu, count = %u\n",
                 initinfo.picWidth, initinfo.picHeight,
                 initinfo.frameRateRes, initinfo.frameRateDiv,
                 initinfo.minFrameBufferCount);
    else
        info_msg("Decoder: width = %d, height = %d, fps = %lu, count = %u\n",
                 initinfo.picWidth, initinfo.picHeight,
                 initinfo.frameRateInfo,
                 initinfo.minFrameBufferCount);

#ifdef COMBINED_VIDEO_SUPPORT
    /* Following lines are sample code to support minFrameBuffer counter
           changed in combined video stream. */
    if (dec->cmdl->format == STD_AVC)
        initinfo.minFrameBufferCount = 19;
#endif
    /*
         * We suggest to add two more buffers than minFrameBufferCount:
         *
         * vpu_DecClrDispFlag is used to control framebuffer whether can be
         * used for decoder again. One framebuffer dequeue from IPU is delayed
         * for performance improvement and one framebuffer is delayed for
         * display flag clear.
         *
         * Performance is better when more buffers are used if IPU performance
         * is bottleneck.
         *
         * Two more buffers may be needed for interlace stream from IPU DVI view
         */
    dec->minfbcount = initinfo.minFrameBufferCount;
    count = getenv("VPU_EXTENDED_BUFFER_COUNT");
    if (count)
        extended_fbcount = atoi(count);
    else
        extended_fbcount = 2;

    if (initinfo.interlace)
        dec->regfbcount = dec->minfbcount + extended_fbcount + 2;
    else
        dec->regfbcount = dec->minfbcount + extended_fbcount;
    dprintf(4, "minfb %d, extfb %d\n", dec->minfbcount, extended_fbcount);

    if (cpu_is_mx6x() && (dec->cmdl->format == STD_MJPG)) {
        origPicWidth = initinfo.picWidth >> dec->mjpegScaleDownRatioWidth;
        origPicHeight = initinfo.picHeight >> dec->mjpegScaleDownRatioHeight;
    } else {
        origPicWidth = initinfo.picWidth;
        origPicHeight = initinfo.picHeight;
    }

    dec->picwidth = ((origPicWidth + 15) & ~15);

    align = 16;
    if ((dec->cmdl->format == STD_MPEG2 ||
         dec->cmdl->format == STD_VC1 ||
         dec->cmdl->format == STD_AVC ||
         dec->cmdl->format == STD_VP8) && initinfo.interlace == 1)
        align = 32;

    dec->picheight = ((origPicHeight + align - 1) & ~(align - 1));

#ifdef COMBINED_VIDEO_SUPPORT
    /* Following lines are sample code to support resolution change
           from small to large in combined video stream. MAX_FRAME_WIDTH
           and MAX_FRAME_HEIGHT must be defined per user requirement. */
    if (dec->picwidth < MAX_FRAME_WIDTH)
        dec->picwidth = MAX_FRAME_WIDTH;
    if (dec->picheight < MAX_FRAME_HEIGHT)
        dec->picheight =  MAX_FRAME_HEIGHT;
#endif

    if ((dec->picwidth == 0) || (dec->picheight == 0))
        return -1;

    /*
         * Information about H.264 decoder picture cropping rectangle which
         * presents the offset of top-left point and bottom-right point from
         * the origin of frame buffer.
         *
         * By using these four offset values, host application can easily
         * detect the position of target output window. When display cropping
         * is off, the cropping window size will be 0.
         *
         * This structure for cropping rectangles is only valid for H.264
         * decoder case.
         */

    /* Add non-h264 crop support, assume left=top=0 */
    if ((dec->picwidth > origPicWidth ||
         dec->picheight > origPicHeight) &&
            (!initinfo.picCropRect.left &&
             !initinfo.picCropRect.top &&
             !initinfo.picCropRect.right &&
             !initinfo.picCropRect.bottom)) {
        initinfo.picCropRect.left = 0;
        initinfo.picCropRect.top = 0;
        initinfo.picCropRect.right = origPicWidth;
        initinfo.picCropRect.bottom = origPicHeight;
    }

    info_msg("CROP left/top/right/bottom %lu %lu %lu %lu\n",
             initinfo.picCropRect.left,
             initinfo.picCropRect.top,
             initinfo.picCropRect.right,
             initinfo.picCropRect.bottom);

    memcpy(&(dec->picCropRect), &(initinfo.picCropRect),
           sizeof(initinfo.picCropRect));

    /* worstSliceSize is in kilo-byte unit */
    dec->phy_slicebuf_size = initinfo.worstSliceSize * 1024;
    dec->stride = dec->picwidth;

    /* Allocate memory for frame status, Mb and Mv report */
    if (dec->frameBufStat.enable) {
        dec->frameBufStat.addr = malloc(initinfo.reportBufSize.frameBufStatBufSize);
        if (!dec->frameBufStat.addr)
            err_msg("malloc_error\n");
    }
    if (dec->mbInfo.enable) {
        dec->mbInfo.addr = malloc(initinfo.reportBufSize.mbInfoBufSize);
        if (!dec->mbInfo.addr)
            err_msg("malloc_error\n");
    }
    if (dec->mvInfo.enable) {
        dec->mvInfo.addr = malloc(initinfo.reportBufSize.mvInfoBufSize);
        if (!dec->mvInfo.addr)
            err_msg("malloc_error\n");
    }

    return 0;
}

int
decoder_open(struct decode *dec)
{
    RetCode ret;
    DecHandle handle = {0};
    DecOpenParam oparam = {0};

    if (dec->cmdl->mapType == LINEAR_FRAME_MAP)
        dec->tiled2LinearEnable = 0;
    else
        /* CbCr interleave must be enabled for tiled map */
        dec->cmdl->chromaInterleave = 1;

    oparam.bitstreamFormat = dec->cmdl->format;
    oparam.bitstreamBuffer = dec->phy_bsbuf_addr;
    oparam.bitstreamBufferSize = STREAM_BUF_SIZE;
    oparam.pBitStream = (Uint8 *)dec->virt_bsbuf_addr;
    oparam.reorderEnable = dec->reorderEnable;
    oparam.mp4DeblkEnable = dec->cmdl->deblock_en;
    oparam.chromaInterleave = dec->cmdl->chromaInterleave;
    oparam.mp4Class = dec->cmdl->mp4_h264Class;
    if (cpu_is_mx6x())
        oparam.avcExtension = dec->cmdl->mp4_h264Class;
    oparam.mjpg_thumbNailDecEnable = 0;
    oparam.mapType = dec->cmdl->mapType;
    oparam.tiled2LinearEnable = dec->tiled2LinearEnable;
    oparam.bitstreamMode = dec->cmdl->bs_mode;
    oparam.jpgLineBufferMode = dec->mjpgLineBufferMode;

    /*
         * mp4 deblocking filtering is optional out-loop filtering for image
         * quality. In other words, mpeg4 deblocking is post processing.
         * So, host application need to allocate new frame buffer.
         * - On MX27, VPU doesn't support mpeg4 out loop deblocking filtering.
         * - On MX5X, VPU control the buffer internally and return one
         *   more buffer after vpu_DecGetInitialInfo().
         * - On MX32, host application need to set frame buffer externally via
         *   the command DEC_SET_DEBLOCK_OUTPUT.
         */
    if (oparam.mp4DeblkEnable == 1) {
        if (cpu_is_mx27()) {
            warn_msg("MP4 deblocking NOT supported on MX27 VPU.\n");
            oparam.mp4DeblkEnable = dec->cmdl->deblock_en = 0;
        } else {	/* do not need extra framebuf */
            dec->cmdl->deblock_en = 0;
        }
    }

    oparam.psSaveBuffer = dec->phy_ps_buf;
    oparam.psSaveBufferSize = PS_SAVE_SIZE;

    ret = vpu_DecOpen(&handle, &oparam);
    if (ret != RETCODE_SUCCESS) {
        err_msg("vpu_DecOpen failed, ret:%d\n", ret);
        return -1;
    }

    dec->handle = handle;
    return 0;
}

void decoder_close(struct decode *dec)
{
    RetCode ret;

    ret = vpu_DecClose(dec->handle);
    if (ret == RETCODE_FRAME_NOT_COMPLETE) {
        vpu_SWReset(dec->handle, 0);
        ret = vpu_DecClose(dec->handle);
        if (ret != RETCODE_SUCCESS)
            err_msg("vpu_DecClose failed\n");
    }
}



int decoder_init(struct vpu_dec_handle *handle)
{    
    cmdl = handle->cmdl;
    dec = handle->dec;
    int ret, eos = 0, fill_end_bs = 0, fillsize = 0;
    dec_beacon = DEC_INIT;
    vpu_v4l_performance_test = 0;
	printf("[%s - %s: %d]----------ddd----------\n",__FILE__,__func__,__LINE__); 
    dec = (struct decode *)calloc(1, sizeof(struct decode));
    if (dec == NULL) {
            err_msg("Failed to allocate decode structure\n");
            ret = -1;
            goto err;
    }

    mem_desc.size = STREAM_BUF_SIZE;
    ret = IOGetPhyMem(&mem_desc);
    if (ret) {
            err_msg("Unable to obtain physical mem\n");
            goto err;
    }

    if (IOGetVirtMem(&mem_desc) == -1) {
            err_msg("Unable to obtain virtual mem\n");
            ret = -1;
            goto err;
    }

    dec->phy_bsbuf_addr = mem_desc.phy_addr;
    dec->virt_bsbuf_addr = mem_desc.virt_uaddr;

    dec->reorderEnable = 1;
    dec->tiled2LinearEnable = 0;

    dec->userData.enable = 0;
    dec->mbInfo.enable = 0;
    dec->mvInfo.enable = 0;
    dec->frameBufStat.enable = 0;
    dec->mjpgLineBufferMode = 0;
    dec->mjpegScaleDownRatioWidth = 0;  /* 0,1,2,3 */
    dec->mjpegScaleDownRatioHeight = 0; /* 0,1,2,3 */

    dec->cmdl = cmdl;

    if (cpu_is_mx6x() && (dec->cmdl->format == STD_MJPG)
                    && dec->mjpgLineBufferMode) {
            dec->mjpg_cached_bsbuf = malloc(STREAM_BUF_SIZE);
            if (dec->mjpg_cached_bsbuf == NULL) {
                    err_msg("Failed to allocate mjpg_cached_bsbuf\n");
                    ret = -1;
                    goto err;
            }
    }

    if (cmdl->format == STD_RV)
            dec->userData.enable = 0; /* RV has no user data */

    if (cmdl->format == STD_AVC) {
            ps_mem_desc.size = PS_SAVE_SIZE;
            ret = IOGetPhyMem(&ps_mem_desc);
            if (ret) {
                    err_msg("Unable to obtain physical ps save mem\n");
                    goto err;
            }
            dec->phy_ps_buf = ps_mem_desc.phy_addr;

    }

    /* open decoder */
    ret = decoder_open(dec);
    if (ret)
            goto err;

    cmdl->complete = 1;
    if (dec->cmdl->src_scheme == PATH_NET)
            fillsize = 1024;

    if (cpu_is_mx6x() && (dec->cmdl->format == STD_MJPG) && dec->mjpgLineBufferMode) {
            ret = mjpg_read_chunk(dec);
            if (ret < 0)
                    goto err1;
            else if (ret == 0) {
                    err_msg("no pic in the clip\n");
                    ret = -1;
                    goto err1;
            }
    } else {
            ret = dec_fill_bsbuffer(dec->handle, cmdl,
                            dec->virt_bsbuf_addr,
                            (dec->virt_bsbuf_addr + STREAM_BUF_SIZE),
                            dec->phy_bsbuf_addr, fillsize, &eos, &fill_end_bs);

            if (fill_end_bs)
                    err_msg("Update 0 before seqinit, fill_end_bs=%d\n", fill_end_bs);

            if (ret < 0) {
                    err_msg("dec_fill_bsbuffer failed\n");
                    goto err1;
            }
    }
    cmdl->complete = 0;

    /* parse the bitstream */
    ret = decoder_parse(dec);
    if (ret) {
            err_msg("decoder parse failed\n");
            goto err1;
    }

    /* allocate slice buf */
    if (cmdl->format == STD_AVC) {
            slice_mem_desc.size = dec->phy_slicebuf_size;
            ret = IOGetPhyMem(&slice_mem_desc);
            if (ret) {
                    err_msg("Unable to obtain physical slice save mem\n");
                    goto err1;
            }
            dec->phy_slice_buf = slice_mem_desc.phy_addr;
    }

    if (cmdl->format == STD_VP8) {
            vp8_mbparam_mem_desc.size = 68 * (dec->picwidth * dec->picheight / 256);
            ret = IOGetPhyMem(&vp8_mbparam_mem_desc);
            if (ret) {
                    err_msg("Unable to obtain physical vp8 mbparam mem\n");
                    goto err1;
            }
            dec->phy_vp8_mbparam_buf = vp8_mbparam_mem_desc.phy_addr;
    }

    /* allocate frame buffers */
    ret = decoder_allocate_framebuffer(dec);
    if (ret)
            goto err1;

    /* start decoding */
//    ret = decoder_start(dec);
    return 0;
err1:
    decoder_close(dec);
    /* free the frame buffers */
    decoder_free_framebuffer(dec);
err:
    if (cmdl->format == STD_AVC) {
            IOFreePhyMem(&slice_mem_desc);
            IOFreePhyMem(&ps_mem_desc);
    }

    if (cmdl->format == STD_VP8)
            IOFreePhyMem(&vp8_mbparam_mem_desc);

    if (dec->mjpg_cached_bsbuf)
            free(dec->mjpg_cached_bsbuf);
    IOFreeVirtMem(&mem_desc);
    IOFreePhyMem(&mem_desc);
    if (dec)
            free(dec);

    return ret;
}
int vpu_open(int versionInfoEn)
{
    int err;
    vpu_versioninfo ver;
    srand((unsigned)time(0)); /* init seed of rand() */

    err = vpu_Init(NULL);
    if (err) {
            err_msg("VPU Init Failure.\n");
            return -1;
    }

    err = vpu_GetVersionInfo(&ver);
    if (err) {
            err_msg("Cannot get version info, err:%d\n", err);
            vpu_UnInit();
            return -1;
    }

    dprintf(versionInfoEn,"VPU firmware version: %d.%d.%d_r%d\n", ver.fw_major, ver.fw_minor,
                                            ver.fw_release, ver.fw_code);
    dprintf(versionInfoEn,"VPU library version: %d.%d.%d\n", ver.lib_major, ver.lib_minor,
                                            ver.lib_release);
    return err;
}
int vpu_close(void)
{
    vpu_UnInit();
}
void decoder_uninit(void)
{
    if(dec) {
        decoder_close(dec);
    }
    /* free the frame buffers */
    if(dec){
        decoder_free_framebuffer((struct decode *)dec);
    }
    if (cmdl->format == STD_AVC) {
            IOFreePhyMem(&slice_mem_desc);
            IOFreePhyMem(&ps_mem_desc);
    }
    if (cmdl->format == STD_VP8)
            IOFreePhyMem(&vp8_mbparam_mem_desc);
    if (dec->mjpg_cached_bsbuf)
            free(dec->mjpg_cached_bsbuf);
    IOFreeVirtMem(&mem_desc);
    IOFreePhyMem(&mem_desc);
    if (dec)
            free(dec);
}

