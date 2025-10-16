/*----------------------------------------------*/
/* TJpgDec System Configurations R0.03          */
/*----------------------------------------------*/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/sjpg/tjpgdcnf.h
#define	JD_SZBUF		512
/* Specifies size of stream input buffer */

#define JD_FORMAT		0
=======
#define JD_SZBUF        512
/* Specifies size of stream input buffer */

#define JD_FORMAT       0
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/sjpg/tjpgdcnf.h
/* Specifies output pixel format.
/  0: RGB888 (24-bit/pix)
/  1: RGB565 (16-bit/pix)
/  2: Grayscale (8-bit/pix)
*/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/sjpg/tjpgdcnf.h
#define	JD_USE_SCALE	1
=======
#define JD_USE_SCALE    0
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/sjpg/tjpgdcnf.h
/* Switches output descaling feature.
/  0: Disable
/  1: Enable
*/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/sjpg/tjpgdcnf.h
#define JD_TBLCLIP		1
=======
#define JD_TBLCLIP      1
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/sjpg/tjpgdcnf.h
/* Use table conversion for saturation arithmetic. A bit faster, but increases 1 KB of code size.
/  0: Disable
/  1: Enable
*/

<<<<<<< Updated upstream:727R_PushBack/include/liblvgl/extra/libs/sjpg/tjpgdcnf.h
#define JD_FASTDECODE	0
=======
#define JD_FASTDECODE   1
>>>>>>> Stashed changes:Drive_727R_PushBack/include/liblvgl/extra/libs/sjpg/tjpgdcnf.h
/* Optimization level
/  0: Basic optimization. Suitable for 8/16-bit MCUs.
/  1: + 32-bit barrel shifter. Suitable for 32-bit MCUs.
/  2: + Table conversion for huffman decoding (wants 6 << HUFF_BIT bytes of RAM)
*/

