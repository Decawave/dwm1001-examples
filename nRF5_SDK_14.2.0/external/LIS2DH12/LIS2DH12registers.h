/*
*
*	Component name:	LIS2DH12
*
*	File name: LIS2DH12registers.h
*
* Description:
* Definitions file for LIS2DH12 Accelerometer registers.
*
* NOTE - this is not a public file - only for use within
* scope of the LIS2DH12 device driver.
*
*/

// Registers, ordered by address
#define	WHO_AM_I		0x0f
#define	I_AM				0x33

#define CTRL_REG1		0x20
#define X_EN				0x01
#define Y_EN				0x02
#define Z_EN				0x04
#define LPEN				0x08

//ODR3 ODR2 ODR1 ODR0 Power mode selection
//0 0 0 0 Power-down mode
//0 0 0 1 HR / Normal / Low-power mode   (1 Hz)
//0 0 1 0 HR / Normal / Low-power mode  (10 Hz)
//0 0 1 1 HR / Normal / Low-power mode  (25 Hz)
//0 1 0 0 HR / Normal / Low-power mode  (50 Hz)
//0 1 0 1 HR / Normal / Low-power mode (100 Hz)
//0 1 1 0 HR / Normal / Low-power mode (200 Hz)
//0 1 1 1 HR / Normal / Low-power mode (400 Hz)
//1 0 0 0 Low-power mode            (1.620 kHz)
//1 0 0 1 HR / Normal               (1.344 kHz);Low-power mode (5.376 kHz)
#define ODR_PDOWN				0
#define ODR_1Hz					0x10
#define ODR_10Hz				0x20
#define ODR_25Hz				0x30
#define ODR_50Hz				0x40
#define ODR_100Hz				0x50
#define ODR_200Hz				0x60
#define ODR_400Hz				0x70
#define ODR_1620Hz			0x80
#define ODR_1344_5376Hz	0x90

#define CTRL_REG2					0x21
#define	HPM_MODE_NORMAL		0x00
#define	HPM_MODE_REFER		0x40
#define	HPM_MODE_AUTO_RS	0xc0
#define	HPF_CUTOFF_F0			0x00
#define	HPF_CUTOFF_F1			0x10
#define	HPF_CUTOFF_F2			0x20
#define	HPF_CUTOFF_F3			0x30
#define	FDS								0x08
#define	HPCLICK						0x04
#define	HP_IA2						0x02
#define	HP_IA1						0x01

#define CTRL_REG3		0x22
#define I1_WTM			0x04
#define	I1_IA1			0x40

#define CTRL_REG4		0x23
#define HIRES_MODE	0x08
#define FS_2G				0x00
#define FS_4G				0x10
#define FS_8G				0x20
#define FS_16G			0x30
#define BDU_ENABLE	0x80

#define CTRL_REG5		0x24
#define LIR_INT1		0x08
#define FIFO_EN			0x40

#define	CTRL_REG6				0x25
#define	REFERENCE_REG		0x26

// Status Register
#define STATUS_REG_ADD	0x27
#define	STATUS_XDATA		0x01

// Data Registers
#define OUT_X_LO		0x28
#define OUT_X_HI		0x29
#define OUT_Y_LO		0x2a
#define OUT_Y_HI		0x2b
#define OUT_Z_LO		0x2c
#define OUT_Z_HI		0x2d

// FIFO Control Register
#define FIFO_CTRL_REG		0x2e
#define BYPASS					0x00
#define FIFO_MODE				0x40
#define STREAM_MODE			0x80
#define STREAM_TO_FIFO	0xc0
#define TR_INT1					0x00
#define TR_INT2					0x20

#define FIFO_WATERMARK_30	30
#define MAX_FIFO_SIZE			32

// FIFO SRC Register
#define FIFO_SRC_REG			0x2f
#define FSS_MASK					0x1f
#define FIFO_EMPTY				0x20
#define	FIFO_OVRN					0x40
#define FIFO_WTM_TRIGGER	0x80

// Interrupt Control
#define READY_PIN		25
#define INT1_SRC		0x31

// Threshold detection configuration register
#define	INT1_CFG	0x30
#define	FUNC_6D		0x40
#define	AOI				0x80

// Threshold register
#define INT1_THS			0x32
#define	THS_FS2G_16mg	1
#define	THS_FS2G_32mg	2
#define	THS_FS2G_48mg	3
#define	THS_FS2G_64mg	4

#define	INT1_DURATION	0x33
