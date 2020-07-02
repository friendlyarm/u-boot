#ifndef _NANOPI_BOARDTYPE_H
#define _NANOPI_BOARDTYPE_H

#if defined(CONFIG_MACH_SUN8I_H3_NANOPI) || defined(CONFIG_MACH_SUN50I_H5_NANOPI)
#define CPU_TYPE_H2_1    (0x42)
#define CPU_TYPE_H2_2    (0x83)
#define CPU_TYPE_H3_1    (0x00)
#define CPU_TYPE_H3_2    (0x81)
#define CPU_TYPE_H3D     (0x58)
#define CPU_TYPE_H5      (0x01)            // H5-CPUID=0x01? NOT reliable

#define BOARD_NAME_LENGTH                 (32)

#define BOARD_TYPE_H3_VARIANT0            (0) // = M1
#define BOARD_TYPE_H3_VARIANT1            (1) // = NEO
#define BOARD_TYPE_H3_VARIANT2            (2) // = NEO-AIR
#define BOARD_TYPE_H3_VARIANT3            (3) // = M1-PLUS

#define BOARD_TYPE_NANOPI_M1              (0)
#define BOARD_TYPE_NANOPI_NEO             (1)
#define BOARD_TYPE_NANOPI_NEO_AIR         (2)
#define BOARD_TYPE_NANOPI_M1_PLUS         (3)
#define BOARD_TYPE_NANOPI_DUO             (4)
#define BOARD_TYPE_NANOPI_NEO_CORE        (5)
#define BOARD_TYPE_NANOPI_K1              (6)
#define BOARD_TYPE_NANOPI_HERO            (7)
#define BOARD_TYPE_NANOPI_DUO2            (8)
#define BOARD_TYPE_NANOPI_R1              (9)
#define BOARD_TYPE_NANOPI_NEO_S           (10)
#define BOARD_TYPE_ZEROPI                 (11)
#define BOARD_TYPE_NANOPI_R1S_H3          (12)
#define BOARD_TYPE_NANOPI_NEO_PLUS2_H3    (13)


#define BOARD_TYPE_H5_VARIANT0            (0) // = NEO-CORE2
#define BOARD_TYPE_H5_VARIANT1            (1) // = NEO2
#define BOARD_TYPE_H5_VARIANT2            (2) // = NEO-PLUS2
#define BOARD_TYPE_H5_VARIANT3            (3) // = M1-PLUS2

#define BOARD_TYPE_NANOPI_NEO_CORE2       (0)       // 00
#define BOARD_TYPE_NANOPI_NEO2            (1)       // 01
#define BOARD_TYPE_NANOPI_NEO_PLUS2       (2)       // 10
#define BOARD_TYPE_NANOPI_M1_PLUS2        (3)       // 11
#define BOARD_TYPE_NANOPI_K1_PLUS         (4)       // 11-(PG12=0)
#define BOARD_TYPE_NANOPI_NEO2_V11        (5)       // 01-(PL3=0)
#define BOARD_TYPE_NANOPI_NEO2_BLACK      (6)
#define BOARD_TYPE_NANOPI_R1S_H5          (7)

#if defined(CONFIG_MACH_SUN8I_H3_NANOPI)
#define BOARD_TYPE_MAX                    (BOARD_TYPE_NANOPI_NEO_PLUS2_H3+1)
#elif defined(CONFIG_MACH_SUN50I_H5_NANOPI)
#define BOARD_TYPE_MAX                    (BOARD_TYPE_NANOPI_R1S_H5+1)
#endif


int nanopi_get_board(void);
extern char nanopi_board[][32];
#endif

#endif
