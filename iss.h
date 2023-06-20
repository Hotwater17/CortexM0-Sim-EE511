/*********************************************************
 *                                                       *
 *  EE511 Project 1                                      *
 *                                                       *
 *  C simulator for Cortex-M0                            *
 *                                                       *
 *********************************************************/

#ifndef ISS_H
#define ISS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>


// Registers
uint32_t R[16];

struct APSR {
  int N;
  int Z;
  int C;
  int V;
} APSR;


#define SP R[13]
#define LR R[14]
#define PC R[15]

#define MEM_SIZE  0x4000000
uint8_t *mem;


// BRANCH should be set to 1 whenever branch occurs
int branch;
//instruction address for EXE pipeline stage(PC-4)
uint32_t EXE_PC;

uint16_t fetch(void);
void process(uint16_t inst);
void updatePC(void);

uint32_t extract32(uint32_t data, int end, int start);
uint32_t extract32_(uint32_t data, int pos);
uint16_t extract16(uint16_t data, int end, int start);
uint16_t extract16_(uint16_t data, int pos);
uint32_t sign_extend(uint32_t a, int length);

/* Shorthand for extracting bits in inst. */
#define INST(x, y) extract16(inst, x, y)
#define INST_(x) extract16_(inst, x)

#define INST32(x, y) extract32(inst, x, y)
#define INST32_(x) extract32_(inst, x)

#define MSB(x) extract32_(x, 31)

// Sign extension
#define zeroExtend32(x) (x)
#define signExtend32(x, n) (((((x) >> ((n)-1)) & 0x1) != 0) ? (~((unsigned int)0) << (n)) | (x) : (x))

/* Memory access functions. */
uint32_t read_word(uint32_t addr);
uint32_t read_halfword(uint32_t addr);
uint32_t read_byte(uint32_t addr);

void write_word(uint32_t addr, uint32_t value);
void write_halfword(uint32_t addr, uint32_t value);
void write_byte(uint32_t addr, uint32_t value);

/* your code here */
//20204787 EE511 Fall 2021 Michal Gorywoda
typedef struct alu_result_t
{
  uint32_t result;
  int carry, overflow;

}alu_result_t;


alu_result_t add_with_carry(uint32_t x, uint32_t y, uint32_t carryIn);

alu_result_t logic_shift_left(uint32_t x, uint32_t shift);
alu_result_t logic_shift_right(uint32_t x, uint32_t shift);
alu_result_t arithmetic_shift_right(uint32_t x, uint32_t shift);
alu_result_t rotate_right(uint32_t x, uint32_t shift);

uint32_t bit_count(uint32_t x); 

#define GET_NEGATIVE(x) MSB(x)
#define GET_ZERO(x) (x == 0x00000000)



// Branch

#define INST_B_SVC 0b1101
#define INST_BX 0b01000111

#define COND_EQ 0x0
#define COND_NE 0x1
#define COND_CS 0x2
#define COND_CC 0x3
#define COND_MI 0x4
#define COND_PL 0x5
#define COND_VS 0x6
#define COND_VC 0x7
#define COND_HI 0x8
#define COND_LS 0x9
#define COND_GE 0xA
#define COND_LT 0xB
#define COND_GT 0xC
#define COND_LE 0xD
#define COND_AL 0xE

void b_conditional(uint16_t inst);
void blx(uint16_t inst);
void bx(uint16_t inst);
void svc(uint16_t inst);


//Immediate data processing

#define INST_IMM 0b00

#define IMM_LSL 0b000
#define IMM_LSR 0b001
#define IMM_ASR 0b010
#define IMM_MOV 0b100
#define IMM_CMP 0b101
#define IMM_ADD8 0b110
#define IMM_SUB8 0b111

#define IMM_3BIT 0b011
#define IMM_ADD3 0b10
#define IMM_SUB3 0b11


void lsl_imm(uint16_t inst);
void lsr_imm(uint16_t inst);
void asr_imm(uint16_t inst);
void add_imm3(uint16_t inst);
void add_imm8(uint16_t inst);
void sub_imm3(uint16_t inst);
void sub_imm8(uint16_t inst);
void mov_imm(uint16_t inst);
void cmp_imm(uint16_t inst);

//Register data processing

#define REG_ADD1 0b0001100
#define REG_SUB1 0b0001101

void add_reg1(uint16_t inst);
void sub_reg1(uint16_t inst);

#define INST_REG 0b010000

#define REG_AND1 0b0000
#define REG_EOR 0b0001
#define REG_LSL 0b0010
#define REG_LSR 0b0011
#define REG_ASR 0b0100
#define REG_ADC 0b0101
#define REG_SBC 0b0110
#define REG_ROR 0b0111
#define REG_TST 0b1000
#define REG_RSB 0b1001
#define REG_CMP1 0b1010
#define REG_CMN 0b1011
#define REG_ORR 0b1100
#define REG_MUL 0b1101
#define REG_BIC 0b1110
#define REG_MVN 0b1111

#define INST_REG2 0b010001

#define REG_ADD2 0b00
#define REG_CMP2 0b01
#define REG_MOV 0b10

void and_reg1(uint16_t inst);
void eor_reg(uint16_t inst);
void lsl_reg(uint16_t inst);
void lsr_reg(uint16_t inst);
void asr_reg(uint16_t inst);
void adc_reg(uint16_t inst);
void sbc_reg(uint16_t inst);
void ror_reg(uint16_t inst);
void tst_reg(uint16_t inst);
void rsb_imm(uint16_t inst);
void cmp_reg1(uint16_t inst);
void cmn_reg(uint16_t inst);
void orr_reg(uint16_t inst);
void mul_reg(uint16_t inst);
void bic_reg(uint16_t inst);
void mvn_reg(uint16_t inst);

void add_reg2(uint16_t inst);
void cmp_reg2(uint16_t inst);
void mov_reg2(uint16_t inst);


//Load and Store

#define INST_LS_PCREL 0b01001
#define INST_LS_REG 0b0101
#define INST_LS_IMM5 0b011
#define INST_LS_IMM8 0b100
#define INST_LDM    0b11001
#define INST_STM    0b11000

#define LS_REG_STR 0b000
#define LS_REG_STRH 0b001
#define LS_REG_STRB 0b010
#define LS_REG_LDRSB 0b011
#define LS_REG_LDR 0b100
#define LS_REG_LDRH 0b101
#define LS_REG_LDRB 0b110
#define LS_REG_LDRSH 0b111

#define LS_IMM5_STR_STRH 0b00
#define LS_IMM5_LDR_LDRH 0b01
#define LS_IMM5_STRB 0b10
#define LS_IMM5_LDRB 0b11

#define LS_IMM8_STR 0b10
#define LS_IMM8_LDR 0b11



void ldr_pcrel(uint16_t inst);

void stm(uint16_t inst);
void ldm(uint16_t inst);

void str_reg(uint16_t inst);
void strh_reg(uint16_t inst);
void strb_reg(uint16_t inst);
void ldrsb_reg(uint16_t inst);
void ldr_reg(uint16_t inst);
void ldrh_reg(uint16_t inst);
void ldrb_reg(uint16_t inst);
void ldrsh_reg(uint16_t inst);

void str_imm5(uint16_t inst);
void ldr_imm5(uint16_t inst);
void strb_imm5(uint16_t inst);
void ldrb_imm5(uint16_t inst);
void strh_imm5(uint16_t inst);
void ldrh_imm5(uint16_t inst);

void str_imm8(uint16_t inst);
void ldr_imm8(uint16_t inst);

//Stack pointer and misc

#define INST_ADR 0b10100
#define INST_ADD_SP_IMM8 0b10101

#define INST_SP_MISC 0b1011

#define SP_ADD_IMM7 0b00000
#define SP_SUB_IMM7 0b00001

#define SP_SXTH 0b001000
#define SP_SXTB 0b001001
#define SP_UXTH 0b001010
#define SP_UXTB 0b001011

#define SP_PUSH 0b010
#define SP_CPS 0b0110
#define SP_REV 0b101000
#define SP_REV16 0b101001
#define SP_REVSH 0b101011
#define SP_POP 0b110
#define SP_BKPT 0b1110
#define SP_NOP 0x00
#define SP_YIELD 0x10
#define SP_WFE 0x20
#define SP_WFI 0x30
#define SP_SEV 0x40

void adr(uint16_t inst);
void add_sp_imm8(uint16_t inst);

void add_sp_imm7(uint16_t inst);
void sub_sp_imm7(uint16_t inst);
void sxth(uint16_t inst);
void sxtb(uint16_t inst);
void uxth(uint16_t inst);
void uxtb(uint16_t inst);
void push(uint16_t inst);
void cps(uint16_t inst);
void rev(uint16_t inst);
void rev16(uint16_t inst);
void revsh(uint16_t inst);
void pop(uint16_t inst);
void bkpt(uint16_t inst);
void nop(uint16_t inst);
void yield(uint16_t inst);
void wfe(uint16_t inst);
void wfi(uint16_t inst);
void sev(uint16_t inst);




//Status register access
#endif
