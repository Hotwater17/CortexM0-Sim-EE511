/*********************************************************
 *                                                       *
 *  EE511 Project 1                                      *
 *                                                       *
 *  C simulator for Cortex-M0                            *
 *                                                       *
 *********************************************************/


#include "iss.h"

void b_unconditional(uint16_t inst);
void bl(uint32_t inst);

void process(uint16_t inst)
{
  uint16_t inst2;
  uint32_t inst32;

  if (INST(15,14) == INST_IMM)
  {
    switch(INST(13,11))
    {
      case IMM_LSL : lsl_imm(inst); break;
      case IMM_LSR : lsr_imm(inst); break;
      case IMM_ASR : asr_imm(inst); break;
      case IMM_3BIT : {
        if(INST(10,9) == 0b00) add_reg1(inst);
        else if(INST(10,9) == 0b01) sub_reg1(inst);
        else if(INST(10,9) == IMM_ADD3) add_imm3(inst);
        else if(INST(10,9) == IMM_SUB3) sub_imm3(inst);

        break;
        }
      case IMM_MOV : mov_imm(inst); break;
      case IMM_CMP : cmp_imm(inst); break;
      case IMM_ADD8 : add_imm8(inst); break;
      case IMM_SUB8 : sub_imm8(inst); break;
    }
  }
  else if(INST(15,9) == REG_ADD1) add_reg1(inst);
  else if(INST(15,9) == REG_SUB1) sub_reg1(inst);
  else if(INST(15,10) == INST_REG) 
  {
    switch(INST(9,6))
    {
      case REG_AND1 : and_reg1(inst); break;
      case REG_EOR : eor_reg(inst); break;
      case REG_LSL : lsl_reg(inst); break;
      case REG_LSR : lsr_reg(inst); break;
      case REG_ASR : asr_reg(inst); break;
      case REG_ADC : adc_reg(inst); break;
      case REG_SBC : sbc_reg(inst); break;
      case REG_ROR : ror_reg(inst); break;
      case REG_TST : tst_reg(inst); break;
      case REG_RSB : rsb_imm(inst); break;
      case REG_CMP1 : cmp_reg1(inst); break;
      case REG_CMN : cmn_reg(inst); break;
      case REG_ORR : orr_reg(inst); break;
      case REG_MUL : mul_reg(inst); break;
      case REG_BIC : bic_reg(inst); break;
      case REG_MVN : mvn_reg(inst); break;
      default : break;
    }
  }
  else if(INST(15,10) == INST_REG2)
  {
    switch(INST(9,8))
    {
      case REG_ADD2 : add_reg2(inst); break;
      case REG_CMP2 : cmp_reg2(inst); break;
      case REG_MOV : mov_reg2(inst); break;
      default : break;
    }
  }
  else if(INST(15,11) == INST_LS_PCREL) ldr_pcrel(inst);
  else if(INST(15,11) == INST_LDM) ldm(inst);
  else if(INST(15,11) == INST_STM) stm(inst);
  else if(INST(15,12) == INST_LS_REG)
  {
    switch(INST(11,9))
    {
      case LS_REG_STR : str_reg(inst); break;
      case LS_REG_STRH : strh_reg(inst); break;
      case LS_REG_STRB : strb_reg(inst); break;
      case LS_REG_LDRSB : ldrsb_reg(inst); break;
      case LS_REG_LDR : ldr_reg(inst); break;
      case LS_REG_LDRH : ldrh_reg(inst); break;
      case LS_REG_LDRB : ldrb_reg(inst); break;
      case LS_REG_LDRSH : ldrsh_reg(inst); break;
      default : break;
    }
  }
  else if(INST(15,13) == INST_LS_IMM5)
  {
    switch(INST(12,11))
    {
      case LS_IMM5_STR_STRH : str_imm5(inst); break;
      case LS_IMM5_LDR_LDRH : ldr_imm5(inst); break;
      case LS_IMM5_STRB : strb_imm5(inst); break;
      case LS_IMM5_LDRB : ldrb_imm5(inst); break;
      default : break;
    }
  }
  else if(INST(15,13) == INST_LS_IMM8)
  {
    switch(INST(12,11))
    {
      case LS_IMM5_STR_STRH : strh_imm5(inst); break;
      case LS_IMM5_LDR_LDRH : ldrh_imm5(inst); break;
      case LS_IMM8_STR : str_imm8(inst); break;
      case LS_IMM8_LDR : ldr_imm8(inst); break;
      default : break;
    }
  }
  else if(INST(15,11) == INST_ADR) adr(inst);
  else if(INST(15,11) == INST_ADD_SP_IMM8) add_sp_imm8(inst);
  else if(INST(15,12) == INST_SP_MISC)
  {
    if(INST(11,7) == SP_ADD_IMM7) add_sp_imm7(inst);
    else if(INST(11,7) == SP_SUB_IMM7) sub_sp_imm7(inst);
    else if(INST(11,8) == 0x2)
    {
      switch(INST(7,6))
      {
        case 0b00 : sxth(inst); break; 
        case 0b01 : sxtb(inst); break;
        case 0b10 : uxth(inst); break;
        case 0b11 : uxtb(inst); break;
      }
    }
    else if(INST(11,8) == 0b1111)
    {
      switch(INST(7,0))
      {
        case 0x00 : nop(inst); break;
        case 0x10 : yield(inst); break;
        case 0x20 : wfe(inst); break;
        case 0x30 : wfi(inst); break;
        case 0x40 : sev(inst); break;
        default : break;
      }
    }
    else
    {
      switch(INST(11,9))
      {
        case 0b010 : push(inst); break; 
        case 0b011 : if((INST(8,5) == 0b0011) && (INST(3,0) == 0b0010)) cps(inst); break;
        case 0b101 : {
          switch(INST(8,6))
          {
            case 0b000 : rev(inst); break;
            case 0b001 : rev16(inst); break;
            case 0b010 : revsh(inst); break;
            default : break;
          }
          break;
        }
        case 0b110 : pop(inst); break;
        case 0b111 : if(INST_(8)) bkpt(inst); break;
        default : break;
      }
    }

  }
  else if(INST(15, 12) == INST_B_SVC)
  {
    if(INST(11,8) == 0xF) svc(inst);
    else b_conditional(inst);
  }
  else if(INST(15, 8) == INST_BX)
  {
    if(INST_(7)) blx(inst);
    else bx(inst);
  } //END OF MY CODE
  else if (INST(15, 11) == 0x1C) {
    b_unconditional(inst);
  }
  else if (INST(15, 11) == 0x1E) {
    inst2 = read_halfword(EXE_PC + 2);
    inst32 = ((uint32_t) inst << 16) | ((uint32_t) inst2);
    if (extract16_(inst2, 14) && extract16_(inst2, 12))
      bl(inst32);
  }
}

    /* your code here (for additional functions)*/
//20204787 EE511 Fall 2021 Michal Gorywoda
/*
  Auxiliary function to perform signed/unsigned addition and subtraction with carry
*/
alu_result_t add_with_carry(uint32_t x, uint32_t y, uint32_t carryIn)
{
  alu_result_t output;
  uint64_t unsigned_sum = (uint64_t)x +  (uint64_t)y +  (uint64_t)carryIn; 
  uint64_t signed_sum =  (int32_t)x + (int32_t)y + carryIn; 
  output.result = (int32_t)unsigned_sum;
  if((uint64_t) output.result == unsigned_sum) output.carry = 0;
  else output.carry = 1;
  if((int32_t)output.result == signed_sum) output.overflow = 0;
  else output.overflow = 1;
  return output;
}

/*
  Auxiliary function to perform logical shift to the left
*/
alu_result_t logic_shift_left(uint32_t x, uint32_t shift)
{
  
  alu_result_t output; 
  output.overflow = 0;
  assert(shift >= 0);
  if(shift == 0)
  {
    output.result = x;
    output.carry = 0;
  }
  else
  {
    output.result = zeroExtend32(x) << shift;
    output.carry = extract32_(x, (32-shift));
  } 

  return output;
}

/*
  Auxiliary function to perform logical shift to the right
*/
alu_result_t logic_shift_right(uint32_t x, uint32_t shift)
{
  alu_result_t output;
  output.overflow = 0;
  assert(shift >= 0);
  if(shift == 0)
  {
    output.result = x;
    output.carry = 0;
  }
  else
  {
    output.result = zeroExtend32(x) >> shift;
    output.carry = extract32_(x, (shift-1)); 
  }

  return output;  
}

/*
  Auxiliary function to perform arithmetic shift to the right
*/
alu_result_t arithmetic_shift_right(uint32_t x, uint32_t shift)
{
  alu_result_t output;
  output.overflow = 0;
  assert(shift >= 0);
  if(shift == 0)
  {
    output.result = x;
    output.carry = 0;    
  }
  else
  {
    output.result = (signExtend32(x, 32) >>shift);
    output.carry = extract32_(output.result, (shift-1)); 
  }

  return output;  
}

/*
  Auxiliary function to perform right rotation
*/
alu_result_t rotate_right(uint32_t x, uint32_t shift)
{
  alu_result_t output, leftShift, rightShift;
  uint32_t shiftMod;
  output.overflow = 0;
  assert(shift >= 0);
  if(shift == 0)
  {
    output.result = x;
    output.carry = 0;  
  }
  else
  {
    shiftMod = shift % 32;
    leftShift = logic_shift_left(x, 32-shiftMod);
    rightShift = logic_shift_right(x, shiftMod);
    output.result =  rightShift.result | leftShift.result;
    output.carry = rightShift.carry | leftShift.carry;
  }

  return output;    
}

/*
  Auxiliary function to count bits set to 1 in data field
*/
uint32_t bit_count(uint32_t x)
{
  uint32_t cnt = 0;
  for(uint32_t i = 0; i < 32; i++)
  {
    if(x & (1<<i)) cnt++;
  }

  return cnt;
}

//Branch

/*
  Conditional branch, jumps to given address if a boolean expression of flags is true.
  Otherwise no action is taken
*/
void b_conditional(uint16_t inst)
{
  uint32_t imm8 = INST(7,0);
  uint32_t addr = PC + signExtend32((imm8 << 1),9);

  uint32_t cond = INST(11,8);

  switch(cond)
  {

    case COND_EQ : branch = APSR.Z; break;
    case COND_NE : branch = ~APSR.Z; break;
    case COND_CS : branch = APSR.C; break;
    case COND_CC : branch = ~APSR.C; break;
    case COND_MI : branch = APSR.N; break;
    case COND_PL : branch = ~APSR.N; break;
    case COND_VS : branch = APSR.V; break;
    case COND_VC : branch = ~APSR.V; break;
    case COND_HI : branch = (APSR.C && ~APSR.Z); break;
    case COND_LS : branch = (~APSR.C || APSR.Z); break;
    case COND_GE : branch = (APSR.N == APSR.V); break;
    case COND_LT : branch = (APSR.N != APSR.V); break; 
    case COND_GT : branch = (~APSR.Z && (APSR.N == APSR.V)); break;
    case COND_LE : branch = (APSR.Z || (APSR.N != APSR.V)); break;
    case COND_AL : branch = 1; break;
    default      : branch = 0;
  }

  if(branch) PC = addr & 0xFFFFFFFE;


}

/*
  Unconditional branch with saving the next instruction address to LR and exchanging instruction set
*/
void blx(uint16_t inst)
{
  uint32_t m = INST(6,3);
  uint32_t addr;
  assert(m != 15);
  
  branch = 1;
  addr = R[m] & 0xFFFFFFFE;
  LR = (PC-2) | 0x00000001;
  PC = addr;
}

/*
  Unconditional branch with exchanging instruction set
*/
void bx(uint16_t inst)
{
  uint32_t m = INST(6,3);
  uint32_t addr;
  assert(m != 15);
  
  branch = 1;
  addr = R[m] & 0xFFFFFFFE;
  PC = addr;
}

/*
  Supervisor call - UNUSED
*/
void svc(uint16_t inst)
{
 printf("UNUSED SVC \r \n");
}

//Immediate data processing

/*
  Logical left shift Rm by 5-bit immediate value, store in Rd
*/
void lsl_imm(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t imm5 = INST(10,6);

  alu_result_t shift_op;  
  shift_op = logic_shift_left(R[m], imm5);

  R[d] = shift_op.result;

  APSR.N = GET_NEGATIVE(shift_op.result); 
  APSR.Z = GET_ZERO(shift_op.result);
  APSR.C = shift_op.carry;
  APSR.V = shift_op.overflow;

}

/*
  Logical right shift Rm by 5-bit immediate value, store in Rd
*/
void lsr_imm(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t imm5 = INST(10,6);

  alu_result_t shift_op;

  shift_op = logic_shift_right(R[m], imm5);

  R[d] = shift_op.result;
  
  APSR.N = GET_NEGATIVE(shift_op.result); 
  APSR.Z = GET_ZERO(shift_op.result);
  APSR.C = shift_op.carry;
  APSR.V = shift_op.overflow;
}

/*
  Arithmetic right shift Rm by 5-bit immediate value, store in Rd
*/
void asr_imm(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t imm5 = INST(10,6);

  alu_result_t shift_op;

  shift_op = arithmetic_shift_right(R[m], imm5);

  R[d] = shift_op.result;
  
  APSR.N = GET_NEGATIVE(shift_op.result); 
  APSR.Z = GET_ZERO(shift_op.result);
  APSR.C = shift_op.carry;
  APSR.V = shift_op.overflow;
}

/*
  Add 3-bit immediate value to Rn, store in Rd
*/
void add_imm3(uint16_t inst)
{
  uint32_t n = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t imm3 = INST(8,6);
  uint32_t imm32 = zeroExtend32(imm3);
  alu_result_t add_op;
  
  add_op = add_with_carry(R[n], imm32, 0);
  
  R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow;
}

/*
  Add 8-bit immediate value to Rdn, store in Rdn
*/
void add_imm8(uint16_t inst)
{
  uint32_t n = INST(10,8);
  uint32_t d = n;
  uint32_t imm8 = INST(7,0);
  uint32_t imm32 = zeroExtend32(imm8);
  alu_result_t add_op;
  
  add_op = add_with_carry(R[n], imm32, 0);
  R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow;
}

/*
  Sub 3-bit immediate value from Rn, store in Rd
*/
void sub_imm3(uint16_t inst)
{
  uint32_t n = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t imm3 = INST(8,6);
  uint32_t imm32 = zeroExtend32(imm3);
  alu_result_t add_op;
  
  add_op = add_with_carry(R[n], ~imm32, 1);
  
  R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow;
}

/*
  Sub 8-bit immediate value from Rdn, store in Rdn
*/
void sub_imm8(uint16_t inst)
{
  uint32_t n = INST(10,8);
  uint32_t d = n;
  uint32_t imm8 = INST(7,0);
  uint32_t imm32 = zeroExtend32(imm8);
  alu_result_t add_op;
  
  add_op = add_with_carry(R[n], ~imm32, 1);
  
  R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow;
}

/*
  Move 8-bit immediate value to Rd
*/
void mov_imm(uint16_t inst)
{
  uint32_t d = INST(10,8);
  uint32_t imm8 = INST(7,0);
  uint32_t imm32 = zeroExtend32(imm8);
  uint32_t result, carry;

  result = imm32;
  R[d] = result;
  carry = APSR.C;

  APSR.N = GET_NEGATIVE(result); 
  APSR.Z = GET_ZERO(result);
  APSR.C = carry; //Carry flag remains the same
  //APSR.V unchanged
}

/*
  Compare value from Rn to 8-bit immediate
*/
void cmp_imm(uint16_t inst)
{

  uint32_t n = INST(10,8);
  uint32_t imm8 = INST(7,0);
  uint32_t imm32 = zeroExtend32(imm8);
  alu_result_t add_op;
  
  add_op = add_with_carry(R[n], ~imm32, 1);

  APSR.N = GET_NEGATIVE(add_op.result);
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry; 
  APSR.V = add_op.overflow;
}

//Register data processing

/*
  Add Rm value to Rn, store in Rd, m and n can not be PC at the same time
*/
void add_reg1(uint16_t inst)
{
  uint32_t m = INST(8,6);
  uint32_t n = INST(5,3);
  uint32_t d = INST(2,0);
  alu_result_t add_op;
  assert(!((n == m) && (n == 15)));
  add_op = add_with_carry(R[n], R[m], 0);
  
  R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow;
}

/*
  Sub Rm value from Rn, store in Rd
*/
void sub_reg1(uint16_t inst)
{
  uint32_t m = INST(8,7);
  uint32_t n = INST(5,3);
  uint32_t d = INST(2,0);
  alu_result_t add_op;
  
  add_op = add_with_carry(R[n], ~R[m], 1);
  
  R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow;
}

/*
  Logical AND Rm value to Rdn, store in Rdn
*/
void and_reg1(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;
  uint32_t result;
  
  result = R[n] & R[m];
  R[d] = result;
 
  APSR.N = GET_NEGATIVE(result); 
  APSR.Z = GET_ZERO(result);
  //Carry unchanged
  //Overflow unchanged
}

/*
  Logical XOR Rm value to Rdn, store in Rdn
*/
void eor_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;
  uint32_t result;
  
  
  result = R[n] ^ R[m];
  R[d] = result;
 
  APSR.N = GET_NEGATIVE(result); 
  APSR.Z = GET_ZERO(result);
  //APSR.C = add_op.carry; //Carry unchanged
  //APSR.V = add_op.overflow; //Overflow unchanged  
}

/*
  Logical shift left Rdn by Rm, store in Rdn
*/
void lsl_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;
  alu_result_t shift_op;

  shift_op = logic_shift_left(R[n], (R[m] & 0x000000FF));

  R[d] = shift_op.result;
  
  APSR.N = GET_NEGATIVE(shift_op.result); 
  APSR.Z = GET_ZERO(shift_op.result);
  APSR.C = shift_op.carry;
}

/*
  Logical shift right Rdn by Rm, store in Rdn
*/
void lsr_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;

  alu_result_t shift_op;

  shift_op = logic_shift_right(R[n], (R[m] & 0x000000FF));

  R[d] = shift_op.result;
  
  APSR.N = GET_NEGATIVE(shift_op.result); 
  APSR.Z = GET_ZERO(shift_op.result);
  APSR.C = shift_op.carry;
}

/*
  Arithmetic shift left Rdn by Rm, store in Rdn
*/
void asr_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;

  alu_result_t shift_op;

  shift_op = arithmetic_shift_right(R[n], (R[m] & 0x000000FF));

  R[d] = shift_op.result;
  
  APSR.N = GET_NEGATIVE(shift_op.result); 
  APSR.Z = GET_ZERO(shift_op.result);
  APSR.C = shift_op.carry;
}

/*
  Add Rm with carry to Rdn, store in Rdn
*/
void adc_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;
  alu_result_t add_op;
  
  add_op = add_with_carry(R[n], R[m], APSR.C);
  
  R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow;  
}

/*
  Subtract Rm with borrow from Rdn, store in Rdn
*/
void sbc_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;
  alu_result_t add_op;
  
  add_op = add_with_carry(R[n], ~R[m], APSR.C);
  
  R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow; 
}

/*
  Rotate right Rdn by Rm, store in Rdn
*/
void ror_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;

  alu_result_t shift_op;

  shift_op = rotate_right(R[n], (R[m] & 0x000000FF));

  R[d] = shift_op.result;
  
  APSR.N = GET_NEGATIVE(shift_op.result); 
  APSR.Z = GET_ZERO(shift_op.result);
  APSR.C = shift_op.carry;  
}

/*
  Test negative and zero conditions in Rn and Rm, discard result
*/
void tst_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t result;
  
  result = R[n] & R[m];
 
  APSR.N = GET_NEGATIVE(result); 
  APSR.Z = GET_ZERO(result);

}

/*
  Reverse subtract Rn from 0, store in Rd
*/
void rsb_imm(uint16_t inst)
{
  uint32_t n = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t imm32 = 0x00000000;
  alu_result_t add_op;
  
  add_op = add_with_carry(~R[n], imm32, 1);
  
  R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow;
}

/*
  Compare Rm and Rn, update flags
*/
void cmp_reg1(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;
  alu_result_t add_op;
  add_op = add_with_carry(R[n], ~R[m], 1);

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow; 
}

/*
  Compare negative Rn and Rm, update flags
*/
void cmn_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  alu_result_t add_op;
  
  add_op = add_with_carry(R[n], R[m], 0);

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow; 
}

/*
  Logical OR Rdn with Rm, store result in Rdn
*/
void orr_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;
  uint32_t result;
  
  
  result = R[n] | R[m];
  R[d] = result;
 
  APSR.N = GET_NEGATIVE(result); 
  APSR.Z = GET_ZERO(result);  
}

/*
  Multiply Rdn by Rm, store result in Rdn
*/
void mul_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;
  uint32_t result;
  uint32_t op1 = (int32_t) R[n];
  uint32_t op2 = (int32_t) R[m];

  result = op1*op2;
  R[d] = result;

  APSR.N = GET_NEGATIVE(result); 
  APSR.Z = GET_ZERO(result); 
}

/*
  Clear Rdn bits specified in Rm, store result in Rdn 
*/
void bic_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t n = INST(2,0);
  uint32_t d = n;
  uint32_t result;


  result = R[n] & ~R[m];
  R[d] = result;

  APSR.N = GET_NEGATIVE(result); 
  APSR.Z = GET_ZERO(result);   
}

/*
  Move negative Rm to Rd
*/
void mvn_reg(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t result;


  result = ~R[m];
  R[d] = result;

  APSR.N = GET_NEGATIVE(result); 
  APSR.Z = GET_ZERO(result);   
}

/*
  Add Rm to Rdn, check if Rdn or Rm point to SP. Rm and Rdn can not point to PC at the same time
  This function simulates add reg, add SP plus reg T1 and add SP plus reg T2
*/
void add_reg2(uint16_t inst)
{
  uint32_t m = INST(6,3);
  uint32_t DN = INST_(7);
  uint32_t n = (DN << 3) | INST(2,0);
  uint32_t d = n;
  assert(!((n == m) && (n == 15)));
  alu_result_t add_op;
  
  //Check if ADD SP plus register
  if((n == 13) != (m == 13))
  {
    //T1 encoding
    if(m == 13)
    {
      //Change register order
      m = n;
      d = m;
      
    }
    //T2 encoding
    else
    {
      d = 13;
    }
    add_op = add_with_carry(SP, R[m], 0);
    if(d == 15) PC = (add_op.result & 0xFFFFFFFE);
    else R[d] = add_op.result;
    
  }
  //ADD register
  else
  {
    add_op = add_with_carry(R[n], R[m], 0);
    
    

    if(d == 15) PC = (add_op.result & 0xFFFFFFFE);
    else
    {
      R[d] = add_op.result;
      APSR.N = GET_NEGATIVE(add_op.result); 
      APSR.Z = GET_ZERO(add_op.result);
      APSR.C = add_op.carry;
      APSR.V = add_op.overflow;
    }
  } 



}

/*
  Compare Rn and Rm. Rn and Rm can not both be lower registers at the same time and none of them can point to PC
*/
void cmp_reg2(uint16_t inst)
{
  uint32_t m = INST(6,3);
  uint32_t N = INST_(7);
  uint32_t n = (N << 3) | INST(2,0);
  assert(!((n < 8 ) && (m < 8)));
  assert((n != 15) && (m != 15));
  alu_result_t add_op;
  
  
  add_op = add_with_carry(R[n], ~R[m], 1);

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
  APSR.C = add_op.carry;
  APSR.V = add_op.overflow;

}

/*
  Move Rm to Rd. If Rd points to PC, write halfword adjusted address
*/
void mov_reg2(uint16_t inst)
{
  uint32_t m = INST(6,3);
  uint32_t D = INST_(7);
  uint32_t d = (D << 3) | INST(2,0);
  alu_result_t add_op;
  
  add_op.result = R[m];
  
  if(d == 15) PC = (add_op.result & 0xFFFFFFFE);
  else R[d] = add_op.result;

  APSR.N = GET_NEGATIVE(add_op.result); 
  APSR.Z = GET_ZERO(add_op.result);
}


//Load and Store

/*
  Load register relative to address relative to PC by 8-bit offset
*/
void ldr_pcrel(uint16_t inst)
{
  uint32_t t = INST(10,8);
  uint32_t imm8 = INST(7,0);
  uint32_t imm32 = zeroExtend32(imm8<<2);
  uint32_t base = PC & 0xFFFFFFFC;
  uint32_t addr = base + imm32;;
  
  
  R[t] = read_word(addr);
 
}

/*
  Store multiple. 8-bit immediate value is a list of registers whose value will be stored.
  Rn is the base address for writes. Address is incremented after each write.
*/
void stm(uint16_t inst)
{
  uint32_t n = INST(10,8);
  uint32_t regList = INST(7,0);
  uint32_t addr = R[n];
  uint32_t lowestSetBit = 0;
  assert(bit_count(regList) >= 1);
  for(lowestSetBit = 0; lowestSetBit < 15; lowestSetBit++)
  {
    //Check for position of rightmost set bit in regList
    if(regList & (1<<lowestSetBit)) break;
  }

  for(uint32_t i = 0; i < 15; i++)
  {
    if(regList & (1<<i))
    {
      if((i == n) && (i != lowestSetBit)) write_word(addr, 0);
      else write_word(addr, R[i]);
    }
    addr = addr + 4;
  }

  R[n] = R[n] + 4*bit_count(regList);
}

/*
  Load multiple. 8-bit immediate value is a list of registers to which the memory contents will be written.
  Rn is the base address for reads. Address is incremented after each read.
*/
void ldm(uint16_t inst)
{
  
  uint32_t n = INST(10,8);
  uint32_t regList = INST(7,0);
  uint32_t addr = R[n];
  uint32_t wback = ~(regList & (1<<n));
  assert(bit_count(regList) >= 1);
  for(uint32_t i = 0; i < 8; i++)
  {
    if(regList & (1<<i))
    {
      R[i] = read_word(addr);
      addr = addr + 4;
    }

    if(!(wback && (regList & (1<<n)))) R[n] = R[n] + 4*bit_count(regList);

  }
}

/*
  Store word from Rt to memory location with address Rn(base) + Rm(offset)
*/
void str_reg(uint16_t inst)
{
  uint32_t m = INST(8,6);
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t addr = R[n]+ R[m];
  
  write_word(addr, R[t]); 

}

/*
  Store lower halfword from Rt to memory location with address Rn(base) + Rm(offset)
*/
void strh_reg(uint16_t inst)
{
  uint32_t m = INST(8,6);
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t addr = R[n]+ R[m];
  
  write_halfword(addr, (R[t] & 0x0000FFFF));   

}

/*
  Store lower byte from Rt to memory location with address Rn(base) + Rm(offset)
*/
void strb_reg(uint16_t inst)
{
  uint32_t m = INST(8,6);
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t addr = R[n]+ R[m];
  
  write_byte(addr, (R[t] & 0x000000FF));   
}

/*
  Load signed lower byte to Rt from memory location with address Rn(base) + Rm(offset)
*/
void ldrsb_reg(uint16_t inst)
{
  uint32_t m = INST(8,6);
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t addr = R[n]+ R[m];

  R[t] = signExtend32(read_byte(addr), 8);
}

/*
  Load word to Rt from memory location with address Rn(base) + Rm(offset)
*/
void ldr_reg(uint16_t inst)
{
  uint32_t m = INST(8,6);
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t addr = R[n]+ R[m];

  R[t] = read_word(addr);

}

/*
  Load lower halfword to Rt from memory location with address Rn(base) + Rm(offset)
*/
void ldrh_reg(uint16_t inst)
{
  uint32_t m = INST(8,6);
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t addr = R[n]+ R[m];

  R[t] = zeroExtend32(read_halfword(addr));  
}

/*
  Load lower byte to Rt from memory location with address Rn(base) + Rm(offset)
*/
void ldrb_reg(uint16_t inst)
{
  uint32_t m = INST(8,6);
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t addr = R[n]+ R[m];

  R[t] = zeroExtend32(read_byte(addr));  

}

/*
  Load lower signed halfword to Rt from memory location with address Rn(base) + Rm(offset)
*/
void ldrsh_reg(uint16_t inst)
{
  uint32_t m = INST(8,6);
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t addr = R[n]+ R[m];

  R[t] = signExtend32(read_halfword(addr), 16);
}

/*
  Store word from Rt to memory location with address Rn(base) + 5-bit offset
*/
void str_imm5(uint16_t inst)
{
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t imm5 = INST(10,6);
  uint32_t imm32 = zeroExtend32(imm5<<2);
  uint32_t addr = R[n]+ imm32;
  
  write_word(addr, R[t]); 
}

/*
  Load word to Rt from memory location with address Rn(base) + 5-bit offset
*/
void ldr_imm5(uint16_t inst)
{
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t imm5 = INST(10,6);
  uint32_t imm32 = zeroExtend32(imm5<<2);
  uint32_t addr = R[n]+ imm32;

  R[t] = read_word(addr);  
}

/*
  Store lower byte from Rt to memory location with address Rn(base) + 5-bit offset
*/
void strb_imm5(uint16_t inst)
{
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t imm5 = INST(10,6);
  uint32_t imm32 = zeroExtend32(imm5);
  uint32_t addr = R[n]+ imm32;
  
  write_byte(addr, (R[t] & 0x000000FF)); 
}

/*
  Load lower byte to Rt from memory location with address Rn(base) + 5-bit offset
*/
void ldrb_imm5(uint16_t inst)
{
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t imm5 = INST(10,6);
  uint32_t imm32 = zeroExtend32(imm5);
  uint32_t addr = R[n]+ imm32;

  R[t] = zeroExtend32(read_byte(addr));  
}

/*
  Store lower halfword from Rt to memory location with address Rn(base) + 5-bit offset
*/
void strh_imm5(uint16_t inst)
{
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t imm5 = INST(10,6);
  uint32_t imm32 = zeroExtend32(imm5<<1);
  uint32_t addr = R[n]+ imm32;
  
  write_halfword(addr, (R[t] & 0x0000FFFF)); 
}

/*
  Load lower halfword to Rt from memory location with address Rn(base) + 5-bit offset
*/
void ldrh_imm5(uint16_t inst)
{
  uint32_t n = INST(5,3);
  uint32_t t = INST(2,0);
  uint32_t imm5 = INST(10,6);
  uint32_t imm32 = zeroExtend32(imm5<<1);
  uint32_t addr = R[n]+ imm32;

  R[t] = zeroExtend32(read_halfword(addr));   
}

/*
  Store word from Rt to memory location with address SP + 8-bit offset
*/
void str_imm8(uint16_t inst)
{
  uint32_t n = 13; //SP
  uint32_t t = INST(10,8);
  uint32_t imm8 = INST(7,0);
  uint32_t imm32 = zeroExtend32(imm8<<2);
  uint32_t addr = R[n]+ imm32;

  write_word(addr, R[t]);
}

/*
  Load word to Rt from memory location with address SP + 8-bit offset
*/
void ldr_imm8(uint16_t inst)
{
  uint32_t n = 13; //SP
  uint32_t t = INST(10,8);
  uint32_t imm8 = INST(7,0);
  uint32_t imm32 = zeroExtend32(imm8<<2);
  uint32_t addr = R[n]+ imm32;

  R[t] = read_word(addr);  
}

//Stack pointer and misc

/*
  Add 8-bit immediate to PC, store result in Rd
*/
void adr(uint16_t inst)
{
  uint32_t d = INST(10,8);
  uint32_t imm8 = INST(7,0);
  uint32_t imm32 = zeroExtend32(imm8<<2);
  uint32_t result = (PC & 0xFFFFFFFC) + imm32;
  R[d] = result; 

}

/*
  Add 8-bit immediate to SP, store result in Rd
*/
void add_sp_imm8(uint16_t inst)
{
  uint32_t d = INST(10,8);
  uint32_t imm8 = INST(7,0);
  uint32_t imm32 = zeroExtend32(imm8<<2);
  alu_result_t add_op;

  add_op = add_with_carry(SP, imm32, 0);
  R[d] = add_op.result;

}

/*
  Add 7-bit immediate to SP, store result in SP
*/
void add_sp_imm7(uint16_t inst)
{
  uint32_t d = 13;
  uint32_t imm7 = INST(6,0);
  uint32_t imm32 = zeroExtend32(imm7<<2);
  alu_result_t add_op;

  add_op = add_with_carry(SP, imm32, 0);
  R[d] = add_op.result; 
}

/*
  Syb 7-bit immediate from SP, store result in SP
*/
void sub_sp_imm7(uint16_t inst)
{
  uint32_t d = 13;
  uint32_t imm7 = INST(6,0);
  uint32_t imm32 = zeroExtend32(imm7<<2);
  alu_result_t add_op;

  add_op = add_with_carry(SP, ~imm32, 1);
  R[d] = add_op.result;  
}

/*
  Sign extend halfword in Rm, store result in Rd
*/
void sxth(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);

  R[d] = signExtend32((R[m] & 0x0000FFFF), 16);
}

/*
  Sign extend byte in Rm, store result in Rd
*/
void sxtb(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);

  R[d] = signExtend32((R[m] & 0x000000FF), 8);  
}

/*
  Zero extend halfword in Rm, store result in Rd
*/
void uxth(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);

  R[d] = zeroExtend32(R[m] & 0x0000FFFF);  
}

/*
  Zero extend halfword in Rm, store result in Rd
*/
void uxtb(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);

  R[d] = zeroExtend32(R[m] & 0x000000FF);  
}

/*
  Push listed registers to stack. Register list must be greater than 0.
  Automatically shifts stack pointer towards memory start 
*/
void push(uint16_t inst)
{
  uint32_t regList = INST(7,0);
  uint32_t m = INST_(8);
  uint32_t registers = (m<<14) | regList;
  uint32_t addr = SP - 4*bit_count(registers); //Next empty cell of the stack
  assert(bit_count(regList) >= 1);
  
  for(uint32_t i = 0; i < 15; i++)
  {
    if(registers & (1<<i))
    {
      write_word(addr, R[i]);
      addr = addr + 4;
    }
  }
  SP = SP - 4*bit_count(registers); //increment stack  
}
/*
  Change processor state. Unused
*/
void cps(uint16_t inst)
{
 printf("UNUSED CPS \r \n");
}

/*
  Byte-reverse word order from Rm, store result in Rd
*/
void rev(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t result = 0;

  result |= ((R[m] & 0x000000FF) << 24);
  result |= ((R[m] & 0x0000FF00) << 8);
  result |= ((R[m] & 0x00FF0000) >> 8);
  result |= ((R[m] & 0xFF000000) >> 24);

  R[d] = result;

}

/*
  Byte-reverse halfword order from Rm, store result in Rd
*/
void rev16(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t result = 0;

  result |= ((R[m] & 0x00FF0000) << 8);
  result |= ((R[m] & 0xFF000000) >> 8);
  result |= ((R[m] & 0x000000FF) << 8);
  result |= ((R[m] & 0x0000FF00) >> 8);

  R[d] = result;
}

/*
  Byte-reverse signed halfword order from Rm, store result in Rd
*/
void revsh(uint16_t inst)
{
  uint32_t m = INST(5,3);
  uint32_t d = INST(2,0);
  uint32_t result = 0;

  result |= (signExtend32((R[m] & 0x000000FF), 8) << 8);
  result |= (R[m] & 0x0000FF00);

  R[d] = result;
}

/*
  Pop stack to listed registers. Register list must be greater than 0.
  Automatically shifts stack pointer towards memory end 
*/
void pop(uint16_t inst)
{
  uint32_t regList = INST(7,0);
  uint32_t p = INST_(8);
  uint32_t registers = (p<<15) | regList;
  uint32_t addr = SP; //Top of stack pointer
  assert(bit_count(regList) >= 1);

  for(uint32_t i = 0; i < 8; i++)
  {
    if(registers & (1<<i))
    {
      R[i] = read_word(addr);
      addr = addr + 4;
    }
  }
  if(registers & (1<<15))
  {
    //check thumb state, write valid address
    assert(read_word(addr) & (1<<0)); 
    PC = (read_word(addr) & 0xFFFFFFFE);
  }  

  SP = SP + 4*bit_count(registers); //decrement stack
}

/*
  Breakpoint. Unused
*/
void bkpt(uint16_t inst)
{
 printf("UNUSED BKPT \r \n");
}

/*
  No operation
*/
void nop(uint16_t inst)
{ 
  printf("NOP \r \n");
}

/*
  Yield hint. Unused
*/
void yield(uint16_t inst)
{
 printf("UNUSED YIELD \r \n");
}

/*
  Wait for event. Unused
*/
void wfe(uint16_t inst)
{
  printf("UNUSED WFE \r \n");
}

/*
  Wait for interrupt. Unused
*/
void wfi(uint16_t inst)
{
  printf("UNUSED WFI \r \n");
}

/*
  Send event hint. Unused
*/
void sev(uint16_t inst)
{
  printf("UNUSED SEV \r \n");
}

// END OF MY FUNCTIONS

    

void b_unconditional(uint16_t inst)
{
  uint32_t imm11 = INST(10, 0);
  uint32_t address;

  address = PC + signExtend32((imm11 << 1), 12);
  branch = 1;
  PC = address & 0xFFFFFFFE;
}
void bl(uint32_t inst)
{
  uint32_t S = INST32_(10 + 16);
  uint32_t imm10 = INST32(9 + 16, 0 + 16);
  uint32_t J1 = INST32_(13);
  uint32_t J2 = INST32_(11);
  uint32_t imm11 = INST32(10, 0);
  uint32_t I1, I2, imm32, address;
  
  I1 = !(J1 ^ S);
  I2 = !(J2 ^ S);
  imm32 = sign_extend((S << 24) | (I1 << 23) | (I2 << 22) | (imm10 << 12) | (imm11 << 1), 25);

  LR = PC | 0x00000001;

  address = PC + imm32;
  branch = 1;
  PC = address & 0xFFFFFFFE;
}

