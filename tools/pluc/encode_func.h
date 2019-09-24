#include "fsm.h"
#include "code_list.h"

int encode_Fan_In(fsm_type fsm );
int encode_IC_Relaxe(fsm_type fsm);
int encode_IC_All( fsm_type fsm );


int encode_BuildMachineWithFF(fsm_type fsm, char * ffType, MIN_SYMB_LIST constrList,
  CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode);
int encode_BuildMachine(fsm_type fsm, MIN_SYMB_LIST constrList,
  CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode);
int encode_BuildMachineToggleFF(fsm_type fsm, MIN_SYMB_LIST constrList,
  CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode);
