#include "project.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>


/* Macro Definitions */

#define compute_INV(result,val) \
  { \
    switch (val) { \
    case LOGIC_0: \
      result = LOGIC_1; \
      break; \
    case LOGIC_1: \
      result = LOGIC_0; \
      break; \
    case LOGIC_X: \
      result = LOGIC_X; \
      break; \
    default: \
      assert(0); \
    } \
  }

#define compute_AND(result,val1,val2) \
    switch (val1) { \
    case LOGIC_0: \
      result = LOGIC_0; \
      break; \
    case LOGIC_1: \
      result = val2; \
      break; \
    case LOGIC_X: \
      if ( val2 == LOGIC_0 ) result = LOGIC_0; \
      else result = LOGIC_X; \
      break; \
    default: \
      assert(0); \
    } 

#define compute_OR(result,val1,val2) \
  { \
    switch (val1) { \
    case LOGIC_1: \
      result = LOGIC_1; \
      break; \
    case LOGIC_0: \
      result = val2; \
      break; \
    case LOGIC_X: \
      if ( val2 == LOGIC_1 ) result = LOGIC_1; \
      else result = LOGIC_X; \
      break; \
    default: \
      assert(0); \
    } \
  }

#define evaluate(gate) \
  { \
    switch( gate.type ) { \
    case PI: \
      break; \
    case PO: \
    case BUF: \
      gate.out_val = gate.in_val[0]; \
      break; \
    case PO_GND: \
      gate.out_val = LOGIC_0; \
      break; \
    case PO_VCC: \
      gate.out_val = LOGIC_1; \
      break; \
    case INV: \
      compute_INV(gate.out_val,gate.in_val[0]); \
      break; \
    case AND: \
      compute_AND(gate.out_val,gate.in_val[0],gate.in_val[1]); \
      break; \
    case NAND: \
      compute_AND(gate.out_val,gate.in_val[0],gate.in_val[1]); \
      compute_INV(gate.out_val,gate.out_val); \
      break; \
    case OR: \
      compute_OR(gate.out_val,gate.in_val[0],gate.in_val[1]); \
      break; \
    case NOR: \
      compute_OR(gate.out_val,gate.in_val[0],gate.in_val[1]); \
      compute_INV(gate.out_val,gate.out_val); \
      break; \
    default: \
      assert(0); \
    } \
  }


/*************************************************************************

Function:  three_val_fault_simulate

Purpose:  This function performs fault simulation on 3-valued input patterns.

pat.out[][] is filled with the fault-free output patterns corresponding to
the input patterns in pat.in[][].

Return:  List of faults that remain undetected.

*************************************************************************/

fault_list_t *three_val_fault_simulate(ckt,pat,undetected_flist)
     circuit_t *ckt;
     pattern_t *pat;
     fault_list_t *undetected_flist;
{
  int p;  /* looping variable for pattern number */
  int i;
  fault_list_t *fptr, *prev_fptr;
  int detected_flag;

  /*************************/
  /* fault-free simulation */
  /*************************/

  /* loop through all patterns */
  for (p = 0; p < pat->len; p++) {
    /* initialize all gate values to UNDEFINED */
    for (i = 0; i < ckt->ngates; i++) {
      ckt->gate[i].in_val[0] = UNDEFINED;
      ckt->gate[i].in_val[1] = UNDEFINED;
      ckt->gate[i].out_val = UNDEFINED;
    }
    /* assign primary input values for pattern */
    for (i = 0; i < ckt->npi; i++) {
      ckt->gate[ckt->pi[i]].out_val = pat->in[p][i];
    }
    /* evaluate all gates */
    for (i = 0; i < ckt->ngates; i++) {
      /* get gate input values */
      switch ( ckt->gate[i].type ) {
      /* gates with no input terminal */
      case PI:
      case PO_GND:
      case PO_VCC:
	break;
      /* gates with one input terminal */
      case INV:
      case BUF:
      case PO:
	ckt->gate[i].in_val[0] = ckt->gate[ckt->gate[i].fanin[0]].out_val;
	break;
      /* gates with two input terminals */
      case AND:
      case NAND:
      case OR:
      case NOR:
	ckt->gate[i].in_val[0] = ckt->gate[ckt->gate[i].fanin[0]].out_val;
	ckt->gate[i].in_val[1] = ckt->gate[ckt->gate[i].fanin[1]].out_val;
	break;
      default:
	assert(0);
      }
      /* compute gate output value */
      evaluate(ckt->gate[i]);
    }
    /* put fault-free primary output values into pat data structure */
    for (i = 0; i < ckt->npo; i++) {
      pat->out[p][i] = ckt->gate[ckt->po[i]].out_val;
    }
  }

  /********************/
  /* fault simulation */
  /********************/

  /* loop through all undetected faults */
  prev_fptr = (fault_list_t *)NULL;
  for (fptr=undetected_flist; fptr != (fault_list_t *)NULL; fptr=fptr->next) {
    /* loop through all patterns */
    detected_flag = FALSE;
    for (p = 0; (p < pat->len) && !detected_flag; p++) {
      /* initialize all gate values to UNDEFINED */
      for (i = 0; i < ckt->ngates; i++) {
	ckt->gate[i].in_val[0] = UNDEFINED;
	ckt->gate[i].in_val[1] = UNDEFINED;
	ckt->gate[i].out_val = UNDEFINED;
      }
      /* assign primary input values for pattern */
      for (i = 0; i < ckt->npi; i++) {
	ckt->gate[ckt->pi[i]].out_val = pat->in[p][i];
      }
      /* evaluate all gates */
      for (i = 0; i < ckt->ngates; i++) {
	/* get gate input values */
	switch ( ckt->gate[i].type ) {
	  /* gates with no input terminal */
	case PI:
	case PO_GND:
	case PO_VCC:
	  break;
	  /* gates with one input terminal */
	case INV:
	case BUF:
	case PO:
	  ckt->gate[i].in_val[0] = ckt->gate[ckt->gate[i].fanin[0]].out_val;
	  break;
	  /* gates with two input terminals */
	case AND:
	case NAND:
	case OR:
	case NOR:
	  ckt->gate[i].in_val[0] = ckt->gate[ckt->gate[i].fanin[0]].out_val;
	  ckt->gate[i].in_val[1] = ckt->gate[ckt->gate[i].fanin[1]].out_val;
	  break;
	default:
	  assert(0);
	}
	/* check if faulty gate */
	if ( i == fptr->gate_index ) {
	  /* check if fault at input */
	  if ( fptr->input_index >= 0 ) {
	    /* inject fault */
	    if ( fptr->type == S_A_0 ) {
	      ckt->gate[i].in_val[fptr->input_index] = LOGIC_0;
	    }
	    else { /* S_A_1 */
	      ckt->gate[i].in_val[fptr->input_index] = LOGIC_1;
	    }
	    /* compute gate output value */
	    evaluate(ckt->gate[i]);
	  }
	  else { /* fault at output */
	    evaluate(ckt->gate[i]);
	    /* inject fault */
	    if ( fptr->type == S_A_0 ) {
	      ckt->gate[i].out_val = LOGIC_0;
	    }
	    else { /* S_A_1 */
	      ckt->gate[i].out_val = LOGIC_1;
	    }
	  }
	}
	else { /* not faulty gate */
	  /* compute gate output value */
	  evaluate(ckt->gate[i]);
	}
      }
      /* check if fault detected */
      for (i = 0; i < ckt->npo; i++) {
	if ( (ckt->gate[ckt->po[i]].out_val == LOGIC_0)
	     && ( pat->out[p][i] == LOGIC_1) )
	  detected_flag = TRUE;
	if ( (ckt->gate[ckt->po[i]].out_val == LOGIC_1)
	     && ( pat->out[p][i] == LOGIC_0) )
	  detected_flag = TRUE;
      }
    }
    if ( detected_flag ) {
      /* remove fault from undetected fault list */
      if ( prev_fptr == (fault_list_t *)NULL ) {
	/* if first fault in fault list, advance head of list pointer */
	undetected_flist = fptr->next;
      }
      else { /* if not first fault in fault list, then remove link */
	prev_fptr->next = fptr->next;
      }
    }
    else { /* fault remains undetected, keep on list */
      prev_fptr = fptr;
    }
  }
  return(undetected_flist);
}
