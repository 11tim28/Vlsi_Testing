#include "project.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/times.h>
#include <stdlib.h>
#include <assert.h>

/* Global Variables */

circuit_t ckt;
pattern_t pat;
int debug;

extern char *pi_order_name_array[];
extern int pi_order_num;

fault_list_t *init_fault_list();
fault_list_t *add_fault();
void read_patterns();
void write_output();
extern void read_circuit(); /* defined in read_ckt.c */
extern fault_list_t *three_val_fault_simulate(); /* defined in project.c */

void print_usage()
{
  printf("usage:  3fsim [-h] circuit_file pattern_file output_file\n");
  printf("\t-h shows usage\n");
  printf("\tcircuit_file is the circuit description to read in\n");
  printf("\tpattern_file is the pattern file to read\n\n");
  printf("\toutput_file is the output file to write\n\n");
}

int main (argc,argv)
     int argc;
     char *argv[];
{
  FILE *pat_file, *ckt_file, *out_file;
  char ckt_filename[256], pat_filename[256], out_filename[256];
  struct rusage start_time, finish_time;  
  unsigned long time;
  fault_list_t *flist,*undetected_flist, *ptr, **fault_array;
  int num_faults,i;

  for (i = 1; i < argc; i++) {
    if ( argv[i][0] == '-' ) {
      switch ( argv[i][1] ) {
      case 'h':
	print_usage();
	exit(0);
      case 'd':
	printf("debug_mode = ON\n");
	debug = TRUE;
	break;
      }
    }
    else {
      break;
    }
  }
  if ( i >= (argc-2) ) {
    print_usage();
    exit(-1);
  }
  strcpy(ckt_filename,argv[i]);
  i++;
  ckt_file = fopen(ckt_filename,"r");
  if ( ckt_file == (FILE *)NULL ) {
    fprintf(stderr,"ERROR:  can't open %s for reading\n",ckt_filename);
    exit(-1);
  }
  printf("\nReading Circuit:  %s\n\n",ckt_filename);
  read_circuit(ckt_file);
  fclose(ckt_file);
  strcpy(pat_filename,argv[i]);
  i++;
  pat_file = fopen(pat_filename,"r");
  if ( pat_file == (FILE *)NULL ) {
    fprintf(stderr,"ERROR:  can't open %s for reading\n",pat_filename);
    exit(-1);
  }
  printf("\nReading Patterns:  %s\n\n",pat_filename);
  read_patterns(&ckt,pat_file);
  fclose(pat_file);
  strcpy(out_filename,argv[i]);
  i++;
  out_file = fopen(out_filename,"w");
  if ( out_file == (FILE *)NULL ) {
    fprintf(stderr,"ERROR:  can't open %s for writing\n",out_filename);
    exit(-1);
  }
  flist = init_fault_list(&ckt);
  for (num_faults = 0,ptr = flist; (ptr != (fault_list_t *)NULL); num_faults++, ptr = ptr->next);
  /*
  for (num_faults = 0,ptr = flist; (ptr != (fault_list_t *)NULL) && (num_faults < 10000); num_faults++) {
    ptr = ptr->next;
  }
  if (num_faults == 10000) {
    printf("num_faults == 10000\n");
    ptr->next = (fault_list_t *)NULL;
  }
  */
  //fault_array = (fault_list_t **)malloc(num_faults*sizeof(fault_list_t *));
  //for (i = 0,ptr = flist; i < num_faults; i++,ptr=ptr->next) {
  //  fault_array[i] = ptr;
  //}
  assert(ptr == (fault_list_t *)NULL);
  printf("Number of PI = %d\n",ckt.npi);
  printf("Number of PO = %d\n",ckt.npo);
  printf("Number of gates = %d\n",ckt.ngates);
  printf("Number of faults = %d\n",num_faults);
  printf("Number of patterns = %d\n",pat.len);

  printf("\nRunning Simulation...\n\n");
  getrusage(RUSAGE_SELF,&start_time);
  undetected_flist = three_val_fault_simulate(&ckt,&pat,flist);
  getrusage(RUSAGE_SELF,&finish_time);
  time = ((finish_time.ru_utime.tv_sec*1e6)+finish_time.ru_utime.tv_usec)
         - ((start_time.ru_utime.tv_sec*1e6)+start_time.ru_utime.tv_usec);
  printf("Finished Simulation.\n\n");
  printf("Simulation Time = %f sec\n\n",(float)time/(float)1e6);
  printf("\nWriting Output:  %s\n\n",out_filename);
  write_output(&ckt,&pat,undetected_flist,num_faults,out_file);
  fclose(out_file);
  /* free data structures */
  //free(fault_array);
  //for (i = 0; i < num_faults; i++) {
  //  free(fault_array[i]);
 // }
  for (i = 0; i < ckt.ngates; i++) {
    free(ckt.gate[i].fanout);
  }
  free(ckt.gate);
  for (i = 0; i < pat.len; i++) {
    free(pat.in[i]);
    free(pat.out[i]);
  }
}

void read_patterns(ckt,pat_file)
     circuit_t *ckt;
     FILE *pat_file; /* file already open and ready to read (don't close it) */
{
  int count, bit, loop, line_count=0, i=0;
  char line[ckt->npi+10];
 
  /*********** Find pattern length *************/
  while (fgets(line, ckt->npi+10, pat_file) != NULL)
    line_count++;
  
  pat.len = line_count;
  rewind(pat_file);
  
  /************ MAIN LOOP ****************/
  for (loop = 0; loop<pat.len; loop++) {
    pat.in[loop] = (int *)malloc(ckt->npi*sizeof(int));
    pat.out[loop] = (int *)malloc(ckt->npo*sizeof(int));
    for (count = 0; count < ckt->npi; count++) {
      pat.in[loop][count] = (fgetc(pat_file)-48);
    }
    if ( loop < pat.len - 1 ) {
      if (fgetc(pat_file)!='\n'){
	printf("\nError: Pattern file is incompatible with circuit primary inputs!\n\n");
	exit(1);
      }
    }
  }  /* end for */
}  /* end read_patterns() */

int fcount = 0;

fault_list_t *init_fault_list(ckt)
     circuit_t *ckt;
{
  fault_list_t *flist;
  int i,j;
  int pi_count = 0;
  int po_count = 0;

  ckt->pi = (int *)malloc(ckt->npi*sizeof(int));
  ckt->po = (int *)malloc(ckt->npo*sizeof(int));
  flist = (fault_list_t *)NULL;
  for (i = 0; (i < ckt->ngates) ; i++) {
    switch ( ckt->gate[i].type ) {
    case AND:
    case NAND:
      flist = add_fault(flist,i,0,S_A_1);
      flist = add_fault(flist,i,1,S_A_1);
      flist = add_fault(flist,i,-1,S_A_0);
      flist = add_fault(flist,i,-1,S_A_1);
      fcount += 4;
      break;
    case OR:
    case NOR:
      flist = add_fault(flist,i,0,S_A_0);
      flist = add_fault(flist,i,1,S_A_0);
      flist = add_fault(flist,i,-1,S_A_0);
      flist = add_fault(flist,i,-1,S_A_1);
      fcount += 4;
      break;
    case INV:
    case BUF:
      flist = add_fault(flist,i,-1,S_A_0);
      flist = add_fault(flist,i,-1,S_A_1);
      fcount += 2;
      break;
    case PI:
      for (j = 0; j < ckt->npi; j++) {
	if ( strcmp(pi_order_name_array[j],ckt->gate[i].name) == 0 ) {
	  ckt->pi[j] = i;
	}
      }
      /* ckt->pi[pi_count] = i; */
      pi_count++;
      flist = add_fault(flist,i,-1,S_A_0);
      flist = add_fault(flist,i,-1,S_A_1);
      fcount += 2;
      break;
    case PO:
      ckt->po[po_count] = i;
      po_count++;
      flist = add_fault(flist,i,0,S_A_0);
      flist = add_fault(flist,i,0,S_A_1);
      fcount += 2;
      break;
    case PO_GND:
    case PO_VCC:
      ckt->po[po_count] = i;
      po_count++;
      break;
    default:
      printf("ERROR:  Unknown gate!\n");
      exit(-1);
    }
  }
  return (flist);
}

fault_list_t *add_fault(flist,gate_index,input_index,type)
     fault_list_t *flist;
     int gate_index;
     int input_index;
     stuck_val_t type;
{
  fault_list_t *new_fault;

  /*
  if (fcount >= 5000) {
    return (flist);
  }
  */
  new_fault = (fault_list_t *)malloc(sizeof(fault_list_t));
  new_fault->gate_index = gate_index;
  new_fault->input_index = input_index;
  new_fault->type = type;
  new_fault->next = flist;
  return(new_fault);
}

void write_output(ckt,pat,flist,num_faults,out_file)
     circuit_t *ckt;
     pattern_t *pat;
     fault_list_t *flist;
     int num_faults;
     FILE *out_file;
{
  int i,j,count;
  fault_list_t *ptr;

  for (i = 0; i < pat->len; i++) {
    for (j = 0; j < ckt->npi; j++) {
      fprintf(out_file,"%d",pat->in[i][j]);
    }
    fprintf(out_file," -> ");
    for (j = 0; j < ckt->npo; j++) {
      fprintf(out_file,"%d",pat->out[i][j]);
    }
    fprintf(out_file,"\n");
  }
  fprintf(out_file,"\nList of Undetected Faults:\n");
  if ( flist == (fault_list_t *)NULL ) {
    fprintf(out_file,"(Empty)\n");
  }
  count = 0;
  for (ptr = flist; ptr != (fault_list_t *)NULL; ptr = ptr->next) {
    count++;
    fprintf(out_file,"Gate %s ",ckt->gate[ptr->gate_index].name);
    switch (ckt->gate[ptr->gate_index].type) {
    case AND:
      fprintf(out_file,"(AND) ");
      break;
    case OR:
      fprintf(out_file,"(OR)  ");
      break;
    case NAND:
      fprintf(out_file,"(NAND)");
      break;
    case NOR:
      fprintf(out_file,"(NOR) ");
      break;
    case INV:
      fprintf(out_file,"(INV) ");
      break;
    case BUF:
      fprintf(out_file,"(BUF) ");
      break;
    case PO:
      fprintf(out_file,"(PO)  ");
      break;
    case PI:
      fprintf(out_file,"(PI)  ");
      break;
    case PO_GND:
      fprintf(out_file,"(PO_GND)");
      break;
    case PO_VCC:
      fprintf(out_file,"(PO_VCC)");
      break;
    default:
      fprintf(out_file,"(UNKNOWN)");
      break;
    }
    if ( ptr->input_index < 0 ) {
      fprintf(out_file,"- output, ");
    }
    else {
      fprintf(out_file,"- input%d (%s), ",ptr->input_index,
         ckt->gate[ckt->gate[ptr->gate_index].fanin[ptr->input_index]].name);
    }
    fprintf(out_file,"%s\n",(ptr->type == S_A_0) ? "S_A_0" : "S_A_1");
  }
  fprintf(out_file,"\nTotal Number of Faults = %d\n",num_faults);
  fprintf(out_file,"Number of Undetected Faults = %d\n",count);
  fprintf(out_file,"Fault Coverage = %d.%d%%\n\n",
	  ((num_faults-count)*100)/num_faults,
	  ((num_faults-count)*1000/num_faults)%10);
}
