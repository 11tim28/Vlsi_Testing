/* 
 *
 *
 * This is free software, licensed under the GNU Public License V2.
 * See the file COPYING for details.
 *
 * This code is provided as is, with no warranty implied or expressed.
 * Use at your own risk.  This software may contain bugs which could
 * cause it to malfunction in possibly dangerous ways.
 *  
 * Neither the author, nor any affiliates of the author are responsible for 
 * any damages caused by use of this software.
 *
 * Copyright Heling Yi, Narayanan Krishnamurthy 1997
 *
 */


/*
 * This code was developed as a part of the project of digital system simulation 
 * taught by Dr. Nur Touba at the University of Texas at Austin.
 * The project is delay fault simulation. This code is to build structure of 
 * a circuit.
 * 
 * There are two public functions:
 * Add_Gate(Gate_Info_t *) is to add a gate to an circuit. After add all
 *             gates to the circuit, call function Build_Ckt()
 * Build_Ckt() levelizes the circuit and translates to desired format. It also
 *             removes "scaffold"  
 */


/* program will print out redundant information, for testing purpose, if 
 * compiling the code with -DDEBUG_BUILD_CKT.
 *
 * To test this program itself, compile with -DSELFTEST_BUILD_CKT. If 
 *  -DTEST_FROM_FILE is also defined, the program will read information of gates
 *  from a file, other it will use a very small build-in circuit.
 */

#include <string.h>
#include <stdio.h>
#include "read_ckt.h"

#define NO_WARNING 1

/* Global variable */
circuit_t ckt; /* Whole simulation will rely on this variable:-) */
char *pi_order_name_array[10000];
int pi_order_num = 0;

static List_Node_Cell_t  *ListNodeStart = NULL;
static List_Gate_Cell_t  *ListPIStart = NULL; /*unnecessary right now */
static List_Gate_Cell_t  *ListPOStart = NULL;
static List_Level_Cell_t *ListLevelStart = NULL;
static int    NumOfGate = 0;
static int    NumOfPI = 0;
static int    NumOfPO = 0;
static int    CktMaxLevel = UnLevel;  /* unnecessary right now */

/* following three memory acllocate functions are the same as those in 
   standard C functions except that print out error message and assert
   program when there is not enough memory
   */
inline void * mallocm(size_t size){
  void * ma;
  ma=malloc(size);
  if (ma == NULL ) { perror("Memory Error!\n"); assert(0);}
  return ma;
}

inline void * reallocm(void* ptr, size_t size){
  void * ma;
  ma=realloc(ptr,size);
  if (ma == NULL ) { perror("Memory Error!\n"); assert(0);}
  return ma;
}

inline void * callocm(size_t nmemb, size_t size){
  void * ma;
  ma=calloc(nmemb,size);
  if (ma == NULL ) { perror("Memory Error!\n"); assert(0);}
  return ma;
}


/* call add_gate() whenever add a new gate to the circuit.  
 *  The Gate Info should be provided, see header file of detail.
 */
void Add_Gate(const Gate_Info_t * GateInfo ){
  Gate_Struct_t       *CurrGateStruct, *TmpGateStruct;
  List_Node_Cell_t    *ListNodeIter;
  List_Gate_Cell_t    *ListGateIter, *ListGateCurr;
  int                  i;
  enum Node_Type            CurrNodeType;
  
  /* allocate memory for this gate, and initilize it */
#ifdef DEBUG_BUILD_CKT
  {
    int j;
    printf("\nIn Add_Gate():(before malloc) Gate->name ");
    for (j=0;j<GateInfo->NumOfNode;j++){
      printf("%s, ",GateInfo->NameOfNode[j]);
    }
  }
#endif
  CurrGateStruct = (Gate_Struct_t *)  mallocm(sizeof(Gate_Struct_t));
  CurrGateStruct->GateType = GateInfo->GateType;
  CurrGateStruct->NumOfFanin = 0;
  CurrGateStruct->NumOfFanout = 0;
  CurrGateStruct->FaninStruct = NULL;
  CurrGateStruct->FanoutStruct = NULL;
  CurrGateStruct->level = UnLevel; 
  /*set gate name to the name of output node of the gate */
  CurrGateStruct->name = strdup(GateInfo->NameOfNode[GateInfo->NumOfNode-1]);
  /* loop each node (signal) of the gate */
  for ( i = 0 ; i < GateInfo->NumOfNode; i ++ ){
    if ( i == (GateInfo->NumOfNode-1) ) CurrNodeType = NodeOut;
    else CurrNodeType = NodeIn;      /* The last node of gate is considered 
					as output of the node */
    if ( GateInfo->GateType==PO) CurrNodeType = NodeIn; /* Primary output is
							   also considered as
							   nodein */
    /* allocate List_Gate_Cell_t for current node, since we need information about
       the node such as it is a output node(signal) or input node(signal) */
    ListGateCurr = (List_Gate_Cell_t *) mallocm(sizeof(List_Gate_Cell_t));
    ListGateCurr->next = NULL;
    ListGateCurr->NodeType = CurrNodeType;
    ListGateCurr->GateStruct = CurrGateStruct;
    ListNodeIter = ListNodeStart ;
    while ( ListNodeIter != NULL ){
      if ( strcmp( ListNodeIter->NodeName, GateInfo->NameOfNode[i] ) == 0 ){
	ListGateIter=ListNodeIter->GateStart;
	ListNodeIter->GateStart = ListGateCurr;
	ListGateCurr->next = ListGateIter;
	while ( ListGateIter != NULL ){
	  if ( CurrNodeType == ListGateIter->NodeType) {
	    ListGateIter = ListGateIter->next;
	    continue;
	  }
	  TmpGateStruct=ListGateIter->GateStruct;
	  if ( CurrNodeType == NodeIn){
	    CurrGateStruct->FaninStruct =
	      (Gate_Struct_t**) reallocm(CurrGateStruct->FaninStruct,
		     (CurrGateStruct->NumOfFanin+1)*sizeof( Gate_Struct_t *));
	    CurrGateStruct->FaninStruct[CurrGateStruct->NumOfFanin++] = 
	      TmpGateStruct;
	    TmpGateStruct->FanoutStruct =
	      (Gate_Struct_t**) reallocm(TmpGateStruct->FanoutStruct,
		     (TmpGateStruct->NumOfFanout+1)*sizeof( Gate_Struct_t* ));
	    TmpGateStruct->FanoutStruct[TmpGateStruct->NumOfFanout++]=
	      CurrGateStruct;
	  }else {
	    CurrGateStruct->FanoutStruct =
	      (Gate_Struct_t**) reallocm(CurrGateStruct->FanoutStruct,
		     (CurrGateStruct->NumOfFanout+1)*sizeof( Gate_Struct_t *));
	    CurrGateStruct->FanoutStruct[CurrGateStruct->NumOfFanout++] = 
	      TmpGateStruct;
	    TmpGateStruct->FaninStruct =
	      (Gate_Struct_t**)reallocm(TmpGateStruct->FaninStruct ,
		     (TmpGateStruct->NumOfFanin+1)*sizeof(Gate_Struct_t *));
	    TmpGateStruct->FaninStruct[TmpGateStruct->NumOfFanin++]=
	      CurrGateStruct;
	  }
	  ListGateIter = ListGateIter->next;
	}
	break;
      }
      ListNodeIter = ListNodeIter->next;
    }
    if (ListNodeIter == NULL ){
      ListNodeIter = (List_Node_Cell_t*) mallocm(sizeof(List_Node_Cell_t));
      ListNodeIter->next = ListNodeStart;
      ListNodeStart = ListNodeIter;
      ListNodeIter->NodeName = strdup(GateInfo->NameOfNode[i]);
      ListNodeIter->GateStart = ListGateCurr;
    }
  }
  /* It seems that list of PI is unnecessary for current implementation */

  if (CurrGateStruct->GateType == PI ){
    pi_order_name_array[pi_order_num] = strdup(CurrGateStruct->name);
    pi_order_num++;
    ListGateIter = (List_Gate_Cell_t*) mallocm(sizeof(List_Gate_Cell_t));
    ListGateIter->next = ListPIStart;
    ListPIStart = ListGateIter;
    ListGateIter->GateStruct = CurrGateStruct;
    NumOfPI++;
  }

  if (CurrGateStruct->GateType == PO ){
    ListGateIter = (List_Gate_Cell_t*) mallocm(sizeof(List_Gate_Cell_t));
    ListGateIter->next = ListPOStart;
    ListPOStart = ListGateIter;
    ListGateIter->GateStruct = CurrGateStruct;
    NumOfPO++;
  }
  NumOfGate++;
#ifdef DEBUG_BUILD_CKT
  {
    int j;
    printf("\nIn Add_Gate():( after malloc) Gate->name ");
    for (j=0;j<GateInfo->NumOfNode;j++){
      printf("%s, ",GateInfo->NameOfNode[j]);
    }
  }
#endif
/* primary input list and primary output list should be here */
}

/* levelize each gate, start from PO. Algorithm is that looking fanins of the
 *  gate and level of this gate is the maximum of level of fanins plus one. If
 *  any fanin has no level, unlevelized fanis will be evaluated before evaluate 
 *  current gate and do it recurisively until reach primary input which will 
 *  set to FirstLevel 
 */
static int Set_Level_Gate(Gate_Struct_t * CurrGate){
  int curmaxlevel,i,level;
  List_Level_Cell_t *ListLevelIter, *ListLevelTmp;

#ifdef DEBUG_BUILD_CKT
  printf("\nLeveling the Gate:%s",CurrGate->name);
#endif
  /*VVVVVVVVVVVVVVVVVVVVV??????????????????????????????????*/
  /* Not sure about following lines */
  /* something need to be done, since I consider constant zero and one as primary 
     input. which will be identical to PI, however, for evaluate it should be 
     place after PI */
  if ( CurrGate->GateType == PI ){
    CurrGate->level = FirstLevel;
  }else if( CurrGate->GateType==PO_GND  ||CurrGate->GateType==PO_VCC ){
    /* consider constant 1 or 0 as one level higher than primary input, may not
       a good idea*/
    CurrGate->level = FirstLevel + 1;
  }
  else{
    assert(CurrGate->NumOfFanin);
    curmaxlevel = UnLevel;
    for (i=0; i<CurrGate->NumOfFanin;i++){
      level=((CurrGate->FaninStruct)[i])->level;
      if ( level== UnLevel) level=Set_Level_Gate(CurrGate->FaninStruct[i]);
      if (level > curmaxlevel ) curmaxlevel=level;
    }
    CurrGate->level=curmaxlevel+1;
  }
  if (CurrGate->GateType == PO ) level = MaxLevel;
  else {
    level = CurrGate->level;
    if (level > CktMaxLevel ) CktMaxLevel = level;
  }
  ListLevelIter = ListLevelStart;
  while (ListLevelIter!=NULL ){
    if (level == ListLevelIter->level ) {
      ListLevelIter->GateStruct = (Gate_Struct_t**) 
	reallocm(ListLevelIter->GateStruct,
		 (ListLevelIter->NumOfGate+1)*sizeof(Gate_Struct_t *));
      ListLevelIter->GateStruct[ListLevelIter->NumOfGate++]=CurrGate;
      break;
    }
    if (level < ListLevelIter->level ){
      ListLevelTmp = (List_Level_Cell_t* ) 
	mallocm(sizeof(List_Level_Cell_t));
      ListLevelTmp->level = level;
      ListLevelTmp->NumOfGate=1;
      ListLevelTmp->GateStruct=(Gate_Struct_t**) 
	mallocm(sizeof(Gate_Struct_t*));
      (ListLevelTmp->GateStruct)[0]=CurrGate;
      ListLevelTmp->next = ListLevelIter;
      ListLevelTmp->prev = ListLevelIter->prev;
      ListLevelIter->prev = ListLevelTmp;
      if ( ListLevelTmp->prev == NULL ) ListLevelStart = ListLevelTmp;
      else ListLevelTmp->prev->next = ListLevelTmp;
      break;
    }   
    ListLevelIter = ListLevelIter->next;
  }
  return CurrGate->level;
}


/* Levelizing the circuit and set index to each gate */
static void Levelize_Ckt(){
  List_Gate_Cell_t  *ListGateIter;
  List_Level_Cell_t *ListLevelIter;
  int i,count;

#ifdef DEBUG_BUILD_CKT
  printf("\nLevelizing the circuit..............");
#endif
  /* levelize the circuit by set level of each primary output */
  ListLevelStart = (List_Level_Cell_t*) mallocm(sizeof(List_Level_Cell_t));
  ListLevelStart->next= NULL;
  ListLevelStart->prev=NULL;
  ListLevelStart->level=MaxLevel;
  ListLevelStart->NumOfGate=0;
  ListLevelStart->GateStruct=NULL;
  ListGateIter = ListPOStart;
  while (ListGateIter != NULL){
    Set_Level_Gate(ListGateIter->GateStruct);
    ListGateIter = ListGateIter->next;
  }
  ListLevelIter = ListLevelStart;
  count=0;
  while (ListLevelIter!=NULL ){
    for ( i=0 ; i<ListLevelIter->NumOfGate; i++ ){
      (ListLevelIter->GateStruct[i])->index=count++;
#ifdef DEBUG_BUILD_CKT
      {
	int j;
	printf("\nGate(%s) has %d fanins and %d fanouts, level is %d:",
	       ListLevelIter->GateStruct[i]->name,
	       ListLevelIter->GateStruct[i]->NumOfFanin,
	       ListLevelIter->GateStruct[i]->NumOfFanout,
	       ListLevelIter->GateStruct[i]->level);
	printf("\n   Fanin:");
	for (j=0;j<ListLevelIter->GateStruct[i]->NumOfFanin;j++){
	  printf(" %s",ListLevelIter->GateStruct[i]->FaninStruct[j]->name);
	}
	printf("\n   Fanout:");
	for (j=0;j<ListLevelIter->GateStruct[i]->NumOfFanout;j++){
	  printf(" %s",ListLevelIter->GateStruct[i]->FanoutStruct[j]->name);
	}
      }
#endif
    }
    ListLevelIter=ListLevelIter->next;
  }
#ifdef DEBUG_BUILD_CKT
  printf("\nTotal %d gates, %d PI, %d PO, %d gates, %d levels\n",
	 count,NumOfPI,NumOfPO,NumOfGate,CktMaxLevel);
#endif
  if ( count < NumOfGate ) {
#ifndef NO_WARNING
    /* if the total number of gates that have been levelized is less than
     * total number of gates in the circuit, there exists some gates that
     * will not affect primary output. Print warnning information unless it
     * has been disabled when compiling the code.
     */
    printf("\nWarning: Some Gates may not connect to primary output\n");
#endif
    NumOfGate = count;
  }
  if ( count > NumOfGate ){
    /* if the total number of gates that have been levelized is greater than
     * total number of gates in the circuit, some weird thing happens. In this
     * case, we simply assert the program, since I am not sure the program is
     * good enough to handle all possible cases:-( 
     */
    printf("\nError: Can't levelize circuit\n");
    assert(0);
  }
}

/* remove "scaffold" of circuit which is used when building the circuit */ 
static void Clean_Ckt(){
  List_Node_Cell_t  *ListNodeIter, *ListNodeTmp;
  List_Gate_Cell_t  *ListGateIter, *ListGateTmp;

#ifdef DEBUG_BUILD_CKT
  printf("\nCleaning the circuit................");
#endif
  /* remove unnecessary link list of date structure */
  ListNodeIter = ListNodeStart;
  while ( ListNodeIter!=NULL ){
    ListGateIter=ListNodeIter->GateStart;
    while (ListGateIter!=NULL){
      ListGateTmp=ListGateIter;
      ListGateIter=ListGateIter->next;
      free(ListGateTmp);
    }
    ListNodeTmp=ListNodeIter;
    ListNodeIter= ListNodeIter->next;
    free(ListNodeTmp);
  }
  /* since list of primary inputs is unnecessary, we'd better comment out
   * following codes:-)
   */
  ListGateIter=ListPIStart;
  while (ListGateIter!=NULL){
    ListGateTmp=ListGateIter;
    ListGateIter=ListGateIter->next;
    free(ListGateTmp);
  }
  
  ListGateIter=ListPOStart;
  while (ListGateIter!=NULL){
    ListGateTmp=ListGateIter;
    ListGateIter=ListGateIter->next;
    free(ListGateTmp);
  } 
}  

/* put gates to default date structure. 
 *  since the circuit structure that used for building circuit may not efficient for
 *  simulation, we need translate it another circuit struction, which is put all
 *  ordered gate structures in contiunous memory suggested by Dr.Touba. 
 */ 
static void Convert_Ckt(Circuit_Struct_t * ckt_t){
  int i,j,count;
  List_Level_Cell_t *ListLevelIter, *ListLevelTmp;

#ifdef DEBUG_BUILD_CKT
  printf("\nConverting the circuit.........................");
#endif
  ckt_t->ngates=NumOfGate;
  ckt_t->npi=NumOfPI;
  ckt_t->npo=NumOfPO;
  ListLevelIter = ListLevelStart;
  count=0;

  while (ListLevelIter!=NULL ){
    for ( i=0 ; i<ListLevelIter->NumOfGate; i++ ){
      ckt_t->gate[count].name = ListLevelIter->GateStruct[i]->name;
      ckt_t->gate[count].type = ListLevelIter->GateStruct[i]->GateType;
      ckt_t->gate[count].index = ListLevelIter->GateStruct[i]->index;
      ckt_t->gate[count].num_fanout=ListLevelIter->GateStruct[i]->NumOfFanout;
      if( ListLevelIter->GateStruct[i]->NumOfFanout ){
	ckt_t->gate[count].fanout =
	  (int*)callocm(ListLevelIter->GateStruct[i]->NumOfFanout,sizeof(int));
      }else{
	ckt_t->gate[count].fanout = NULL;
      }
      for (j=0; j<ListLevelIter->GateStruct[i]->NumOfFanin;j++){
	ckt_t->gate[count].fanin[j] = 
	  ListLevelIter->GateStruct[i]->FaninStruct[j]->index;
      }
      for (;j<MAX_GATE_FANIN;j++) ckt_t->gate[count].fanin[j]=UnLevel;
      for (j=0; j<ListLevelIter->GateStruct[i]->NumOfFanout;j++){
	ckt_t->gate[count].fanout[j] = 
	  ListLevelIter->GateStruct[i]->FanoutStruct[j]->index;
      }

      count++;
    }
    ListLevelTmp=ListLevelIter;
    ListLevelIter=ListLevelIter->next;
    free(ListLevelTmp->GateStruct);
    free(ListLevelTmp);
  }
  
}


/* call build_ckt() after adding all gates into circuit */
void Build_Ckt(){
#ifdef DEBUG_BUILD_CKT
  printf("\nBuilding the circuit.........................");
#endif
  Levelize_Ckt();
  Clean_Ckt();
  ckt.gate = (gate_t *) callocm(NumOfGate,sizeof(gate_t));
  Convert_Ckt(&ckt);
#ifdef DEBUG_BUILD_CKT
  {
    int i,j;
    for (i = 0;i<ckt.ngates;i++){
      printf("\nGate (%s) has %d fanouts.",ckt.gate[i].name,
	     ckt.gate[i].num_fanout);
      printf("\n  Fanins: ");
      for (j=0;j<MAX_GATE_FANIN;j++){
	if (ckt.gate[i].fanin[j] == UnLevel) break;
	printf("%s ",ckt.gate[ckt.gate[i].fanin[j]].name);
      }
      printf("\n  Fanouts:");
      for (j=0;j<ckt.gate[i].num_fanout;j++){
	printf("%s ",ckt.gate[ckt.gate[i].fanout[j]].name);
      }
    }
    printf("\n\n");
  }
#endif
}


/* following code is to test this program */
#ifdef SELFTEST_BUILD_CKT
int main(int argv, char **argc){
#ifdef TEST_FROM_FILE
  FILE *openfile;
  int i, j=0;
  char a1[80],a2[80],a3[80];
  char *t1[3]={a1,a2,a3};
  Gate_Info_t gate={PI,0,t1};

  if (argv<2) {
	printf ("\nUsage: %s filename\n\n",argc[0]);
	exit(1);
}
  if ( (openfile=fopen(argc[1],"r"))==NULL ){
    printf("\n Error: Can't open %s\n\n",argc[1]);
    exit(1);
  }
  while (!feof(openfile )){
    fscanf(openfile,"%d\n",&gate.GateType);
    fscanf(openfile,"%d\n",&gate.NumOfNode);
    for (i=0;i<gate.NumOfNode;i++){
      fscanf(openfile,"%s\n",gate.NameOfNode[i]);
    }
#ifdef DEBUG_BUILD_CKT
    printf("\nline %d, Gate has %d nodes, type is %d, nodes are ",
	   ++j,gate.NumOfNode,gate.GateType);
    for (i=0;i<gate.NumOfNode;i++){
      printf("%s,",gate.NameOfNode[i]);
    }
#endif
    Add_Gate(&gate);
  }
  fclose(openfile);
  printf ("\n");
#else
  int i,j;
  char *g1="A<0>0",*g2="B<0>0",*g3="[601]",
    *g4="[701]",*g5="z<0>0";
  char *t1[]={g1};
  char *t2[]={g2};
  char *t3[]={g1,g3};
  char *t4[]={g1,g2,g4};
  char *t5[]={g3,g4,g5};
  char *t6[]={g5};

  Gate_Info_t a1={PI,1,t1};
  Gate_Info_t a2={PI,1,t2};
  Gate_Info_t a3={INV,2,t3};
  Gate_Info_t a4={AND,3,t4};
  Gate_Info_t a5={OR,3,t5};
  Gate_Info_t a6={PO,1,t6};
  Add_Gate(&a1);
  Add_Gate(&a2);
  Add_Gate(&a3);
  Add_Gate(&a4);
  Add_Gate(&a5);
  Add_Gate(&a6);
#endif
  Build_Ckt();
  for (i = 0;i<ckt.ngates;i++){
    printf("\nGate (%s) has %d fanouts.",ckt.gate[i].name,
	   ckt.gate[i].num_fanout);
    printf("\n  Fanins: ");
    for (j=0;j<MAX_GATE_FANIN;j++){
      if (ckt.gate[i].fanin[j] == UnLevel) break;
      printf("%s ",ckt.gate[ckt.gate[i].fanin[j]].name);
    }
    printf("\n  Fanouts:");
    for (j=0;j<ckt.gate[i].num_fanout;j++){
      printf("%s ",ckt.gate[ckt.gate[i].fanout[j]].name);
    }
  }
  printf("\n\n");

}
#endif
