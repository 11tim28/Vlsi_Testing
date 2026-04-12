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
 * Copyright Heling Yi, Narayanan Krishnamurthy  1997
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


#ifndef _READ_CKT_H
#define _READ_CKT_H

#include <assert.h>
#include <stdlib.h>
#include <limits.h>
#include "project.h"

/* redefine types  */
typedef enum gate_type_enum Gate_Type_Enum;     /* change to my own convention */
typedef struct circuit_struct Circuit_Struct_t;

typedef struct Gate_Info Gate_Info_t;
struct Gate_Info{
  Gate_Type_Enum  GateType;
  int             NumOfNode;
  char            **NameOfNode;
};

/* use following functions to build circuit */
extern void Add_Gate(const Gate_Info_t *);
extern void Build_Ckt();

/*Global variable(s) */ 
extern circuit_t ckt;   /* Whole simulation will rely on this variable:-) */


/* following definition and type is used only by read_ckt.c program */

/* define some constants */
#define  UnLevel    -1
#define  FirstLevel 0
#define  MaxLevel INT_MAX


enum Node_Type { 
    NodeIn, NodeOut
};

typedef struct Gate_Struct Gate_Struct_t;
typedef struct List_Node_Cell List_Node_Cell_t;
typedef struct List_Gate_Cell List_Gate_Cell_t;
typedef struct List_Level_Cell List_Level_Cell_t;

/* my own version of structure of the gate */ 
struct Gate_Struct{
  Gate_Type_Enum     GateType;
  char               *name;
  int                NumOfFanin;
  int                NumOfFanout;
  struct Gate_Struct **FaninStruct;
  struct Gate_Struct **FanoutStruct;
  int                level;
  int                index;
};

/* List_Cell is a general sturcture to build a list */
struct List_Node_Cell {
  char                  *NodeName;
  struct List_Node_Cell *next;
  struct List_Gate_Cell *GateStart;
};

/* Keep tracing of Node of Circuit in order to build circuit structure */
struct List_Gate_Cell {
  struct List_Gate_Cell *next;
  Gate_Struct_t         *GateStruct;
  enum Node_Type             NodeType;
};

struct List_Level_Cell{
  struct List_Level_Cell *prev;
  struct List_Level_Cell *next;
  int level;
  int NumOfGate;
  struct Gate_Struct  **GateStruct;
};

/* in order to compile with -ansi option of gcc, we need write strdup which
   is not in ANSI C */
#ifdef __STRICT_ANSI__
#include <string.h>
char *strdup(const char *src){
    char *des=(char *) malloc(strlen(des)+1);
    return strcpy(des,src);}    
#endif

#endif
