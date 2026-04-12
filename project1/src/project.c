#include "project.h"
#include <stdint.h>
#include <stddef.h>   
#include <stdlib.h>   
#include <stdio.h>

/*************************************************************************

Function:  three_val_transition_fault_simulate

Purpose:  This function performs transition fault simulation on 3-valued
          input patterns.

pat.out[][] is filled with the fault-free output patterns corresponding to
the input patterns in pat.in[][].

Return:  List of faults that remain undetected.

*************************************************************************/


/*
I use parallel-pattern single-fault propagation (PPSFP) + event driven method to accelerate
fault simulation. By using event-driven method, the simulator would only compute gate
that have different value with fault-free circuit (and its fanout). Since the event region
 of each fault is about the same across different pattern, event-driven is naturally
compatible with parallel-pattern single-fault propagation (PPSFP) to gain huge speedup. 

In my code, I use PPSFP with pattern batch size = 32, and use event_queue for event-driven method.
*/

fault_list_t *three_val_fault_simulate(circuit_t *ckt, pattern_t *pat, fault_list_t *undetected_flist)
{
    const int W = 32;
    uint32_t val0[MAX_GATES], val1[MAX_GATES];
    int nwords = (pat->len + W - 1) / W;

    int event_queue[MAX_GATES];
    int queue_start, queue_end;

    for (int w = 0; w < nwords; w++) {
        // Divide patterns into batch of size 32
        int start = w * W;
        int end = (start + W > pat->len) ? pat->len : (start + W);
        uint32_t mask_batch = (end - start < W) ? ((1U << (end - start)) - 1) : 0xFFFFFFFF;

        // =====================================
        // 1. Fault free simulation for this batch
        // =====================================
        for (int i = 0; i < ckt->ngates; i++)
            val0[i] = val1[i] = 0;

        for (int p = start; p < end; p++) {
            int bit = 1U << (p - start);
            for (int pi = 0; pi < ckt->npi; pi++) {
                int gate = ckt->pi[pi];
                // Initialization on PI
                if (pat->in[p][pi] == LOGIC_0) val0[gate] |= bit;
                else if (pat->in[p][pi] == LOGIC_1) val1[gate] |= bit;
            }
        }

        // Propagate fault-free through the circuit
        for (int i = 0; i < ckt->ngates; i++) {
            gate_t g = ckt->gate[i];
            if (g.type == PI) continue;

            uint32_t a0 = val0[g.fanin[0]], a1 = val1[g.fanin[0]];
            uint32_t b0 = 0, b1 = 0;
            if (g.type == AND || g.type == OR || g.type == NAND || g.type == NOR) {
                // 2-input gate -> assign b0 & b1
                b0 = val0[g.fanin[1]];
                b1 = val1[g.fanin[1]];
            }

            uint32_t o0=0, o1=0;
            switch(g.type) {
                case AND: o0 = a0 | b0; o1 = a1 & b1; break;
                case OR:  o1 = a1 | b1; o0 = a0 & b0; break;
                case NAND:o0 = a1 & b1; o1 = a0 | b0; break;
                case NOR: o1 = a0 & b0; o0 = a1 | b1; break;
                case INV: o0 = a1; o1 = a0; break;
                case BUF: case PO: o0 = a0; o1 = a1; break;
                case PO_GND: o0=mask_batch; o1=0; break;
                case PO_VCC: o0=0; o1=mask_batch; break;
                default: continue;
            }
            val0[i] = o0;
            val1[i] = o1;
        }

        // Store fault-free outputs into pat->out
        for (int p = start; p < end; p++) {
            int bit = 1U << (p - start);
            for (int po = 0; po < ckt->npo; po++) {
                int gate = ckt->po[po];
                if ((val1[gate] & bit)) pat->out[p][po] = LOGIC_1;
                else if ((val0[gate] & bit)) pat->out[p][po] = LOGIC_0;
                else pat->out[p][po] = LOGIC_X;
            }
        }

        // =============================
        // 2. Event-driven PPSFP
        // =============================
        fault_list_t *prev = NULL;
        fault_list_t *curr = undetected_flist;

        while (curr) {
            fault_list_t *f = curr;
            int g = f->gate_index; // f: current fault index

            uint32_t val0_faulty[MAX_GATES], val1_faulty[MAX_GATES];
            for (int i=0;i<ckt->ngates;i++){
                // Copy fault-free value into faulty (initialization)
                val0_faulty[i] = val0[i];
                val1_faulty[i] = val1[i];
            }

            queue_start = queue_end = 0;

            // ----------------------------
            //  Inject Fault & Enqueue Event
            // ----------------------------
            if (f->input_index == -1) {
                // output fault
                if (f->type == S_A_0) {
                    val0_faulty[g] |= mask_batch;
                    val1_faulty[g] &= ~mask_batch;
                } else {
                    val0_faulty[g] &= ~mask_batch;
                    val1_faulty[g] |= mask_batch;
                }
                // enqueue fanout
                for (int k=0; k<ckt->gate[g].num_fanout; k++)
                    event_queue[queue_end++] = ckt->gate[g].fanout[k];
            } else {
                // input fault → recompute gate before enqueuing
                int i = g;
                gate_t gi = ckt->gate[i];

                uint32_t a0 = val0_faulty[gi.fanin[0]], a1 = val1_faulty[gi.fanin[0]];
                uint32_t b0=0, b1=0;

                if (gi.type==AND||gi.type==OR||gi.type==NAND||gi.type==NOR) {
                    b0 = val0_faulty[gi.fanin[1]];
                    b1 = val1_faulty[gi.fanin[1]];
                }

                if (f->input_index == 0) {
                    if (f->type == S_A_0) { a0 |= mask_batch; a1 &= ~mask_batch; }
                    else { a0 &= ~mask_batch; a1 |= mask_batch; }
                } else {
                    if (f->type == S_A_0) { b0 |= mask_batch; b1 &= ~mask_batch; }
                    else { b0 &= ~mask_batch; b1 |= mask_batch; }
                }

                uint32_t o0=0,o1=0;
                switch(gi.type){
                    case AND: o0=a0|b0; o1=a1&b1; break;
                    case OR:  o1=a1|b1; o0=a0&b0; break;
                    case NAND:o0=a1&b1; o1=a0|b0; break;
                    case NOR: o1=a0&b0; o0=a1|b1; break;
                    case INV: o0=a1; o1=a0; break;
                    case BUF: case PO: o0=a0; o1=a1; break;
                    default: break;
                }

                val0_faulty[i] = o0;
                val1_faulty[i] = o1;

                // enqueue fanout
                for (int k=0;k<gi.num_fanout;k++)
                    event_queue[queue_end++] = gi.fanout[k];
            }

            // ----------------------------
            // Event Propagation
            // ----------------------------
            while (queue_start < queue_end) {
                int i = event_queue[queue_start++];
                gate_t gi = ckt->gate[i];
                if (gi.type == PI) continue;

                uint32_t a0 = val0_faulty[gi.fanin[0]];
                uint32_t a1 = val1_faulty[gi.fanin[0]];
                uint32_t b0=0,b1=0;

                if (gi.type==AND||gi.type==OR||gi.type==NAND||gi.type==NOR) {
                    b0 = val0_faulty[gi.fanin[1]];
                    b1 = val1_faulty[gi.fanin[1]];
                }

                uint32_t o0=0,o1=0;
                switch(gi.type){
                    case AND: o0=a0|b0; o1=a1&b1; break;
                    case OR:  o1=a1|b1; o0=a0&b0; break;
                    case NAND:o0=a1&b1; o1=a0|b0; break;
                    case NOR: o1=a0&b0; o0=a1|b1; break;
                    case INV: o0=a1; o1=a0; break;
                    case BUF: case PO: o0=a0; o1=a1; break;
                    default: continue;
                }

                if (o0 != val0_faulty[i] || o1 != val1_faulty[i]) {
                    // Overwrite & enqueue event ONLY when faulty != fault-free
                    val0_faulty[i] = o0;
                    val1_faulty[i] = o1;
                    for (int k=0;k<gi.num_fanout;k++)
                        event_queue[queue_end++] = gi.fanout[k];
                }
            }

            // Detection
            int detected = 0;
            for (int i=0; i<ckt->npo; i++){
                int po = ckt->po[i];
                if ((val0[po] ^ val0_faulty[po]) & (val1[po] ^ val1_faulty[po])) {
                    detected = 1;
                    break;
                }
            }

            // Fault dropping
            if (detected) {
                if (prev == NULL) undetected_flist = curr->next;
                else prev->next = curr->next;
                curr = (prev == NULL) ? undetected_flist : prev->next;
            } else {
                prev = curr;
                curr = curr->next;
            }
        }
    }

    return undetected_flist;
}











// Serial Baseline
// fault_list_t *three_val_fault_simulate(ckt,pat,undetected_flist)
//      circuit_t *ckt;
//      pattern_t *pat;
//      fault_list_t *undetected_flist;
// {
//     int ff_value[MAX_GATES];
//     int faulty_value[MAX_GATES];

//     for (int p = 0; p < pat->len; p++) {

//         // ================================
//         // 1. Fault-free simulation
//         // ================================
//         for (int i = 0; i < ckt->ngates; i++) {
//             ff_value[i] = LOGIC_X;      // initialize non-PI gates
//             faulty_value[i] = LOGIC_X;  // initialize faulty_value
//         }

//         for (int i = 0; i < ckt->npi; i++) {
//             ff_value[ckt->pi[i]] = pat->in[p][i];
//         }

//         for (int i = 0; i < ckt->ngates; i++) {
//             gate_t g = ckt->gate[i];
//             if (g.type == PI) continue;

//             int in0 = ff_value[g.fanin[0]];
//             int in1 = 0;
//             if (g.type == AND || g.type == OR || g.type == NAND || g.type == NOR)
//                 in1 = ff_value[g.fanin[1]];

//             int out;
//             switch (g.type) {
//                 case AND:
//                     if (in0 == LOGIC_0 || in1 == LOGIC_0) out = LOGIC_0;
//                     else if (in0 == LOGIC_1 && in1 == LOGIC_1) out = LOGIC_1;
//                     else out = LOGIC_X;
//                     break;
//                 case OR:
//                     if (in0 == LOGIC_1 || in1 == LOGIC_1) out = LOGIC_1;
//                     else if (in0 == LOGIC_0 && in1 == LOGIC_0) out = LOGIC_0;
//                     else out = LOGIC_X;
//                     break;
//                 case NAND:
//                     if (in0 == LOGIC_0 || in1 == LOGIC_0) out = LOGIC_1;
//                     else if (in0 == LOGIC_1 && in1 == LOGIC_1) out = LOGIC_0;
//                     else out = LOGIC_X;
//                     break;
//                 case NOR:
//                     if (in0 == LOGIC_1 || in1 == LOGIC_1) out = LOGIC_0;
//                     else if (in0 == LOGIC_0 && in1 == LOGIC_0) out = LOGIC_1;
//                     else out = LOGIC_X;
//                     break;
//                 case INV:
//                     if (in0 == LOGIC_0) out = LOGIC_1;
//                     else if (in0 == LOGIC_1) out = LOGIC_0;
//                     else out = LOGIC_X;
//                     break;
//                 case BUF:
//                 case PO:
//                     out = in0;
//                     break;
//                 case PO_GND:
//                     out = LOGIC_0;
//                     break;
//                 case PO_VCC:
//                     out = LOGIC_1;
//                     break;
//                 default:
//                     continue;
//             }

//             ff_value[i] = out;
//         }

//         for (int i = 0; i < ckt->npo; i++) {
//             pat->out[p][i] = ff_value[ckt->po[i]];
//         }

//         // ================================
//         // 2. Serial fault simulation
//         // ================================
//         fault_list_t *curr = undetected_flist;
//         fault_list_t *prev = NULL;

//         while (curr != NULL) {

//             // copy fault-free values
//             for (int i = 0; i < ckt->ngates; i++)
//                 faulty_value[i] = ff_value[i];

//             int g_idx = curr->gate_index;
//             int stuck_val = (curr->type == S_A_0) ? LOGIC_0 : LOGIC_1;

//             // ============================
//             // 3. Inject fault
//             // ============================
//             if (curr->input_index == -1) {
//                 // output fault
//                 faulty_value[g_idx] = stuck_val;
//             }

//             // ============================
//             // 4. Forward propagation
//             // ============================
//             int start = (curr->input_index == -1) ? g_idx + 1 : g_idx;
//             for (int i = start; i < ckt->ngates; i++) {
//                 gate_t g = ckt->gate[i];
//                 if (g.type == PI) continue;

//                 int in0 = faulty_value[g.fanin[0]];
//                 int in1 = 0;
//                 if (g.type == AND || g.type == OR || g.type == NAND || g.type == NOR)
//                     in1 = faulty_value[g.fanin[1]];

//                 // input fault injection
//                 if (i == g_idx && curr->input_index >= 0) {
//                     if (curr->input_index == 0) in0 = stuck_val;
//                     else if (curr->input_index == 1) in1 = stuck_val;
//                 }

//                 int out;
//                 switch (g.type) {
//                     case AND:
//                         if (in0 == LOGIC_0 || in1 == LOGIC_0) out = LOGIC_0;
//                         else if (in0 == LOGIC_1 && in1 == LOGIC_1) out = LOGIC_1;
//                         else out = LOGIC_X;
//                         break;
//                     case OR:
//                         if (in0 == LOGIC_1 || in1 == LOGIC_1) out = LOGIC_1;
//                         else if (in0 == LOGIC_0 && in1 == LOGIC_0) out = LOGIC_0;
//                         else out = LOGIC_X;
//                         break;
//                     case NAND:
//                         if (in0 == LOGIC_0 || in1 == LOGIC_0) out = LOGIC_1;
//                         else if (in0 == LOGIC_1 && in1 == LOGIC_1) out = LOGIC_0;
//                         else out = LOGIC_X;
//                         break;
//                     case NOR:
//                         if (in0 == LOGIC_1 || in1 == LOGIC_1) out = LOGIC_0;
//                         else if (in0 == LOGIC_0 && in1 == LOGIC_0) out = LOGIC_1;
//                         else out = LOGIC_X;
//                         break;
//                     case INV:
//                         if (in0 == LOGIC_0) out = LOGIC_1;
//                         else if (in0 == LOGIC_1) out = LOGIC_0;
//                         else out = LOGIC_X;
//                         break;
//                     case BUF:
//                     case PO:
//                         out = in0;
//                         break;
//                     case PO_GND:
//                         out = LOGIC_0;
//                         break;
//                     case PO_VCC:
//                         out = LOGIC_1;
//                         break;
//                     default:
//                         continue;
//                 }

//                 faulty_value[i] = out;
//             }

//             // ============================
//             // 5. Detection check
//             // ============================
//             int detected = 0;
//             for (int i = 0; i < ckt->npo; i++) {
//                 int po = ckt->po[i];
//                 if ((faulty_value[po] == LOGIC_0 && ff_value[po] == LOGIC_1) ||
//                     (faulty_value[po] == LOGIC_1 && ff_value[po] == LOGIC_0)) {
//                     detected = 1;
//                     // printf("Pattern %d detects fault at gate %d", p, curr->gate_index);
//                     // if (curr->input_index == -1) printf(" (output)");
//                     // else printf(" (input %d)", curr->input_index);
//                     // printf(" stuck-at-%d\n", (curr->type == S_A_0) ? 0 : 1);
//                     break;
//                 }
//             }

//             // ============================
//             // 6. Fault dropping (do not free)
//             // ============================
//             if (detected) {
//                 if (prev == NULL) undetected_flist = curr->next;
//                 else prev->next = curr->next;

//                 curr = (prev == NULL) ? undetected_flist : prev->next;
//             } else {
//                 prev = curr;
//                 curr = curr->next;
//             }
//         }
//     }

//     return undetected_flist;
// }













//     int eval_gate(gate_t g, int in0, int in1)
//     {
//         switch (g.type) {
//             case AND:
//                 if (in0 == LOGIC_0 || in1 == LOGIC_0) return LOGIC_0;
//                 if (in0 == LOGIC_1 && in1 == LOGIC_1) return LOGIC_1;
//                 return LOGIC_X;

//             case OR:
//                 if (in0 == LOGIC_1 || in1 == LOGIC_1) return LOGIC_1;
//                 if (in0 == LOGIC_0 && in1 == LOGIC_0) return LOGIC_0;
//                 return LOGIC_X;

//             case NAND:
//                 if (in0 == LOGIC_0 || in1 == LOGIC_0) return LOGIC_1;
//                 if (in0 == LOGIC_1 && in1 == LOGIC_1) return LOGIC_0;
//                 return LOGIC_X;

//             case NOR:
//                 if (in0 == LOGIC_1 || in1 == LOGIC_1) return LOGIC_0;
//                 if (in0 == LOGIC_0 && in1 == LOGIC_0) return LOGIC_1;
//                 return LOGIC_X;

//             case INV:
//                 if (in0 == LOGIC_0) return LOGIC_1;
//                 if (in0 == LOGIC_1) return LOGIC_0;
//                 return LOGIC_X;

//             case BUF:
//             case PO:
//                 return in0;

//             case PO_GND:
//                 return LOGIC_0;

//             case PO_VCC:
//                 return LOGIC_1;

//             default:
//                 return LOGIC_X;
//         }
//     }



// // ------------ //
// // Event-Driven //
// // -------------//

// fault_list_t *three_val_fault_simulate(
//     circuit_t *ckt,
//     pattern_t *pat,
//     fault_list_t *undetected_flist)
// {
//     int ff_value[MAX_GATES];
//     int faulty_value[MAX_GATES];
//     // -------------------------------
//     // Event-driven propagation
//     // -------------------------------
//     void propagate_event(int start_gate)
//     {
//         int queue[MAX_GATES];
//         int head = 0, tail = 0;

//         queue[tail++] = start_gate;

//         while (head < tail) {
//             int g_idx = queue[head++];
//             gate_t g = ckt->gate[g_idx];

//             // propagate to fanouts
//             for (int k = 0; k < g.num_fanout; k++) {
//                 int fo = g.fanout[k];
//                 gate_t fg = ckt->gate[fo];

//                 if (fg.type == PI) continue;

//                 int in0 = faulty_value[fg.fanin[0]];
//                 int in1 = 0;

//                 if (fg.type == AND || fg.type == OR ||
//                     fg.type == NAND || fg.type == NOR)
//                     in1 = faulty_value[fg.fanin[1]];

//                 int new_val = eval_gate(fg, in0, in1);

//                 // ONLY propagate if value changes
//                 if (new_val != faulty_value[fo]) {
//                     faulty_value[fo] = new_val;
//                     queue[tail++] = fo;

//                     // safety (optional)
//                     if (tail >= MAX_GATES) {
//                         printf("Queue overflow!\n");
//                         exit(1);
//                     }
//                 }
//             }
//         }
//     }

//     // ================================
//     // Pattern loop
//     // ================================
//     for (int p = 0; p < pat->len; p++) {

//         // ----------------------------
//         // 1. Fault-free simulation
//         // ----------------------------
//         for (int i = 0; i < ckt->ngates; i++) {
//             ff_value[i] = LOGIC_X;
//             faulty_value[i] = LOGIC_X;
//         }

//         // assign PI
//         for (int i = 0; i < ckt->npi; i++)
//             ff_value[ckt->pi[i]] = pat->in[p][i];

//         // levelized evaluation
//         for (int i = 0; i < ckt->ngates; i++) {
//             gate_t g = ckt->gate[i];
//             if (g.type == PI) continue;

//             int in0 = ff_value[g.fanin[0]];
//             int in1 = (g.type == AND || g.type == OR ||
//                        g.type == NAND || g.type == NOR)
//                       ? ff_value[g.fanin[1]] : 0;

//             ff_value[i] = eval_gate(g, in0, in1);
//         }

//         // copy outputs
//         for (int i = 0; i < ckt->npo; i++)
//             pat->out[p][i] = ff_value[ckt->po[i]];

//         // ----------------------------
//         // 2. Fault simulation
//         // ----------------------------
//         fault_list_t *curr = undetected_flist;
//         fault_list_t *prev = NULL;

//         while (curr != NULL) {

//             // copy good values
//             for (int i = 0; i < ckt->ngates; i++)
//                 faulty_value[i] = ff_value[i];

//             int g_idx = curr->gate_index;
//             int stuck_val = (curr->type == S_A_0) ? LOGIC_0 : LOGIC_1;

//             // ------------------------
//             // 3. Inject + propagate
//             // ------------------------
//             if (curr->input_index == -1) {
//                 // output fault
//                 if (faulty_value[g_idx] != stuck_val) {
//                     faulty_value[g_idx] = stuck_val;
//                     propagate_event(g_idx);
//                 }
//             } else {
//                 // input fault → recompute gate first
//                 gate_t g = ckt->gate[g_idx];

//                 int in0 = faulty_value[g.fanin[0]];
//                 int in1 = (g.type == AND || g.type == OR ||
//                            g.type == NAND || g.type == NOR)
//                           ? faulty_value[g.fanin[1]] : 0;

//                 if (curr->input_index == 0) in0 = stuck_val;
//                 else in1 = stuck_val;

//                 int new_val = eval_gate(g, in0, in1);

//                 if (new_val != faulty_value[g_idx]) {
//                     faulty_value[g_idx] = new_val;
//                     propagate_event(g_idx);
//                 }
//             }

//             // ------------------------
//             // 4. Detection check
//             // ------------------------
//             int detected = 0;
//             for (int i = 0; i < ckt->npo; i++) {
//                 int po = ckt->po[i];
//                 if ((faulty_value[po] == LOGIC_0 && ff_value[po] == LOGIC_1) ||
//                     (faulty_value[po] == LOGIC_1 && ff_value[po] == LOGIC_0)) {
//                     detected = 1;
//                     break;
//                 }
//             }

//             // ------------------------
//             // 5. Fault dropping
//             // ------------------------
//             if (detected) {
//                 if (prev == NULL)
//                     undetected_flist = curr->next;
//                 else
//                     prev->next = curr->next;

//                 curr = (prev == NULL) ? undetected_flist : prev->next;
//             } else {
//                 prev = curr;
//                 curr = curr->next;
//             }
//         }
//     }

//     return undetected_flist;
// }






















// ------------ //
// Bit-parallel //
// -------------//
// fault_list_t *
// three_val_fault_simulate(circuit_t *ckt, pattern_t *pat, fault_list_t *undetected_flist)
// {
//     const int W = 32;

//     uint32_t val0[MAX_GATES];  // bit=1 means logic 0
//     uint32_t val1[MAX_GATES];  // bit=1 means logic 1

//     int ff_value[MAX_GATES];

//     for (int p = 0; p < pat->len; p++) {

//         // =====================================================
//         // 1. Fault-free simulation (same as your original code)
//         // =====================================================
//         for (int i = 0; i < ckt->ngates; i++)
//             ff_value[i] = LOGIC_X;

//         for (int i = 0; i < ckt->npi; i++)
//             ff_value[ckt->pi[i]] = pat->in[p][i];

//         for (int i = 0; i < ckt->ngates; i++) {

//             gate_t g = ckt->gate[i];
//             if (g.type == PI) continue;

//             int in0 = ff_value[g.fanin[0]];
//             int in1 = (g.type == AND || g.type == OR || g.type == NAND || g.type == NOR)
//                       ? ff_value[g.fanin[1]] : 0;

//             int out;

//             switch (g.type) {
//                 case AND:
//                     if (in0 == LOGIC_0 || in1 == LOGIC_0) out = LOGIC_0;
//                     else if (in0 == LOGIC_1 && in1 == LOGIC_1) out = LOGIC_1;
//                     else out = LOGIC_X;
//                     break;
//                 case OR:
//                     if (in0 == LOGIC_1 || in1 == LOGIC_1) out = LOGIC_1;
//                     else if (in0 == LOGIC_0 && in1 == LOGIC_0) out = LOGIC_0;
//                     else out = LOGIC_X;
//                     break;
//                 case NAND:
//                     if (in0 == LOGIC_0 || in1 == LOGIC_0) out = LOGIC_1;
//                     else if (in0 == LOGIC_1 && in1 == LOGIC_1) out = LOGIC_0;
//                     else out = LOGIC_X;
//                     break;
//                 case NOR:
//                     if (in0 == LOGIC_1 || in1 == LOGIC_1) out = LOGIC_0;
//                     else if (in0 == LOGIC_0 && in1 == LOGIC_0) out = LOGIC_1;
//                     else out = LOGIC_X;
//                     break;
//                 case INV:
//                     if (in0 == LOGIC_0) out = LOGIC_1;
//                     else if (in0 == LOGIC_1) out = LOGIC_0;
//                     else out = LOGIC_X;
//                     break;
//                 case BUF:
//                 case PO:
//                     out = in0;
//                     break;
//                 case PO_GND:
//                     out = LOGIC_0;
//                     break;
//                 case PO_VCC:
//                     out = LOGIC_1;
//                     break;
//                 default:
//                     continue;
//             }

//             ff_value[i] = out;
//         }
//         for (int i = 0; i < ckt->npo; i++) {
//             pat->out[p][i] = ff_value[ckt->po[i]];
//         }

//         // =====================================================
//         // 2. Bit-parallel fault simulation
//         // =====================================================

//         fault_list_t *curr = undetected_flist;

//         while (curr) {

//             // ===============================
//             // 1. Pack faults into a batch
//             // ===============================
//             fault_list_t *batch[W];
//             int nf = 0;
//             fault_list_t *tmp = curr;

//             while (tmp && nf < W) {
//                 batch[nf++] = tmp;
//                 tmp = tmp->next;
//             }

//             // ===============================
//             // 2. Initialize from fault-free
//             // ===============================
//             for (int i = 0; i < ckt->ngates; i++) {
//                 if (ff_value[i] == LOGIC_0) {
//                     val0[i] = 0xFFFFFFFF;
//                     val1[i] = 0;
//                 } else if (ff_value[i] == LOGIC_1) {
//                     val0[i] = 0;
//                     val1[i] = 0xFFFFFFFF;
//                 } else {
//                     val0[i] = 0;
//                     val1[i] = 0;
//                 }
//             }

//             // ===============================
//             // 3. Inject PI faults FIRST
//             // ===============================
//             for (int f = 0; f < nf; f++) {
//                 int g = batch[f]->gate_index;

//                 if (ckt->gate[g].type == PI) {
//                     uint32_t mask = (1u << f);

//                     if (batch[f]->type == S_A_0) {
//                         val0[g] |= mask;
//                         val1[g] &= ~mask;
//                     } else {
//                         val0[g] &= ~mask;
//                         val1[g] |= mask;
//                     }
//                 }
//             }

//             // ===============================
//             // 4. Propagate through circuit
//             // ===============================
//             for (int i = 0; i < ckt->ngates; i++) {

//                 gate_t g = ckt->gate[i];
//                 if (g.type == PI) continue;

//                 uint32_t a0 = val0[g.fanin[0]];
//                 uint32_t a1 = val1[g.fanin[0]];
//                 uint32_t b0 = 0, b1 = 0;

//                 if (g.type == AND || g.type == OR || g.type == NAND || g.type == NOR) {
//                     b0 = val0[g.fanin[1]];
//                     b1 = val1[g.fanin[1]];
//                 }

//                 // ---- input fault injection ----
//                 for (int f = 0; f < nf; f++) {
//                     if (batch[f]->gate_index == i &&
//                         batch[f]->input_index >= 0) {

//                         uint32_t mask = (1u << f);

//                         if (batch[f]->type == S_A_0) {
//                             if (batch[f]->input_index == 0) {
//                                 a0 |= mask; a1 &= ~mask;
//                             } else {
//                                 b0 |= mask; b1 &= ~mask;
//                             }
//                         } else {
//                             if (batch[f]->input_index == 0) {
//                                 a0 &= ~mask; a1 |= mask;
//                             } else {
//                                 b0 &= ~mask; b1 |= mask;
//                             }
//                         }
//                     }
//                 }

//                 uint32_t o0 = 0, o1 = 0;

//                 // ---- gate evaluation ----
//                 switch (g.type) {
//                     case AND:
//                         o0 = a0 | b0;
//                         o1 = a1 & b1;
//                         break;
//                     case OR:
//                         o1 = a1 | b1;
//                         o0 = a0 & b0;
//                         break;
//                     case NAND:
//                         o0 = a1 & b1;
//                         o1 = a0 | b0;
//                         break;
//                     case NOR:
//                         o1 = a0 & b0;
//                         o0 = a1 | b1;
//                         break;
//                     case INV:
//                         o0 = a1;
//                         o1 = a0;
//                         break;
//                     case BUF:
//                     case PO:
//                         o0 = a0;
//                         o1 = a1;
//                         break;
//                     case PO_GND:
//                         o0 = 0xFFFFFFFF;
//                         o1 = 0;
//                         break;
//                     case PO_VCC:
//                         o0 = 0;
//                         o1 = 0xFFFFFFFF;
//                         break;
//                     default:
//                         continue;
//                 }

//                 // ---- output fault injection ----
//                 for (int f = 0; f < nf; f++) {
//                     if (batch[f]->gate_index == i &&
//                         batch[f]->input_index == -1) {

//                         uint32_t mask = (1u << f);

//                         if (batch[f]->type == S_A_0) {
//                             o0 |= mask;
//                             o1 &= ~mask;
//                         } else {
//                             o0 &= ~mask;
//                             o1 |= mask;
//                         }
//                     }
//                 }

//                 val0[i] = o0;
//                 val1[i] = o1;
//             }

//             // ===============================
//             // 5. Detection
//             // ===============================
//             int detected[W];
//             for (int f = 0; f < W; f++) detected[f] = 0;

//             for (int f = 0; f < nf; f++) {

//                 for (int i = 0; i < ckt->npo; i++) {
//                     int po = ckt->po[i];

//                     int good = ff_value[po];

//                     int faulty;
//                     if ((val0[po] >> f) & 1) faulty = LOGIC_0;
//                     else if ((val1[po] >> f) & 1) faulty = LOGIC_1;
//                     else faulty = LOGIC_X;

//                     // CORRECT detection condition
//                     if (good != LOGIC_X && faulty != LOGIC_X && good != faulty) {
//                         detected[f] = 1;
//                         break;
//                     }
//                 }
//             }

//             // ===============================
//             // 6. Fault dropping
//             // ===============================
//             fault_list_t *prev = NULL;
//             fault_list_t *scan = undetected_flist;

//             while (scan) {

//                 int remove = 0;
//                 for (int f = 0; f < nf; f++) {
//                     if (batch[f] == scan && detected[f]) {
//                         remove = 1;
//                         break;
//                     }
//                 }

//                 if (remove) {
//                     if (prev == NULL)
//                         undetected_flist = scan->next;
//                     else
//                         prev->next = scan->next;

//                     scan = (prev == NULL) ? undetected_flist : prev->next;
//                 } else {
//                     prev = scan;
//                     scan = scan->next;
//                 }
//             }

//             curr = tmp;
//         }
//     }

//     return undetected_flist;
// }














// PPSFP

// fault_list_t *
// three_val_fault_simulate(circuit_t *ckt, pattern_t *pat, fault_list_t *undetected_flist)
// {
//     const int W = 32; // batch size = 32 patterns per word
//     uint32_t val0[MAX_GATES]; // bit=1 means logic 0
//     uint32_t val1[MAX_GATES]; // bit=1 means logic 1
//     int ff_value[MAX_GATES];  // temporary for gate evaluation per batch

//     int nwords = (pat->len + W - 1) / W; // number of pattern batches
//     int event_queue[MAX_GATES];
//     int queue_start, queue_end;

//     // Loop over batches of patterns
//     for (int w = 0; w < nwords; w++) {
//         int start = w * W;
//         int end = (start + W > pat->len) ? pat->len : (start + W);
//         uint32_t mask_batch = (end - start < W) ? ((1U << (end - start)) - 1) : 0xFFFFFFFF;

//         // =====================================
//         // 1. Fault-free simulation for this batch
//         // =====================================
//         for (int i = 0; i < ckt->ngates; i++)
//             val0[i] = val1[i] = 0;

//         for (int p = start; p < end; p++) {
//             int bit = 1U << (p - start);
//             for (int pi = 0; pi < ckt->npi; pi++) {
//                 int gate = ckt->pi[pi];
//                 if (pat->in[p][pi] == LOGIC_0) val0[gate] |= bit;
//                 else if (pat->in[p][pi] == LOGIC_1) val1[gate] |= bit;
//                 // LOGIC_X: leave val0/val1 as 0
//             }
//         }

//         // Propagate fault-free through the circuit
//         for (int i = 0; i < ckt->ngates; i++) {
//             gate_t g = ckt->gate[i];
//             if (g.type == PI) continue;

//             uint32_t a0 = val0[g.fanin[0]], a1 = val1[g.fanin[0]];
//             uint32_t b0 = 0, b1 = 0;
//             if (g.type == AND || g.type == OR || g.type == NAND || g.type == NOR) {
//                 b0 = val0[g.fanin[1]];
//                 b1 = val1[g.fanin[1]];
//             }

//             uint32_t o0=0, o1=0;
//             switch(g.type) {
//                 case AND: o0 = a0 | b0; o1 = a1 & b1; break;
//                 case OR:  o1 = a1 | b1; o0 = a0 & b0; break;
//                 case NAND:o0 = a1 & b1; o1 = a0 | b0; break;
//                 case NOR: o1 = a0 & b0; o0 = a1 | b1; break;
//                 case INV: o0 = a1; o1 = a0; break;
//                 case BUF: case PO: o0 = a0; o1 = a1; break;
//                 case PO_GND: o0=mask_batch; o1=0; break;
//                 case PO_VCC: o0=0; o1=mask_batch; break;
//                 default: continue;
//             }
//             val0[i] = o0;
//             val1[i] = o1;
//         }

//         // Store fault-free outputs into pat->out
//         for (int p = start; p < end; p++) {
//             int bit = 1U << (p - start);
//             for (int po = 0; po < ckt->npo; po++) {
//                 int gate = ckt->po[po];
//                 if ((val1[gate] & bit)) pat->out[p][po] = LOGIC_1;
//                 else if ((val0[gate] & bit)) pat->out[p][po] = LOGIC_0;
//                 else pat->out[p][po] = LOGIC_X;
//             }
//         }

//         // =====================================
//         // 2. Fault simulation: one fault at a time per batch
//         // =====================================
//         fault_list_t *prev = NULL;
//         fault_list_t *curr = undetected_flist;

//         while (curr) {
//             fault_list_t *f = curr;
//             int g = f->gate_index;
//             int input_index = f->input_index;
//             stuck_val_t type = f->type;

//             // Copy fault-free batch
//             uint32_t val0_batch[MAX_GATES], val1_batch[MAX_GATES];
//             for (int i = 0; i < ckt->ngates; i++) {
//                 val0_batch[i] = val0[i];
//                 val1_batch[i] = val1[i];
//             }

//             // ------------------------
//             // PI fault injection BEFORE propagation
//             // ------------------------
//             if (ckt->gate[g].type == PI && input_index == -1) {
//                 if (type == S_A_0) { val0_batch[g] |= mask_batch; val1_batch[g] &= ~mask_batch; }
//                 else { val0_batch[g] &= ~mask_batch; val1_batch[g] |= mask_batch; }
//             }

//             // ------------------------
//             // Propagate gates
//             // ------------------------
//             for (int i = 0; i < ckt->ngates; i++) {
//                 gate_t gatei = ckt->gate[i];
//                 if (gatei.type == PI) continue;

//                 uint32_t a0 = val0_batch[gatei.fanin[0]];
//                 uint32_t a1 = val1_batch[gatei.fanin[0]];
//                 uint32_t b0 = 0, b1 = 0;
//                 if (gatei.type == AND || gatei.type == OR || gatei.type == NAND || gatei.type == NOR) {
//                     b0 = val0_batch[gatei.fanin[1]];
//                     b1 = val1_batch[gatei.fanin[1]];
//                 }

//                 if(i == g && input_index >= 0){
//                   uint32_t mask = mask_batch;
//                   if(input_index == 0){
//                     if(f->type == S_A_0) {a0 |= mask; a1 &= ~mask;}
//                     else {a0 &= ~mask; a1 |= mask;}
//                   } else{
//                     if(f->type == S_A_0) {b0 |= mask; b1 &= ~mask;}
//                     else {b0 &= ~mask; b1 |= mask;}
//                   }
//                 }

//                 uint32_t o0=0, o1=0;
//                 switch(gatei.type) {
//                     case AND: o0 = a0 | b0; o1 = a1 & b1; break;
//                     case OR:  o1 = a1 | b1; o0 = a0 & b0; break;
//                     case NAND:o0 = a1 & b1; o1 = a0 | b0; break;
//                     case NOR: o1 = a0 & b0; o0 = a1 | b1; break;
//                     case INV: o0 = a1; o1 = a0; break;
//                     case BUF: case PO: o0 = a0; o1 = a1; break;
//                     case PO_GND: o0=mask_batch; o1=0; break;
//                     case PO_VCC: o0=0; o1=mask_batch; break;
//                     default: continue;
//                 }

//                 if(f->gate_index == i && f->input_index == -1){
//                   uint32_t mask = mask_batch;
//                   if(f->type == S_A_0){o0 |= mask; o1 &= ~mask;}
//                   else {o0 &= ~mask; o1 |= mask;}
//                 }

//                 val0_batch[i] = o0;
//                 val1_batch[i] = o1;
//             }

//             // ------------------------
//             // Detection
//             // ------------------------
//             int detected = 0;
//             for(int i = 0; i < ckt->npo; i++){
//               int po = ckt->po[i];
//               if((val0[po] ^ val0_batch[po]) & (val1[po] ^ val1_batch[po])){
//                 detected = 1;
//                 break;
//               } 
//             }

//             // ------------------------
//             // Fault dropping
//             // ------------------------
//             if (detected) {
//                 if (prev == NULL) undetected_flist = curr->next;
//                 else prev->next = curr->next;
//                 curr = (prev == NULL) ? undetected_flist : prev->next;
//             } else {
//                 prev = curr;
//                 curr = curr->next;
//             }
//         } // end while undetected faults
//     } // end pattern batch

//     return undetected_flist;
// }













































