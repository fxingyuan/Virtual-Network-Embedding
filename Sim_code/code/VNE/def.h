/*
 * Def.h
 *
 *  Created on: May 15, 2015
 *      Author: Lixing (based on nmmkchow)
 */


#ifndef DEF_H_
#define DEF_H_

#define DEBUG 0

//#define MYCPP
#define MY_INFINITY         (INT_MAX / 2)
#define EPSILON             1E-6

#define SUCCESS             0
#define NODE_MAP_SUCCESS    0
#define EDGE_MAP_SUCCESS    0
#define BOTH_MAP_SUCCESS    0

#define COULD_NOT_OPEN_FILE -1

#define NOT_MAPPED_YET      -1

#define BOTH_MAP_FAILED     -1
#define NODE_MAP_FAILED     -2
#define EDGE_MAP_FAILED     -3

// node mapping methods
#define INVALID_NM_METHOD   -1
#define NM_GREEDY_BEST_FIT  0
#define NM_GREEDY_WORST_FIT 1             //
#define NM_RANDOM           2
#define NM_RANDOM_P2_BEST   3
#define NM_RANDOM_P2_WORST  4
#define NM_DETERMINISTIC    5            //
#define NM_RANDOMIZED       6           //
//my_defination
#define RECONFIG_T               500  //÷ÿ≈‰÷√÷‹∆⁄
#define MY_ONESTAGE_MAPPING       7
#define MY_NODE_MAPPING_DEPTH     8
#define MY_NODE_MAPPING_BREADTH   9
#define MY_NEW_NODE_MAPPING 10
#define DAMPING 0.15
// VNode ordering
#define VNODE_ORDER_ASC   0
#define VNODE_ORDER_DESC  1
#define VNODE_ORDER_RAND  2

// edge mapping methods
#define INVALID_EM_METHOD   -1
#define EM_GREEDY_BEST_FIT  0
#define EM_GREEDY_WORST_FIT 1
#define EM_RANDOM           2
#define EM_RANDOM_P2_BEST   3
#define EM_RANDOM_P2_WORST  4
#define EM_MCF              5

// VNode ordering
#define VEDGE_ORDER_ASC   0
#define VEDGE_ORDER_DESC  1
#define VEDGE_ORDER_RAND  2

#define MAX_PATHS_TO_CONSIDER 20

#define LEN_FILENAME 128

#define MAX_CPU 100
#define MAX_BW  100

#define META_EDGE_BW  1000000

#define MAX_NODES_IN_A_SUBSTRATE  1000
#define MAX_NODES_IN_A_REQUEST    100

// topology types
#define TOPO_GENERAL  0
#define TOPO_STAR     1
#define TOPO_TREE     2
#define TOPO_HUBS     3

#define LINK_SPLITTABLE   1
#define LINK_UNSPLITTABLE 0

#define xJ_STATICS_FILENAME "Temp_xJ.txt"
#define KSP_INPUT_FILENAME  "ksp_input.txt"

#define CNLM_MIP_MODEL_FILE "MIPs/CNLM-MIP.mod"
#define CNLM_LP_MODEL_FILE  "MIPs/CNLM-LP.mod"
#define CNLM_DATA_FILE      "MIPs/CNLM.dat"
#define CNLM_OUTPUT_FILE    "MIPs/CNLM.out"

#define MCF_MODEL_FILE     "MIPs/MCF.mod"
#define MCF_DATA_FILE      "MIPs/MCF.dat"
#define MCF_OUTPUT_FILE    "MIPs/MCF.out"

#endif /* DEF_H_ */
