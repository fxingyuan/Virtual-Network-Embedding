/*
 * Utility.h
 *
 *  Created on: May 15, 2015
 *      Author: Lixing (based on nmmkchow)
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <cassert>
#include <cmath>
#include <malloc.h>
#include <iostream>
#include <fstream>
#include <deque>
#include <vector>
#include <map>
#include <time.h>
#include "def.h"

#include "QYKShortestPaths.h"	// k-shortest-path



using namespace std;
using namespace asu_emit_qyan;

class Node;
class Edge;

class SubstrateNode;
class SubstrateEdge;
class SubstrateGraph;

class VNNode;
class VNEdge;
class VNRequest;

// function declarations
void printMapping(VNRequest &VNR, SubstrateGraph &SG);
void randomPermutation(vector<int> &series);

void getDifferentStress(SubstrateGraph &SG, double &mNS, double &aNS,
    double &mLS, double &aLS, double &sdNS, double &sdLS);

double getRevenue(VNRequest &aRequest, double __MULT,
    double &nodeRev, double &edgeRev);

double getCost(VNRequest &VNR, SubstrateGraph &SG, double __MULT,
    double &nodeCost, double &edgeCost, bool aOne, bool bOne);

class Node {
public:
  int x, y;
  //double connectivity,con0,sum_adj_con0,pagerfd;
  //con0初始连通度
  //connectivity为markov连通度
  double cpu;

  Node(int _x, int _y, double _cpu);

#ifdef MYCPP
  Node(const Node &o);
  virtual ~Node();
  const Node& operator=(const Node &o);
#endif
};

class Edge {
public:
  int from, to;
  //double connectivity,con0,sum_adj_con0,pagerfd;
  double bw, dlay;

  Edge(int _from, int _to, double _bw, double _dlay);

#ifdef MYCPP
  Edge(const Edge &o);
  virtual ~Edge();
  const Edge& operator=(const Edge &o);
#endif
};

class SubstrateNode: public Node {
public:
  int count;
  double rest_cpu;
  bool touched;
  double ratio_temp;
  double ratio,resi_ratio,rfd;
  double xJ,connectivity,con0,sum_adj_con0,pagerfd;
  //con0初始连通度
  //connectivity为markov连通度
  vector<int> edgeIDs;

  vector<int> req_ids;
  vector<int> node_ids;
  vector<double> used_cpu;

  SubstrateNode(int _x, int _y, double _cpu);

#ifdef MYCPP
  SubstrateNode(const SubstrateNode &o);
  virtual ~SubstrateNode();
  const SubstrateNode& operator=(const SubstrateNode &o);
#endif

  double distanceFrom(Node &aNode);
  double distanceFrom(SubstrateNode &aNode);
};

class SubstrateEdge: public Edge {
public:
  int count;
  double rest_bw;
  double ratio_temp;
  double ratio,resi_ratio;//ratio占用比例，resi_ratio剩余资源比例
  double xJ,connectivity,con0,sum_adj_con0,pagerfd;
  double rfd;
  vector<int> req_ids;
  vector<int> edge_ids;
  vector<double> used_bw;

  SubstrateEdge(int _from, int _to, double _bw, double _dlay);

#ifdef MYCPP
  SubstrateEdge(const SubstrateEdge &o);
  virtual ~SubstrateEdge();
  const SubstrateEdge& operator=(const SubstrateEdge &o);
#endif
};

class SubstrateGraph {
public:
  string fileName;
  int nodeNum, edgeNum, substrateID;
  vector<SubstrateNode> nodes;
  vector<SubstrateEdge> edges;
  map< pair<int, int>, size_t > hop;
  map< pair<int, int>, int > edgeMap;
  //my defination
  void calcu_status();
  double init_node_con(int nodeID);
  double init_edge_con(int edgeID);
  double Edge_rfd(int edgeID);
  double Node_rfd(int nodeID);
  void inithop();
  void initBias();
  bool fileinitBias();
  void clean(VNRequest &aRequest,bool tag=true);
  void rank();
  void pagerank(double sum_alln_con0,double sum_alll_con0);
  int judge (double* backup,int is_node);
  void my_sortNodesAscending(vector<int> &nodeProcessOrder);
  void my_sortNodesdescending(vector<int> &nodeProcessOrder);
  void my_sortEdgesAscending(vector<int> &nodeProcessOrder);
  void my_sortEdgesdescending(vector<int> &nodeProcessOrder);
  int my_nodemap(VNRequest &aRequest,int mappingMethod);
  int my_new_nodemap(VNRequest &aRequest);
  double calculate_nbandwidth(int nodeID);
  int getshortestpath(int n_from, int n_to, double bw,CQYDirectedPath &shortest_path);
  double getRFD_cost(CQYDirectedPath &path);
  double get_newRFD_cost(CQYDirectedPath &path);
  void findNb_Node(int nodeID,vector<int> &Nb_VNodes, vector<int> &Nb_Edges);
  int calucate_dis(int tempSnode,vector<int> Snodeset);
  int calucate_dis_Weighted(VNRequest &aRequest,int tempSnode[],int len,vector<int> Snodeset,vector<int> Nb_edges);
  int firstnodemap(VNRequest &aRequest,int nodeID);
  int new_node_and_path(VNRequest &aRequest,int nodeID,vector<int> &SNodes,int &vnodeID);
  int node_and_path(VNRequest &aRequest,int nodeID,vector<int> &SNodes,int &vnodeID);
  int Costlowest(VNRequest &aRequest,int nodeID,vector<int> &SNodes);
  void reconfiguration(vector<VNRequest> &VNR, int curtime,vector<int> &re_reqID);
  //bool lastConfig(VNRequest aRequest,vector<int> FailNode,vector<int> FailEdge);
  //my defination
  void addVNMapping(VNRequest &aRequest);
  void removeVNMapping(const VNRequest &aRequest);
 
  
  int mapNodes(VNRequest &aRequest, int VNodeOrdering = VNODE_ORDER_ASC,
      int mappingMethod = NM_GREEDY_BEST_FIT);
  int mapEdges(VNRequest &aRequest, int VEdgeOrdering = VEDGE_ORDER_ASC,
      int mappingMethod = EM_GREEDY_BEST_FIT);

  int mapNodes_ViNE(VNRequest &aRequest, int nodeMapStyle, bool aOne, bool bOne);
  int mapEdges_ViNE(VNRequest &aRequest, bool aOne, bool bOne);

  int mapNodes_ViNE_v2(VNRequest &aRequest, int nodeMapStyle, bool aOne, bool bOne);

  int mapByMIP(VNRequest &aRequest);
  void get_hops_n(int* dist,int s_node);//by lzx
  //SubstrateGraph() {};
  SubstrateGraph(string _fileName, int _substrateID = -1);

#ifdef MYCPP
  SubstrateGraph(const SubstrateGraph &o);
  virtual ~SubstrateGraph();
  const SubstrateGraph& operator=(const SubstrateGraph &o);
#endif

  int initGraph();

  int findNodesWithinConstraints(Node &aNode, int reqID, int maxD,
      vector<int> &validNodeIDs);
  double getNodePotential(int nodeID);
  double getPathPotential(CQYDirectedPath* &aPath);
  double getPathPotentialRfd(CQYDirectedPath* &aPath,double bw);
  void createKSPInputFile(double maxBW);
  void create_xJ_StaticFile();
  int solveMultiCommodityFlow(VNRequest &aRequest,
      int VNodeOrdering = VNODE_ORDER_ASC);

  void printNodeStatus();
  void printEdgeStatus();
};

class VNNode: public Node {
public:
  int substrateID;
  int subNodeID;
  int touched;
  vector<int> edgeIDs;

  VNNode(int _x, int _y, double _cpu);

#ifdef MYCPP
  VNNode(const VNNode &o);
  virtual ~VNNode();
  const VNNode& operator=(const VNNode &o);
#endif
};

class VNEdge: public Edge {
public:
  int substrateID;
  int pathLen;
  double pathDelay;
  int touched;
  vector < int> subPath;
  vector <double> subBW;
  
  VNEdge(int _from, int _to, double _bw, double _dlay);

#ifdef MYCPP
  VNEdge(const VNEdge &o);
  virtual ~VNEdge();
  const VNEdge& operator=(const VNEdge &o);
#endif
};

class VNRequest {
public:
  string fileName;
  int split, time, duration, topology, maxD;
  double revenue;
  int nodeNum, edgeNum, reqID;

  vector<int> Reconfig_nodes;
  vector<VNNode> nodes;
  vector<VNEdge> edges;
  map< pair<int, int>, int > edgeMap;
  VNRequest(string _fileName, int _reqID);

#ifdef MYCPP
  VNRequest(const VNRequest &o);
  virtual ~VNRequest();
  const VNRequest& operator=(const VNRequest &o);
#endif

  int initGraph();
  int  findmax_degree();
  void getCost(double &nodecost,double &edgecost);
 // bool topology();
  void findNb_Node(int nodeID,vector<int> &Nb_VNodes, vector<int> &Nb_Edges);
  double calculate_nbandwidth(int nodeID);
  void breadth_searching(int nodeID,vector<int> &nodeProcessor);
  void depth_searching(int nodeID,vector<int> &nodeProcessor);
  void my_sortNodesAscending(vector<int> &nodeProcessOrder);
  void my_sortNodesdescending(vector<int> &nodeProcessOrder);
  void my_sortEdgesAscending(vector<int> &nodeProcessOrder);
  void my_sortEdgesdescending(vector<int> &nodeProcessOrder);

  void sortNodesAscending(vector<int> &nodeProcessOrder);
  void sortNodesDescending(vector<int> &nodeProcessOrder);
  void sortEdgesAscending(vector<int> &edgeProcessOrder);
  void sortEdgesDescending(vector<int> &edgeProcessOrder);
};

class VNEdgeComparerASC {
public:
  bool operator()(const VNEdge& n1, const VNEdge& n2) const {
    return (n1.bw < n2.bw);
  }
};

class VNEdgeComparerDESC {
public:
  bool operator()(const VNEdge& n1, const VNEdge& n2) const {
    return (n1.bw > n2.bw);
  }
};

#endif /* UTILITY_H_ */
