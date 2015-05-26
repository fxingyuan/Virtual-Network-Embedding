/*
 * Utility.cpp
 *
 *  Created on: May 17, 2015
 *      Author: Lixing
 */

#include "utility.h"
#include "def.h"

///////////////////////////////////////////////////////////////////////////////
// Virtual Network Embedding Functions                                                   //
// Description: 虚拟网络映射功能函数                                                             //
///////////////////////////////////////////////////////////////////////////////

void printMapping(VNRequest &VNR, SubstrateGraph &SG) {
  int i, j;

  // considering only one InP

  cout << "Node Mapping" << endl;
  for (i = 0; i < VNR.nodeNum; i++) {
    printf("VNode: %2d @(%2d, %2d) CPU = %.4lf <--> PNode: %2d @(%2d, %2d) RCPU = \n",
        i, VNR.nodes[i].x, VNR.nodes[i].y, VNR.nodes[i].cpu,
        /*VNR.nodes[i].substrateID,*/ VNR.nodes[i].subNodeID,
        SG.nodes[VNR.nodes[i].subNodeID].x, SG.nodes[VNR.nodes[i].subNodeID].y,
        SG.nodes[VNR.nodes[i].subNodeID].rest_cpu);
  }

  cout << "Edge Mapping" << endl;
  for (i = 0; i < VNR.edgeNum; i++) {
    printf("VEdge: %2d @(%2d, %2d) BW = %.4lf <--> PEdges:",
        i, VNR.edges[i].from, VNR.edges[i].to, VNR.edges[i].bw);
    for (j = 0; j < VNR.edges[i].pathLen; j++) {
      printf(" %3d[%.4lf] @(%2d, %2d)", VNR.edges[i].subPath[j],
          VNR.edges[i].subBW[j],
          SG.edges[VNR.edges[i].subPath[j]].from,
          SG.edges[VNR.edges[i].subPath[j]].to);
    }
    printf("\n");
  }
}

void getDifferentStress(SubstrateGraph &SG, double &mNS, double &aNS, double &mLS, double &aLS,
    double &sdNS, double &sdLS) {
		//统计底层物理网络负载状况
  int i;
  double tStress;
  mNS = mLS = aNS = aLS = sdNS = sdLS = 0;

  for (i = 0; i < SG.nodeNum; i++) {
    tStress = (SG.nodes[i].cpu - SG.nodes[i].rest_cpu) / SG.nodes[i].cpu;
    if (tStress > mNS) {
      mNS = tStress;
    }
    aNS += tStress;
  }
  aNS /= SG.nodeNum;

  for (i = 0; i < SG.nodeNum; i++) {
    tStress = (SG.nodes[i].cpu - SG.nodes[i].rest_cpu) / SG.nodes[i].cpu;
    sdNS += ((aNS - tStress) * (aNS - tStress));
  }
  sdNS = sqrt((sdNS / SG.nodeNum));

  for (i = 0; i < SG.edgeNum; i++) {
    tStress = (SG.edges[i].bw - SG.edges[i].rest_bw) / SG.edges[i].bw;
    if (tStress > mLS) {
      mLS = tStress;
    }
    aLS += tStress;
  }
  aLS /= SG.edgeNum;

  for (i = 0; i < SG.edgeNum; i++) {
    tStress = (SG.edges[i].bw - SG.edges[i].rest_bw) / SG.edges[i].bw;
    sdLS += ((aLS - tStress) * (aLS - tStress));
  }
  sdLS = sqrt((sdLS / SG.edgeNum));
}

double getRevenue(VNRequest &aRequest, double __MULT,
    double &nodeRev, double &edgeRev) {
		//虚拟网络收益

  int i;
  nodeRev = edgeRev = 0;

  for (i = 0; i < aRequest.nodeNum; i++) {
    nodeRev += aRequest.nodes[i].cpu;
  }

  for (i = 0; i < aRequest.edgeNum; i++) {
    edgeRev += aRequest.edges[i].bw;
  }

  return nodeRev + __MULT * edgeRev;
}

double getCost(VNRequest &VNR, SubstrateGraph &SG, double __MULT,
    double &nodeCost, double &edgeCost, bool aOne, bool bOne) {
		//虚拟网络代价
  int i, j;
  double temp;
  nodeCost = edgeCost = 0;

  for (i = 0; i < VNR.nodeNum; i++) {
    /*if (bOne) {
      temp = 1.0 / (SG.nodes[VNR.nodes[i].subNodeID].rest_cpu + EPSILON)
        * VNR.nodes[i].cpu;
    }
    else {*/
      temp = VNR.nodes[i].cpu;
    //}
    nodeCost += temp;
  }

  for (i = 0; i < VNR.edgeNum; i++) {
    temp = 0;
    for (j = 0; j < VNR.edges[i].pathLen; j++) {
      /*if (aOne) {
        temp += (1.0 / (SG.edges[VNR.edges[i].subPath[j]].rest_bw + EPSILON)
          * VNR.edges[i].subBW[j]);
      }
      else {*/
        temp += VNR.edges[i].subBW[j];
      //}
    }
    edgeCost += temp;
  }

  return nodeCost + __MULT * edgeCost;
}

void randomPermutation(vector<int> &series) {
	//产生随机序列
  int j;
  size_t sLen = series.size();
  vector<int> temp(sLen, 0), ret;

  while(sLen) {
    j = rand() % sLen;
    ret.push_back(series[j]);
    series[j] = series[--sLen];
  }

  ret.resize(series.size());
  copy(ret.begin(), ret.end(), series.begin());
}

///////////////////////////////////////////////////////////////////////////////
// Class      : Node                                                         //
// Description:  位置x，y  资源cpu                                                           //
///////////////////////////////////////////////////////////////////////////////

Node::Node(int _x, int _y, double _cpu) :
  x(_x), y(_y), cpu(_cpu) {
  if(DEBUG) {
    cout << "Node" << endl;
  }
}

#ifdef MYCPP
Node::Node(const Node &o) :
  x(o.x), y(o.y), cpu(o.cpu){
  if(DEBUG) {
    cout << "Node Copy" << endl;
  }
}

Node::~Node() {
  if(DEBUG) {
    cout << "~Node" << endl;
  }
}

const Node& Node::operator =(const Node &o) {
  if(DEBUG) {
    cout << "Node=" << endl;
  }
  if (this != &o) {
    x = o.x;
    y = o.y;
    cpu = o.cpu;
  }

  return *this;
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Class      : Edge                                                         //
// Description:  端点from，to     资源：bw   链路特性：延迟dlay                                                            //
///////////////////////////////////////////////////////////////////////////////

Edge::Edge(int _from, int _to, double _bw, double _dlay) :
  from(_from), to(_to), bw(_bw), dlay(_dlay) {
  if(DEBUG) {
    cout << "Edge" << endl;
  }
}

#ifdef MYCPP
Edge::Edge(const Edge &o) :
  from(o.from), to(o.to), bw(o.bw), dlay(o.dlay) {
  if(DEBUG) {
    cout << "Edge Copy" << endl;
  }
}

Edge::~Edge() {
  if(DEBUG) {
    cout << "~Edge" << endl;
  }
}

const Edge& Edge::operator=(const Edge &o) {
  if(DEBUG) {
    cout << "Edge=" << endl;
  }
  if (this != &o) {
    from = o.from;
    to = o.to;
    bw = o.bw;
    dlay = o.dlay;
  }

  return *this;
}
#endif

//////////////////////////////////////////////////////////////////////////////
// Class      : SubstrateGraph                                              //
// Description: 底层物理网络初始化											//
//              initGraph()读取subXXXX.txt，构造底层网络拓扑；				//
//				inithop()初始化节点距离										//
//				initBias()初始化全局因子									//
//////////////////////////////////////////////////////////////////////////////

SubstrateGraph::SubstrateGraph(string _fileName, int _substrateID) :
  fileName(_fileName), substrateID(_substrateID) {
	//底层物理网络构造函数
  if(DEBUG) {
    cout << "SubGraph" << endl;
  }

  nodeNum = 0;
  edgeNum = 0;

  nodes.clear();
  edges.clear();

  edgeMap.clear();
  //底层物理网络初始化
  initGraph();
  inithop();
  initBias();
  
}

#ifdef MYCPP
SubstrateGraph::SubstrateGraph(const SubstrateGraph &o) {
  if(DEBUG) {
    cout << "SubGraph Copy" << endl;
  }

  nodeNum = o.nodeNum;
  edgeNum = o.edgeNum;
  substrateID = o.substrateID;
  fileName = o.fileName;

  //copy(o.nodes.begin(), o.nodes.end(), nodes.begin());
  for (int i = 0; i < o.nodeNum; i++)
    nodes.push_back(o.nodes[i]);
  //copy(o.edges.begin(), o.edges.end(), edges.begin());
  for (int i = 0; i < edgeNum; i++)
    edges.push_back(o.edges[i]);
}

SubstrateGraph::~SubstrateGraph() {
  if(DEBUG) {
    cout << "~SubGraph" << endl;
  }

  nodes.clear();
  edges.clear();
}

const SubstrateGraph& SubstrateGraph::operator=(const SubstrateGraph &o) {
  if(DEBUG) {
    cout << "SubGraph=" << endl;
  }

  if (this != &o) {
    nodeNum = o.nodeNum;
    edgeNum = o.edgeNum;
    substrateID = o.substrateID;
    fileName = o.fileName;

    //copy(o.nodes.begin(), o.nodes.end(), nodes.begin());
    for (int i = 0; i < o.nodeNum; i++)
      nodes.push_back(o.nodes[i]);
    //copy(o.edges.begin(), o.edges.end(), edges.begin());
    for (int i = 0; i < edgeNum; i++)
      edges.push_back(o.edges[i]);
  }

  return *this;
}
#endif

void SubstrateGraph::createKSPInputFile(double maxBW) {
//创建KSP文件  用于k-shortest-paths计算k最短路径
  int i;
  ofstream ofs;
  ofs.open(KSP_INPUT_FILENAME, ios::out);

  if (!ofs) {
    cout << "failed to open file: " << KSP_INPUT_FILENAME << endl;
    exit(COULD_NOT_OPEN_FILE);
  }

  ofs << nodeNum << endl << endl;
  for (i = 0; i < edgeNum; i++) {
    if (edges[i].rest_bw >= maxBW) {
      ofs << edges[i].from << " " << edges[i].to << " " << edges[i].dlay << endl;
      ofs << edges[i].to << " " << edges[i].from << " " << edges[i].dlay << endl;
    }
  }

  ofs.close();
}

void SubstrateGraph::printNodeStatus() {
//输出节点状态：承载虚拟节点数量，剩余CPU
  int i;
  cout << "Substrate Graph: " << substrateID <<  " Node Status" << endl;
  for (i = 0; i < nodeNum; i++) {
    printf("(%2d, %6.3lf)", nodes[i].count, nodes[i].rest_cpu);
  }
  cout << endl;
}

void SubstrateGraph::printEdgeStatus() {
//输出链路状态：承载虚拟链路数量，剩余CPU
  int i;
  cout << "Substrate Graph: " << substrateID <<  " Edge Status" << endl;
  for (i = 0; i < edgeNum; i++) {
    printf("(%2d, %6.3lf)", edges[i].count, edges[i].rest_bw);
  }
  cout << endl;
}
double SubstrateGraph::Node_rfd(int nodeID){
//计算节点局部碎片度：首先计算连通度
	int i;
	double edge_ratio,node_ratio,rfd=0;
	size_t nb_size=nodes[nodeID].edgeIDs.size();
	for(i=0;i<nb_size;i++){
		int cur_edgeID=nodes[nodeID].edgeIDs[i];
		edge_ratio=1-edges[cur_edgeID].ratio_temp;
		//printf("edge_ratio=%lf\n",edge_ratio);
		if(edges[cur_edgeID].from!=nodeID){
			node_ratio=1-nodes[edges[cur_edgeID].from].ratio_temp;
			//printf("node_ratio=%lf\n",node_ratio);
			rfd+=node_ratio*edge_ratio/nb_size;
		}
		else{
			node_ratio=1-nodes[edges[cur_edgeID].to].ratio_temp;
			rfd+=node_ratio*edge_ratio/nb_size;
		}
	}
	//printf("rfd=%lf\n",1-rfd);
	return 1-rfd; //返回局部碎片度
}
double SubstrateGraph::Edge_rfd(int edgeID){
//计算链路碎片度：首先计算连通度
	int i;
	int nb_from=edges[edgeID].from,nb_to=edges[edgeID].to;
	double node_ratio,rfd=0;
	node_ratio=1-nodes[nb_from].ratio_temp;
	for(i=0;i<nodes[nb_from].edgeIDs.size();i++){
		rfd+=node_ratio*(1-edges[nodes[nb_from].edgeIDs[i]].ratio_temp);
	}
	rfd-=node_ratio*(1-edges[edgeID].ratio_temp);
	node_ratio=1-nodes[nb_to].ratio_temp;
	for(i=0;i<nodes[nb_to].edgeIDs.size();i++){
		rfd+=node_ratio*(1-edges[nodes[nb_to].edgeIDs[i]].ratio_temp);
	}
	rfd-=node_ratio*(1-edges[edgeID].ratio_temp);
	rfd/=nodes[nb_from].edgeIDs.size()+nodes[nb_to].edgeIDs.size()-2;
	//printf("rfd=%lf\n",1-rfd);
	return 1-rfd; //返回链路碎片度
}
void SubstrateGraph::my_sortNodesAscending(vector<int> &nodeProcessOrder) {
//节点按照碎片度 升序排序
  int i, j;
  for (i = 0; i < nodeNum; i++) {
    for (j = i + 1; j < nodeNum; j++) {
		if (nodes[nodeProcessOrder[i]].rfd > nodes[nodeProcessOrder[j]].rfd) {
        int t = nodeProcessOrder[i];
        nodeProcessOrder[i] = nodeProcessOrder[j];
        nodeProcessOrder[j] = t;
      }
    }
  }
}
void SubstrateGraph::my_sortNodesdescending(vector<int> &nodeProcessOrder) {
//节点按照碎片度 降序排序
  int i, j;
  for (i = 0; i < nodeNum; i++) {
    for (j = i + 1; j < nodeNum; j++) {
		if (nodes[nodeProcessOrder[i]].rfd < nodes[nodeProcessOrder[j]].rfd) {
        int t = nodeProcessOrder[i];
        nodeProcessOrder[i] = nodeProcessOrder[j];
        nodeProcessOrder[j] = t;
      }
    }
  }
}
void SubstrateGraph::my_sortEdgesAscending(vector<int> &edgeProcessOrder) {
//链路按照碎片度 升序排序
  int i, j;
  for (i = 0; i < edgeNum; i++) {
    for (j = i + 1; j < edgeNum; j++) {
		if (edges[edgeProcessOrder[i]].rfd > edges[edgeProcessOrder[j]].rfd) {
        int t = edgeProcessOrder[i];
        edgeProcessOrder[i] = edgeProcessOrder[j];
        edgeProcessOrder[j] = t;
      }
    }
  }
}

void SubstrateGraph::my_sortEdgesdescending(vector<int> &edgeProcessOrder) {
//链路按照碎片度 降序排序
  int i, j;
  for (i = 0; i < edgeNum; i++) {
    for (j = i + 1; j < edgeNum; j++) {
		if (edges[edgeProcessOrder[i]].rfd< edges[edgeProcessOrder[j]].rfd) {
        int t = edgeProcessOrder[i];
        edgeProcessOrder[i] = edgeProcessOrder[j];
        edgeProcessOrder[j] = t;
      }
    }
  }
}

void SubstrateGraph::addVNMapping(VNRequest &aRequest) {
//添加虚拟网络
  int i, j;
  // add to the substrate network nodes
  for (i = 0; i < aRequest.nodeNum; i++) {
    if (aRequest.nodes[i].substrateID == substrateID) {
      nodes[aRequest.nodes[i].subNodeID].rest_cpu -= aRequest.nodes[i].cpu;
      assert(nodes[aRequest.nodes[i].subNodeID].rest_cpu + EPSILON >= 0);
      nodes[aRequest.nodes[i].subNodeID].count++;

      // save request information
      nodes[aRequest.nodes[i].subNodeID].req_ids.push_back(aRequest.reqID);
      nodes[aRequest.nodes[i].subNodeID].node_ids.push_back(i);
      nodes[aRequest.nodes[i].subNodeID].used_cpu.push_back(aRequest.nodes[i].cpu);
    }
	nodes[aRequest.nodes[i].subNodeID].ratio = 1-nodes[aRequest.nodes[i].subNodeID].rest_cpu/nodes[aRequest.nodes[i].subNodeID].cpu;
	nodes[aRequest.nodes[i].subNodeID].ratio_temp=nodes[aRequest.nodes[i].subNodeID].ratio;
	nodes[aRequest.nodes[i].subNodeID].rfd=Node_rfd(aRequest.nodes[i].subNodeID);
  }

  // add to the substrate network links
  for (i = 0; i < aRequest.edgeNum; i++) {
    for (j = 0; j < aRequest.edges[i].pathLen; j++) {
      if (aRequest.edges[i].substrateID == substrateID) {
        edges[aRequest.edges[i].subPath[j]].rest_bw -= aRequest.edges[i].subBW[j];
        assert(edges[aRequest.edges[i].subPath[j]].rest_bw + EPSILON >= 0);
        edges[aRequest.edges[i].subPath[j]].count++;

        // save request information
        edges[aRequest.edges[i].subPath[j]].req_ids.push_back(aRequest.reqID);
        edges[aRequest.edges[i].subPath[j]].edge_ids.push_back(i);
        edges[aRequest.edges[i].subPath[j]].used_bw.push_back(aRequest.edges[i].subBW[j]);
      }
	  edges[aRequest.edges[i].subPath[j]].ratio=1-edges[aRequest.edges[i].subPath[j]].rest_bw/edges[aRequest.edges[i].subPath[j]].bw;
	  edges[aRequest.edges[i].subPath[j]].ratio_temp=edges[aRequest.edges[i].subPath[j]].ratio;
	  edges[aRequest.edges[i].subPath[j]].rfd=Edge_rfd(aRequest.edges[i].subPath[j]);
    }
  }
}

void SubstrateGraph::removeVNMapping(const VNRequest &aRequest) {
//移除虚拟网络
  int i, j;
  vector<int>::iterator iter1, iter2;
  vector<double>::iterator iter3;
  // remove from the substrate network nodes
  for (i = 0; i < aRequest.nodeNum; i++) {
    if (aRequest.nodes[i].substrateID == substrateID) {
      //remove request information
      iter1 = nodes[aRequest.nodes[i].subNodeID].req_ids.begin();
      iter2 = nodes[aRequest.nodes[i].subNodeID].node_ids.begin();
      iter3 = nodes[aRequest.nodes[i].subNodeID].used_cpu.begin();
	  assert(nodes[aRequest.nodes[i].subNodeID].req_ids.size()==nodes[aRequest.nodes[i].subNodeID].node_ids.size()&&nodes[aRequest.nodes[i].subNodeID].node_ids.size()==nodes[aRequest.nodes[i].subNodeID].used_cpu.size());
      while (iter1 != nodes[aRequest.nodes[i].subNodeID].req_ids.end()) {
        if (*iter1 == aRequest.reqID) {
			nodes[aRequest.nodes[i].subNodeID].used_cpu.erase(iter3);
			nodes[aRequest.nodes[i].subNodeID].req_ids.erase(iter1);
			nodes[aRequest.nodes[i].subNodeID].node_ids.erase(iter2);
			nodes[aRequest.nodes[i].subNodeID].rest_cpu += aRequest.nodes[i].cpu;
			assert(nodes[aRequest.nodes[i].subNodeID].rest_cpu <= nodes[aRequest.nodes[i].subNodeID].cpu + EPSILON);
			nodes[aRequest.nodes[i].subNodeID].count--;
          break;	 // can break, since only one VNode can map to a PNode
        }
        iter1++;
        iter2++;
        iter3++;
      }
	  nodes[aRequest.nodes[i].subNodeID].ratio=1-nodes[aRequest.nodes[i].subNodeID].rest_cpu/nodes[aRequest.nodes[i].subNodeID].cpu;
	  nodes[aRequest.nodes[i].subNodeID].ratio_temp=nodes[aRequest.nodes[i].subNodeID].ratio;
	  nodes[aRequest.nodes[i].subNodeID].rfd=Node_rfd(aRequest.nodes[i].subNodeID);
    }
  }

  // remove from the substrate network links
  for (i = 0; i < aRequest.edgeNum; i++) {
    for (j = 0; j < aRequest.edges[i].pathLen; j++) {
      if (aRequest.edges[i].substrateID == substrateID) {
        //remove request information
        iter1 = edges[aRequest.edges[i].subPath[j]].req_ids.begin();
        iter2 = edges[aRequest.edges[i].subPath[j]].edge_ids.begin();
        iter3 = edges[aRequest.edges[i].subPath[j]].used_bw.begin();
        while (iter1 != edges[aRequest.edges[i].subPath[j]].req_ids.end()) {
          if (*iter1 == aRequest.reqID && *iter2 == i) {
            edges[aRequest.edges[i].subPath[j]].req_ids.erase(iter1);
            edges[aRequest.edges[i].subPath[j]].edge_ids.erase(iter2);
            edges[aRequest.edges[i].subPath[j]].used_bw.erase(iter3);
			edges[aRequest.edges[i].subPath[j]].rest_bw += aRequest.edges[i].subBW[j];
			assert(edges[aRequest.edges[i].subPath[j]].rest_bw <= edges[aRequest.edges[i].subPath[j]].bw + EPSILON);
			edges[aRequest.edges[i].subPath[j]].count--;

            break;
          }
          iter1++;
          iter2++;
          iter3++;
        }
		edges[aRequest.edges[i].subPath[j]].ratio=1-edges[aRequest.edges[i].subPath[j]].rest_bw/edges[aRequest.edges[i].subPath[j]].bw;
		edges[aRequest.edges[i].subPath[j]].ratio_temp=edges[aRequest.edges[i].subPath[j]].ratio;
		edges[aRequest.edges[i].subPath[j]].rfd=Edge_rfd(aRequest.edges[i].subPath[j]);
      }
	}
  }
}
/*
//虚拟网络映射失败时，重配置  接口....
bool SubstrateGraph::lastConfig(VNRequest aRequest,vector<int> FailNode,vector<int> FailEdge)
{
	//添加虚拟网络映射失败时，重配置操作......待完成
	int i,vID;
	vector<int> SNodes;
	for(i=0;i<aRequest.Reconfig_nodes.size();i++)
	{
		vID=aRequest.Reconfig_nodes[i];

	}
}
*/
void SubstrateGraph::calcu_status()
{
//校验底层物理网络节点和链路上  状态信息  （属于代码调试辅助函数，检查错误）
	int i,j;
	double used_cpu=0, total_cpu=0,res_cpu=0,used_bw=0,res_bw=0,total_bw=0;
	for(i=0;i<nodeNum;i++)
	{
		for(j=0;j<nodes[i].used_cpu.size();j++)
			used_cpu=used_cpu+nodes[i].used_cpu[j];
		total_cpu+=nodes[i].cpu;
		res_cpu+=nodes[i].rest_cpu;

	}
	for(i=0;i<edgeNum;i++)
	{
		for(j=0;j<edges[i].used_bw.size();j++)
			used_bw+=edges[i].used_bw[j];
		total_bw+=edges[i].bw;
		res_bw+=edges[i].rest_bw;
	}
	assert((res_cpu+used_cpu<=total_cpu+0.1)&&(res_cpu+used_cpu-0.1<=total_cpu));
	assert((res_bw+used_bw<=total_bw+0.1)&&(res_bw+used_bw-0.1<=total_bw));
	cout<<"used_cpu:"<<used_cpu<<", total_cpu:"<<total_cpu<<endl;
	cout<<"used_bw:"<<used_bw<<", total_bw:"<<total_bw<<endl;
}

void SubstrateGraph::reconfiguration(vector<VNRequest> &VNR,int curtime, vector<int> &re_reqID) {
//虚拟网络重配置算法
  int i, j, nID, vID;
  double max_rfd,nodecost,edgecost;
  vector<int>::iterator iter1, iter2, iter4, iter5;		//节点 链路 状态 迭代器
  vector<double>::iterator iter3, iter6;
  //search_max_fragmented resources max_load
  max_rfd=nodes[0].pagerfd*nodes[0].rest_cpu*nodes[0].count;
  nID=0;	//重配置节点
  for(i=1;i<nodeNum;i++)
  {
	  if(nodes[i].pagerfd*nodes[i].rest_cpu*nodes[i].count>max_rfd)
	  {
		  assert(nodes[i].count==nodes[i].req_ids.size());
		  max_rfd=nodes[i].pagerfd*nodes[i].rest_cpu*nodes[i].count;
		  nID=i;
	  }

  }
	if(nodes[nID].req_ids.size()==0)
		return;
	iter1 = nodes[nID].req_ids.begin();
	iter2 = nodes[nID].node_ids.begin();
    iter3 = nodes[nID].used_cpu.begin();
	assert(nodes[nID].req_ids.size()==nodes[nID].node_ids.size()&&nodes[nID].node_ids.size()==nodes[nID].used_cpu.size());
	size_t k=0,len=nodes[nID].req_ids.size();//底层物理节点承载的 虚拟网络（虚拟节点）数量
	while (k<len) {
		//对每一个虚拟节点进行重配置操作
		VNRequest Retemp=VNR[(*iter1)];
		VNRequest &aRequest=VNR[(*iter1)];
		if((curtime-aRequest.time)*1.0/aRequest.duration<0.6){//对剩余生存时间率 大于0.4的虚拟节点进行重配置
			cout<<"Pre config SN status..............."<<endl;	//统计重配置前  虚拟网络映射状态
			calcu_status(); 
			aRequest.getCost(nodecost,edgecost);
			cout<<"Pre config VN cost............"<<endl;
			cout<<"reqID:"<<aRequest.reqID<<"nodecost:"<<nodecost<<",edgecost:"<<edgecost<<endl;
			//remove nodes and nb links
			vID=*iter2;	//虚拟节点ID
			//初始化 底层物理网络与 待重配置虚拟网络 的节点映射关系（原则：同一虚拟网络中，多个虚拟节点不能映射到一个底层物理节点上）
			for(i=0;i<nodeNum;i++){
			nodes[i].touched=false;
			}
			for(i=0;i<aRequest.nodeNum;i++)
			{
				if(i!=vID) nodes[aRequest.nodes[i].subNodeID].touched=true;
			}
			//.........................................
			//Node  移除当前虚拟节点在 物理节点上的 配置信息
			nodes[nID].rest_cpu += aRequest.nodes[vID].cpu;
			assert(nodes[nID].rest_cpu <= nodes[nID].cpu + EPSILON);
			nodes[nID].count--;
			iter1=nodes[nID].req_ids.erase(iter1);
			iter2=nodes[nID].node_ids.erase(iter2);
			iter3=nodes[nID].used_cpu.erase(iter3);
			assert(nodes[nID].req_ids.size()==nodes[nID].node_ids.size()&&nodes[nID].node_ids.size()==nodes[nID].used_cpu.size());
			k++;
			nodes[nID].ratio=1-nodes[nID].rest_cpu/nodes[nID].cpu;
		  
			//Edge 移除虚拟节点邻接链路  在SN上的配置信息
			for(i=0;i<aRequest.nodes[vID].edgeIDs.size();i++){
				int edgeID=aRequest.nodes[vID].edgeIDs[i];
				for (j = 0; j < aRequest.edges[edgeID].pathLen; j++) {
					if (aRequest.edges[edgeID].substrateID == substrateID) {
					edges[aRequest.edges[edgeID].subPath[j]].rest_bw += aRequest.edges[edgeID].subBW[j];
					assert(edges[aRequest.edges[edgeID].subPath[j]].rest_bw <= edges[aRequest.edges[edgeID].subPath[j]].bw + EPSILON);
					edges[aRequest.edges[edgeID].subPath[j]].count--;

					//remove request information
					iter4 = edges[aRequest.edges[edgeID].subPath[j]].req_ids.begin();
					iter5 = edges[aRequest.edges[edgeID].subPath[j]].edge_ids.begin();
					iter6 = edges[aRequest.edges[edgeID].subPath[j]].used_bw.begin();
					while (iter4 != edges[aRequest.edges[edgeID].subPath[j]].req_ids.end()) {
						if (*iter4 == aRequest.reqID && *iter5 == edgeID) {
						edges[aRequest.edges[edgeID].subPath[j]].req_ids.erase(iter4);
						edges[aRequest.edges[edgeID].subPath[j]].edge_ids.erase(iter5);
						edges[aRequest.edges[edgeID].subPath[j]].used_bw.erase(iter6);

						break;
						}
						iter4++;
						iter5++;
						iter6++;
					}
					edges[aRequest.edges[edgeID].subPath[j]].ratio=1-edges[aRequest.edges[edgeID].subPath[j]].rest_bw/edges[aRequest.edges[edgeID].subPath[j]].bw;
					}
					
				}
				aRequest.edges[edgeID].subPath.clear();
				aRequest.edges[edgeID].subBW.clear();
				aRequest.edges[edgeID].pathLen=0;
				aRequest.edges[edgeID].touched=false;
			}

			
			//计算节点vID的重配置
			rank();
			//nodes[nID].touched=false;
			int iterators=0;
			vector<int> SNodes;
			int SNodeCount,tempNodeID;
			SNodes.clear();
			SNodeCount = findNodesWithinConstraints(aRequest.nodes[vID],  //备选x,y,cpu物理节点集合
			aRequest.reqID, aRequest.maxD, SNodes);
			if (SNodeCount == 0){
				std::cout<<"For virtual node %d:   "<<vID;
				cerr << "findSNodesWithinConstraints() returned 0" << endl;
				assert(SNodeCount==0);
				return  ;
			}
			int Fail_Nb_nID=-1;
			tempNodeID=new_node_and_path(aRequest,vID,SNodes,Fail_Nb_nID);
			if(tempNodeID<0){
			//未找到 重配置方案 则恢复之前的配置信息
				for(i=0;i<aRequest.nodes[vID].edgeIDs.size();i++){
					int edgeID=aRequest.nodes[vID].edgeIDs[i];
					aRequest.edges[edgeID].pathLen=Retemp.edges[edgeID].pathLen;
					for (j = 0; j < Retemp.edges[edgeID].pathLen; j++) {
						aRequest.edges[edgeID].subPath.push_back(Retemp.edges[edgeID].subPath[j]);
						aRequest.edges[edgeID].subBW.push_back(Retemp.edges[edgeID].subPath[j]);
					}
				}
					tempNodeID=nID;

			}
			std::cout<<"Map<"<<vID<<","<<tempNodeID<<">"<<std::endl;
			if(tempNodeID!=nID) re_reqID.push_back(aRequest.reqID);		//新配置节点tempNodeID与原配置节点nID不同时， 加入重配置队列re_reqID，用于更新重配置后 虚拟网络映射代价
			// 为底层节点tempNodeID和虚拟节点vID 更新映射信息
			aRequest.nodes[vID].substrateID = substrateID;
			aRequest.nodes[vID].subNodeID = tempNodeID;
			nodes[aRequest.nodes[vID].subNodeID].touched = true;
			aRequest.nodes[vID].touched=true;
			nodes[aRequest.nodes[vID].subNodeID].rest_cpu -= aRequest.nodes[vID].cpu;
			nodes[aRequest.nodes[vID].subNodeID].ratio=1.0-nodes[aRequest.nodes[vID].subNodeID].rest_cpu/nodes[aRequest.nodes[vID].subNodeID].cpu;
			nodes[aRequest.nodes[vID].subNodeID].count++;
			// save request information
			nodes[aRequest.nodes[vID].subNodeID].req_ids.push_back(aRequest.reqID);
			nodes[aRequest.nodes[vID].subNodeID].node_ids.push_back(vID);
			nodes[aRequest.nodes[vID].subNodeID].used_cpu.push_back(aRequest.nodes[vID].cpu);
			//计算重配置后代价....................................
			cout<<"After config SN status..............."<<endl;
			calcu_status();
			aRequest.getCost(nodecost,edgecost);
			cout<<"After config VN cost............"<<endl;
			cout<<"reqID:"<<aRequest.reqID<<"nodecost:"<<nodecost<<",edgecost:"<<edgecost<<endl;
		}
		else
		{
			iter1++;
			iter2++;
			iter3++;
			k++;
		}
	}
}

int SubstrateGraph::mapNodes(VNRequest &aRequest, int VNodeOrdering,
    int mappingMethod) {

  int i, j;
  int validNodeCount;
  vector<int> validNodesWithinReach;
  vector<int> nodeProcessOrder;

  // make all substrate nodes untouched
  for (i = 0; i < nodeNum; i++) {
    nodes[i].touched = false;
  }

  for (i = 0; i < aRequest.nodeNum; i++) {
    nodeProcessOrder.push_back(i);
  }

  // set the virtual nodes' processing order
  switch (VNodeOrdering) {
  case VNODE_ORDER_ASC:
    aRequest.sortNodesAscending(nodeProcessOrder);
    break;
  case VNODE_ORDER_DESC:
    aRequest.sortNodesDescending(nodeProcessOrder);
    break;
  case VNODE_ORDER_RAND:
    randomPermutation(nodeProcessOrder);
    break;
  default:
    break;
  }

  /*for (i = 0; i < aRequest.nodeNum; i++)
    cout << nodeProcessOrder[i] << " ";
  cout << endl;*/

  // map the virtual nodes using specific algorithms
  switch (mappingMethod) {

  case NM_GREEDY_BEST_FIT:

    for (i = 0; i < aRequest.nodeNum; i++) {
      validNodeCount = findNodesWithinConstraints(aRequest.nodes[nodeProcessOrder[i]],
          aRequest.reqID, aRequest.maxD, validNodesWithinReach);

      if (validNodeCount == 0) {
        cerr << "findNodesWithinConstraints() returned 0" << endl;
        break;
      }

      // find the substrate node with the minimum potential
      int minPos = NOT_MAPPED_YET;
      //double minVal = INFINITY;
	  double minVal = std::numeric_limits<double>::max();//wu

      for (j = 0; j < validNodeCount; j++) {
        double nodePotential = getNodePotential(validNodesWithinReach[j]);
        if (nodePotential < minVal) {
          minVal = nodePotential;
          minPos = validNodesWithinReach[j];
        }
      }

      assert (minPos != NOT_MAPPED_YET);

      // update the virtual node's mapping information
      aRequest.nodes[nodeProcessOrder[i]].substrateID = substrateID;
      aRequest.nodes[nodeProcessOrder[i]].subNodeID = minPos;

      nodes[aRequest.nodes[nodeProcessOrder[i]].subNodeID].touched = true;
    }

    break;

  case NM_GREEDY_WORST_FIT:

    for (i = 0; i < aRequest.nodeNum; i++) {
      validNodeCount = findNodesWithinConstraints(aRequest.nodes[nodeProcessOrder[i]],
          aRequest.reqID, aRequest.maxD, validNodesWithinReach);

      if (validNodeCount == 0) {
        cerr << "findNodesWithinConstraints() returned 0" << endl;
        break;
      }

      // find the substrate node with the maximum potential
      int maxPos = NOT_MAPPED_YET;
      double maxVal = 0;

      for (j = 0; j < validNodeCount; j++) {
        double nodePotential = getNodePotential(validNodesWithinReach[j]);
        if (nodePotential > maxVal) {
          maxVal = nodePotential;
          maxPos = validNodesWithinReach[j];
        }
      }

      assert (maxPos != NOT_MAPPED_YET);

      // update the virtual node's mapping information
      aRequest.nodes[nodeProcessOrder[i]].substrateID = substrateID;
      aRequest.nodes[nodeProcessOrder[i]].subNodeID = maxPos;

      nodes[aRequest.nodes[nodeProcessOrder[i]].subNodeID].touched = true;
    }

    break;

  case NM_RANDOM:

    for (i = 0; i < aRequest.nodeNum; i++) {
      validNodeCount = findNodesWithinConstraints(aRequest.nodes[nodeProcessOrder[i]],
          aRequest.reqID, aRequest.maxD, validNodesWithinReach);

      if (validNodeCount == 0) {
        cerr << "findNodesWithinConstraints() returned 0" << endl;
        break;
      }

      // find the substrate node with random potential
      int randPos = rand() % validNodeCount;

      // update the virtual node's mapping information
      aRequest.nodes[nodeProcessOrder[i]].substrateID = substrateID;
      aRequest.nodes[nodeProcessOrder[i]].subNodeID = validNodesWithinReach[randPos];

      nodes[aRequest.nodes[nodeProcessOrder[i]].subNodeID].touched = true;
    }

    break;

  case NM_RANDOM_P2_BEST:

    for (i = 0; i < aRequest.nodeNum; i++) {
      validNodeCount = findNodesWithinConstraints(aRequest.nodes[nodeProcessOrder[i]],
          aRequest.reqID, aRequest.maxD, validNodesWithinReach);

      if (validNodeCount == 0) {
        cerr << "findNodesWithinConstraints() returned 0" << endl;
        break;
      }

      // find the substrate node with random potential
      int randPos1, randPos2, maxPos;
      double randVal1, randVal2;

      // select two random id from 'validNodesWithinReach'
      randPos1 = rand() % validNodeCount;
      randPos2 = rand() % validNodeCount;

      randVal1 = getNodePotential(randPos1);
      randVal2 = getNodePotential(randPos2);

      maxPos = (randVal1 > randVal2) ? randPos1 : randPos2;

      // update the virtual node's mapping information
      aRequest.nodes[nodeProcessOrder[i]].substrateID = substrateID;
      aRequest.nodes[nodeProcessOrder[i]].subNodeID = validNodesWithinReach[maxPos];

      nodes[aRequest.nodes[nodeProcessOrder[i]].subNodeID].touched = true;
    }

    break;

  case NM_RANDOM_P2_WORST:

    for (i = 0; i < aRequest.nodeNum; i++) {
      validNodeCount = findNodesWithinConstraints(aRequest.nodes[nodeProcessOrder[i]],
          aRequest.reqID, aRequest.maxD, validNodesWithinReach);

      if (validNodeCount == 0) {
        cerr << "findNodesWithinConstraints() returned 0" << endl;
        break;
      }

      // find the substrate node with random potential
      int randPos1, randPos2, maxPos;
      double randVal1, randVal2;

      // select two random id from 'validNodesWithinReach'
      randPos1 = rand() % validNodeCount;
      randPos2 = rand() % validNodeCount;

      randVal1 = getNodePotential(randPos1);
      randVal2 = getNodePotential(randPos2);

      maxPos = (randVal1 < randVal2) ? randPos1 : randPos2;

      // update the virtual node's mapping information
      aRequest.nodes[nodeProcessOrder[i]].substrateID = substrateID;
      aRequest.nodes[nodeProcessOrder[i]].subNodeID = validNodesWithinReach[maxPos];

      nodes[aRequest.nodes[nodeProcessOrder[i]].subNodeID].touched = true;
    }

    break;

  default:
    return INVALID_NM_METHOD;

    break;
  }

  if (i != aRequest.nodeNum) {
    return NODE_MAP_FAILED;
  }

  return NODE_MAP_SUCCESS;
}

double SubstrateGraph::getNodePotential(int nodeID) {
  // TODO: implement better node potential function

  return nodes[nodeID].rest_cpu;
}
double SubstrateGraph::getPathPotentialRfd(CQYDirectedPath* &aPath,double bw) {
  // TODO: implement better path potential and Rfd function
  // now using average bandwidth of the path

  int i, len;
  const vector<int> vertexList(aPath->GetVertexList());
  len = vertexList.size();
  //pre_allocation
  for (i = 1; i < len; i++) {
    int eID = edgeMap[make_pair(vertexList[i - 1], vertexList[i])];
	edges[eID].rest_bw-=bw;
	assert(edges[eID].rest_bw + EPSILON >0);
  }
  //
  double RfdCost=0.;
  for (i = 1; i < len; i++) {
    int eID = edgeMap[make_pair(vertexList[i - 1], vertexList[i])];
	printf("%lf, %lf\n",edges[eID].rest_bw,Edge_rfd(eID));
	RfdCost += edges[eID].rest_bw*Edge_rfd(eID);
  }
  
  //reset
  for (i = 1; i < len; i++) {
    int eID = edgeMap[make_pair(vertexList[i - 1], vertexList[i])];
	edges[eID].rest_bw+=bw;
	assert(edges[eID].rest_bw <edges[eID].bw+ EPSILON);
  }
  return RfdCost;
}
double SubstrateGraph::getPathPotential(CQYDirectedPath* &aPath) {
  // TODO: implement better path potential function
  // now using average bandwidth of the path

  int i, len;
  const vector<int> vertexList(aPath->GetVertexList());
  len = vertexList.size();
  double minBW = MAX_BW + 1., maxBW = 0., avgBW = 0.;

  for (i = 1; i < len; i++) {
    int eID = edgeMap[make_pair(vertexList[i - 1], vertexList[i])];
    if (edges[eID].bw < minBW) {
      minBW = edges[eID].bw;
    }

    if (edges[eID].bw > maxBW) {
      maxBW = edges[eID].bw;
    }

    avgBW += edges[eID].bw;
  }
  avgBW /= (len - 1);

  return avgBW;
}

int SubstrateGraph::findNodesWithinConstraints(Node &aNode, int reqID, int maxD,
    vector<int> &validNodeIDs) {

  int i, count = 0;
  validNodeIDs.clear();

  for (i = 0; i < nodeNum; i++) {
    if ((nodes[i].distanceFrom(aNode) <= maxD) &&
        (nodes[i].rest_cpu >= aNode.cpu) && (nodes[i].touched == false)) {
      validNodeIDs.push_back(i);
      count++;
    }
  }

  return count;
}
double SubstrateGraph::calculate_nbandwidth(int nodeID){
//计算物理节点nodeID的邻接带宽总量
	int i;
	double nb_bw=0;
	for(i=0;i<nodes[nodeID].edgeIDs.size();i++)
		nb_bw+=edges[nodes[nodeID].edgeIDs[i]].rest_bw;
	return nb_bw;
}
int SubstrateGraph::getshortestpath(int n_from, int n_to, double bw,CQYDirectedPath &shortest_path){
//获取n_from到n_to，带宽为bw的最短路径
	int len=10000000;
	createKSPInputFile(bw);	//重建网络拓扑
	CQYDirectedGraph dg(KSP_INPUT_FILENAME);
	CQYKShortestPaths ksp(dg, n_from, n_to, MAX_PATHS_TO_CONSIDER);
	vector<CQYDirectedPath*> topK_shortest_paths = ksp.GetTopKShortestPaths();
	if (topK_shortest_paths.empty()) {
		cerr << "topK_shortest_paths is empty" << endl;
		return -1;
	}
	CQYDirectedPath* minPos = NULL;
	double minVal = std::numeric_limits<double>::max();
	for (vector<CQYDirectedPath*>::iterator topKIter = topK_shortest_paths.begin();
		topKIter != topK_shortest_paths.end();  ++topKIter) {
		double pathPotential = getPathPotential(*topKIter);
		if((*topKIter)->GetLength()<len)
		{
			minVal = pathPotential;
            minPos = *topKIter;
			len=(*topKIter)->GetLength();
		}
		else
        if ((*topKIter)->GetLength()==len&&pathPotential < minVal) {
          minVal = pathPotential;
          minPos = *topKIter;
        }
	}
	shortest_path=*minPos;
	return 1;
}
void SubstrateGraph::findNb_Node(int nodeID,vector<int> &Nb_VNodes, vector<int> &Nb_Edges){
//获取节点nodeID的邻居节点和邻接链路 Nb_VNodes,Nb_Edges
	int i;
	Nb_VNodes.clear();
	Nb_Edges.clear();
	for(i=0;i<nodes[nodeID].edgeIDs.size();i++){
		Nb_Edges.push_back(nodes[nodeID].edgeIDs[i]);
		if(edges[nodes[nodeID].edgeIDs[i]].from!=nodeID)
			Nb_VNodes.push_back(edges[nodes[nodeID].edgeIDs[i]].from);
		else
			Nb_VNodes.push_back(edges[nodes[nodeID].edgeIDs[i]].to);
	}

}
double SubstrateGraph::get_newRFD_cost(CQYDirectedPath &path){
//计算路径的碎片化代价 （全局碎片度）
	const vector<int> vertexList((&path)->GetVertexList());
	vector<int> Nb_link;
	vector<int> Nb_node;
	int i,j,eID0,eID1,nID0,nID1;
	int len = vertexList.size();
	double cost=0;
	for (i = 0; i < len; i++) {
		findNb_Node(vertexList[i],Nb_node,Nb_link);
		if(i>=1){
			eID0 = edgeMap[make_pair(vertexList[i-1], vertexList[i])];
			nID0=vertexList[i-1];
			cost+=edges[eID0].pagerfd*edges[eID0].rest_bw;
			cost+=nodes[nID0].pagerfd*nodes[nID0].rest_cpu;
		}else
		{
			eID0=-1;
			nID0=-1;
		}
		if(i<len-1) {
			eID1 = edgeMap[make_pair(vertexList[i], vertexList[i+1])];
			nID1= vertexList[i+1];
		}else{
			eID1=-1;
			nID1=-1;
		}
		for(j=0;j<Nb_node.size();j++){
			//if(Nb_node[i]==nID0||Nb_node[i]==nID1) continue;----by lx
			if(Nb_node[j]==nID0||Nb_node[j]==nID1) continue;
			cost+=nodes[Nb_node[j]].pagerfd*nodes[Nb_node[j]].rest_cpu;
		}
		for(j=0;j<Nb_link.size();j++){
			if(Nb_link[j]==eID0||Nb_link[j]==eID1) continue;
			cost+=edges[Nb_link[j]].pagerfd*edges[Nb_link[j]].rest_bw;
		}
				
	}
	return 0.01*cost;
}
double SubstrateGraph::getRFD_cost(CQYDirectedPath &path){
//计算路径的碎片化代价 （局部碎片度）
	const vector<int> vertexList((&path)->GetVertexList());
	vector<int> Nb_link;
	vector<int> Nb_node;
	int i,j,eID0,eID1,nID0,nID1;
	int len = vertexList.size();
	double cost=0;
	for (i = 0; i < len; i++) {
		findNb_Node(vertexList[i],Nb_node,Nb_link);
		if(i>=1){
			eID0 = edgeMap[make_pair(vertexList[i-1], vertexList[i])];
			nID0=vertexList[i-1];
			cost+=Edge_rfd(eID0)*edges[eID0].rest_bw*(1-edges[eID0].ratio_temp);
			cost+=Node_rfd(nID0)*nodes[nID0].rest_cpu*(1-nodes[nID0].ratio_temp);
		}else
		{
			eID0=-1;
			nID0=-1;
		}
		if(i<len-1) {
			eID1 = edgeMap[make_pair(vertexList[i], vertexList[i+1])];
			nID1= vertexList[i+1];
		}else{
			eID1=-1;
			nID1=-1;
		}
		for(j=0;j<Nb_node.size();j++){
			if(Nb_node[j]==nID0||Nb_node[j]==nID1) continue;
			cost+=Node_rfd(Nb_node[j])*nodes[Nb_node[j]].rest_cpu*(1-nodes[Nb_node[j]].ratio_temp);
			//printf("curcost=%lf\n",1-nodes[Nb_node[j]].ratio_temp);
		}
		for(j=0;j<Nb_link.size();j++){
			if(Nb_link[j]==eID0||Nb_link[j]==eID1) continue;
			cost+=Edge_rfd(Nb_link[j])*edges[Nb_link[j]].rest_bw*(1-edges[Nb_link[j]].ratio_temp);
			//printf("curcost=%lf\n",Edge_rfd(Nb_link[j])*edges[Nb_link[j]].rest_bw*(1-edges[Nb_link[j]].ratio_temp));
		}
				
	}
	return cost;
}
int SubstrateGraph::calucate_dis(int tempnode,vector<int> Snodeset){
//计算Snodeset中  与tempnode距离最近的节点
	int i,min_dis=0;
	for(i=1;i<Snodeset.size();i++){
		if(hop[make_pair(tempnode,Snodeset[i])]<hop[make_pair(tempnode,Snodeset[min_dis])])
			min_dis=i;
	}
	return Snodeset[min_dis];
}
int SubstrateGraph::calucate_dis_Weighted(VNRequest &aRequest,int tempSnode[],int len,vector<int> Snodeset,vector<int> Nb_edges){
//计算Snodeset中   与tempSnode[]的加权和最小的节点  权重为虚拟链路带宽bw
	int i,j;
	double dis=0;
	double mindis=1000000;
	int mindisID;
	for(i=0;i<Snodeset.size();i++){
		for(j=0;j<len;j++) dis+=hop[make_pair(Snodeset[i],tempSnode[j])]*aRequest.edges[Nb_edges[j]].bw;
		if(dis<mindis){
			mindis=dis;
			mindisID=i;
		}
	}
	return Snodeset[mindisID];
}
int SubstrateGraph::firstnodemap(VNRequest &aRequest,int nodeID){
//首节点映射 迭代求解
	vector<int> *Snodeset = new vector<int>[nodeNum];
	int i,SNodeCount,tempnodeID;
	vector<int> Nb_VNodes, Nb_Edges;
	Nb_VNodes.clear();
	Nb_Edges.clear();
	aRequest.findNb_Node(nodeID,Nb_VNodes,Nb_Edges);
	size_t len=Nb_VNodes.size();
	int *tempSnode=new int[len];
	memset(tempSnode,-1,sizeof(tempSnode));  //初始化-1
	for(i=0;i<=len;i++){
		if(i==len) tempnodeID=nodeID;
		else
			tempnodeID=Nb_VNodes[i];
		SNodeCount = findNodesWithinConstraints(aRequest.nodes[tempnodeID],  //备选x,y,cpu物理节点集合
        aRequest.reqID, aRequest.maxD, Snodeset[i]);
		if (SNodeCount == 0){
			cerr << "findSNodesWithinConstraints() returned 0" << endl;
			return -1;
		}
	}
	int Centernode=Snodeset[len][0];//中心点初始化
	int iteratortime=0;
	while(iteratortime<5*len){
		iteratortime++;
		for(i=0;i<len;i++){
			if(i==nodeID) continue;
			tempSnode[i]=calucate_dis(Centernode,Snodeset[i]); //确定邻居最近点
		}
		Centernode=calucate_dis_Weighted(aRequest,tempSnode,len,Snodeset[len],Nb_Edges);//利用邻居最近点  计算新中心点
	}
	return Centernode;
}
double SubstrateGraph::init_node_con(int nodeID){
	int i;
	double edge_ratio;//链路剩余资源比例
	double node_ratio;//节点剩余资源比例
	double con=0; //初始连通度
	size_t nb_size=nodes[nodeID].edgeIDs.size();//节点的度(邻居节点数目)
	for(i=0;i<nb_size;i++){
		int cur_edgeID=nodes[nodeID].edgeIDs[i];
		edge_ratio=1-edges[cur_edgeID].ratio;
		if(edges[cur_edgeID].from!=nodeID){
			node_ratio=1-nodes[edges[cur_edgeID].from].ratio;
			con+=node_ratio*edge_ratio/nb_size;
		}
		else{
			node_ratio=1-nodes[edges[cur_edgeID].to].ratio;
			con+=node_ratio*edge_ratio/nb_size;
		}
	}
	//printf("con=%lf\n",con);
	return con;
}
double SubstrateGraph::init_edge_con(int edgeID){
	int i;
	int nb_from=edges[edgeID].from,nb_to=edges[edgeID].to;
	double node_ratio,con=0;
	node_ratio=1-nodes[nb_from].ratio;
	for(i=0;i<nodes[nb_from].edgeIDs.size();i++){
		con+=node_ratio*(1-edges[nodes[nb_from].edgeIDs[i]].ratio);
	}
	con-=node_ratio*(1-edges[edgeID].ratio);
	node_ratio=1-nodes[nb_to].ratio;
	for(i=0;i<nodes[nb_to].edgeIDs.size();i++){
		con+=node_ratio*(1-edges[nodes[nb_to].edgeIDs[i]].ratio);
	}
	con-=node_ratio*(1-edges[edgeID].ratio);
	con/=nodes[nb_from].edgeIDs.size()+nodes[nb_to].edgeIDs.size()-2;
	//printf("con=%lf\n",con);
	return con;
}
void SubstrateGraph::clean(VNRequest &aRequest,bool tag){
	int edgeID,j;
	vector<int>::iterator iter4, iter5;
	vector<double>::iterator iter6;
	for(edgeID=0;edgeID<aRequest.edgeNum;edgeID++)
	{
		if(aRequest.edges[edgeID].touched)
		{
			for (j = 0; j < aRequest.edges[edgeID].pathLen; j++) {
					if (aRequest.edges[edgeID].substrateID == substrateID) {
					edges[aRequest.edges[edgeID].subPath[j]].rest_bw += aRequest.edges[edgeID].subBW[j];
					assert(edges[aRequest.edges[edgeID].subPath[j]].rest_bw <= edges[aRequest.edges[edgeID].subPath[j]].bw + EPSILON);
					edges[aRequest.edges[edgeID].subPath[j]].count--;

					//remove request information
					iter4 = edges[aRequest.edges[edgeID].subPath[j]].req_ids.begin();
					iter5 = edges[aRequest.edges[edgeID].subPath[j]].edge_ids.begin();
					iter6 = edges[aRequest.edges[edgeID].subPath[j]].used_bw.begin();
					while (iter4 != edges[aRequest.edges[edgeID].subPath[j]].req_ids.end()) {
						if (*iter4 == aRequest.reqID && *iter5 == edgeID) {
						edges[aRequest.edges[edgeID].subPath[j]].req_ids.erase(iter4);
						edges[aRequest.edges[edgeID].subPath[j]].edge_ids.erase(iter5);
						edges[aRequest.edges[edgeID].subPath[j]].used_bw.erase(iter6);

						break;
						}
						iter4++;
						iter5++;
						iter6++;
					}
					edges[aRequest.edges[edgeID].subPath[j]].ratio=1-edges[aRequest.edges[edgeID].subPath[j]].rest_bw/edges[aRequest.edges[edgeID].subPath[j]].bw;
					}
					
				}
			if(tag){
				aRequest.edges[edgeID].subPath.clear();
				aRequest.edges[edgeID].subBW.clear();
				aRequest.edges[edgeID].pathLen=0;
			}

		}
	}
}
void SubstrateGraph::initBias(){
	int i,j,k;
	double bw=0.1;
	if(fileinitBias())
		return;
	//initialize
	for(i=0;i<nodeNum;i++)
			nodes[i].xJ=0;
	for(j=0;j<edgeNum;j++)
		edges[j].xJ=0;
	CQYDirectedPath shortest_path[20];
	int len,n_from,n_to;
	createKSPInputFile(bw);
	CQYDirectedGraph dg(KSP_INPUT_FILENAME);
	for(n_from=0;n_from<nodeNum;n_from++)
		for(n_to=n_from+1;n_to<nodeNum;n_to++)
		{
			CQYKShortestPaths ksp(dg, n_from, n_to, MAX_PATHS_TO_CONSIDER);
			vector<CQYDirectedPath*> topK_shortest_paths = ksp.GetTopKShortestPaths();
			if (topK_shortest_paths.empty()) {
				cerr << "topK_shortest_paths is empty" << endl;
				continue;
			}
			int tags=0;
			CQYDirectedPath* minPos = NULL;
			double minVal = std::numeric_limits<double>::max();
			len=(*topK_shortest_paths.begin())->GetLength();
			for (vector<CQYDirectedPath*>::iterator topKIter = topK_shortest_paths.begin();
				topKIter != topK_shortest_paths.end();  ++topKIter) 
			{
				if((*topKIter)->GetLength()<=len)
				{
					if((*topKIter)->GetLength()<len)
					{
						len=(*topKIter)->GetLength();
						tags=0;
					}
					shortest_path[tags] = **topKIter;
					tags++;
				}
			}
			for(i=0;i<tags;i++)
			{
				vector<int> vertexList(shortest_path[i].GetVertexList());
				size_t len = vertexList.size();
				for (k = 1; k < len; k++) 
				{
				
					int eID = edgeMap[make_pair(vertexList[k-1], vertexList[k])];
					edges[eID].xJ+=1/tags;
					int nID = vertexList[k-1];
					if(nID!=n_from&&nID!=n_to)
						nodes[nID].xJ+=1/tags;
				}
			}
		}
		for(i=0;i<nodeNum;i++)
			nodes[i].xJ=nodes[i].xJ/((nodeNum-1)*(nodeNum-2));
		for(j=0;j<edgeNum;j++)
			edges[j].xJ=edges[j].xJ/(nodeNum*(nodeNum-1));
		create_xJ_StaticFile();

}
bool SubstrateGraph::fileinitBias(){
	int nNUM,eNUM,i=0,j=0;
	double temp;
	const char* file_name = xJ_STATICS_FILENAME;
	std::ifstream ifs(file_name);
	if (!ifs)
	{
		std::cout << "The xJ file " << file_name << " is not exist!" << std::endl;
		return false;
	}
	ifs >> nNUM;
	ifs >> eNUM;
	for(i=0;i<nNUM;i++){
		ifs >> temp;
		if (temp==-1)
		{
			return false;
		}
		nodes[i].xJ=temp;
	}
	for(i=0;i<eNUM;i++){
		ifs >> temp;
		if (temp==-1)
		{
			return false;
		}
		edges[i].xJ=temp;
	}
	ifs.close();
	return true;
}
void SubstrateGraph::create_xJ_StaticFile() {
  int i;
  ofstream ofs;
  ofs.open(xJ_STATICS_FILENAME, ios::out);

  if (!ofs) {
    cout << "failed to open file: " << KSP_INPUT_FILENAME << endl;
    exit(COULD_NOT_OPEN_FILE);
  }

  ofs << nodeNum << endl ;
  ofs << edgeNum << endl;
  for (i = 0; i < nodeNum; i++) {
	  ofs << nodes[i].xJ << endl;
  }
  for (i = 0; i < edgeNum; i++) {
	  ofs << edges[i].xJ << endl;
  }
  ofs.close();
}
void SubstrateGraph::rank()
{

    int i,k;
    double sum_alln_con0 = 0;
	double sum_alll_con0 = 0;
	//struct timeval tpstart,tpend;
	//double timeuse;
	//gettimeofday(&tpstart,null);
	//time_t now;
	//time(& now);    /* get time */
	//printf("before pagerank time is:     %s\n",  ctime(&now));   /* ctime() return char string */
        for(i = 0; i < nodeNum; i ++) //初始化节点连通度，计算节点连通度总和
        {
            
            nodes[i].con0 = init_node_con(i);
			nodes[i].connectivity = nodes[i].con0;
            sum_alln_con0 += nodes[i].con0;
            if(sum_alln_con0 < 0 ||nodes[i].con0 < 0)
            {
                printf("error! nodes[i].con0 is non negative\n");
            }

        }
        for(i = 0;i < nodeNum;i++) //计算 邻居节点连通度 总和
        {
            nodes[i].sum_adj_con0  = 0;
			vector<int> temp_Nb_nodes,temp_Nb_edges;
			findNb_Node(i,temp_Nb_nodes, temp_Nb_edges);
            for(k = 0;k<temp_Nb_nodes.size(); k++)
            {
				nodes[i].sum_adj_con0 += nodes[temp_Nb_nodes[k]].con0;
            }
        }

		for(i = 0; i < edgeNum; i ++)   //初始化链路连通度，计算链路连通度总和
        {
            
            edges[i].con0 = init_edge_con(i);
			edges[i].connectivity = edges[i].con0;
            sum_alll_con0 += edges[i].con0;
            if(sum_alll_con0 < 0 ||edges[i].con0 < 0)
            {
                printf("error! nodes[i].con0 is non negative\n");
            }

        }
        for(i = 0;i < edgeNum;i++)   //计算 邻居链路连通度 总和
        {
            edges[i].sum_adj_con0  = 0;
			vector<int> Nb_nodes,temp_Nb_edges0,temp_Nb_edges1;
			findNb_Node(edges[i].from, Nb_nodes,temp_Nb_edges0);
			Nb_nodes.clear();
			findNb_Node(edges[i].to, Nb_nodes,temp_Nb_edges1);
            for(k = 0;k<temp_Nb_edges0.size(); k++)
            {
				if(temp_Nb_edges0[k]!=i)
				edges[i].sum_adj_con0 += edges[temp_Nb_edges0[k]].con0;
            }
			for(k = 0;k<temp_Nb_edges1.size(); k++)
            {
				if(temp_Nb_edges1[k]!=i)
				edges[i].sum_adj_con0 += edges[temp_Nb_edges1[k]].con0;
            }
        }

		pagerank(sum_alln_con0,sum_alll_con0);
        //time(& now);    /* get time */
        //printf("after pagerank time is:     %s\n",  ctime(&now));   /* ctime() return char string */
        //gettimeofday(&tpend,NULL);
        //timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec)+tpend.tv_usec-tpstart.tv_usec;
        //timeuse/=1000000;
        //printf("Used Time:%f\n",timeuse);
		//printf("node pagerfd\n");
		for(i=0;i<nodeNum;i++){
			nodes[i].pagerfd=1-nodes[i].connectivity;
			//if(!i%10) printf("\n");
			//printf("%0.2f ",nodes[i].pagerfd);
			
		}
		//printf("\nedge pagerfd\n");
		for(i=0;i<edgeNum;i++){
			edges[i].pagerfd=1-edges[i].connectivity;
			//if(!i%10) printf("\n");
			//printf("%0.2f ",edges[i].pagerfd);
			
		}
		
}
//Dijkstra shortest path algorithm-----by lzx
void SubstrateGraph::get_hops_n(int* dist,int s_node){
	int i,j,k;
	int INF=1000000;
	int** Edge;
	int *S;
	Edge=new int*[nodeNum];
	for(i=0;i<nodeNum;i++)
	{
		Edge[i]=new int[nodeNum];
		memset(Edge[i],INF,sizeof(int)*nodeNum);
	}
	S=new int[nodeNum];
	memset(S,0,sizeof(int)*nodeNum);
	for(i=0;i<edgeNum;i++)
	{
		if(edges[i].rest_bw<=0)
		{
			Edge[edges[i].from][edges[i].to]=INF;
			Edge[edges[i].to][edges[i].from]=INF;
		}
		else
		{
			Edge[edges[i].from][edges[i].to]=1;
			Edge[edges[i].to][edges[i].from]=1;
		}
	
	}
	for(i=0;i<nodeNum;i++)
		dist[i]=Edge[s_node][i];
	S[s_node]=1;dist[s_node]=0;
	for(i=0;i<nodeNum-1;i++)
	{
		int min=INF,u=s_node;
		for(j=0;j<nodeNum;j++)
		{
			if(!S[j] && (dist[j]<min))
			{
				u=j;
				min=dist[j];
			}
		}
		S[u]=1;
		for(k=0;k<nodeNum;k++)
		{
			if(!S[k]&&(Edge[u][k]<INF)&&(dist[u]+Edge[u][k]<dist[k]))
				dist[k]=dist[u]+Edge[u][k];
		}
	}
	for(i=0;i<nodeNum;i++)
		delete[] Edge[i];
	delete[] Edge;
	delete[] S;
}
/*pagerank algorithm */
void SubstrateGraph::pagerank(double sum_alln_con0,double sum_alll_con0)
{
	//double xJ=0.5,xF=0.5;
    int i, j, k;
    double *backup;
    double ** pJ;
	/*  //lzx
	int **hops_n;   
	hops_n=new int*[nodeNum];
	for(i=0;i<nodeNum;i++)
		hops_n[i]=new int[nodeNum];

	//get_hops
	for(i=0;i<nodeNum;i++)
	    get_hops_n(hops_n[i],i);

	double new_DAMPING;
	*/
	vector<int> Nb_nodes, Nb_edges;
    pJ = (double**) malloc (nodeNum * sizeof (double*));
    for (i=0; i<nodeNum; i++)
        pJ[i] = (double*) malloc (nodeNum * sizeof (double)); 
	
    for (i=0; i<nodeNum; i++)                                                
    {
		
        for (j=0; j<nodeNum; j++)
        {
			 pJ[i][j] = nodes[i].xJ*(double)nodes[j].con0/(sum_alln_con0+EPSILON);
			//new_DAMPING=DAMPING*exp(-double(hops_n[i][j]));
            //pJ[i][j] = new_DAMPING*(double)nodes[j].con0/(sum_alln_con0+EPSILON);//modify pJ[i][j]------by lzx
			//printf("%f  %f\n",nodes[j].con0,pJ[i][j]);
	
				
        }
		findNb_Node(i,Nb_nodes,Nb_edges);
		for(j=0;j<Nb_nodes.size();j++)
		{
			pJ[i][Nb_nodes[j]]+=(1-nodes[i].xJ)*nodes[Nb_nodes[j]].con0/(nodes[i].sum_adj_con0+EPSILON);
			//pJ[i][Nb_nodes[j]]+=(1-DAMPING)*nodes[Nb_nodes[j]].con0/(nodes[i].sum_adj_con0+EPSILON);
			//printf("%f  %f %f\n",nodes[Nb_nodes[j]].con0,nodes[i].sum_adj_con0,pJ[i][Nb_nodes[j]]);
		}
    }
	/*for(i=0;i<nodeNum;i++)
	{
		double sum=0;
		for(j=0;j<nodeNum;j++){
			sum+=pJ[i][j];
			printf("%0.5f   ",pJ[i][j]);
			if(!j%10) printf("\n");
		}
		printf("sum=%lf\n",sum);
	}*/
    backup = (double*) malloc (nodeNum * sizeof (double));
    for (i=0; i<nodeNum; i++)
        backup[i] = 0;
    int iteration_time = 0;
    while (judge (backup,1))
    {
        iteration_time++;
        for (i=0; i<nodeNum; i++)
        {
			backup[i] =nodes[i].connectivity;
			nodes[i].connectivity = 0;
        }

        for (i=0; i<nodeNum; i++)

        {
			double sum=0;
            for (j=0; j<nodeNum; j++){
				//printf("%f %f  \n",backup[j],nodes[i].connectivity);
                nodes[i].connectivity += backup[j] * pJ [j][i]; 
				//sum+=pJ[i][j];
			
			}
			//printf("sum=%f\n",sum);
			//printf("%f \n",nodes[i].connectivity);
        }
    }
	double max_connectivity=0;
	for(i=0;i<nodeNum;i++)
	{
		if(nodes[i].connectivity>max_connectivity)
			max_connectivity=nodes[i].connectivity;
		/*if(!i%10) printf("\n");
		printf("%0.5f   ",nodes[i].connectivity);*/
		
	}
	for(i=0;i<nodeNum;i++)
	{
		nodes[i].connectivity=nodes[i].connectivity/(max_connectivity+EPSILON);
		/*if(!i%10) printf("\n");
		printf("%0.5f   ",nodes[i].connectivity);*/
		
	}
	free (backup);
	for(i = 0;i<nodeNum;i++)
        free(pJ[i]);
    free(pJ);



	//edge connectivity
	pJ = (double**) malloc (edgeNum * sizeof (double*));
    for (i=0; i<edgeNum; i++)
        pJ[i] = (double*) malloc (edgeNum * sizeof (double)); 
	for (i=0; i<edgeNum; i++)                                                
    {
        for (j=0; j<edgeNum; j++)
        {
			//new_DAMPING=DAMPING*exp(-double(hops_n[edges[i].from][edges[j].from]));
            //pJ[i][j] = new_DAMPING*(double)edges[j].con0/sum_alll_con0 ;//modify pJ[i][j]---------by lzx
			pJ[i][j] = edges[i].xJ*(double)edges[j].con0/sum_alll_con0;
        }
		findNb_Node(edges[i].from, Nb_nodes,Nb_edges);
		for(k = 0;k<Nb_edges.size(); k++)
        {
			if(Nb_edges[k]!=i)
			//pJ[i][Nb_edges[k]]+=(1-DAMPING)*edges[Nb_edges[k]].con0/(edges[i].sum_adj_con0+EPSILON);
			pJ[i][Nb_edges[k]]+=(1-edges[i].xJ)*edges[Nb_edges[k]].con0/(edges[i].sum_adj_con0+EPSILON);
        }
		findNb_Node(edges[i].to, Nb_nodes,Nb_edges);
		for(k = 0;k<Nb_edges.size(); k++)
        {
			if(Nb_edges[k]!=i)
			//pJ[i][Nb_edges[k]]+=(1-DAMPING)*edges[Nb_edges[k]].con0/(edges[i].sum_adj_con0+EPSILON);
			pJ[i][Nb_edges[k]]+=(1-edges[i].xJ)*edges[Nb_edges[k]].con0/(edges[i].sum_adj_con0+EPSILON);
        }
    }

    backup = (double*) malloc (edgeNum * sizeof (double));
    for (i=0; i<edgeNum; i++)
        backup[i] = 0;
    iteration_time = 0;
    while (judge(backup,0))
    {
        iteration_time++;
        for (i=0; i<edgeNum; i++)
        {
			backup[i] =edges[i].connectivity;
			edges[i].connectivity = 0;
        }

        for (i=0; i<edgeNum; i++)

        {
            for (j=0; j<edgeNum; j++)
				edges[i].connectivity = edges[i].connectivity + backup[j] * pJ[j][i]; 
        }
    }
	max_connectivity=0;
	for(i=0;i<edgeNum;i++)
	{
		if(edges[i].connectivity>max_connectivity)
			max_connectivity=edges[i].connectivity;
		/*printf("%0.5f   ",edges[i].connectivity);
		if(!i%10) printf("\n");*/
	}
	for(i=0;i<edgeNum;i++)
	{
		edges[i].connectivity=edges[i].connectivity/(max_connectivity+EPSILON);
		/*printf("%0.5f   ",nodes[i].connectivity);
		if(!i%10) printf("\n");*/
	}
    free (backup);
    for(i = 0;i<edgeNum;i++)
        free(pJ[i]);
    free(pJ);
/*
	for(i=0;i<nodeNum;i++)
		delete[] hops_n[i];
	delete[] hops_n;
*/
}

//decide whether continue iteration
int SubstrateGraph::judge (double* backup,int is_node)
{
    int i;
	if(is_node)
		for (i=0; i<nodeNum; i++)
		{
			if (fabs (nodes[i].connectivity-backup[i]) >= EPSILON)
			{
				//double f = fabs (result[i] - backup[i]) ;
				return 1; 
			}
		}
	else
	{
		for (i=0; i<edgeNum; i++)
		{
			if (fabs (edges[i].connectivity-backup[i]) >= EPSILON)
			{
				//double f = fabs (result[i] - backup[i]) ;
				return 1; 
			}
		}
	}
    return 0;    
}

int SubstrateGraph::new_node_and_path(VNRequest &aRequest,int nodeID,vector<int> &SNodes, int &vnodeID){
	int i,j,k,n_to,n_from,min_id,tag,n;
	vector<double> Cost;
	Cost.clear();
	//路径存储
		int ***pathedgeID=new int**[nodeNum];//三维数组b[m][n][t]
		for(i=0;i<nodeNum;i++){
			*(pathedgeID+i)=new int*[aRequest.nodeNum];
		}
		for(i=0;i<nodeNum;i++){
			for(int j=0;j<aRequest.nodeNum;j++)
			{
			*(*(pathedgeID+i)+j)=new int[edgeNum];
			}
		}
		for(i=0;i<nodeNum;i++)
			for(j=0;j<aRequest.nodeNum;j++)
				for(k=0;k<edgeNum;k++)
					pathedgeID[i][j][k]=-1;

   //.....................
	vector<int> Nb_nodes,Nb_edges,preSnodes; //虚拟节点邻接点
	Nb_nodes.clear();
	Nb_edges.clear();
	aRequest.findNb_Node(nodeID,Nb_nodes,Nb_edges);
	int a[50];
	for(i=0;i<50;i++)
		a[i]=0;
	//int *a=new int[Nb_nodes.size()];
	//memset(a,0,sizeof(a));
	double total_cost;
	printf("Cost for virtual node %d:\n",nodeID);
	for(i=0;i<SNodes.size();i++){
		n_from=SNodes[i];  //底层节点标号
		if(nodes[n_from].touched==true) continue;
		if(aRequest.calculate_nbandwidth(nodeID)>calculate_nbandwidth(n_from)) continue;
		//nodes[n_from].ratio_temp=nodes[n_from].ratio_temp+aRequest.nodes[nodeID].cpu/nodes[n_from].rest_cpu;
		preSnodes.push_back(n_from);
		total_cost=0;
		//prea_ction
		tag=0;
		for(j=0;j<Nb_nodes.size();j++){
			//cout<<"Nb_nodes.size():"<<Nb_nodes.size()<<endl;
			if(aRequest.nodes[Nb_nodes[j]].touched==false) continue;
			tag=1;
			CQYDirectedPath shortest_path;
			n_to=aRequest.nodes[Nb_nodes[j]].subNodeID; //底层节点标号
			int temp=getshortestpath(n_from,n_to,aRequest.edges[Nb_edges[j]].bw,shortest_path);
			if (temp==-1) {
				cerr << "shortest_path is empty" << endl;
				a[j]++;
				total_cost=MY_INFINITY;
				break;
			}
			//path pre_processing......................
			vector<int> vertexList(shortest_path.GetVertexList());
			size_t len = vertexList.size();
			for (k = 1; k < len; k++) {
				
			int eID = edgeMap[make_pair(vertexList[k-1], vertexList[k])];
			//edges[eID].ratio_temp=edges[eID].ratio_temp+aRequest.edges[Nb_edges[j]].bw/edges[eID].rest_bw;
			pathedgeID[i][j][k-1]=eID;
			edges[eID].rest_bw-=aRequest.edges[Nb_edges[j]].bw;
			}

			//.........................................
				double bw_cost=(len-1)*aRequest.edges[Nb_edges[j]].bw;
				double rfd_cost=get_newRFD_cost(shortest_path);
				total_cost+=bw_cost+rfd_cost;
				//cout<<total_cost<<endl;
				printf("<Vnode:%d, Snode:%d, Vedge:%d>, bwcost: %lf  rfd_lcost: %lf \n",nodeID,n_from,Nb_edges[j],bw_cost,rfd_cost);
		}
		for(j=0;j<Nb_nodes.size();j++)
			for(k=1;k<50;k++)
			{ 
				if(pathedgeID[i][j][k-1]!=-1)
					edges[pathedgeID[i][j][k-1]].rest_bw+=aRequest.edges[Nb_edges[j]].bw;
			}
		//nodes[n_from].ratio_temp=nodes[n_from].ratio_temp-aRequest.nodes[nodeID].cpu/nodes[n_from].rest_cpu;
		Cost.push_back(total_cost);
		//cout<<"total_cost:"<<total_cost<<endl;
			
	}
	 if(preSnodes.empty())  return -1;
	/*
	 cout<<"Cost.........."<<endl;
	for(i=0;i<Cost.size();i++)
		cout<<Cost[i]<<endl;
		*/
	int temp=0;
	for(i=1;i<Nb_nodes.size();i++){
		if(a[i]>a[temp]) temp=i;
	}
	vnodeID=Nb_nodes[temp];
	assert(Cost.size()==preSnodes.size());
	assert(vnodeID<aRequest.nodeNum);
	min_id=0;
	for (i = 1; i < preSnodes.size(); i++) {
		if (Cost[i] < Cost[min_id])
			min_id=i;
	}
	double tempcost;
	tempcost=Cost[min_id];
	for (i = 0; i < preSnodes.size(); i++) {
		if ((fabs(Cost[i]-tempcost)<1e-1)&&(nodes[preSnodes[i]].rest_cpu>nodes[preSnodes[min_id]].rest_cpu))
			min_id=i;
	}
	
	//delete []a;
	if(preSnodes.empty()||Cost[min_id]>100000){
		 //三维数组内存释放
		 for(i=0;i<nodeNum;i++)
		 for(int j=0;j<aRequest.nodeNum;j++) 
			 delete [](*(*(pathedgeID+i)+j));
		 for(i=0;i<aRequest.nodeNum;i++)	
			 delete [](*(pathedgeID+i));
		 delete []pathedgeID;
			 return -2;
	}
	for(j=0;j<Nb_nodes.size();j++){
		if(aRequest.nodes[Nb_nodes[j]].touched==false) continue;
		int eIDs = aRequest.edgeMap[make_pair(Nb_nodes[j], nodeID)];
		for(k=0;k<edgeNum;k++){
			if(pathedgeID[min_id][j][k]==-1) break;
			aRequest.edges[eIDs].substrateID = substrateID;
			aRequest.edges[eIDs].pathLen = k+1;
			aRequest.edges[eIDs].subPath.push_back(pathedgeID[min_id][j][k]);
			aRequest.edges[eIDs].subBW.push_back(aRequest.edges[eIDs].bw);			
		}
		// add to the substrate network links
		aRequest.edges[eIDs].touched=true;
		for (n = 0; n < aRequest.edges[eIDs].pathLen; n++) {
			if (aRequest.edges[eIDs].substrateID == substrateID) {
				edges[aRequest.edges[eIDs].subPath[n]].rest_bw -= aRequest.edges[eIDs].subBW[n];
				assert(edges[aRequest.edges[eIDs].subPath[n]].rest_bw + EPSILON >0);
				edges[aRequest.edges[eIDs].subPath[n]].count++;
				// save request information
				edges[aRequest.edges[eIDs].subPath[n]].req_ids.push_back(aRequest.reqID);
				edges[aRequest.edges[eIDs].subPath[n]].edge_ids.push_back(eIDs);
				edges[aRequest.edges[eIDs].subPath[n]].used_bw.push_back(aRequest.edges[eIDs].subBW[n]);
				edges[aRequest.edges[eIDs].subPath[n]].ratio=1-edges[aRequest.edges[eIDs].subPath[n]].rest_bw/edges[aRequest.edges[eIDs].subPath[n]].bw;
			}
		}
	}
	//三维数组内存释放
	for(i=0;i<nodeNum;i++)
	for(int j=0;j<aRequest.nodeNum;j++) 
		delete [](*(*(pathedgeID+i)+j));
	for(i=0;i<aRequest.nodeNum;i++)	
		delete [](*(pathedgeID+i));
	delete []pathedgeID;
	
	return preSnodes[min_id];
}
int SubstrateGraph::node_and_path(VNRequest &aRequest,int nodeID,vector<int> &SNodes, int &vnodeID){
	int i,j,k,n,n_to,n_from,min_id,tag;
	vector<double> Cost;
	       
	//delete []a; 
	//路径存储
		int ***pathedgeID=new int**[nodeNum];//三维数组b[m][n][t]
		for(i=0;i<nodeNum;i++){
			*(pathedgeID+i)=new int*[aRequest.nodeNum];
		}
		for(i=0;i<nodeNum;i++){
			for(int j=0;j<aRequest.nodeNum;j++)
			{
			*(*(pathedgeID+i)+j)=new int[edgeNum];
			}
		}
		for(i=0;i<nodeNum;i++)
			for(j=0;j<aRequest.nodeNum;j++)
				for(k=0;k<edgeNum;k++)
					pathedgeID[i][j][k]=-1;

   //.....................
	vector<int> Nb_nodes,Nb_edges,preSnodes; //虚拟节点邻接点
	Nb_nodes.clear();
	Nb_edges.clear();
	aRequest.findNb_Node(nodeID,Nb_nodes,Nb_edges);
	int *a=new int[Nb_nodes.size()];
	memset(a,0,sizeof(a));
	double total_cost;
	printf("Cost for virtual node %d:\n",nodeID);
	for(i=0;i<SNodes.size();i++){
		n_from=SNodes[i];  //底层节点标号
		if(nodes[n_from].touched==true) continue;
		if(aRequest.calculate_nbandwidth(nodeID)>calculate_nbandwidth(n_from)) continue;
		nodes[n_from].ratio_temp=nodes[n_from].ratio_temp+aRequest.nodes[nodeID].cpu/nodes[n_from].rest_cpu;
		preSnodes.push_back(n_from);
		total_cost=0;
		//prea_ction
		tag=0;
		for(j=0;j<Nb_nodes.size();j++){
			if(aRequest.nodes[Nb_nodes[j]].touched==false) continue;
			tag=1;
			CQYDirectedPath shortest_path;
			n_to=aRequest.nodes[Nb_nodes[j]].subNodeID; //底层节点标号
			int temp=getshortestpath(n_from,n_to,aRequest.edges[Nb_edges[j]].bw,shortest_path);
			if (temp==-1) {
				cerr << "shortest_path is empty" << endl;
				a[j]++;
				total_cost=MY_INFINITY;
				break;
			}
			//path pre_processing......................
			vector<int> vertexList(shortest_path.GetVertexList());
			size_t len = vertexList.size();
			for (k = 1; k < len; k++) {
			int eID = edgeMap[make_pair(vertexList[k-1], vertexList[k])];
			edges[eID].ratio_temp=edges[eID].ratio_temp+aRequest.edges[Nb_edges[j]].bw/edges[eID].rest_bw;
			pathedgeID[i][j][k-1]=eID;
			edges[eID].rest_bw-=aRequest.edges[Nb_edges[j]].bw;
			}

			//.........................................
				double bw_cost=(len-1)*aRequest.edges[Nb_edges[j]].bw;
				double rfd_cost=getRFD_cost(shortest_path);
				total_cost+=bw_cost+rfd_cost;
				printf("<%d, %d>, bwcost: %lf  rfd_lcost: %lf \n",nodeID,n_from,bw_cost,rfd_cost);
				
					//path reset......................
				for (k = 1; k < len; k++) {
				int eID = edgeMap[make_pair(vertexList[k - 1], vertexList[k])];
				edges[eID].ratio_temp=edges[eID].ratio_temp-aRequest.edges[Nb_edges[j]].bw/edges[eID].rest_bw;
				}
		}
		for(j=0;j<Nb_nodes.size();j++)
			for(k=1;k<50;k++)
			{ 
				if(pathedgeID[i][j][k-1]!=-1)
					edges[pathedgeID[i][j][k-1]].rest_bw+=aRequest.edges[Nb_edges[j]].bw;
			}
		nodes[n_from].ratio_temp=nodes[n_from].ratio_temp-aRequest.nodes[nodeID].cpu/nodes[n_from].rest_cpu;
		Cost.push_back(total_cost);
			
	}
	int temp=0;
	for(i=1;i<Nb_nodes.size();i++){
		if(a[i]>a[temp]) temp=i;
	}
	vnodeID=Nb_nodes[temp];
	assert(vnodeID<aRequest.nodeNum);
	min_id=0;
	for (i = 1; i < preSnodes.size(); i++) {
		if (Cost[i] < Cost[min_id])
			min_id=i;
	}
	for (i = 0; i < preSnodes.size(); i++) {
		if (fabs(Cost[i]-Cost[min_id])<1e-1&&nodes[preSnodes[i]].rest_cpu>nodes[preSnodes[min_id]].rest_cpu)
			min_id=i;
	}
	//delete []a;
	if(preSnodes.empty()||Cost[min_id]>100000){
		 //三维数组内存释放
		 for(i=0;i<nodeNum;i++)
		 for(int j=0;j<aRequest.nodeNum;j++) 
			 delete [](*(*(pathedgeID+i)+j));
		 for(i=0;i<aRequest.nodeNum;i++)	
			 delete [](*(pathedgeID+i));
		 delete []pathedgeID;
		 if(preSnodes.empty())
			 return -1;
		 else
			 return -2;
	}
	//mapping		
	for(j=0;j<Nb_nodes.size();j++){
		if(aRequest.nodes[Nb_nodes[j]].touched==false) continue;
		int eIDs = aRequest.edgeMap[make_pair(Nb_nodes[j], nodeID)];
		for(k=0;k<edgeNum;k++){
			if(pathedgeID[min_id][j][k]==-1) break;
			aRequest.edges[eIDs].substrateID = substrateID;
			aRequest.edges[eIDs].pathLen = k+1;
			aRequest.edges[eIDs].subPath.push_back(pathedgeID[min_id][j][k]);
			aRequest.edges[eIDs].subBW.push_back(aRequest.edges[eIDs].bw);
			edges[pathedgeID[min_id][j][k]].ratio_temp=edges[pathedgeID[min_id][j][k]].ratio_temp+aRequest.edges[Nb_edges[j]].bw/edges[pathedgeID[min_id][j][k]].rest_bw;
			
		}
		// add to the substrate network links
		aRequest.edges[eIDs].touched=true;
		for (n = 0; n < aRequest.edges[eIDs].pathLen; n++) {
			if (aRequest.edges[eIDs].substrateID == substrateID) {
				edges[aRequest.edges[eIDs].subPath[n]].rest_bw -= aRequest.edges[eIDs].subBW[n];
				assert(edges[aRequest.edges[eIDs].subPath[n]].rest_bw + EPSILON >0);
				edges[aRequest.edges[eIDs].subPath[n]].count++;
				// save request information
				edges[aRequest.edges[eIDs].subPath[n]].req_ids.push_back(aRequest.reqID);
				edges[aRequest.edges[eIDs].subPath[n]].edge_ids.push_back(eIDs);
				edges[aRequest.edges[eIDs].subPath[n]].used_bw.push_back(aRequest.edges[eIDs].subBW[n]);
				edges[aRequest.edges[eIDs].subPath[n]].ratio=1-edges[aRequest.edges[eIDs].subPath[n]].rest_bw/edges[aRequest.edges[eIDs].subPath[n]].bw;
			}
		}
	}
	//三维数组内存释放
		for(i=0;i<nodeNum;i++)
		for(int j=0;j<aRequest.nodeNum;j++) 
			delete [](*(*(pathedgeID+i)+j));
		for(i=0;i<aRequest.nodeNum;i++)	
			delete [](*(pathedgeID+i));
		delete []pathedgeID;
	return preSnodes[min_id];
}
int SubstrateGraph::Costlowest(VNRequest &aRequest,int nodeID,vector<int> &SNodes){
	int i,j,n_to,n_from,min_id;
	vector<double> Cost; 
	vector<int> Nb_nodes,Nb_edges,preSnodes; //虚拟节点邻接点
	double total_cost;
	printf("Cost for virtual node %d:\n",nodeID);
	for(i=0;i<SNodes.size();i++){
		n_from=SNodes[i];  //底层节点标号
		if(nodes[n_from].touched==true) continue;
		if(aRequest.calculate_nbandwidth(nodeID)>calculate_nbandwidth(n_from)) continue;
		preSnodes.push_back(n_from);
		Nb_nodes.clear();
		Nb_edges.clear();
		aRequest.findNb_Node(nodeID,Nb_nodes,Nb_edges);
		total_cost=0;
		//prea_ction
		nodes[n_from].rest_cpu-=aRequest.nodes[nodeID].cpu;
		for(j=0;j<Nb_nodes.size();j++){
			if(aRequest.nodes[Nb_nodes[j]].touched==false) continue;
			n_to=aRequest.nodes[Nb_nodes[j]].subNodeID; //底层节点标号
			//calcualte for RFD_cost
			createKSPInputFile(aRequest.edges[Nb_edges[j]].bw);
			CQYDirectedGraph dg(KSP_INPUT_FILENAME);
			CQYKShortestPaths ksp(dg, n_from, n_to, MAX_PATHS_TO_CONSIDER);
			vector<CQYDirectedPath*> topK_shortest_paths = ksp.GetTopKShortestPaths();
			if (topK_shortest_paths.empty()) {
				cerr << "topK_shortest_paths is empty" << endl;
				total_cost=MY_INFINITY;
				break;
			}
			CQYDirectedPath* minPos = NULL;
			double minVal = std::numeric_limits<double>::max();
			for (vector<CQYDirectedPath*>::iterator topKIter = topK_shortest_paths.begin();
				topKIter != topK_shortest_paths.end();  ++topKIter) {
				double bw_cost=((*topKIter)->GetLength()-1)*aRequest.edges[Nb_edges[j]].bw;
				double RFD_lcost=getPathPotentialRfd(*topKIter,aRequest.edges[Nb_edges[j]].bw);
				//double RFD_ncost=nodes[n_from].rest_cpu*Node_rfd(n_from);
				double pathPotential_add_RFD=bw_cost+RFD_lcost;
				printf("<%d, %d, %d>, bcost: %lf  r_lcost: %lf \n",nodeID,n_from,j,bw_cost,RFD_lcost);
				if (pathPotential_add_RFD< minVal) {
					minVal = pathPotential_add_RFD;
					minPos = *topKIter;
				}
			}
			
			assert(minPos != NULL);
			
		    //mapedge 
			total_cost+=minVal;	
		}
		total_cost+=nodes[n_from].rest_cpu*Node_rfd(n_from);
		nodes[n_from].rest_cpu+=aRequest.nodes[nodeID].cpu;
		assert(nodes[n_from].rest_cpu<=nodes[n_from].cpu+EPSILON);
		Cost.push_back(total_cost);
		printf("Map<%d, %d>: %lf\n",nodeID,n_from,total_cost);
		//reset
		
	}
	min_id=0;
	for (i = 1; i < preSnodes.size(); i++) {
		if (Cost[i] < Cost[min_id])
			min_id=i;
	}
	for (i = 0; i < preSnodes.size(); i++) {
		if (fabs(Cost[i]-Cost[min_id])<1e-1&&nodes[preSnodes[i]].rest_cpu>nodes[preSnodes[min_id]].rest_cpu)
			min_id=i;
	}
	if(preSnodes.empty())
		return -1;
	else
		return preSnodes[min_id];
	
}
int SubstrateGraph::mapEdges(VNRequest &aRequest, int VEdgeOrdering,
    int mappingMethod) {
  int i, j;
  int fromSub, toSub, fromSubNode, toSubNode;
  vector<int> edgeProcessOrder;
  vector<double> list_rest_bw(edgeNum);

  for (i = 0; i < aRequest.edgeNum; i++) {
    edgeProcessOrder.push_back(i);
  }

  for (i = 0; i < edgeNum; i++) {
    list_rest_bw[i] = edges[i].rest_bw;
  }

  // set the virtual nodes' processing order
  switch (VEdgeOrdering) {
  case VEDGE_ORDER_ASC:
    aRequest.sortEdgesAscending(edgeProcessOrder);
    break;
  case VEDGE_ORDER_DESC:
    aRequest.sortEdgesDescending(edgeProcessOrder);
    break;
  case VEDGE_ORDER_RAND:
    randomPermutation(edgeProcessOrder);
    break;
  default:
    break;
  }

  // for (i = 0; i < aRequest.edgeNum; i++)
    // cout << edgeProcessOrder[i] << " ";
  // cout << endl;

  switch (mappingMethod) {
  case EM_GREEDY_BEST_FIT:

    for (i = 0; i < aRequest.edgeNum; i++) {
      fromSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].substrateID;
      fromSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].subNodeID;

      toSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].substrateID;
      toSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].subNodeID;

      // if (fromSub != toSub) {
        // multiple InP scenario
        // may handle later
      // }

      createKSPInputFile(aRequest.edges[edgeProcessOrder[i]].bw);

      CQYDirectedGraph dg(KSP_INPUT_FILENAME);
      CQYKShortestPaths ksp(dg, fromSubNode, toSubNode, MAX_PATHS_TO_CONSIDER);
      vector<CQYDirectedPath*> topK_shortest_paths = ksp.GetTopKShortestPaths();

      if (topK_shortest_paths.empty()) {
        cerr << "topK_shortest_paths is empty" << endl;
        break;
      }

      // find the substrate path with the minimum potential

      CQYDirectedPath* minPos = NULL;
      //double minVal = INFINITY;
	  double minVal = std::numeric_limits<double>::max();
      for (vector<CQYDirectedPath*>::iterator topKIter = topK_shortest_paths.begin();
        topKIter != topK_shortest_paths.end();  ++topKIter) {

        double pathPotential = getPathPotential(*topKIter);
        if (pathPotential < minVal) {
          minVal = pathPotential;
          minPos = *topKIter;
        }
      }

      assert(minPos != NULL);

      // TODO: improve substrate info saving mechanism for multiple InP

      aRequest.edges[edgeProcessOrder[i]].substrateID = substrateID;
      aRequest.edges[edgeProcessOrder[i]].pathLen = minPos->GetLength() - 1;
      aRequest.edges[edgeProcessOrder[i]].pathDelay = minPos->GetCost();

      //minPos->PrintOut(cout);

      int minPathLen = minPos->GetLength() - 1;
      vector<int> minPathNodes = minPos->GetVertexList();
      for (int j = 0; j < minPathLen; j++) {
        int eID = edgeMap[make_pair(minPathNodes[j], minPathNodes[j + 1])];
        aRequest.edges[edgeProcessOrder[i]].subPath.push_back(eID);
        aRequest.edges[edgeProcessOrder[i]].subBW.push_back(aRequest.edges[edgeProcessOrder[i]].bw);

        // remove used bandwidth from the substrate edge
        edges[eID].rest_bw -= aRequest.edges[edgeProcessOrder[i]].bw;
        assert(edges[eID].rest_bw + EPSILON >= 0);
      }
    }

    break;

  case EM_GREEDY_WORST_FIT:

    for (i = 0; i < aRequest.edgeNum; i++) {
      fromSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].substrateID;
      fromSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].subNodeID;

      toSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].substrateID;
      toSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].subNodeID;

      // if (fromSub != toSub) {
        // multiple InP scenario
        // may handle later
      // }

      createKSPInputFile(aRequest.edges[edgeProcessOrder[i]].bw);

      CQYDirectedGraph dg(KSP_INPUT_FILENAME);
      CQYKShortestPaths ksp(dg, fromSubNode, toSubNode, MAX_PATHS_TO_CONSIDER);
      vector<CQYDirectedPath*> topK_shortest_paths = ksp.GetTopKShortestPaths();

      if (topK_shortest_paths.empty()) {
        cerr << "topK_shortest_paths is empty" << endl;
        break;
      }

      // find the substrate path with the minimum potential

      CQYDirectedPath* maxPos = NULL;
      double maxVal = 0.;

      for (vector<CQYDirectedPath*>::iterator topKIter = topK_shortest_paths.begin();
        topKIter != topK_shortest_paths.end();  ++topKIter) {

        double pathPotential = getPathPotential(*topKIter);
        if (pathPotential > maxVal) {
          maxVal = pathPotential;
          maxPos = *topKIter;
        }
      }

      assert(maxPos != NULL);

      // TODO: improve substrate info saving mechanism for multiple InP

      aRequest.edges[edgeProcessOrder[i]].substrateID = substrateID;
      aRequest.edges[edgeProcessOrder[i]].pathLen = maxPos->GetLength() - 1;
      aRequest.edges[edgeProcessOrder[i]].pathDelay = maxPos->GetCost();

      int maxPathLen = maxPos->GetLength() - 1;
      vector<int> maxPathNodes = maxPos->GetVertexList();
      for (int j = 0; j < maxPathLen; j++) {
        int eID = edgeMap[make_pair(maxPathNodes[j], maxPathNodes[j + 1])];

        aRequest.edges[edgeProcessOrder[i]].subPath.push_back(eID);
        aRequest.edges[edgeProcessOrder[i]].subBW.push_back(aRequest.edges[edgeProcessOrder[i]].bw);

        // remove used bandwidth from the substrate edge
        edges[eID].rest_bw -= aRequest.edges[edgeProcessOrder[i]].bw;
        assert(edges[eID].rest_bw + EPSILON >= 0);
      }
    }

    break;

  case EM_RANDOM:

    for (i = 0; i < aRequest.edgeNum; i++) {
      fromSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].substrateID;
      fromSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].subNodeID;

      toSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].substrateID;
      toSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].subNodeID;

      // if (fromSub != toSub) {
        // multiple InP scenario
        // may handle later
      // }

      createKSPInputFile(aRequest.edges[edgeProcessOrder[i]].bw);

      CQYDirectedGraph dg(KSP_INPUT_FILENAME);
      CQYKShortestPaths ksp(dg, fromSubNode, toSubNode, MAX_PATHS_TO_CONSIDER);
      vector<CQYDirectedPath*> topK_shortest_paths = ksp.GetTopKShortestPaths();

      if (topK_shortest_paths.empty()) {
        cerr << "topK_shortest_paths is empty" << endl;
        break;
      }

      // find the substrate path with the minimum potential

      int randIndex = rand() % topK_shortest_paths.size();
      CQYDirectedPath* randPos = topK_shortest_paths[randIndex];

      assert(randPos != NULL);

      // TODO: improve substrate info saving mechanism for multiple InP

      aRequest.edges[edgeProcessOrder[i]].substrateID = substrateID;
      aRequest.edges[edgeProcessOrder[i]].pathLen = randPos->GetLength() - 1;
      aRequest.edges[edgeProcessOrder[i]].pathDelay = randPos->GetCost();

      int randPathLen = randPos->GetLength() - 1;
      vector<int> randPathNodes = randPos->GetVertexList();
      for (int j = 0; j < randPathLen; j++) {
        int eID = edgeMap[make_pair(randPathNodes[j], randPathNodes[j + 1])];

        aRequest.edges[edgeProcessOrder[i]].subPath.push_back(eID);
        aRequest.edges[edgeProcessOrder[i]].subBW.push_back(aRequest.edges[edgeProcessOrder[i]].bw);

        // remove used bandwidth from the substrate edge
        edges[eID].rest_bw -= aRequest.edges[edgeProcessOrder[i]].bw;
        assert(edges[eID].rest_bw + EPSILON >= 0);
      }
    }

    break;

  case EM_RANDOM_P2_BEST:

    for (i = 0; i < aRequest.edgeNum; i++) {
      fromSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].substrateID;
      fromSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].subNodeID;

      toSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].substrateID;
      toSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].subNodeID;

      // if (fromSub != toSub) {
        // multiple InP scenario
        // may handle later
      // }

      createKSPInputFile(aRequest.edges[edgeProcessOrder[i]].bw);

      CQYDirectedGraph dg(KSP_INPUT_FILENAME);
      CQYKShortestPaths ksp(dg, fromSubNode, toSubNode, MAX_PATHS_TO_CONSIDER);
      vector<CQYDirectedPath*> topK_shortest_paths = ksp.GetTopKShortestPaths();

      if (topK_shortest_paths.empty()) {
        cerr << "topK_shortest_paths is empty" << endl;
        break;
      }

      // find the substrate path with the minimum potential

      int randIndex1 = rand() % topK_shortest_paths.size();
      int randIndex2 = rand() % topK_shortest_paths.size();

      CQYDirectedPath* randPos1 = topK_shortest_paths[randIndex1];
      CQYDirectedPath* randPos2 = topK_shortest_paths[randIndex2];

      double randVal1 = getPathPotential(randPos1);
      double randVal2 = getPathPotential(randPos2);

      CQYDirectedPath* maxPos = (randVal1 > randVal2) ? randPos1 : randPos2;

      assert(maxPos != NULL);

      // TODO: improve substrate info saving mechanism for multiple InP

      aRequest.edges[edgeProcessOrder[i]].substrateID = substrateID;
      aRequest.edges[edgeProcessOrder[i]].pathLen = maxPos->GetLength() - 1;
      aRequest.edges[edgeProcessOrder[i]].pathDelay = maxPos->GetCost();

      int maxPathLen = maxPos->GetLength() - 1;
      vector<int> maxPathNodes = maxPos->GetVertexList();
      for (int j = 0; j < maxPathLen; j++) {
        int eID = edgeMap[make_pair(maxPathNodes[j], maxPathNodes[j + 1])];

        aRequest.edges[edgeProcessOrder[i]].subPath.push_back(eID);
        aRequest.edges[edgeProcessOrder[i]].subBW.push_back(aRequest.edges[edgeProcessOrder[i]].bw);

        // remove used bandwidth from the substrate edge
        edges[eID].rest_bw -= aRequest.edges[edgeProcessOrder[i]].bw;
        assert(edges[eID].rest_bw + EPSILON >= 0);
      }
    }

    break;

  case EM_RANDOM_P2_WORST:

    for (i = 0; i < aRequest.edgeNum; i++) {
      fromSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].substrateID;
      fromSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].from].subNodeID;

      toSub = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].substrateID;
      toSubNode = aRequest.nodes[aRequest.edges[edgeProcessOrder[i]].to].subNodeID;

      // if (fromSub != toSub) {
        // multiple InP scenario
        // may handle later
      // }

      createKSPInputFile(aRequest.edges[edgeProcessOrder[i]].bw);

      CQYDirectedGraph dg(KSP_INPUT_FILENAME);
      CQYKShortestPaths ksp(dg, fromSubNode, toSubNode, MAX_PATHS_TO_CONSIDER);
      vector<CQYDirectedPath*> topK_shortest_paths = ksp.GetTopKShortestPaths();

      if (topK_shortest_paths.empty()) {
        cerr << "topK_shortest_paths is empty" << endl;
        break;
      }

      // find the substrate path with the minimum potential

      int randIndex1 = rand() % topK_shortest_paths.size();
      int randIndex2 = rand() % topK_shortest_paths.size();

      CQYDirectedPath* randPos1 = topK_shortest_paths[randIndex1];
      CQYDirectedPath* randPos2 = topK_shortest_paths[randIndex2];

      double randVal1 = getPathPotential(randPos1);
      double randVal2 = getPathPotential(randPos2);

      CQYDirectedPath* maxPos = (randVal1 < randVal2) ? randPos1 : randPos2;

      assert(maxPos != NULL);

      // TODO: improve substrate info saving mechanism for multiple InP

      aRequest.edges[edgeProcessOrder[i]].substrateID = substrateID;
      aRequest.edges[edgeProcessOrder[i]].pathLen = maxPos->GetLength() - 1;
      aRequest.edges[edgeProcessOrder[i]].pathDelay = maxPos->GetCost();

      int maxPathLen = maxPos->GetLength() - 1;
      vector<int> maxPathNodes = maxPos->GetVertexList();
      for (int j = 0; j < maxPathLen; j++) {
        int eID = edgeMap[make_pair(maxPathNodes[j], maxPathNodes[j + 1])];

        aRequest.edges[edgeProcessOrder[i]].subPath.push_back(eID);
        aRequest.edges[edgeProcessOrder[i]].subBW.push_back(aRequest.edges[edgeProcessOrder[i]].bw);

        // remove used bandwidth from the substrate edge
        edges[eID].rest_bw -= aRequest.edges[edgeProcessOrder[i]].bw;
        assert(edges[eID].rest_bw + EPSILON  >= 0);
      }
    }

    break;

  default:
    return INVALID_EM_METHOD;

    break;
  }

  for (j = 0; j < edgeNum; j++) {
    edges[j].rest_bw = list_rest_bw[j];
  }

  if (i != aRequest.edgeNum) {
    return EDGE_MAP_FAILED;
  }

  return EDGE_MAP_SUCCESS;
}

int SubstrateGraph::solveMultiCommodityFlow(VNRequest &aRequest,
    int VNodeOrdering) {
  FILE * fp;
  fp = fopen("ltest.dat", "w");

  int i, j;
  int com_count = aRequest.edgeNum;
  int arc_count;
  int node_count = nodeNum + aRequest.nodeNum;
  int edge_count = edgeNum;

  int validNodeCount;
  vector <int> validNodesWithinReach, nodeProcessOrder;
  vector< vector <int> > validNodesWithinReachVectors;

  // make all substrate nodes untouched
  for (i = 0; i < nodeNum; i++) {
    nodes[i].touched = false;
  }

  for (i = 0; i < aRequest.nodeNum; i++) {
    nodeProcessOrder.push_back(i);
  }

  // set the virtual nodes' processing order
  switch (VNodeOrdering) {
  case VNODE_ORDER_ASC:
    aRequest.sortNodesAscending(nodeProcessOrder);
    break;
  case VNODE_ORDER_DESC:
    aRequest.sortNodesDescending(nodeProcessOrder);
    break;
  case VNODE_ORDER_RAND:
    randomPermutation(nodeProcessOrder);
    break;
  default:
    break;
  }

  for (i = 0; i < aRequest.nodeNum; i++) {
    validNodeCount = findNodesWithinConstraints(aRequest.nodes[nodeProcessOrder[i]],
        aRequest.reqID, aRequest.maxD, validNodesWithinReach);

    if (validNodeCount == 0) {
      cerr << "findNodesWithinConstraints() failed" << endl;

      fclose(fp);
      return NODE_MAP_FAILED;
    }

    validNodesWithinReachVectors.push_back(validNodesWithinReach);

    edge_count += validNodeCount;
  }

  arc_count = edge_count * 2;

  fprintf(fp, "%d %d %d %d %d\n", node_count, arc_count, com_count, edge_count,
      com_count * arc_count);

  fprintf(fp, "ARC COSTS BY COMMODITIES\n");
  for (i = 0; i < com_count; i++) {
    for (j = 0; j < arc_count; j++) {
      fprintf(fp, "1.0 ");
    }
    fprintf(fp, "\n");
  }

  fprintf(fp, "ARC CAPACITIES BY COMMODITIES\n");
  for (i = 0; i < com_count; i++) {
    for (j = 0; j < edgeNum * 2; j++) {
      fprintf(fp, "%.2lf ", edges[j/2].rest_bw);
    }
    for (j = 0; j < aRequest.nodeNum * 2; j++) {
      fprintf(fp, "%.2lf ", (double)MY_INFINITY);
    }
    fprintf(fp, "\n");
  }

  fprintf(fp, "NODE INJECTIONS BY COMMODITIES\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    for (j = 0; j < nodeNum; j++) {
      fprintf(fp, "0 ");
    }
    for (j = 0; j < aRequest.nodeNum; j++) {
      if (aRequest.edges[i].from == j) {
        fprintf(fp, "%.2lf ", aRequest.edges[i].bw);
      } else if (aRequest.edges[i].to == j) {
        fprintf(fp, "%.2lf ", -aRequest.edges[i].bw);
      } else {
        fprintf(fp, "0 ");
      }
    }
    fprintf(fp, "\n");
  }

  fprintf(fp, "ARC MUTUAL CAPACITIES\n");
  for (j = 0; j < edgeNum * 2; j++) {
    fprintf(fp, "%.2lf ", edges[j/2].rest_bw);
  }
  for (j = 0; j < aRequest.nodeNum * 2; j++) {
    fprintf(fp, "%.2lf ", (double)MY_INFINITY);
  }
  fprintf(fp, "\n");

  fprintf(fp, "NETWORK TOPOLOGY\n");
  for (i = 0; i < edgeNum; i ++) {
    fprintf(fp, "%d %d\n", edges[i].from + 1, edges[i].to + 1);
    fprintf(fp, "%d %d\n", edges[i].to + 1, edges[i].from + 1);
  }
  for (i = 0; i < aRequest.nodeNum; i++) {
    int vNWRVSize = validNodesWithinReachVectors[i].size();
    for (j = 0; j < vNWRVSize; j++) {
      fprintf(fp, "%d %d\n", nodeNum + i + 1, validNodesWithinReachVectors[i][j] + 1);
      fprintf(fp, "%d %d\n", validNodesWithinReachVectors[i][j] + 1, nodeNum + i + 1);
    }
  }

  /*for (i = start; i <= end; i ++) {
    if (v2s[i].map == 1) {
      for (j = 0; j < req[i].links; j ++) {
        fprintf(fp, "%d %d\n", v2s[i].snode[req[i].link[j].to]+1, v2s[i].snode[req[i].link[j].from]+1);
      }
    }
    }*/

  /*fprintf(fp, "LOWER AND UPPER BOUNDS\n");
  for (i = 0; i < edgeNum; i ++)
    fprintf(fp, "0 ");
  fprintf(fp, "\n");
  for (i = 0; i < edgeNum; i ++) {
    if (s2v_l[i].rest_bw < 0) s2v_l[i].rest_bw = 0;
    fprintf(fp, "%.2f ", s2v_l[i].rest_bw);
  }
  fprintf(fp, "\n");

  fprintf(fp, "SIDE CONSTRAINTS\n");
  for (i = 0; i < edgeNum; i ++) {
    int p = 0;
    for (j = start; j <= end; j ++) {
      if (req[j].split == LINK_SPLITTABLE && (v2s[j].map == STATE_MAP_NODE || (v2s[j].map == STATE_MAP_LINK && option == ROUTE_MIGRATION))) {
        for (k = 0; k < req[j].links; k ++) {
          fprintf(fp, "%d %d %d 1\n", 2 * i + 1, p +1, i + 1);
          fprintf(fp, "%d %d %d 1\n", 2 * i + 2, p +1, i + 1);
          p +=1;
        }
      }
    }
  }*/

  fclose(fp);
  //system("./lintest");
  //sleep(1);
  //printf("lintest ends\n");

  return 0;
}

int SubstrateGraph::initGraph() {
  int i;

  int x, y;
  double cpu;

  int from, to;
  double bw, dlay;

  FILE *fp = fopen(fileName.c_str(), "rt");

  if (!fp) {
    cout << "failed to open file: " << fileName << endl;
    return COULD_NOT_OPEN_FILE;
  }
  fscanf_s(fp, "%d %d", &nodeNum, &edgeNum);

  // read nodes
  for (i = 0; i < nodeNum; i++) {
    fscanf_s(fp, "%d %d %lf", &x, &y, &cpu);
    nodes.push_back(SubstrateNode(x, y, cpu));
    //printf("%d %d %lf\n", x, y, cpu);
    nodes[i].rest_cpu = cpu;
	nodes[i].ratio=0;
	nodes[i].resi_ratio=1;
	nodes[i].xJ=0;
  }

  // read edges
  for (i = 0; i < edgeNum; i++) {
    fscanf_s(fp, "%d %d %lf %lf", &from, &to, &bw, &dlay);
    edges.push_back(SubstrateEdge(from, to, bw, dlay));
    //printf("%d %d %lf %lf\n", from, to, bw, dlay);
    edges[i].rest_bw = bw;
	edges[i].ratio=0;
	edges[i].resi_ratio=1;
	edges[i].xJ=0;

    // save edge information in nodes
    nodes[from].edgeIDs.push_back(i);
    nodes[to].edgeIDs.push_back(i);

    // save the edge map
    edgeMap[make_pair(from, to)] = i;
    edgeMap[make_pair(to, from)] = i;
  }

    //printf("%d %d %lf\n", x, y, cpu);
   // nodes[i].rest_cpu = cpu;

  fclose(fp);

  return SUCCESS;
}
void SubstrateGraph::inithop(){
	int i,j;
	hop.clear();
	createKSPInputFile(1);
	CQYDirectedGraph dg(KSP_INPUT_FILENAME);
	for(i=0;i<nodeNum;i++)
		for(j=i;j<nodeNum;j++){
		CQYKShortestPaths ksp(dg, i, j, MAX_PATHS_TO_CONSIDER);
		  vector<CQYDirectedPath*> topK_shortest_paths = ksp.GetTopKShortestPaths();

		  if (topK_shortest_paths.empty()) {
			cerr << "topK_shortest_paths is empty" << endl;
			break;
		  }
		  if(topK_shortest_paths.size()>0){
			  hop[make_pair(i,j)]=topK_shortest_paths[0]->GetLength()-1;
			  hop[make_pair(j,i)]=topK_shortest_paths[0]->GetLength()-1;
		  }
		  else{
			  hop[make_pair(i,j)]=0;
			  hop[make_pair(j,i)]=0;
		  }
		}
}

int SubstrateGraph::mapNodes_ViNE(VNRequest &aRequest, int nodeMapStyle, bool aOne, bool bOne){
  int i, j;
  int validNodeCount;
  char commandString[1024], solStatus[1024];
  double objectiveValue;
  //vector<int> validNodesWithinReach[aRequest.nodeNum];
  vector<int> *validNodesWithinReach = new vector<int>[aRequest.nodeNum];  //wu
  // open data file
  FILE *fpDat = fopen(CNLM_DATA_FILE, "w");
  if (fpDat == NULL) {
    cout << "failed to open file: " << CNLM_DATA_FILE << endl;
    return COULD_NOT_OPEN_FILE;
  }

  // create data file
  fprintf(fpDat, "data;\n\n");

  // substrate node set
  fprintf(fpDat, "set N:=");
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, " %d", i);
  }
  fprintf(fpDat, ";\n");

  // virtual node set
  fprintf(fpDat, "set M:=");
  for (i = 0; i < aRequest.nodeNum; i++) {
    fprintf(fpDat, " %d", nodeNum + i);
  }
  fprintf(fpDat, ";\n");

  // flow / virtual edge set
  fprintf(fpDat, "set F:=");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, " f%d", i);
  }
  fprintf(fpDat, ";\n\n");

  // cpu capacity of the substrate and meta nodes
  fprintf(fpDat, "param p:=\n");
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, "%d %.4lf\n", i, nodes[i].rest_cpu);
  }
  for (i = 0; i < aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d %.4lf\n", nodeNum + i, aRequest.nodes[i].cpu);
  }
  fprintf(fpDat, ";\n\n");

  // bandwidth capacity of the substrate and meta edges
  fprintf(fpDat, "param b:\n");
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
  }
  fprintf(fpDat, ":=\n");

  vector<double> bOneDim(nodeNum + aRequest.nodeNum, 0);
  vector< vector<double> > bTwoDim (nodeNum + aRequest.nodeNum, bOneDim);

  for (i = 0; i < edgeNum; i++) {
    bTwoDim[edges[i].from][edges[i].to] = bTwoDim[edges[i].to][edges[i].from] =
      edges[i].rest_bw;
  }

  // make all substrate nodes untouched ( required in findNodesWithinConstraints() )
  for (i = 0; i < nodeNum; i++) {
    nodes[i].touched = false;
  }

  for (i = 0; i < aRequest.nodeNum; i++) {
    printf("%d: ", i);
    validNodeCount = findNodesWithinConstraints(aRequest.nodes[i],
        aRequest.reqID, aRequest.maxD, validNodesWithinReach[i]);

    if (validNodeCount == 0) {
      cerr << "findNodesWithinConstraints() returned 0" << endl;
      break;
    }

    for (j = 0; j < validNodeCount; j++) {
      bTwoDim[validNodesWithinReach[i][j]][nodeNum + i] =
        bTwoDim[nodeNum + i][validNodesWithinReach[i][j]] = META_EDGE_BW;
      printf("%d ", validNodesWithinReach[i][j]);
    }
    printf("\n");
  }

  // if there is any node that can never be mapped
  if (i != aRequest.nodeNum) {
    fclose(fpDat);
    return NODE_MAP_FAILED;
  }

  // write down the b matrix
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
    for (j = 0; j < nodeNum + aRequest.nodeNum; j++) {
      fprintf(fpDat, "%.4lf ", bTwoDim[i][j]);
    }
    fprintf(fpDat, "\n");
  }

  fprintf(fpDat, ";\n\n");

  // alpha
  fprintf(fpDat, "param alpha:\n");
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
  }
  fprintf(fpDat, ":=\n");

  if (aOne == false) {
    for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
      fprintf(fpDat, "%d ", i);
      for (j = 0; j < nodeNum + aRequest.nodeNum; j++) {
        fprintf(fpDat, "%.4lf ", bTwoDim[i][j]);
      }
      fprintf(fpDat, "\n");
    }
  }
  else {
    for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
      fprintf(fpDat, "%d ", i);
      for (j = 0; j < nodeNum + aRequest.nodeNum; j++) {
        fprintf(fpDat, "1 ");
      }
      fprintf(fpDat, "\n");
    }
  }
  fprintf(fpDat, ";\n\n");

  // beta
  fprintf(fpDat, "param beta:=\n");
  if (bOne == false) {
    for (i = 0; i < nodeNum; i++) {
      fprintf(fpDat, "%d %.4lf\n", i, nodes[i].rest_cpu);
    }
    for (i = 0; i < aRequest.nodeNum; i++) {
      fprintf(fpDat, "%d %.4lf\n", nodeNum + i, aRequest.nodes[i].cpu);
    }
  }
  else {
    for (i = 0; i < nodeNum; i++) {
      fprintf(fpDat, "%d 1\n", i);
    }
    for (i = 0; i < aRequest.nodeNum; i++) {
      fprintf(fpDat, "%d 1\n", nodeNum + i);
    }
  }
  fprintf(fpDat, ";\n\n");

  // flow source
  fprintf(fpDat, "param fs:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %d\n", i, aRequest.edges[i].from + nodeNum);
  }
  fprintf(fpDat, ";\n\n");

  // flow destination
  fprintf(fpDat, "param fe:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %d\n", i, aRequest.edges[i].to + nodeNum);
  }
  fprintf(fpDat, ";\n\n");

  // flow requirements
  fprintf(fpDat, "param fd:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %.4lf\n", i, aRequest.edges[i].bw);
  }
  fprintf(fpDat, ";\n\n");

  fprintf(fpDat, "end;\n\n");

  fclose(fpDat);

  // call glpsol
  sprintf(commandString, "glpsol --model %s --data %s --output %s",
      CNLM_LP_MODEL_FILE, CNLM_DATA_FILE, CNLM_OUTPUT_FILE);
  cout << commandString << endl;
  system(commandString);

  // open output file
  FILE *fpOut = fopen(CNLM_OUTPUT_FILE, "r");
  if (fpOut == NULL) {
    cout << "failed to open file: " << CNLM_OUTPUT_FILE << endl;
    return COULD_NOT_OPEN_FILE;
  }

  // parse result from the output file
  for (i = 0; i < 10; i++) {
    fscanf_s(fpOut, "%s", solStatus);
  }

  if (strcmp(solStatus, "OPTIMAL") != 0) {
    fclose(fpOut);

    return NODE_MAP_FAILED;
  }

  for (i = 0; i < 4; i++) {
    fscanf_s(fpOut, "%s", solStatus);
  }

  // LP minimized value
  objectiveValue = atof(solStatus);

  // find the first occurrence of "Marginal"
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (strcmp(solStatus, "Marginal") == 0)
      break;
  }

  // find the second occurrence of "Marginal"
  // that means we are just on top of f and x values
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (strcmp(solStatus, "Marginal") == 0)
      break;
  }

  vector<double> fOneDim(nodeNum + aRequest.nodeNum, 0);
  vector< vector<double> > fTwoDim (nodeNum + aRequest.nodeNum, fOneDim);

  // find the first 'f'
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (solStatus[0] == 'f')
      break;
  }

  // now find out the values
  int fID, from, to, brk;
  double fVal = 0;

  while (1) {
    sscanf(solStatus, "f[f%d,%d,%d]", &fID, &from, &to);

    fscanf_s(fpOut, "%s", solStatus);
    fscanf_s(fpOut, "%s", solStatus);

    fVal = atof(solStatus);
    if (fVal > EPSILON) {
      printf("%d %d %.4lf\n", from, to, fVal);
    }

    fTwoDim[from][to] += fVal;

    while ((brk = fscanf_s(fpOut, "%s", solStatus)) == 1) {
      if (solStatus[0] == 'f' || solStatus[0] == 'x') {
        break;
      }
    }

    if (solStatus[0] == 'x') {
      break;
    }
  }

  // find the first 'x'
  /*while (fscanf(fpOut, "%s", solStatus) == 1) {
    if (solStatus[0] == 'x')
      break;
  }*/

  // now find out the values
  // int from, to, brk;
  double xVal;
  //vector< pair<int, double> > possibleMaps[aRequest.nodeNum];
  vector< pair<int, double> > *possibleMaps = new vector< pair<int, double> >[aRequest.nodeNum];//wu
  vector<double> xOneDim(nodeNum + aRequest.nodeNum, 0);
  vector< vector<double> > xTwoDim (nodeNum + aRequest.nodeNum, fOneDim);

  while (1) {
    sscanf(solStatus, "x[%d,%d]", &from, &to);

    fscanf_s(fpOut, "%s", solStatus);
    fscanf_s(fpOut, "%s", solStatus);

    xVal = atof(solStatus);
    xTwoDim[from][to] = xVal * (fTwoDim[from][to] + fTwoDim[to][from]);

    if (from >= nodeNum && to < nodeNum && xVal > EPSILON &&
        (fTwoDim[from][to] + fTwoDim[to][from] > EPSILON)) {
      possibleMaps[from - nodeNum].push_back(make_pair(to, xVal));
    }

    while ((brk = fscanf_s(fpOut, "%s", solStatus)) == 1) {
      if (solStatus[0] == 'x') {
        break;
      }
    }

    if (brk != 1) {
      break;
    }
  }

  // got the x values
  // now do Node Mapping based on nodeMapStyle value

  assert(nodeMapStyle == NM_DETERMINISTIC || nodeMapStyle == NM_RANDOMIZED);

  for (i = 0; i < aRequest.nodeNum; i++) {
    for (j = 0; j < possibleMaps[i].size(); j++) {
      printf("(%3d,%.6lf) ", possibleMaps[i][j].first, possibleMaps[i][j].second);
    }
    printf("\n");
  }

  for (i = 0; i < aRequest.nodeNum; i++) {
    double total = 0;
    for (j = 0; j < validNodesWithinReach[i].size(); j++) {
      total += xTwoDim[nodeNum + i][validNodesWithinReach[i][j]];
    }

    if (total < EPSILON) {
      total = 1.0;
    }

    for (j = 0; j < validNodesWithinReach[i].size(); j++) {
      xTwoDim[nodeNum + i][validNodesWithinReach[i][j]] /= total;
      printf("(%3d,%.6lf) ", validNodesWithinReach[i][j], xTwoDim[nodeNum + i][validNodesWithinReach[i][j]]);
    }
    printf("\n");
  }

  if (nodeMapStyle == NM_DETERMINISTIC) {
    for (i = 0; i < aRequest.nodeNum; i++) {
      int maxPos = NOT_MAPPED_YET;
      double maxVal = 0;

      /*for (j = 0; j < possibleMaps[i].size(); j++) {
        if (possibleMaps[i][j].second >= maxVal &&
            nodes[possibleMaps[i][j].first].touched == false) {
          maxPos = possibleMaps[i][j].first;
          maxVal = possibleMaps[i][j].second;
        }
      }*/
      for (j = 0; j < validNodesWithinReach[i].size(); j++) {
        if (xTwoDim[nodeNum + i][validNodesWithinReach[i][j]] >= maxVal &&
            nodes[validNodesWithinReach[i][j]].touched == false) {
          maxPos = validNodesWithinReach[i][j];
          maxVal = xTwoDim[nodeNum + i][validNodesWithinReach[i][j]];
        }
      }

      if (maxPos == NOT_MAPPED_YET) {
        fclose(fpOut);

        return NODE_MAP_FAILED;
      }

      // update the virtual node's mapping information
      aRequest.nodes[i].substrateID = substrateID;
      aRequest.nodes[i].subNodeID = maxPos;

      nodes[aRequest.nodes[i].subNodeID].touched = true;
    }
  }
  else {
    for (i = 0; i < aRequest.nodeNum; i++) {
      int randTry = 0;
      int randPos = NOT_MAPPED_YET;

      while (randTry < 16) {
        double randVal = (double)rand() / (double)INT_MAX;

        /*for (j = 0; j < possibleMaps[i].size(); j++) {
          randVal -= possibleMaps[i][j].second;
          if (randVal <= 0.0 && nodes[possibleMaps[i][j].first].touched == false) {
            randPos = possibleMaps[i][j].first;
            break;
          }
        }*/

        for (j = 0; j < validNodesWithinReach[i].size(); j++) {
          randVal -= xTwoDim[nodeNum + i][validNodesWithinReach[i][j]];
          if (randVal <= 0.0 && nodes[validNodesWithinReach[i][j]].touched == false) {
            randPos = validNodesWithinReach[i][j];
            break;
          }
        }

        if (randPos != NOT_MAPPED_YET) {
          break;
        }
        randTry++;
      }

      randTry = 0;
      if (randPos == NOT_MAPPED_YET) {
        while (randTry < 16) {
          int randVal = rand() % validNodesWithinReach[i].size();

          if (nodes[validNodesWithinReach[i][randVal]].touched == false) {
            randPos = validNodesWithinReach[i][randVal];
            break;
          }

          randTry++;
        }
      }

      if (randPos == NOT_MAPPED_YET) {
        fclose(fpOut);

        return NODE_MAP_FAILED;
      }

      // update the virtual node's mapping information
      aRequest.nodes[i].substrateID = substrateID;
      aRequest.nodes[i].subNodeID = randPos;

      nodes[aRequest.nodes[i].subNodeID].touched = true;
    }
  }

  fclose(fpOut);

  return NODE_MAP_SUCCESS;
}
//my mapping
/*
int SubstrateGraph::maponestage(VNRequest &aRequest, int MapStyle, bool aOne, bool bOne) {
}

*/
int SubstrateGraph::my_nodemap(VNRequest &aRequest,int mappingMethod){
  vector<int> Snode_in;
  vector<int> Vnode_in;
  Snode_in.clear();
  Vnode_in.clear();
  int iterators=0;
  LABEL_INIT:
  int i,j,firstnode;
  
  vector<int> nodeProcessOrder;
  // make all substrate nodes untouched
  for (i = 0; i < nodeNum; i++) {
    nodes[i].touched = false;
	nodes[i].ratio_temp=0;
  }
  for(i = 0; i < edgeNum; i++){
	  edges[i].ratio_temp=0;
  }
  // make all Virtual nodes untouched
  for (i = 0; i < aRequest.nodeNum; i++) {
	  aRequest.nodes[i].touched = false;
  }
  // make all Virtual edges untouched
  for (i = 0; i < aRequest.edgeNum; i++) {
	  aRequest.edges[i].touched = false;
  }
  firstnode=aRequest.findmax_degree();
  // map the virtual nodes using specific algorithms
  switch(mappingMethod) {
  case MY_NODE_MAPPING_DEPTH:
	  aRequest.depth_searching(firstnode,nodeProcessOrder);
	  break;
  case MY_NODE_MAPPING_BREADTH:
	  aRequest.breadth_searching(firstnode,nodeProcessOrder);
	  break;
  default:
	  {
		  for (i = 0; i < aRequest.nodeNum; i++)
			 nodeProcessOrder[i]=i;
	  }
  }
    for (i = 0; i < aRequest.nodeNum; i++){
	  aRequest.nodes[i].touched = false;
	}
	vector<int> SNodes,Nb_nodes,Nb_edges;
	int SNodeCount,tempNodeID;
	vector<int>::iterator iter;
	for(i=0;i<aRequest.nodeNum;i++){
		int nodeID=nodeProcessOrder[i];
		SNodes.clear();
		SNodeCount = findNodesWithinConstraints(aRequest.nodes[nodeID],  //备选x,y,cpu物理节点集合
        aRequest.reqID, aRequest.maxD, SNodes);
		for(j=0;j<Vnode_in.size();j++){
			if(Vnode_in[j]==nodeID){
				for(iter=SNodes.begin();iter != SNodes.end();)
				{
					if((*iter)==Snode_in[j]){
						iter=SNodes.erase(iter);
						SNodeCount --;
					}
					else
					{
						iter++;
					}
				}
			}
		}
		if (SNodeCount == 0){
			cerr << "findSNodesWithinConstraints() returned 0" << endl;
			clean(aRequest);
			return NODE_MAP_FAILED ;
		}
		printf("%d:",nodeID);
		for(j=0;j<SNodeCount;j++){
			printf(" %d",SNodes[j]);
		}
		printf("\n");

		int vnode=-1;//
		if(i==0) 
			tempNodeID=firstnodemap(aRequest,nodeID);
		else
			tempNodeID=node_and_path(aRequest,nodeID,SNodes,vnode);
		if(tempNodeID==-1){ 
			clean(aRequest);
			return NODE_MAP_FAILED;
		}
		if(tempNodeID==-2){
			clean(aRequest);
			iterators++;
			printf("iterators=%d\n",iterators);
			printf("vnode=%d\n",vnode);
			printf("Num=%d\n",aRequest.nodeNum);
			Snode_in.push_back(aRequest.nodes[vnode].subNodeID);
			Vnode_in.push_back(vnode);
			printf("iterators=%d\n",iterators);
			if(iterators<7) goto LABEL_INIT; 
			return NODE_MAP_FAILED;
		}
		printf("Map<%d, %d>\n",nodeID,tempNodeID);
		aRequest.nodes[nodeID].substrateID = substrateID;
		aRequest.nodes[nodeID].subNodeID = tempNodeID;
		nodes[aRequest.nodes[nodeID].subNodeID].touched = true;
		nodes[aRequest.nodes[nodeID].subNodeID].ratio_temp=nodes[aRequest.nodes[nodeID].subNodeID].ratio_temp+aRequest.nodes[nodeID].cpu/nodes[aRequest.nodes[nodeID].subNodeID].rest_cpu;
		aRequest.nodes[nodeID].touched=true;
	}
	clean(aRequest,false);
	return NODE_MAP_SUCCESS;
}



int SubstrateGraph::my_new_nodemap(VNRequest &aRequest){
  vector<int> Snode_in;
  vector<int> Vnode_in;
  Snode_in.clear();
  Vnode_in.clear();
  int iterators=0;
LABEL_INIT:
  int i,j,firstnode;
  
  vector<int> nodeProcessOrder;
  // make all substrate nodes untouched
  for (i = 0; i < nodeNum; i++) {
    nodes[i].touched = false;
	nodes[i].ratio_temp=0;
  }
  for(i = 0; i < edgeNum; i++){
	  edges[i].ratio_temp=0;
  }
  // make all Virtual nodes untouched
  for (i = 0; i < aRequest.nodeNum; i++) {
	  aRequest.nodes[i].touched = false;
  }
  // make all Virtual edges untouched
  for (i = 0; i < aRequest.edgeNum; i++) {
	  aRequest.edges[i].touched = false;
  }
  firstnode=aRequest.findmax_degree();
  //if(aRequest.topology())
	  aRequest.depth_searching(firstnode,nodeProcessOrder);
 // else
	//  aRequest.breadth_searching(firstnode,nodeProcessOrder);

  /*default:
	  {
		  for (i = 0; i < aRequest.nodeNum; i++)
			 nodeProcessOrder[i]=i;
	  }*/
    for (i = 0; i < aRequest.nodeNum; i++){
	  aRequest.nodes[i].touched = false;
	}
	vector<int> SNodes,Nb_nodes,Nb_edges;
	vector<int>::iterator iter;
	int SNodeCount,tempNodeID;
	for(i=0;i<aRequest.nodeNum;i++){
		int nodeID=nodeProcessOrder[i];
		SNodes.clear();
		SNodeCount = findNodesWithinConstraints(aRequest.nodes[nodeID],  //备选x,y,cpu物理节点集合
        aRequest.reqID, aRequest.maxD, SNodes);
		for(j=0;j<Vnode_in.size();j++){
			if(Vnode_in[j]==nodeID){
				for(iter=SNodes.begin();iter != SNodes.end();)
				{
					if((*iter)==Snode_in[j]){
						iter=SNodes.erase(iter);
						SNodeCount --;
					}
					else
					{
						iter++;
					}
				}
			}
		}
		if (SNodeCount == 0){
			printf("For virtual node %d:   ",nodeID);
			cerr << "findSNodesWithinConstraints() returned 0" << endl;
			clean(aRequest);
			return NODE_MAP_FAILED ;
		}
		
		for(j=0;j<SNodeCount;j++){
			printf(" %d",SNodes[j]);
		}
		printf("\n");
		int vnode=-1;//
		if(i==0) 
			tempNodeID=firstnodemap(aRequest,nodeID);
		else
			tempNodeID=new_node_and_path(aRequest,nodeID,SNodes,vnode);
		if(tempNodeID==-1){ 
			clean(aRequest);
			return NODE_MAP_FAILED;
		}
		if(tempNodeID==-2){
			iterators++;
			printf("iterators=%d\n",iterators);
			printf("vnode=%d\n",vnode);
			printf("Num=%d\n",aRequest.nodeNum);
			Snode_in.push_back(aRequest.nodes[vnode].subNodeID);
			Vnode_in.push_back(vnode);
			clean(aRequest);
			printf("iterators=%d\n",iterators);
			if(iterators<7) goto LABEL_INIT; 
			return NODE_MAP_FAILED;
		}
		printf("Map<%d, %d>\n",nodeID,tempNodeID);
		aRequest.nodes[nodeID].substrateID = substrateID;
		aRequest.nodes[nodeID].subNodeID = tempNodeID;
		nodes[aRequest.nodes[nodeID].subNodeID].touched = true;
		nodes[aRequest.nodes[nodeID].subNodeID].ratio_temp=nodes[aRequest.nodes[nodeID].subNodeID].ratio_temp+aRequest.nodes[nodeID].cpu/nodes[aRequest.nodes[nodeID].subNodeID].rest_cpu;
		aRequest.nodes[nodeID].touched=true;
	}
	clean(aRequest,false);
	return NODE_MAP_SUCCESS;
}

int SubstrateGraph::mapNodes_ViNE_v2(VNRequest &aRequest, int nodeMapStyle, bool aOne, bool bOne) {
  int i, j;
  //int fromSubNode, toSubNode;
  char commandString[1024], solStatus[1024];
  double objectiveValue;

  int validNodeCount;
  vector<int> validNodesWithinReach;

  // open data file
  FILE *fpDat = fopen(MCF_DATA_FILE, "w");
  if (fpDat == NULL) {
    cout << "failed to open file: " << MCF_DATA_FILE << endl;
    return COULD_NOT_OPEN_FILE;
  }

  // create data file
  fprintf(fpDat, "data;\n\n");

  // substrate node set
  fprintf(fpDat, "set N:=");
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, " %d", i);
  }
  for (i = 0; i < aRequest.nodeNum; i++) {
    fprintf(fpDat, " %d", nodeNum + i);
  }
  fprintf(fpDat, ";\n");

  // flow / virtual edge set
  fprintf(fpDat, "set F:=");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, " f%d", i);
  }
  fprintf(fpDat, ";\n\n");

  // bandwidth capacity of the substrate and meta edges
  fprintf(fpDat, "param b:\n");
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
  }
  fprintf(fpDat, ":=\n");

  vector<double> bOneDim(nodeNum + aRequest.nodeNum, 0);
  vector< vector<double> > bTwoDim (nodeNum + aRequest.nodeNum, bOneDim);

  for (i = 0; i < edgeNum; i++) {
    bTwoDim[edges[i].from][edges[i].to] = bTwoDim[edges[i].to][edges[i].from] =
      edges[i].rest_bw;
  }

  // make all substrate nodes untouched ( required in findNodesWithinConstraints() )
  for (i = 0; i < nodeNum; i++) {
    nodes[i].touched = false;
  }

  for (i = 0; i < aRequest.nodeNum; i++) {
    printf("%d: ", i);
    validNodeCount = findNodesWithinConstraints(aRequest.nodes[i],
        aRequest.reqID, aRequest.maxD, validNodesWithinReach);

    if (validNodeCount == 0) {
      cerr << "findNodesWithinConstraints() returned 0" << endl;
      break;
    }

    for (j = 0; j < validNodeCount; j++) {
      bTwoDim[validNodesWithinReach[j]][nodeNum + i] =
        bTwoDim[nodeNum + i][validNodesWithinReach[j]] = META_EDGE_BW;
      printf("%d ", validNodesWithinReach[j]);
    }
    printf("\n");
  }

  // if there is any node that can never be mapped
  if (i != aRequest.nodeNum) {
    fclose(fpDat);
    return NODE_MAP_FAILED;
  }

  // write down the b matrix
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
    for (j = 0; j < nodeNum + aRequest.nodeNum; j++) {
      fprintf(fpDat, "%.4lf ", bTwoDim[i][j]);
    }
    fprintf(fpDat, "\n");
  }

  fprintf(fpDat, ";\n\n");

  // alpha
  fprintf(fpDat, "param alpha:\n");
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
  }
  fprintf(fpDat, ":=\n");

  if (aOne == false) {
    for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
      fprintf(fpDat, "%d ", i);
      for (j = 0; j < nodeNum + aRequest.nodeNum; j++) {
        fprintf(fpDat, "%.4lf ", bTwoDim[i][j]);
      }
      fprintf(fpDat, "\n");
    }
  }
  else {
    for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
      fprintf(fpDat, "%d ", i);
      for (j = 0; j < nodeNum + aRequest.nodeNum; j++) {
        fprintf(fpDat, "1 ");
      }
      fprintf(fpDat, "\n");
    }
  }
  fprintf(fpDat, ";\n\n");

  // flow source
  fprintf(fpDat, "param fs:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %d\n", i, aRequest.edges[i].from + nodeNum);
  }
  fprintf(fpDat, ";\n\n");

  // flow destination
  fprintf(fpDat, "param fe:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %d\n", i, aRequest.edges[i].to + nodeNum);
  }
  fprintf(fpDat, ";\n\n");

  // flow requirements
  fprintf(fpDat, "param fd:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %.4lf\n", i, aRequest.edges[i].bw);
  }
  fprintf(fpDat, ";\n\n");

  fprintf(fpDat, "end;\n\n");

  fclose(fpDat);

  // call glpsol
  sprintf(commandString, "glpsol --model %s --data %s --output %s",
      MCF_MODEL_FILE, MCF_DATA_FILE, MCF_OUTPUT_FILE);
  cout << commandString << endl;
  system(commandString);

  // open output file
  FILE *fpOut = fopen(MCF_OUTPUT_FILE, "r");
  if (fpOut == NULL) {
    cout << "failed to open file: " << MCF_OUTPUT_FILE << endl;
    return COULD_NOT_OPEN_FILE;
  }

  // parse result from the output file
  for (i = 0; i < 10; i++) {
    fscanf_s(fpOut, "%s", solStatus);
  }

  if (strcmp(solStatus, "OPTIMAL") != 0) {
    fclose(fpOut);

    return NODE_MAP_FAILED;
  }

  for (i = 0; i < 4; i++) {
    fscanf_s(fpOut, "%s", solStatus);
  }

  // LP minimized value
  objectiveValue = atof(solStatus);

  cout << objectiveValue << endl;

  // find the first occurrence of "Marginal"
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (strcmp(solStatus, "Marginal") == 0)
      break;
  }

  // find the second occurrence of "Marginal"
  // that means we are just on top of f values
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (strcmp(solStatus, "Marginal") == 0)
      break;
  }

  vector<double> fOneDim(nodeNum + aRequest.nodeNum, 0);
  vector< vector<double> > fTwoDim (nodeNum + aRequest.nodeNum, fOneDim);

  // find the first 'f'
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (solStatus[0] == 'f')
      break;
  }

  // now find out the values
  int fID, from, to, brk;
  double fVal = 0;

  while (1) {
    sscanf(solStatus, "f[f%d,%d,%d]", &fID, &from, &to);

    fscanf_s(fpOut, "%s", solStatus);
    fscanf_s(fpOut, "%s", solStatus);

    fVal = atof(solStatus);
    if (fVal > EPSILON) {
      printf("%d %d %.4lf\n", from, to, fVal);
    }

    fTwoDim[from][to] += fVal;

    while ((brk = fscanf_s(fpOut, "%s", solStatus)) == 1) {
      if (solStatus[0] == 'f') {
        break;
      }
    }

    if (brk != 1) {
      break;
    }
  }

  for (i = nodeNum; i < nodeNum + aRequest.nodeNum; i++) {
    for (j = 0; j < nodeNum; j++) {
      fTwoDim[i][j] += fTwoDim[j][i];
    }
  }

  //vector< pair<int, double> > possibleMaps[aRequest.nodeNum];
  vector< pair<int, double> > *possibleMaps = new vector< pair<int, double> >[aRequest.nodeNum];//wu
  for (i = nodeNum; i < nodeNum + aRequest.nodeNum; i++) {
    double total = 0;
    for (j = 0; j < nodeNum; j++) {
      total += fTwoDim[i][j];
    }

    if (total < EPSILON) {
      fclose(fpOut);

      return NODE_MAP_FAILED;
    }

    for (j = 0; j < nodeNum; j++) {
      fTwoDim[i][j] /= total;
      printf("%.4lf ", fTwoDim[i][j]);
      if (fTwoDim[i][j] > EPSILON) {
        possibleMaps[i - nodeNum].push_back(make_pair(j, fTwoDim[i][j]));
      }
    }
    printf("\n");
  }

  assert(nodeMapStyle == NM_DETERMINISTIC || nodeMapStyle == NM_RANDOMIZED);

  for (i = 0; i < aRequest.nodeNum; i++) {
    for (j = 0; j < possibleMaps[i].size(); j++) {
      printf("(%3d,%.6lf) ", possibleMaps[i][j].first, possibleMaps[i][j].second);
    }
    printf("\n");
  }

  if (nodeMapStyle == NM_DETERMINISTIC) {
    for (i = 0; i < aRequest.nodeNum; i++) {
      int maxPos = NOT_MAPPED_YET;
      double maxVal = 0;

      for (j = 0; j < possibleMaps[i].size(); j++) {
        if (possibleMaps[i][j].second > maxVal &&
            nodes[possibleMaps[i][j].first].touched == false) {
          maxPos = possibleMaps[i][j].first;
          maxVal = possibleMaps[i][j].second;
        }
      }

      if (maxPos == NOT_MAPPED_YET) {
        fclose(fpOut);

        return NODE_MAP_FAILED;
      }

      // update the virtual node's mapping information
      aRequest.nodes[i].substrateID = substrateID;
      aRequest.nodes[i].subNodeID = maxPos;

      nodes[aRequest.nodes[i].subNodeID].touched = true;
    }
  }
  else {
    for (i = 0; i < aRequest.nodeNum; i++) {
      int randTry = 0;
      int randPos = NOT_MAPPED_YET;

      while (randTry < 16) {
        double randVal = (double)rand() / (double)INT_MAX;

        for (j = 0; j < possibleMaps[i].size(); j++) {
          randVal -= possibleMaps[i][j].second;
          if (randVal <= 0.0 && nodes[possibleMaps[i][j].first].touched == false) {
            randPos = possibleMaps[i][j].first;
            break;
          }
        }

        if (randPos != NOT_MAPPED_YET) {
          break;
        }
        randTry++;
      }

      if (randPos == NOT_MAPPED_YET) {
        fclose(fpOut);

        return NODE_MAP_FAILED;
      }

      // update the virtual node's mapping information
      aRequest.nodes[i].substrateID = substrateID;
      aRequest.nodes[i].subNodeID = randPos;

      nodes[aRequest.nodes[i].subNodeID].touched = true;
    }
  }

  fclose(fpOut);

  return NODE_MAP_SUCCESS;
}

int SubstrateGraph::mapEdges_ViNE(VNRequest &aRequest, bool aOne, bool bOne) {
  int i, j;
  int fromSubNode, toSubNode;
  char commandString[1024], solStatus[1024];
  double objectiveValue;

  // open data file
  FILE *fpDat = fopen(MCF_DATA_FILE, "w");
  if (fpDat == NULL) {
    cout << "failed to open file: " << MCF_DATA_FILE << endl;
    return COULD_NOT_OPEN_FILE;
  }

  // create data file
  fprintf(fpDat, "data;\n\n");

  // substrate node set
  fprintf(fpDat, "set N:=");
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, " %d", i);
  }
  fprintf(fpDat, ";\n");

  // flow / virtual edge set
  fprintf(fpDat, "set F:=");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, " f%d", i);
  }
  fprintf(fpDat, ";\n\n");

  // bandwidth capacity of the substrate edges
  fprintf(fpDat, "param b:\n");
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
  }
  fprintf(fpDat, ":=\n");

  vector<double> bOneDim(nodeNum, 0);
  vector< vector<double> > bTwoDim (nodeNum, bOneDim);

  for (i = 0; i < edgeNum; i++) {
    bTwoDim[edges[i].from][edges[i].to] = bTwoDim[edges[i].to][edges[i].from] =
      edges[i].rest_bw;
  }

  // write down the b matrix
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
    for (j = 0; j < nodeNum; j++) {
      fprintf(fpDat, "%.4lf ", bTwoDim[i][j]);
    }
    fprintf(fpDat, "\n");
  }

  fprintf(fpDat, ";\n\n");

  // alpha
  fprintf(fpDat, "param alpha:\n");
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
  }
  fprintf(fpDat, ":=\n");

  if (aOne == false) {
    for (i = 0; i < nodeNum; i++) {
      fprintf(fpDat, "%d ", i);
      for (j = 0; j < nodeNum; j++) {
        fprintf(fpDat, "%.4lf ", bTwoDim[i][j]);
      }
      fprintf(fpDat, "\n");
    }
  }
  else {
    for (i = 0; i < nodeNum; i++) {
      fprintf(fpDat, "%d ", i);
      for (j = 0; j < nodeNum; j++) {
        fprintf(fpDat, "1 ");
      }
      fprintf(fpDat, "\n");
    }
  }
  fprintf(fpDat, ";\n\n");

  // flow source
  fprintf(fpDat, "param fs:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fromSubNode = aRequest.nodes[aRequest.edges[i].from].subNodeID;
    fprintf(fpDat, "f%d %d\n", i, fromSubNode);
  }
  fprintf(fpDat, ";\n\n");

  // flow destination
  fprintf(fpDat, "param fe:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    toSubNode = aRequest.nodes[aRequest.edges[i].to].subNodeID;
    fprintf(fpDat, "f%d %d\n", i, toSubNode);
  }
  fprintf(fpDat, ";\n\n");

  // flow requirements
  fprintf(fpDat, "param fd:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %.4lf\n", i, aRequest.edges[i].bw);
  }
  fprintf(fpDat, ";\n\n");

  fprintf(fpDat, "end;\n\n");

  fclose(fpDat);

  // call glpsol
  sprintf(commandString, "glpsol --model %s --data %s --output %s",
      MCF_MODEL_FILE, MCF_DATA_FILE, MCF_OUTPUT_FILE);
  cout << commandString << endl;
  system(commandString);

  // open output file
  FILE *fpOut = fopen(MCF_OUTPUT_FILE, "r");
  if (fpOut == NULL) {
    cout << "failed to open file: " << MCF_OUTPUT_FILE << endl;
    return COULD_NOT_OPEN_FILE;
  }

  // parse result from the output file
  for (i = 0; i < 10; i++) {
    fscanf_s(fpOut, "%s", solStatus);
  }

  if (strcmp(solStatus, "OPTIMAL") != 0) {
    fclose(fpOut);

    return EDGE_MAP_FAILED;
  }

  for (i = 0; i < 4; i++) {
    fscanf_s(fpOut, "%s", solStatus);
  }

  // LP minimized value
  objectiveValue = atof(solStatus);

  cout << objectiveValue << endl;

  // find the first occurrence of "Marginal"
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (strcmp(solStatus, "Marginal") == 0)
      break;
  }

  // find the second occurrence of "Marginal"
  // that means we are just on top of f values
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (strcmp(solStatus, "Marginal") == 0)
      break;
  }

  // find the first 'f'
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (solStatus[0] == 'f')
      break;
  }

  // now find out the values
  int fID, from, to, brk;
  double fVal;

  while (1) {
    sscanf(solStatus, "f[f%d,%d,%d]", &fID, &from, &to);

    fscanf_s(fpOut, "%s", solStatus);
    fscanf_s(fpOut, "%s", solStatus);

    fVal = atof(solStatus);

    if (fVal > 0) {
      aRequest.edges[fID].substrateID = substrateID;
      aRequest.edges[fID].pathLen++;
      // aRequest.edges[fID].pathDelay = ;

      int eID = edgeMap[make_pair(from, to)];

      aRequest.edges[fID].subPath.push_back(eID);
      aRequest.edges[fID].subBW.push_back(fVal);

      // cout << fVal << " " << eID << " "  << from << " " << to << endl;
    }

    while ((brk = fscanf_s(fpOut, "%s", solStatus)) == 1) {
      if (solStatus[0] == 'f') {
        break;
      }
    }

    if (brk != 1) {
      break;
    }
  }

  fclose(fpOut);

  return EDGE_MAP_SUCCESS;
}

int SubstrateGraph::mapByMIP(VNRequest &aRequest) {
  int i, j;
  int validNodeCount;
  char commandString[1024], solStatus[1024];
  double objectiveValue;
  vector<int> validNodesWithinReach;

  // open data file
  FILE *fpDat = fopen(CNLM_DATA_FILE, "w");
  if (fpDat == NULL) {
    cout << "failed to open file: " << CNLM_DATA_FILE << endl;
    return COULD_NOT_OPEN_FILE;
  }

  // create data file
  fprintf(fpDat, "data;\n\n");

  // substrate node set
  fprintf(fpDat, "set N:=");
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, " %d", i);
  }
  fprintf(fpDat, ";\n");

  // virtual node set
  fprintf(fpDat, "set M:=");
  for (i = 0; i < aRequest.nodeNum; i++) {
    fprintf(fpDat, " %d", nodeNum + i);
  }
  fprintf(fpDat, ";\n");

  // flow / virtual edge set
  fprintf(fpDat, "set F:=");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, " f%d", i);
  }
  fprintf(fpDat, ";\n\n");

  // cpu capacity of the substrate and meta nodes
  fprintf(fpDat, "param p:=\n");
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, "%d %.4lf\n", i, nodes[i].rest_cpu);
  }
  for (i = 0; i < aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d %.4lf\n", nodeNum + i, aRequest.nodes[i].cpu);
  }
  fprintf(fpDat, ";\n\n");

  // bandwidth capacity of the substrate and meta edges
  fprintf(fpDat, "param b:\n");
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
  }
  fprintf(fpDat, ":=\n");

  vector<double> bOneDim(nodeNum + aRequest.nodeNum, 0);
  vector< vector<double> > bTwoDim (nodeNum + aRequest.nodeNum, bOneDim);

  for (i = 0; i < edgeNum; i++) {
    bTwoDim[edges[i].from][edges[i].to] = bTwoDim[edges[i].to][edges[i].from] =
      edges[i].rest_bw;
  }

  // make all substrate nodes untouched ( required in findNodesWithinConstraints() )
  for (i = 0; i < nodeNum; i++) {
    nodes[i].touched = false;
  }

  for (i = 0; i < aRequest.nodeNum; i++) {
    validNodeCount = findNodesWithinConstraints(aRequest.nodes[i],
        aRequest.reqID, aRequest.maxD, validNodesWithinReach);

    if (validNodeCount == 0) {
      cerr << "findNodesWithinConstraints() returned 0" << endl;
      break;
    }

    for (j = 0; j < validNodeCount; j++) {
      bTwoDim[validNodesWithinReach[j]][nodeNum + i] =
        bTwoDim[nodeNum + i][validNodesWithinReach[j]] = META_EDGE_BW;
    }
  }

  // if there is any node that can never be mapped
  if (i != aRequest.nodeNum) {
    fclose(fpDat);
    return NODE_MAP_FAILED;
  }

  // write down the b matrix
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
    for (j = 0; j < nodeNum + aRequest.nodeNum; j++) {
      fprintf(fpDat, "%.4lf ", bTwoDim[i][j]);
    }
    fprintf(fpDat, "\n");
  }

  fprintf(fpDat, ";\n\n");

  // alpha
  fprintf(fpDat, "param alpha:\n");
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
  }
  fprintf(fpDat, ":=\n");

  // TODO: using same values as b :-S
  for (i = 0; i < nodeNum + aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d ", i);
    for (j = 0; j < nodeNum + aRequest.nodeNum; j++) {
      fprintf(fpDat, "%.4lf ", bTwoDim[i][j]);
    }
    fprintf(fpDat, "\n");
  }

  fprintf(fpDat, ";\n\n");

  // beta
  fprintf(fpDat, "param beta:=\n");
  // TODO: using same values as p :-S
  for (i = 0; i < nodeNum; i++) {
    fprintf(fpDat, "%d %.4lf\n", i, nodes[i].rest_cpu);
  }
  for (i = 0; i < aRequest.nodeNum; i++) {
    fprintf(fpDat, "%d %.4lf\n", nodeNum + i, aRequest.nodes[i].cpu);
  }
  fprintf(fpDat, ";\n\n");

  // flow source
  fprintf(fpDat, "param fs:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %d\n", i, aRequest.edges[i].from + nodeNum);
  }
  fprintf(fpDat, ";\n\n");

  // flow destination
  fprintf(fpDat, "param fe:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %d\n", i, aRequest.edges[i].to + nodeNum);
  }
  fprintf(fpDat, ";\n\n");

  // flow requirements
  fprintf(fpDat, "param fd:=\n");
  for (i = 0; i < aRequest.edgeNum; i++) {
    fprintf(fpDat, "f%d %.4lf\n", i, aRequest.edges[i].bw);
  }
  fprintf(fpDat, ";\n\n");

  fprintf(fpDat, "end;\n\n");

  fclose(fpDat);

  // call glpsol
  sprintf(commandString, "glpsol --model %s --data %s --output %s --nomip",
      CNLM_MIP_MODEL_FILE, CNLM_DATA_FILE, CNLM_OUTPUT_FILE);
  cout << commandString << endl;
  system(commandString);

  // open output file
  FILE *fpOut = fopen(CNLM_OUTPUT_FILE, "r");
  if (fpOut == NULL) {
    cout << "failed to open file: " << CNLM_OUTPUT_FILE << endl;
    return COULD_NOT_OPEN_FILE;
  }

  // parse result from the output file
  for (i = 0; i < 15; i++) {
    fscanf_s(fpOut, "%s", solStatus);
  }

  if (strcmp(solStatus, "OPTIMAL") != 0) {
    fclose(fpOut);

    return BOTH_MAP_FAILED;
  }

  for (i = 0; i < 4; i++) {
    fscanf_s(fpOut, "%s", solStatus);
  }

  // LP minimized value
  objectiveValue = atof(solStatus);

  // find the first occurrence of "Upper"
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (strcmp(solStatus, "Upper") == 0)
      break;
  }

  // find the second occurrence of "Upper"
  // that means we are just on top of f and x values
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (strcmp(solStatus, "Upper") == 0)
      break;
  }

  // find the first 'f'
  while (fscanf_s(fpOut, "%s", solStatus) == 1) {
    if (solStatus[0] == 'f') {
      break;
    }
  }

  // now find out the values
  int fID, from, to, brk;
  double fVal;

  while (1) {
    sscanf_s(solStatus, "f[f%d,%d,%d]", &fID, &from, &to);

    fscanf_s(fpOut, "%s", solStatus);

    fVal = atof(solStatus);

    if (fVal > 0) {
      aRequest.edges[fID].substrateID = substrateID;
      aRequest.edges[fID].pathLen++;
      // aRequest.edges[fID].pathDelay = ;

      int eID = edgeMap[make_pair(from, to)];

      aRequest.edges[fID].subPath.push_back(eID);
      aRequest.edges[fID].subBW.push_back(fVal);

      // cout << fVal << " " << eID << " "  << from << " " << to << endl;
    }

    while ((brk = fscanf_s(fpOut, "%s", solStatus)) == 1) {
      if (solStatus[0] == 'f' || solStatus[0] == 'x') {
        break;
      }
    }

    if (solStatus[0] == 'x') {
      break;
    }
  }

  // got the first 'x'; now find out the values
  int xVal;

  while (1) {
    sscanf(solStatus, "x[%d,%d]", &from, &to);

    fscanf_s(fpOut, "%s", solStatus);
    fscanf_s(fpOut, "%s", solStatus);

    xVal = atoi(solStatus);

    if (from >= nodeNum && to < nodeNum && xVal == 1) {
      assert(nodes[aRequest.nodes[from - nodeNum].subNodeID].touched == false);

      // update the virtual node's mapping information
      aRequest.nodes[from - nodeNum].substrateID = substrateID;
      aRequest.nodes[from - nodeNum].subNodeID = to;

      nodes[aRequest.nodes[from - nodeNum].subNodeID].touched = true;
    }

    while ((brk = fscanf_s(fpOut, "%s", solStatus)) == 1) {
      if (solStatus[0] == 'x') {
        break;
      }
    }

    if (brk != 1) {
      break;
    }
  }

  fclose(fpOut);

  return NODE_MAP_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// Class      : VNRequest                                                    //
// Description:                                                              //
///////////////////////////////////////////////////////////////////////////////

VNRequest::VNRequest(string _fileName, int _reqID) :
  fileName(_fileName), reqID(_reqID) {
  if(DEBUG) {
    cout << "VNReq" << endl;
  }

  nodes.clear();
  edges.clear();

  initGraph();

  revenue = 0;
}

#ifdef MYCPP
VNRequest::VNRequest(const VNRequest &o) {
  if(DEBUG) {
    cout << "VNReq Copy" << endl;
  }

  nodeNum = o.nodeNum;
  edgeNum = o.edgeNum;
  reqID = o.reqID;
  fileName = o.fileName;
  revenue = o.revenue;

  split = o.split;
  time = o.time;
  duration = o.duration;
  topology = o.topology;
  maxD = o.maxD;

  //copy(o.nodes.begin(), o.nodes.end(), nodes.begin());
  for (int i = 0; i < o.nodeNum; i++)
    nodes.push_back(o.nodes[i]);
  //copy(o.edges.begin(), o.edges.end(), edges.begin());
  for (int i = 0; i < edgeNum; i++)
    edges.push_back(o.edges[i]);
}

VNRequest::~VNRequest() {
  if(DEBUG) {
    cout << "~VNReq" << endl;
  }
  nodes.clear();
  edges.clear();
}

const VNRequest& VNRequest::operator=(const VNRequest &o) {
  if(DEBUG) {
    cout << "VNReq=" << endl;
  }

  if (this != &o) {
    nodeNum = o.nodeNum;
    edgeNum = o.edgeNum;
    reqID = o.reqID;
    fileName = o.fileName;
    revenue = o.revenue;

    split = o.split;
    time = o.time;
    duration = o.duration;
    topology = o.topology;
    maxD = o.maxD;

    //copy(o.nodes.begin(), o.nodes.end(), nodes.begin());
    for (int i = 0; i < o.nodeNum; i++)
      nodes.push_back(o.nodes[i]);
    //copy(o.edges.begin(), o.edges.end(), edges.begin());
    for (int i = 0; i < edgeNum; i++)
      edges.push_back(o.edges[i]);
  }

  return *this;
}
#endif

int VNRequest::initGraph() {
  int i;

  int x, y;
  double cpu;

  int from, to;
  double bw, dlay;

  FILE *fp = fopen(fileName.c_str(), "rt");

  if (!fp) {
    cout << "failed to open file: " << fileName << endl;
    return COULD_NOT_OPEN_FILE;
  }

  fscanf_s(fp, "%d %d %d %d %d %d %d", &nodeNum, &edgeNum, &split, &time,
      &duration, &topology, &maxD);
  //changemaxD
  //maxD=10;
  // read nodes
  for (i = 0; i < nodeNum; i++) {
    fscanf_s(fp, "%d %d %lf", &x, &y, &cpu);
    nodes.push_back(VNNode(x, y, cpu));
    //printf("%d %d %lf\n", x, y, cpu);
  }

  // read edges
  for (i = 0; i < edgeNum; i++) {
    fscanf_s(fp, "%d %d %lf %lf", &from, &to, &bw, &dlay);
    edges.push_back(VNEdge(from, to, bw, dlay));
    //printf("%d %d %lf %lf\n", from, to, bw, dlay);

    // save edge information in nodes
    nodes[from].edgeIDs.push_back(i);
    nodes[to].edgeIDs.push_back(i);
	// save the edge map
    edgeMap[make_pair(from, to)] = i;
    edgeMap[make_pair(to, from)] = i;
  }

  fclose(fp);

  return SUCCESS;
}
//my defination : Resource fragmentation Degree.....................
//bool VNRequest::topology(){
//}
void VNRequest::getCost(double &nodecost,double &edgecost)
{
	int i,j;
	nodecost=0;
	edgecost=0;
	for(i=0;i<nodeNum;i++)
	{
		nodecost+=nodes[i].cpu;

	}
	for(i=0;i<edgeNum;i++)
	{
		for(j=0;j<edges[i].pathLen;j++)

			edgecost+=edges[i].subBW[j];

	}
}
int VNRequest::findmax_degree(){
	int i,j;
	size_t maxdegree=nodes[0].edgeIDs.size();
	double max_res=0;
	double cur_res;
	vector<int> Nb_Vnodes,Nb_Vedges;
	int nodeID=0;
	for(i=1;i<nodeNum;i++){
		cur_res=0;
		if(nodes[i].edgeIDs.size()>maxdegree){
			maxdegree=nodes[i].edgeIDs.size();
			nodeID=i;
		}
		findNb_Node(i,Nb_Vnodes,Nb_Vedges);
		for(j=0;j<Nb_Vedges.size();j++)
			cur_res+=edges[Nb_Vedges[j]].bw;
		cur_res+=nodes[i].cpu;
		if(nodes[i].edgeIDs.size()==maxdegree && cur_res>max_res){
			maxdegree=nodes[i].edgeIDs.size();
			max_res=cur_res;
			nodeID=i;
		}
		
	}
	return nodeID;
}
void VNRequest::findNb_Node(int nodeID,vector<int> &Nb_VNodes, vector<int> &Nb_Edges){
	int i;
	for(i=0;i<nodes[nodeID].edgeIDs.size();i++){
		Nb_Edges.push_back(nodes[nodeID].edgeIDs[i]);
		if(edges[nodes[nodeID].edgeIDs[i]].from!=nodeID)
			Nb_VNodes.push_back(edges[nodes[nodeID].edgeIDs[i]].from);
		else
			Nb_VNodes.push_back(edges[nodes[nodeID].edgeIDs[i]].to);
	}

}
double VNRequest::calculate_nbandwidth(int nodeID){
	int i;
	double nb_bw=0;
	for(i=0;i<nodes[nodeID].edgeIDs.size();i++)
		nb_bw+=edges[nodes[nodeID].edgeIDs[i]].bw;
	return nb_bw;
}
void VNRequest::breadth_searching(int nodeID, vector<int> &nodeProcessor){
	int front,rear,v;
	vector<int> Nb_nodes,Nb_edges;
	front=rear=-1;
    nodeProcessor.push_back(nodeID);
	nodes[nodeID].touched=true;
	++rear;
	 while(front!=rear)
	 {
		 v=nodeProcessor[++front];
		 Nb_nodes.clear();
		 findNb_Node(v,Nb_nodes,Nb_edges);
		 sortNodesDescending(Nb_nodes);
		 for(int i=0;i<Nb_nodes.size();i++)
	   {
		   if(nodes[Nb_nodes[i]].touched==false)
		   {
               nodes[Nb_nodes[i]].touched=true;
			   nodeProcessor.push_back(Nb_nodes[i]);
			   ++rear;
		   }
	   }
	 }
}
void VNRequest::depth_searching(int nodeID, vector<int> &nodeProcessor){
	int i;
	vector<int> Nb_nodes,Nb_edges;
	nodeProcessor.push_back(nodeID);
	nodes[nodeID].touched=true;
	findNb_Node(nodeID, Nb_nodes,Nb_edges);
	sortNodesDescending(Nb_nodes);
		  for(i=0;i<Nb_nodes.size();i++){    //遍历邻居节点
			  if(nodes[Nb_nodes[i]].touched==false)
				  depth_searching(Nb_nodes[i],nodeProcessor);
		  }  
}
void VNRequest::sortNodesAscending(vector<int> &nodeProcessOrder) {
  int i, j;

  for (i = 0; i < nodeNum; i++) {
    for (j = i + 1; j < nodeNum; j++) {
      if (nodes[nodeProcessOrder[i]].cpu > nodes[nodeProcessOrder[j]].cpu) {
        int t = nodeProcessOrder[i];
        nodeProcessOrder[i] = nodeProcessOrder[j];
        nodeProcessOrder[j] = t;
      }
    }
  }
}
void VNRequest::sortNodesDescending(vector<int> &nodeProcessOrder) {
  int i, j;

  for (i = 0; i < nodeProcessOrder.size(); i++) {
    for (j = i + 1; j < nodeProcessOrder.size(); j++) {
      if (nodes[nodeProcessOrder[i]].cpu < nodes[nodeProcessOrder[j]].cpu) {
        int t = nodeProcessOrder[i];
        nodeProcessOrder[i] = nodeProcessOrder[j];
        nodeProcessOrder[j] = t;
      }
    }
  }
}

void VNRequest::sortEdgesAscending(vector<int> &edgeProcessOrder) {
  int i, j;

  for (i = 0; i < edgeNum; i++) {
    for (j = i + 1; j < edgeNum; j++) {
      if (edges[edgeProcessOrder[i]].bw > edges[edgeProcessOrder[j]].bw) {
        int t = edgeProcessOrder[i];
        edgeProcessOrder[i] = edgeProcessOrder[j];
        edgeProcessOrder[j] = t;
      }
    }
  }
}

void VNRequest::sortEdgesDescending(vector<int> &edgeProcessOrder) {
  int i, j;

  for (i = 0; i < edgeNum; i++) {
    for (j = i + 1; j < edgeNum; j++) {
      if (edges[edgeProcessOrder[i]].bw < edges[edgeProcessOrder[j]].bw) {
        int t = edgeProcessOrder[i];
        edgeProcessOrder[i] = edgeProcessOrder[j];
        edgeProcessOrder[j] = t;
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Class      : SubstrateNode                                                //
// Description:                                                              //
///////////////////////////////////////////////////////////////////////////////

SubstrateNode::SubstrateNode(int _x, int _y, double _cpu) :
  Node(_x, _y, _cpu) {
  if(DEBUG) {
    cout << "SubNode" << endl;
  }

  count = 0;
  rest_cpu = 0;
  ratio=0;
  req_ids.clear();
  node_ids.clear();
  used_cpu.clear();
}

#ifdef MYCPP
SubstrateNode::SubstrateNode(const SubstrateNode &o) :
  Node(o), count(o.count), rest_cpu(o.rest_cpu) {
  if(DEBUG) {
    cout << "SubNode Copy" << endl;
  }

  edgeIDs.resize(o.edgeIDs.size());
  copy(o.edgeIDs.begin(), o.edgeIDs.end(), edgeIDs.begin());
  req_ids.resize(o.req_ids.size());
  copy(o.req_ids.begin(), o.req_ids.end(), req_ids.begin());
  node_ids.resize(o.node_ids.size());
  copy(o.node_ids.begin(), o.node_ids.end(), node_ids.begin());
  used_cpu.resize(o.used_cpu.size());
  copy(o.used_cpu.begin(), o.used_cpu.end(), used_cpu.begin());
}

SubstrateNode::~SubstrateNode() {
  if(DEBUG) {
    cout << "~SubNode" << endl;
  }
  edgeIDs.clear();
  req_ids.clear();
  node_ids.clear();
  used_cpu.clear();
}

const SubstrateNode& SubstrateNode::operator=(const SubstrateNode &o) {
  if(DEBUG) {
    cout << "SubNode=" << endl;
  }

  if (this != &o) {
    Node::operator=(o);

    count = o.count;
    rest_cpu = o.rest_cpu;

    edgeIDs.resize(o.edgeIDs.size());
    copy(o.edgeIDs.begin(), o.edgeIDs.end(), edgeIDs.begin());
    req_ids.resize(o.req_ids.size());
    copy(o.req_ids.begin(), o.req_ids.end(), req_ids.begin());
    node_ids.resize(o.node_ids.size());
    copy(o.node_ids.begin(), o.node_ids.end(), node_ids.begin());
    used_cpu.resize(o.used_cpu.size());
    copy(o.used_cpu.begin(), o.used_cpu.end(), used_cpu.begin());
  }

  return *this;
}
#endif
/*
double SubstrateNode::distanceFrom(Node &aNode) {
  return sqrt((x - aNode.x) * (x - aNode.x) +
      (y - aNode.y) * (y - aNode.y));
}

double SubstrateNode::distanceFrom(SubstrateNode &aNode) {
  return sqrt((x - aNode.x) * (x - aNode.x) +
      (y - aNode.y) * (y - aNode.y));
}
*/
//wu
double SubstrateNode::distanceFrom(Node &aNode) {
  return sqrt( double((x - aNode.x) * (x - aNode.x) +
      (y - aNode.y) * (y - aNode.y)) );
}

double SubstrateNode::distanceFrom(SubstrateNode &aNode) {
  return sqrt( double((x - aNode.x) * (x - aNode.x) +
      (y - aNode.y) * (y - aNode.y)));
}
///////////////////////////////////////////////////////////////////////////////
// Class      : SubstrateEdge                                                //
// Description:                                                              //
///////////////////////////////////////////////////////////////////////////////

SubstrateEdge::SubstrateEdge(int _from, int _to, double _bw, double _dlay) :
  Edge(_from, _to, _bw, _dlay) {
  if(DEBUG) {
    cout << "SubEdge" << endl;
  }
  count = 0;
  rest_bw = 0;
  ratio=0;
  req_ids.clear();
  edge_ids.clear();
  used_bw.clear();
}

#ifdef MYCPP
SubstrateEdge::SubstrateEdge(const SubstrateEdge &o) :
  Edge(o), count(o.count), rest_bw(o.rest_bw) {
  if(DEBUG) {
    cout << "SubEdge Copy" << endl;
  }

  req_ids.resize(o.req_ids.size());
  copy(o.req_ids.begin(), o.req_ids.end(), req_ids.begin());
  edge_ids.resize(o.edge_ids.size());
  copy(o.edge_ids.begin(), o.edge_ids.end(), edge_ids.begin());
  used_bw.resize(o.used_bw.size());
  copy(o.used_bw.begin(), o.used_bw.end(), used_bw.begin());
}

SubstrateEdge::~SubstrateEdge() {
  if(DEBUG) {
    cout << "~SubEdge" << endl;
  }
  req_ids.clear();
  edge_ids.clear();
  used_bw.clear();
}

const SubstrateEdge& SubstrateEdge::operator=(const SubstrateEdge &o) {
  if(DEBUG) {
    cout << "SubEdge=" << endl;
  }
  if (this != &o) {
    Edge::operator=(o);

    count = o.count;
    rest_bw = o.rest_bw;

    req_ids.resize(o.req_ids.size());
    copy(o.req_ids.begin(), o.req_ids.end(), req_ids.begin());
    edge_ids.resize(o.edge_ids.size());
    copy(o.edge_ids.begin(), o.edge_ids.end(), edge_ids.begin());
    used_bw.resize(o.used_bw.size());
    copy(o.used_bw.begin(), o.used_bw.end(), used_bw.begin());
  }

  return *this;
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Class      : VNNode                                                       //
// Description:                                                              //
///////////////////////////////////////////////////////////////////////////////

VNNode::VNNode(int _x, int _y, double _cpu) :
  Node(_x, _y, _cpu) {
  if(DEBUG) {
    cout << "VNNode" << endl;
  }

  substrateID = NOT_MAPPED_YET;
  subNodeID = NOT_MAPPED_YET;
}

#ifdef MYCPP
VNNode::VNNode(const VNNode &o) :
  Node(o), substrateID(o.substrateID), subNodeID(o.subNodeID) {
  if(DEBUG) {
    cout << "VNNode Copy" << endl;
  }

  edgeIDs.resize(o.edgeIDs.size());
  copy(o.edgeIDs.begin(), o.edgeIDs.end(), edgeIDs.begin());
}

VNNode::~VNNode() {
  if(DEBUG) {
    cout << "~VNNode" << endl;
  }

  edgeIDs.clear();
}

const VNNode& VNNode::operator=(const VNNode &o) {
  if(DEBUG) {
    cout << "VNNode=" << endl;
  }

  if (this != &o) {
    Node::operator=(o);

    substrateID = o.substrateID;
    subNodeID = o.subNodeID;

    edgeIDs.resize(o.edgeIDs.size());
    copy(o.edgeIDs.begin(), o.edgeIDs.end(), edgeIDs.begin());
  }

  return *this;
}
#endif

///////////////////////////////////////////////////////////////////////////////
// Class      : VNEdge                                                       //
// Description:                                                              //
///////////////////////////////////////////////////////////////////////////////

VNEdge::VNEdge(int _from, int _to, double _bw, double _dlay) :
  Edge(_from, _to, _bw, _dlay) {
  if(DEBUG) {
    cout << "VNEdge" << endl;
  }

  pathLen = 0;
  pathDelay = NOT_MAPPED_YET;

  substrateID = NOT_MAPPED_YET;
  subPath.clear();
  subBW.clear();
}

#ifdef MYCPP
VNEdge::VNEdge(const VNEdge &o) :
  Edge(o), substrateID(o.substrateID), pathLen(o.pathLen),
  pathDelay(o.pathDelay) {
  if(DEBUG) {
    cout << "VNEdge Copy" << endl;
  }

  subPath.resize(o.subPath.size());
  copy(o.subPath.begin(), o.subPath.end(), subPath.begin());
  subBW.resize(o.subBW.size());
  copy(o.subBW.begin(), o.subBW.end(), subBW.begin());
}

VNEdge::~VNEdge() {
  if(DEBUG) {
    cout << "~VNEdge" << endl;
  }

  subPath.clear();
  subBW.clear();
}

const VNEdge& VNEdge::operator=(const VNEdge &o) {
  if(DEBUG) {
    cout << "VNEdge=" << endl;
  }

  if (this != &o) {
    Edge::operator=(o);

    substrateID = o.substrateID;
    pathLen = o.pathLen;
    pathDelay = o.pathDelay;

    subPath.resize(o.subPath.size());
    copy(o.subPath.begin(), o.subPath.end(), subPath.begin());
    subBW.resize(o.subBW.size());
    copy(o.subBW.begin(), o.subBW.end(), subBW.begin());
  }

  return *this;
}
#endif
