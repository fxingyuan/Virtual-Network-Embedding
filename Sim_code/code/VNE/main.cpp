//============================================================================
// Name        : MySimulation
// Author      : LiXing(Framework derived from Mosharaf K Chowdhury)
// Version     : Final
// Copyright   : No
// Description : Virtual Network Embedding and Reconfigration
//============================================================================

#include "utility.h"
#include "def.h"
#include "simulator.h"
#include <time.h>
using namespace std;

//SubstrateGraph SG;
vector<VNRequest> VNR;                  //虚拟请求队列
int reqCount;							//虚拟请求数量
char reqFolderName[LEN_FILENAME];       //虚拟请求所在文件夹  可以相对路径
char outputFileName[LEN_FILENAME];		//虚拟网络映射统计文件
Simulator mySim;						//事件发生器

int main(int argc, char *argv[]) { //输入： VN请求数量  VNs所在文件夹 统计输出文件 节点映射 链路映射 虚拟请求分类开关 负载均衡开关 cpu&bw比例因子 重配置开关
  int i;
  int curtime=0,reconfig_time=1;	//重配置标志参数 
  char reqFileName[LEN_FILENAME];	//请求文件名
  bool aOne = false, bOne = false;	//负载均衡开关
  double __MULT = 1.0;				//cpu&bw比例因子
  bool fullOrPartialSplit,Reconfig=true;			//是否支持虚拟网络请求中的分裂
  int nodeMapMethod, edgeMapMethod;	//节点&链路映射方法（Note：8,9,10为节点和链路交替映射，此时edgeMapMethod设置无效）
  double mNS, aNS, mLS, aLS, sdNS, sdLS;	//负载状态统计值
  double nodeRev, edgeRev, totRev, nodeCost, edgeCost, totCost;	//虚拟网络映射收益与代价 统计值
  
  if (argc != 10 && argc != 11) {
    cerr << "usage: VNE <sN> <rC> <rD> <oF> <nM> <eM> <fS> <lB> <MT>" << endl;
	cerr << "<sN>: Substrate network" << endl;
    cerr << "<rC>: total number of requests" << endl;
    cerr << "<rD>: directory containing the requests " << endl;
    cerr << "<oF>: output file to dump the results" << endl;
	cerr << "<nM>: node mapping method (1: GREEDY. 5: D-ViNE 6: R-ViNE 8:VNE-RFD-D 9:VNE-RFD-B 10：VNE-TA(VNE-GRFD & VNR-GRFD))" << endl;
    cerr << "<eM>: edge mapping method (1: GREEDY. 5: MCF)" << endl;
    cerr << "<fS>: whether to ignore (1) or respect(0) VNR's splitting choice [eM must be 5]" << endl;
    cerr << "<lB>: load balancing (1: alpha = beta = 1. 0: alpha = beta = residue)" << endl;
    cerr << "<MT>: multiplier in the revenue and cost functions" << endl;
	cerr << "<Re>: Reconfiguration ON or OFF (Optional)" << endl;
    exit(1);
  }
  //输入参数读取...................................................
	SubstrateGraph SG(argv[1], 0); //输入底层物理网络拓扑，   节点数量100 ，地理位置 25*25
	reqCount = atoi(argv[2]); // total number of requests
	strcpy(reqFolderName, argv[3]); // folder containing the requests
	strcpy(outputFileName, argv[4]);  // where to save output
	nodeMapMethod = atoi(argv[5]);  // 1: GREEDY. 5: D-ViNE 6: R-ViNE 8:VNE-RFD-D 9:VNE-RFD-B 10：VNE-TA(VNE-GRFD & VNR-GRFD))
	edgeMapMethod = atoi(argv[6]);  // 1: GREEDY. 5: MCF
	fullOrPartialSplit = atoi(argv[7]); // true: ignore VNR's choice
	aOne = bOne = atoi(argv[8]);  // true: try to load balance
	__MULT = atof(argv[9]); // cpu vs bw weighting value
	if(argc==11) Reconfig = atoi(argv[10]);	//Reconfiguration ON or OFF

  //输入参数读取...................................................
 
  //创建统计文件
  FILE *outputFP = fopen(outputFileName, "w");
  if (outputFP == NULL) {
    cout << "failed to open file: " << outputFileName << endl;
    return COULD_NOT_OPEN_FILE;
  }

  for (i = 1; i < argc; i++) {
    fprintf(outputFP, "%s ", argv[i]);
  }

  fprintf_s(outputFP, "\n");

  //仿真系统初始化：构建虚拟请求事件
  vector<VNRequest>::iterator VNRIter;  //虚拟请求迭代器
  int curVNR, mappingResult;			//当前虚拟请求编号 映射结果
  bool requestAccepted = false;			//虚拟请求是否被接受

  int nodeMapFailCount = 0, edgeMapFailCount = 0, totalMapFailCount = 0;//节点 链路映射失败统计

  srand((unsigned) time(NULL)); // 初始化随机器

  // 虚拟请求处理
  for (i = 0; i < reqCount; i++) {
	cout<<i<<endl;
	sprintf_s(reqFileName, "%s/req%d.txt", reqFolderName, i);
    // 保存VN request
    VNR.push_back(VNRequest(reqFileName, i));

    // 创建VN 到达事件
    mySim.PQ.push(Event(EVENT_ARRIVE, VNR[i].time, i));
	//cout.flush();
	cout<<i+1<<endl;
  }

  // VN 映射过程.......................................
  while (!mySim.empty()) {
    const Event &curEvent = mySim.top();	//Pop事件
    curVNR = curEvent.index;				//事件编号，虚拟请求编号
    requestAccepted = false;

    cout << curVNR << " " << curEvent.time << " " << curEvent.type << endl;

    if (curEvent.type == EVENT_ARRIVE) { // 虚拟请求到达
	//节点映射
      switch(nodeMapMethod) {	
      case NM_GREEDY_WORST_FIT:
        mappingResult = SG.mapNodes(VNR[curVNR], VNODE_ORDER_ASC, NM_GREEDY_WORST_FIT);
        break;
      case NM_DETERMINISTIC:
        mappingResult = SG.mapNodes_ViNE(VNR[curVNR], NM_DETERMINISTIC, aOne, bOne);
        //mappingResult = SG.mapNodes_ViNE_v2(VNR[curVNR], NM_DETERMINISTIC, aOne, bOne); //版本2
        break;
      case NM_RANDOMIZED:
        mappingResult = SG.mapNodes_ViNE(VNR[curVNR], NM_RANDOMIZED, aOne, bOne);
        //mappingResult = SG.mapNodes_ViNE_v2(VNR[curVNR], NM_RANDOMIZED, aOne, bOne); //版本2
        break;
	  case MY_NODE_MAPPING_DEPTH:
	
		  mappingResult =SG.my_nodemap(VNR[curVNR], MY_NODE_MAPPING_DEPTH);
		  if (fullOrPartialSplit || VNR[curVNR].split) break;
		  if(mappingResult==NODE_MAP_SUCCESS) goto LABEL_ADD_MAP;
		  break;
	  case MY_NODE_MAPPING_BREADTH:///////
		  
		  mappingResult =SG.my_nodemap(VNR[curVNR], MY_NODE_MAPPING_BREADTH);
		  if (fullOrPartialSplit || VNR[curVNR].split) break;
		  if(mappingResult==NODE_MAP_SUCCESS) goto LABEL_ADD_MAP;
		  break;
	case MY_NEW_NODE_MAPPING:
	
		SG.rank();
		mappingResult =SG.my_new_nodemap(VNR[curVNR]);
	if (fullOrPartialSplit || VNR[curVNR].split) break;
	if(mappingResult==NODE_MAP_SUCCESS) goto LABEL_ADD_MAP;
	break;
      default:
        cerr << "invalid node mapping method" << endl;
        exit(1);
        break;
      }

      // node mapping failed, don't admit
      if (mappingResult != NODE_MAP_SUCCESS) {
        nodeMapFailCount++;
        totalMapFailCount++;
        goto LABEL_MAP_FAILED;
      }
	  //链路映射
      if (edgeMapMethod == EM_GREEDY_WORST_FIT) {
        mappingResult = SG.mapEdges(VNR[curVNR], VEDGE_ORDER_ASC, EM_GREEDY_WORST_FIT);
      }
      else if (edgeMapMethod == EM_MCF) {
        if (fullOrPartialSplit || VNR[curVNR].split) {
          mappingResult = SG.mapEdges_ViNE(VNR[curVNR], aOne, bOne);
        }
        else {
          mappingResult=SG.mapEdges(VNR[curVNR], VEDGE_ORDER_ASC, EM_GREEDY_WORST_FIT);
        }
      }
      else {
        cerr << "invalid edge mapping method" << endl;
        exit(1);
      }

      // edge mapping failed, don't admit
      if (mappingResult != EDGE_MAP_SUCCESS) {
        edgeMapFailCount++;
        totalMapFailCount++;
        goto LABEL_MAP_FAILED;
      }
	  LABEL_ADD_MAP:
      requestAccepted = true;
      SG.addVNMapping(VNR[curVNR]);

      //printMapping(VNR[curVNR], SG);

      // 为加入的虚拟网络  创建离开事件
      mySim.PQ.push(Event(EVENT_DEPART, VNR[curVNR].time + VNR[curVNR].duration,
          curVNR));
    }
    else if (curEvent.type == EVENT_DEPART) { 
      SG.removeVNMapping(VNR[curVNR]);	//虚拟请求离开
    }
    else {
		/*可以添加其他的事件，如重配置事件，虚拟请求变化事件，虚拟节点和链路的离开等等*/
    }

LABEL_MAP_FAILED:
	//当前虚拟网络映射统计信息，并输出到统计文件中
	SG.rank();		//底层物理网络节点和链路的碎片度rank计算
    totRev = getRevenue(VNR[curVNR], __MULT, nodeRev, edgeRev);	
    totCost = getCost(VNR[curVNR], SG, __MULT, nodeCost, edgeCost, aOne, bOne);
    getDifferentStress(SG, mNS, aNS, mLS, aLS, sdNS, sdLS);
	double mean2_Nrfd=0,mean2_Lrfd=0,mean_Nrfd=0,std_Nrfd=0,mean_Lrfd=0,std_Lrfd=0;
	for(i=0;i<SG.nodeNum;i++){
		//SG.nodes[i].rfd=SG.Node_rfd(i);	//局部碎片度
		mean2_Nrfd+=SG.nodes[i].pagerfd*SG.nodes[i].pagerfd;
		mean_Nrfd+=SG.nodes[i].pagerfd;
	}
	for(i=0;i<SG.edgeNum;i++){
		//SG.edges[i].rfd=SG.Edge_rfd(i);	//局部碎片度
		mean_Lrfd+=SG.edges[i].pagerfd;
		mean2_Lrfd+=SG.edges[i].pagerfd*SG.edges[i].pagerfd;
	}
	std_Nrfd=sqrt(fabs(mean2_Nrfd-mean_Nrfd)/SG.nodeNum);
	std_Lrfd=sqrt(fabs(mean2_Lrfd-mean_Lrfd)/SG.edgeNum);
	mean_Nrfd=mean_Nrfd/SG.nodeNum;
	mean_Lrfd=mean_Lrfd/SG.edgeNum;
    fprintf(outputFP, "%4d %6d %6d %d %d %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf\n",
        curEvent.index, curEvent.time, VNR[curEvent.index].duration, curEvent.type, requestAccepted, nodeRev, edgeRev, totRev, nodeCost, edgeCost, totCost, mNS, aNS, mLS, aLS, sdNS, sdLS, mean_Nrfd, std_Nrfd, mean_Lrfd, std_Lrfd);
    // pop the request
    mySim.pop();	//事件处理结束，离开队列


	//虚拟网络映射方法为MY_NEW_NODE_MAPPING（10），添加周期性重配置机制
	if(nodeMapMethod==MY_NEW_NODE_MAPPING && Reconfig){
		vector<int> re_REQ;
		vector<int>::iterator iter;
		re_REQ.clear();
		const Event &reEvent = mySim.top();
		if(reconfig_time*RECONFIG_T<reEvent.time){
			SG.reconfiguration(VNR,reconfig_time*RECONFIG_T,re_REQ); //重配置机制，返回重配置的虚拟网络编号re_REQ
			//重配置后 统计结果
			SG.rank();
			for(iter=re_REQ.begin();iter!=re_REQ.end();iter++){
			int CurID=*iter;
			totRev = getRevenue(VNR[CurID], __MULT, nodeRev, edgeRev);
			totCost = getCost(VNR[CurID], SG, __MULT, nodeCost, edgeCost, aOne, bOne);
			getDifferentStress(SG, mNS, aNS, mLS, aLS, sdNS, sdLS);
			mean2_Nrfd=0,mean2_Lrfd=0,mean_Nrfd=0,std_Nrfd=0,mean_Lrfd=0,std_Lrfd=0;
			for(i=0;i<SG.nodeNum;i++){
				//SG.nodes[i].rfd=SG.Node_rfd(i);
				mean2_Nrfd+=SG.nodes[i].pagerfd*SG.nodes[i].pagerfd;
				mean_Nrfd+=SG.nodes[i].pagerfd;
			}
			for(i=0;i<SG.edgeNum;i++){
				//SG.edges[i].rfd=SG.Edge_rfd(i);
				mean_Lrfd+=SG.edges[i].pagerfd;
				mean2_Lrfd+=SG.edges[i].pagerfd*SG.edges[i].pagerfd;
			}
			std_Nrfd=sqrt(fabs(mean2_Nrfd-mean_Nrfd)/SG.nodeNum);
			std_Lrfd=sqrt(fabs(mean2_Lrfd-mean_Lrfd)/SG.edgeNum);
			mean_Nrfd=mean_Nrfd/SG.nodeNum;
			mean_Lrfd=mean_Lrfd/SG.edgeNum;
			fprintf(outputFP, "%4d %6d %6d %d %d %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf\n",
				CurID, reconfig_time*RECONFIG_T, VNR[CurID].duration-(reconfig_time*RECONFIG_T-VNR[CurID].time), 2, re_REQ.size(), nodeRev, edgeRev, totRev, nodeCost, edgeCost, totCost, mNS, aNS, mLS, aLS, sdNS, sdLS, mean_Nrfd, std_Nrfd, mean_Lrfd, std_Lrfd);
		}
			reconfig_time++;
		}
	}
	//*/
}
	//输出虚拟网络映射失败 统计信息
  cout << "Node Mapping Failed: " << nodeMapFailCount << endl;
  cout << "Edge Mapping Failed: " << edgeMapFailCount << endl;
  cout << "Mapping Failed: " << totalMapFailCount << endl;

  fclose(outputFP);

  return 0;
}
