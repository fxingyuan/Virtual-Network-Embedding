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
vector<VNRequest> VNR;                  //�����������
int reqCount;							//������������
char reqFolderName[LEN_FILENAME];       //�������������ļ���  �������·��
char outputFileName[LEN_FILENAME];		//��������ӳ��ͳ���ļ�
Simulator mySim;						//�¼�������

int main(int argc, char *argv[]) { //���룺 VN��������  VNs�����ļ��� ͳ������ļ� �ڵ�ӳ�� ��·ӳ�� ����������࿪�� ���ؾ��⿪�� cpu&bw�������� �����ÿ���
  int i;
  int curtime=0,reconfig_time=1;	//�����ñ�־���� 
  char reqFileName[LEN_FILENAME];	//�����ļ���
  bool aOne = false, bOne = false;	//���ؾ��⿪��
  double __MULT = 1.0;				//cpu&bw��������
  bool fullOrPartialSplit,Reconfig=true;			//�Ƿ�֧���������������еķ���
  int nodeMapMethod, edgeMapMethod;	//�ڵ�&��·ӳ�䷽����Note��8,9,10Ϊ�ڵ����·����ӳ�䣬��ʱedgeMapMethod������Ч��
  double mNS, aNS, mLS, aLS, sdNS, sdLS;	//����״̬ͳ��ֵ
  double nodeRev, edgeRev, totRev, nodeCost, edgeCost, totCost;	//��������ӳ����������� ͳ��ֵ
  
  if (argc != 10 && argc != 11) {
    cerr << "usage: VNE <sN> <rC> <rD> <oF> <nM> <eM> <fS> <lB> <MT>" << endl;
	cerr << "<sN>: Substrate network" << endl;
    cerr << "<rC>: total number of requests" << endl;
    cerr << "<rD>: directory containing the requests " << endl;
    cerr << "<oF>: output file to dump the results" << endl;
	cerr << "<nM>: node mapping method (1: GREEDY. 5: D-ViNE 6: R-ViNE 8:VNE-RFD-D 9:VNE-RFD-B 10��VNE-TA(VNE-GRFD & VNR-GRFD))" << endl;
    cerr << "<eM>: edge mapping method (1: GREEDY. 5: MCF)" << endl;
    cerr << "<fS>: whether to ignore (1) or respect(0) VNR's splitting choice [eM must be 5]" << endl;
    cerr << "<lB>: load balancing (1: alpha = beta = 1. 0: alpha = beta = residue)" << endl;
    cerr << "<MT>: multiplier in the revenue and cost functions" << endl;
	cerr << "<Re>: Reconfiguration ON or OFF (Optional)" << endl;
    exit(1);
  }
  //���������ȡ...................................................
	SubstrateGraph SG(argv[1], 0); //����ײ������������ˣ�   �ڵ�����100 ������λ�� 25*25
	reqCount = atoi(argv[2]); // total number of requests
	strcpy(reqFolderName, argv[3]); // folder containing the requests
	strcpy(outputFileName, argv[4]);  // where to save output
	nodeMapMethod = atoi(argv[5]);  // 1: GREEDY. 5: D-ViNE 6: R-ViNE 8:VNE-RFD-D 9:VNE-RFD-B 10��VNE-TA(VNE-GRFD & VNR-GRFD))
	edgeMapMethod = atoi(argv[6]);  // 1: GREEDY. 5: MCF
	fullOrPartialSplit = atoi(argv[7]); // true: ignore VNR's choice
	aOne = bOne = atoi(argv[8]);  // true: try to load balance
	__MULT = atof(argv[9]); // cpu vs bw weighting value
	if(argc==11) Reconfig = atoi(argv[10]);	//Reconfiguration ON or OFF

  //���������ȡ...................................................
 
  //����ͳ���ļ�
  FILE *outputFP = fopen(outputFileName, "w");
  if (outputFP == NULL) {
    cout << "failed to open file: " << outputFileName << endl;
    return COULD_NOT_OPEN_FILE;
  }

  for (i = 1; i < argc; i++) {
    fprintf(outputFP, "%s ", argv[i]);
  }

  fprintf_s(outputFP, "\n");

  //����ϵͳ��ʼ�����������������¼�
  vector<VNRequest>::iterator VNRIter;  //�������������
  int curVNR, mappingResult;			//��ǰ���������� ӳ����
  bool requestAccepted = false;			//���������Ƿ񱻽���

  int nodeMapFailCount = 0, edgeMapFailCount = 0, totalMapFailCount = 0;//�ڵ� ��·ӳ��ʧ��ͳ��

  srand((unsigned) time(NULL)); // ��ʼ�������

  // ����������
  for (i = 0; i < reqCount; i++) {
	cout<<i<<endl;
	sprintf_s(reqFileName, "%s/req%d.txt", reqFolderName, i);
    // ����VN request
    VNR.push_back(VNRequest(reqFileName, i));

    // ����VN �����¼�
    mySim.PQ.push(Event(EVENT_ARRIVE, VNR[i].time, i));
	//cout.flush();
	cout<<i+1<<endl;
  }

  // VN ӳ�����.......................................
  while (!mySim.empty()) {
    const Event &curEvent = mySim.top();	//Pop�¼�
    curVNR = curEvent.index;				//�¼���ţ�����������
    requestAccepted = false;

    cout << curVNR << " " << curEvent.time << " " << curEvent.type << endl;

    if (curEvent.type == EVENT_ARRIVE) { // �������󵽴�
	//�ڵ�ӳ��
      switch(nodeMapMethod) {	
      case NM_GREEDY_WORST_FIT:
        mappingResult = SG.mapNodes(VNR[curVNR], VNODE_ORDER_ASC, NM_GREEDY_WORST_FIT);
        break;
      case NM_DETERMINISTIC:
        mappingResult = SG.mapNodes_ViNE(VNR[curVNR], NM_DETERMINISTIC, aOne, bOne);
        //mappingResult = SG.mapNodes_ViNE_v2(VNR[curVNR], NM_DETERMINISTIC, aOne, bOne); //�汾2
        break;
      case NM_RANDOMIZED:
        mappingResult = SG.mapNodes_ViNE(VNR[curVNR], NM_RANDOMIZED, aOne, bOne);
        //mappingResult = SG.mapNodes_ViNE_v2(VNR[curVNR], NM_RANDOMIZED, aOne, bOne); //�汾2
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
	  //��·ӳ��
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

      // Ϊ�������������  �����뿪�¼�
      mySim.PQ.push(Event(EVENT_DEPART, VNR[curVNR].time + VNR[curVNR].duration,
          curVNR));
    }
    else if (curEvent.type == EVENT_DEPART) { 
      SG.removeVNMapping(VNR[curVNR]);	//���������뿪
    }
    else {
		/*��������������¼������������¼�����������仯�¼�������ڵ����·���뿪�ȵ�*/
    }

LABEL_MAP_FAILED:
	//��ǰ��������ӳ��ͳ����Ϣ���������ͳ���ļ���
	SG.rank();		//�ײ���������ڵ����·����Ƭ��rank����
    totRev = getRevenue(VNR[curVNR], __MULT, nodeRev, edgeRev);	
    totCost = getCost(VNR[curVNR], SG, __MULT, nodeCost, edgeCost, aOne, bOne);
    getDifferentStress(SG, mNS, aNS, mLS, aLS, sdNS, sdLS);
	double mean2_Nrfd=0,mean2_Lrfd=0,mean_Nrfd=0,std_Nrfd=0,mean_Lrfd=0,std_Lrfd=0;
	for(i=0;i<SG.nodeNum;i++){
		//SG.nodes[i].rfd=SG.Node_rfd(i);	//�ֲ���Ƭ��
		mean2_Nrfd+=SG.nodes[i].pagerfd*SG.nodes[i].pagerfd;
		mean_Nrfd+=SG.nodes[i].pagerfd;
	}
	for(i=0;i<SG.edgeNum;i++){
		//SG.edges[i].rfd=SG.Edge_rfd(i);	//�ֲ���Ƭ��
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
    mySim.pop();	//�¼�����������뿪����


	//��������ӳ�䷽��ΪMY_NEW_NODE_MAPPING��10������������������û���
	if(nodeMapMethod==MY_NEW_NODE_MAPPING && Reconfig){
		vector<int> re_REQ;
		vector<int>::iterator iter;
		re_REQ.clear();
		const Event &reEvent = mySim.top();
		if(reconfig_time*RECONFIG_T<reEvent.time){
			SG.reconfiguration(VNR,reconfig_time*RECONFIG_T,re_REQ); //�����û��ƣ����������õ�����������re_REQ
			//�����ú� ͳ�ƽ��
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
	//�����������ӳ��ʧ�� ͳ����Ϣ
  cout << "Node Mapping Failed: " << nodeMapFailCount << endl;
  cout << "Edge Mapping Failed: " << edgeMapFailCount << endl;
  cout << "Mapping Failed: " << totalMapFailCount << endl;

  fclose(outputFP);

  return 0;
}
