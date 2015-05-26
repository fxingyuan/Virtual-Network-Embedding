#pragma once
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "def.h"
using namespace std;
class curEvent
{
public:
	int index,time,duration,type, requestAccepted;
	double nodeRev, edgeRev, totRev, nodeCost, edgeCost, totCost, mNS, aNS, mLS, aLS, sdNS, sdLS, mean_Nrfd, std_Nrfd, mean_Lrfd, std_Lrfd;
};
class curFile:public curEvent{
public:
  string fileName;
  int fileID,acceptNum,rejectNum,time_from,time_to;
  double nodeLos,edgeLos,totLos, average_Rev,average_Cos,average_ratio_R2C;
  vector<curEvent> reqEvent;
  curFile(string _fileName, int _fileID = -1);
  void initFile();
  void member_clear();
  void calcu_param(int time_from, int time_to);
  int outputfile(char outputFileName[LEN_FILENAME]);
};



