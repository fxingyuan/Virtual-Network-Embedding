#include "In_Event.h"
int getNextLinePos(FILE *p)
{
	int ch;	
	int curpos = 0;

	ch=getc(p);
	curpos = ftell(p);
	while(ch!=EOF)
	{
		putchar(ch);
		ch=getc(p);
		if(ch == '\n')
		{
			break;
		}
	}

	return (ftell(p) - curpos + 1);
}
curFile::curFile(string _fileName, int _fileID) :
  fileName(_fileName), fileID(_fileID) {
  reqEvent.clear();
  initFile();
}
  void curFile::initFile() {
  //int i;
  FILE *fp = fopen(fileName.c_str(), "rt");
  if (!fp) {
	  cout << "failed to open file: "<< fileName.c_str() << endl;
      exit(-1);
  }
  curEvent cur_event;
  fseek(fp, getNextLinePos(fp), SEEK_SET); // 文件定位到下一行
  while(1)
  {
	  if(EOF==fscanf(fp, "%d %d %d %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
	  &cur_event.index, &cur_event.time,&cur_event.duration,&cur_event.type,&cur_event.requestAccepted,
	  &cur_event.nodeRev,&cur_event.edgeRev,&cur_event.totRev,
	  &cur_event.nodeCost,&cur_event.edgeCost,&cur_event.totCost,
	  &cur_event.mNS,&cur_event.aNS,&cur_event.mLS,&cur_event.aLS,&cur_event.sdNS,&cur_event.sdLS,&cur_event.mean_Nrfd,&cur_event.std_Nrfd,&cur_event.mean_Lrfd,&cur_event.std_Lrfd))
	  break;
      reqEvent.push_back(cur_event);
  }
 

  fclose(fp);
  }
void curFile::member_clear(){
	nodeRev=0;
	nodeCost=0;
	edgeRev=0;
	edgeCost=0;
	totRev=0;
	totCost=0;
	acceptNum=0;
	nodeLos=0;
	edgeLos=0;
	totLos=0;
	average_Rev=0;
	average_Cos=0;
	average_ratio_R2C=0;
	rejectNum=0;
	mNS=aNS=mLS=aLS=sdNS=sdLS=0;
	mean_Nrfd=0;
	std_Nrfd=0;
	mean_Lrfd=0;
	std_Lrfd=0;
  }
  void curFile::calcu_param(int from, int to){
	  int i;
	  time_from=from;
	  time_to=to;
	  //int interval_time;
	  //初始化
	  bool flag=false;
	  int interval_time;
	  member_clear();
	  for(i=0;i<reqEvent.size();i++){
		  if(time_to<=reqEvent[i].time||reqEvent[i].time<=time_from) 
		  {
			  if(flag) break; 
			  continue;
		  }
		  flag=true;
		  if(i<reqEvent.size()-1) interval_time=reqEvent[i+1].time-reqEvent[i].time;
		  else
			  interval_time=20;
		  switch(reqEvent[i].type)
		  {
			    case ARRIVE:
					{
					  switch(reqEvent[i].requestAccepted)
					  {
						  case ACCEPT:
							  {
							  nodeRev+=reqEvent[i].nodeRev;
							  nodeCost+=reqEvent[i].nodeCost;
							  edgeRev+=reqEvent[i].edgeRev;
							  edgeCost+=reqEvent[i].edgeCost;
							  totRev+=reqEvent[i].totRev;
							  totCost+=reqEvent[i].totCost;
							  average_Rev+=reqEvent[i].totRev;
							  average_Cos+=reqEvent[i].totCost;
							  acceptNum++;
							  //rfd NS
								mNS+=reqEvent[i].mNS;
								aNS+=reqEvent[i].aNS;
								mLS+=reqEvent[i].mLS;
								aLS+=reqEvent[i].aLS;
								sdNS+=reqEvent[i].sdNS;
								sdLS+=reqEvent[i].sdLS;
								mean_Nrfd+=reqEvent[i].mean_Nrfd;
								std_Nrfd+=reqEvent[i].std_Nrfd;
								mean_Lrfd+=reqEvent[i].mean_Lrfd;
								std_Lrfd+=reqEvent[i].std_Lrfd;
							  //....
							  break;
							  }
						  case REJECT://损失
							  {
							  nodeLos+=reqEvent[i].nodeRev;
							  edgeLos+=reqEvent[i].edgeRev;
							  totLos+=reqEvent[i].totRev;
							  rejectNum++;

							  //rfd NS
								mNS+=reqEvent[i].mNS;
								aNS+=reqEvent[i].aNS;
								mLS+=reqEvent[i].mLS;
								aLS+=reqEvent[i].aLS;
								sdNS+=reqEvent[i].sdNS;
								sdLS+=reqEvent[i].sdLS;
								mean_Nrfd+=reqEvent[i].mean_Nrfd;
								std_Nrfd+=reqEvent[i].std_Nrfd;
								mean_Lrfd+=reqEvent[i].mean_Lrfd;
								std_Lrfd+=reqEvent[i].std_Lrfd;
							  //....
							  break;
							  }
						  default:
							  continue;
					  }
					  break;
					}
				  case DEPART://节点压力
					  {
						  if(reqEvent[i].time-reqEvent[i].duration>time_from)
						  {
								nodeRev-=reqEvent[i].nodeRev;
								nodeCost-=reqEvent[i].nodeCost;
								edgeRev-=reqEvent[i].edgeRev;
								edgeCost-=reqEvent[i].edgeCost;
								totRev-=reqEvent[i].totRev;
								totCost-=reqEvent[i].totCost;

								//rfd NS
								mNS+=reqEvent[i].mNS;
								aNS+=reqEvent[i].aNS;
								mLS+=reqEvent[i].mLS;
								aLS+=reqEvent[i].aLS;
								sdNS+=reqEvent[i].sdNS;
								sdLS+=reqEvent[i].sdLS;
								mean_Nrfd+=reqEvent[i].mean_Nrfd;
								std_Nrfd+=reqEvent[i].std_Nrfd;
								mean_Lrfd+=reqEvent[i].mean_Lrfd;
								std_Lrfd+=reqEvent[i].std_Lrfd;
							  //....
						  }
						  break;
					  }
				  default:
					  cout<<"reconfiguration"<<endl;
		  }
		  /*
		  mNS=reqEvent[i].mNS;
		  aNS=reqEvent[i].aNS;
		  mLS=reqEvent[i].mLS;
		  aLS=reqEvent[i].aLS;
		  sdNS=reqEvent[i].sdNS;
		  sdLS=reqEvent[i].sdLS;
		  mean_Nrfd=reqEvent[i].mean_Nrfd;
		  std_Nrfd=reqEvent[i].std_Nrfd;
		  mean_Lrfd=reqEvent[i].mean_Lrfd;
		  std_Lrfd=reqEvent[i].std_Lrfd;
		  */
			
	  }
	  average_Rev=1.0*average_Rev/time_to;
	  average_Cos=1.0*average_Cos/time_to;
	  average_ratio_R2C=average_Rev/average_Cos;
		mNS=1.0*mNS/time_to;
		aNS=1.0*aNS/time_to;
		mLS=1.0*mLS/time_to;
		aLS=1.0*aLS/time_to;
		sdNS=1.0*sdNS/time_to;
		sdLS=1.0*sdLS/time_to;
		mean_Nrfd=1.0*mean_Nrfd/time_to;
		std_Nrfd=1.0*std_Nrfd/time_to;
		mean_Lrfd=1.0*mean_Lrfd/time_to;
		std_Lrfd=1.0*std_Lrfd/time_to;

  }
  int curFile::outputfile(char outputFileName[LEN_FILENAME]){
	 FILE *fp = fopen(outputFileName, "a+");
	 if (!fp) {
		 cout << "failed to open file: "<<outputFileName << endl;
		 return COULD_NOT_OPEN_FILE;
	 }
	 double reject_ratio=rejectNum*1.0/(acceptNum+rejectNum);
	 fprintf(fp, "%2d %6d %4d %4d %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf %10.4lf\n",time_from, time_to, acceptNum, rejectNum, reject_ratio, nodeRev, edgeRev, totRev, average_Rev, nodeCost, edgeCost, totCost, average_Cos,average_ratio_R2C, nodeLos, edgeLos, totLos, mNS, aNS, mLS, aLS, sdNS, sdLS, mean_Nrfd, std_Nrfd, mean_Lrfd, std_Lrfd);
	 fclose(fp);
	 return SUCCESS;
  
  }

 