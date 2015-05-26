#include<Windows.h>
#include "In_Event.h"
using namespace std;
char Input_FolderName[LEN_FILENAME];
char Input_FileName[LEN_FILENAME];
char OutputFileName[LEN_FILENAME];
int main(int argc, char *argv[]) {
	int i,interval_time;
	if (argc != 4) {
    cerr << "usage: SIM <Input Filefolder> <Ourput filefolder> <interval_time>" << endl;
    exit(1);
	}
	sprintf(Input_FolderName, "%s\\*.*", argv[1]); //指定搜索目录和文件类型
	interval_time=atoi(argv[3]);
	HANDLE hFile;		
	WIN32_FIND_DATA pNextInfo;	//搜索得到的文件信息将储存在pNextInfo中;	
	hFile = FindFirstFile(Input_FolderName,&pNextInfo);//请注意是 &pNextInfo , 不是 pNextInfo;	
	if(hFile == INVALID_HANDLE_VALUE){		
		//搜索失败		
		exit(-1);	
	}
	while(FindNextFile(hFile,&pNextInfo)){		
		if(pNextInfo.cFileName[0] == '.')//过滤.和..		  
			continue;
		sprintf(Input_FileName,"%s\\%s",argv[1],pNextInfo.cFileName);
		curFile curfile(Input_FileName);
		for(i=interval_time;i<curfile.reqEvent[curfile.reqEvent.size()-1].time;i=i+interval_time){
		curfile.calcu_param(0,i);
		sprintf(OutputFileName,"%s\\%s.statics",argv[2],pNextInfo.cFileName);
		curfile.outputfile(OutputFileName);
		}
	}
}
