function   static2xls(importdir,outputdir)          
if ~ischar(importdir)
    error('fname should be a char string');
end
if ~ischar(outputdir)
    error('fname should be a char string');
end
file=dir(fullfile(importdir,'*.statics')); %文档路径（文件夹）
total_file=fullfile(outputdir,'Total.xlsx');

% range='A1:Z';
    for i=1:size(file) 
        totline=[];
       [ pathstr, name, ext, versn] = fileparts(fullfile(importdir,file(i).name));
       fid = fopen(strcat(fullfile(pathstr,name),'.statics'));
       while(1)
           [cur_line,count]=fscanf(fid,'%d %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f',27);
               if(~count) break;
               end
           totline=[totline,cur_line];

       end
       fclose(fid);     %关闭统计文件
       totline=totline';
%        strcat(range,num2str(size(totline,1)));
       filename=strcat(fullfile(outputdir,name),'.xlsx');
       xlswrite(filename,totline,1);
       if(i==1)
           %accept_ratio
           xlswrite(total_file,{'time'},'accept_ratio','A1');
           xlswrite(total_file,totline(:,2),'accept_ratio',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %average_Rev
           xlswrite(total_file,{'time'},'average_Rev','A1');
           xlswrite(total_file,totline(:,2),'average_Rev',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %average_Cos
           xlswrite(total_file,{'time'},'average_Cos','A1');
           xlswrite(total_file,totline(:,2),'average_Cos',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %average_ratio_R2C
           xlswrite(total_file,{'time'},'average_ratio_R2C','A1');
           xlswrite(total_file,totline(:,2),'average_ratio_R2C',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %totLos 
           xlswrite(total_file,{'time'},'totLos','A1');
           xlswrite(total_file,totline(:,2),'totLos',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %aNS
           xlswrite(total_file,{'time'},'aNS','A1');
           xlswrite(total_file,totline(:,2),'aNS',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %aLS
           xlswrite(total_file,{'time'},'aLS','A1');
           xlswrite(total_file,totline(:,2),'aLS',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %mean_Nrfd
           xlswrite(total_file,{'time'},'mean_Nrfd','A1');
           xlswrite(total_file,totline(:,2),'mean_Nrfd',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %std_Nrfd
           xlswrite(total_file,{'time'},'std_Nrfd','A1');
           xlswrite(total_file,totline(:,2),'std_Nrfd',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %mean_Lrfd
           xlswrite(total_file,{'time'},'mean_Lrfd','A1');
           xlswrite(total_file,totline(:,2),'mean_Lrfd',['A2:','A',num2str(size(totline(:,2),1)+1)]);
           %std_Lrfd
           xlswrite(total_file,{'time'},'std_Lrfd','A1');
           xlswrite(total_file,totline(:,2),'std_Lrfd',['A2:','A',num2str(size(totline(:,2),1)+1)]);
       end
       %accept_ratio   5列
       xlswrite(total_file,{name(10:27)},'accept_ratio',[setstr(65+i),'1']);
       xlswrite(total_file,1-totline(:,5),'accept_ratio',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,5),1)+1)]);
       %average_Rev    9列
       xlswrite(total_file,{name(10:27)},'average_Rev',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,9),'average_Rev',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,9),1)+1)]);
       %average_Cos 13列
       xlswrite(total_file,{name(10:27)},'average_Cos',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,13),'average_Cos',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,13),1)+1)]);
       %average_ratio_R2C 14列
       xlswrite(total_file,{name(10:27)},'average_ratio_R2C',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,14),'average_ratio_R2C',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,14),1)+1)]);
       %totLos 17
       xlswrite(total_file,{name(10:27)},'totLos',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,17),'totLos',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,17),1)+1)]);
       %aNS 19
       xlswrite(total_file,{name(10:27)},'aNS',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,19),'aNS',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,19),1)+1)]);
       %aLS 21
       xlswrite(total_file,{name(10:27)},'aLS',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,21),'aLS',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,21),1)+1)]);
       %mean_Nrfd 24
       xlswrite(total_file,{name(10:27)},'mean_Nrfd',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,24),'mean_Nrfd',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,24),1)+1)]);
       %std_Nrfd 25
       xlswrite(total_file,{name(10:27)},'std_Nrfd',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,25),'std_Nrfd',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,25),1)+1)]);
       %mean_Lrfd 26
       xlswrite(total_file,{name(10:27)},'mean_Lrfd',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,26),'mean_Lrfd',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,26),1)+1)]);
       %std_Lrfd 27
       xlswrite(total_file,{name(10:27)},'std_Lrfd',[setstr(65+i),'1']);
       xlswrite(total_file,totline(:,27),'std_Lrfd',[setstr(65+i),'2:',setstr(65+i),num2str(size(totline(:,27),1)+1)]);
      % time_from, time_to, acceptNum, rejectNum, reject_ratio, 
      % nodeRev, edgeRev, totRev, average_Rev, nodeCost,
      % edgeCost, totCost, average_Cos,average_ratio_R2C, nodeLos,
      % edgeLos, totLos, mNS, aNS, mLS, 
      % aLS, sdNS, sdLS, mean_Nrfd, std_Nrfd,
      % mean_Lrfd, std_Lrfd

    end
end
