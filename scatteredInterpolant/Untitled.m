close all;
clc
format long
%对于类似的txt文件，不含有字符，只有数字
data1=load('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\样本点坐标.txt');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
scatter3(x,y,z);

data2=load('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\样本点.txt');
v = data2(:,1);
F = scatteredInterpolant(x,y,z,v,'linear');
%linear
data3=load('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\插值点坐标.txt');
xq = data3(:,1);
yq = data3(:,2);
zq = data3(:,3);
t=0;
tic;
vq = F(xq,yq,zq);
t=t+toc;
%把插值结果写入txt文件
 fid = fopen('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\matlab插值点value.txt','wt');
 fprintf(fid,'%g\n',vq);
 fclose(fid);
data4=load('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\真实插值点value.txt');
v_real=data4(:,1);
abs_error = abs(vq-v_real);
abs_error = mean(mean(abs_error));
v_real_mean = mean(mean(abs(v_real)));
Relative_error = abs_error/v_real_mean;

%%
%写入文件
data=load('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\重建点02.txt');
x=data(:,9);
fid = fopen('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\真实插值点value.txt','wt');
fprintf(fid,'%g\n',x);    
fclose(fid);

%%
%样本点坐标
% data=load('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\重建点02.txt');
% x=data(:,6);
% y=data(:,7);
% z=data(:,8);
% fid = fopen('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\插值点坐标.txt','wt');
% for i=1:length(x)
%  %for j=1:c
%  fprintf(fid,'%f %f %f',x(i,1),y(i,1),z(i,1));
%  %end
%  fprintf(fid,'\n');
% end
% fclose(fid);

%%
data1=load('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\真实插值点value.txt');
v_real=data1(:,1);
data2=load('E:\XTOP\XTOPwork\CGAL-NNI\测试数据\插值点value.txt');
vq=data2(:,1);
abs_error = abs(vq-v_real);
abs_error = mean(mean(abs_error));
v_real_mean = mean(mean(abs(v_real)));
Relative_error = abs_error/v_real_mean;

%%
close all;
clc
format long
%对于类似的txt文件，不含有字符，只有数字
data1=load('E:\XTOP\XTOPwork\ICP\测试数据\test\07\PT1.asc');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);
 fid = fopen('E:\XTOP\XTOPwork\ICP\测试数据\test\07\pPT1.asc','wt');
    for i = 1:s
    fprintf(fid,'%f\t%f\t%f\n',x(i,1),y(i,1),z(i,1));
    end
 fclose(fid);
 
 data1=load('E:\XTOP\XTOPwork\ICP\测试数据\test\07\PT2.asc');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);
 fid = fopen('E:\XTOP\XTOPwork\ICP\测试数据\test\07\pPT2.asc','wt');
    for i = 1:s
    fprintf(fid,'%f\t%f\t%f\n',x(i,1),y(i,1),z(i,1));
    end
 fclose(fid);
 
 
 data1=load('E:\XTOP\XTOPwork\ICP\测试数据\test\07\PT3.asc');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);
 fid = fopen('E:\XTOP\XTOPwork\ICP\测试数据\test\07\pPT3.asc','wt');
    for i = 1:s
    fprintf(fid,'%f\t%f\t%f\n',x(i,1),y(i,1),z(i,1));
    end
 fclose(fid);
 
 
 data1=load('E:\XTOP\XTOPwork\ICP\测试数据\test\07\PT4.asc');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);
 fid = fopen('E:\XTOP\XTOPwork\ICP\测试数据\test\07\pPT4.asc','wt');
    for i = 1:s
    fprintf(fid,'%f\t%f\t%f\n',x(i,1),y(i,1),z(i,1));
    end
 fclose(fid);
 
 
 data1=load('E:\XTOP\XTOPwork\ICP\测试数据\test\07\PT5.asc');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);
 fid = fopen('E:\XTOP\XTOPwork\ICP\测试数据\test\07\pPT5.asc','wt');
    for i = 1:s
    fprintf(fid,'%f\t%f\t%f\n',x(i,1),y(i,1),z(i,1));
    end
 fclose(fid);
 
 
 data1=load('E:\XTOP\XTOPwork\ICP\测试数据\test\07\PT6.asc');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);
 fid = fopen('E:\XTOP\XTOPwork\ICP\测试数据\test\07\pPT6.asc','wt');
    for i = 1:s
    fprintf(fid,'%f\t%f\t%f\n',x(i,1),y(i,1),z(i,1));
    end
 fclose(fid);
 
 
 data1=load('E:\XTOP\XTOPwork\ICP\测试数据\test\07\PT7.asc');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);
 fid = fopen('E:\XTOP\XTOPwork\ICP\测试数据\test\07\pPT7.asc','wt');
    for i = 1:s
    fprintf(fid,'%f\t%f\t%f\n',x(i,1),y(i,1),z(i,1));
    end
 fclose(fid);
 
 
 data1=load('E:\XTOP\XTOPwork\ICP\测试数据\test\07\PT8.asc');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);
 fid = fopen('E:\XTOP\XTOPwork\ICP\测试数据\test\07\pPT8.asc','wt');
    for i = 1:s
    fprintf(fid,'%f\t%f\t%f\n',x(i,1),y(i,1),z(i,1));
    end
 fclose(fid);
 
 
 data1=load('E:\XTOP\XTOPwork\ICP\测试数据\test\07\PT9.asc');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);
 fid = fopen('E:\XTOP\XTOPwork\ICP\测试数据\test\07\pPT9.asc','wt');
    for i = 1:s
    fprintf(fid,'%f\t%f\t%f\n',x(i,1),y(i,1),z(i,1));
    end
 fclose(fid);
 %%
%写入文件
data1=load('E:\XTOP\XTOPwork\NN_NNI\NN_NNI\样本点坐标.txt');
x=data1(:,1);
y=data1(:,2);
z=data1(:,3);
s = length(x);

data2=load('E:\XTOP\XTOPwork\NN_NNI\NN_NNI\样本点.txt');
v=data2(:,1);
fid = fopen('E:\XTOP\XTOPwork\NN_NNI\NN_NNI\data.txt','wt');
    for i = 1:s
    fprintf(fid,'%f %f %f %f\n',x(i,1),y(i,1),z(i,1),v(i,1));
    end
fclose(fid);
 
 
 
 
 