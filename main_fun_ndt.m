clear all; close all; clc;
%{
用来生成上采样脚本
NDT
%}
%data_path = 'E:\data\kitti_tracking\training\'; %输入path
%save_path='.\save_path_ndt\';%输出path

data_path = 'H:\kitti\tracking\training'; %输入path
save_path='H:\kitti\tracking\training\upsample\triangle';

for seq_idx=0:0% 20 %数据序列0-20
%创建存储目录
c_save_dir=fullfile(save_path,sprintf('%04d/',seq_idx));%['F:\jiediaosi\code\kitti\frames\tracking\','labels\']; % 新标签
if ~exist(c_save_dir,'dir')
   mkdir(c_save_dir);
end

lidar_dir = fullfile(data_path, sprintf('velodyne/%04d', seq_idx));%当前雷达序列path
calib_dir = fullfile(data_path, 'calib/');%当前校准文件path
img_dir=fullfile(data_path, sprintf('image_02/%04d', seq_idx));%当前图像序列path

st=Fun_open_calib(sprintf('%04d.txt', seq_idx),calib_dir);%获取矫正矩阵
nimages = length(dir(fullfile(lidar_dir, '*.bin')));%总图像
%fst_frame = 0; nt_frames = nimages-1;%
fst_frame = 0; nt_frames = nimages-1;% for icp 叠加，从第二帧开始，用上一帧叠加当前帧

%bias=20;%纵横向偏移，将所有坐标系转为正

%boundary parameters--前后左右裁剪参数
st.x_min= -5;%-20;%left-right
st.x_max= 80;%60;% kitti 到80米还有障碍物，是否有必要
st.y_min=-80;%front-back
st.y_max=80;

interp_method='linear';%nearest linear natural  插值算法，依次速率下降

for frame = fst_frame: 1: nt_frames
    frame
    bt=cputime;
    fd_c = fopen(sprintf('%s/%06d.bin',lidar_dir,frame),'rb');% 当前帧
    fd_l = fopen(sprintf('%s/%06d.bin',lidar_dir,frame-1),'rb');% 上一帧
    if fd_c < 1 || fd_l<1
        fprintf('No LIDAR files !!!\n');
        continue;
        %keyboard
    else
    velo_c = fread(fd_c,[4 inf],'single')';% 读取雷达数据，存为n*4的矩阵
    velo_l = fread(fd_l,[4 inf],'single')';% 读取雷达数据，存为n*4的矩阵
    
    fclose(fd_c);fclose(fd_l);
    end
    
    %velo=distanceFilter(velo,st);
    img = imread(sprintf('%s/%06d.png',img_dir,frame));
    %前向投影，按视野过滤
    %velo_c=velo_preprocess(velo_c,st,size(img));
    velo_c(velo_c(:, 1) < st.x_min, :) = [];
    velo_l(velo_l(:, 1) < st.x_min, :) = [];
    %velo_l=velo_preprocess(velo_l,st,size(img));
    %velo=lidar_merge_icp(velo_c,velo_l);
    [velo,rmse]=lidar_merge_ndt(velo_c,velo_l);
    
    pixel=[velo.Location,velo.Intensity];
    %%%%%%%%对比叠加前后%%%%%%%%%
    %compare(img,velo_l,velo.Location);
    %%%%%%%%
    %velo=lidar_merge1(velo_c,velo_l);
    pixel1=velo_preprocess(pixel,st,size(img));% ndt 是前面不能做投影变换，只能在处理后做
    up_im=Fun_upsample(pixel1,img,st,interp_method);%上采样+稠密化
    time=cputime-bt;
    %imshow(up_im);
    imwrite(up_im,sprintf('%s%06d.png',c_save_dir,frame));%保存表示深度信息的灰度图
    
    %信息输出
    point_num=length(pixel1(:,1));
    info_str=sprintf('%06d %d %fs %0.5f',frame,point_num,time,rmse);% 没有rmse
    writelabels(save_path,seq_idx,info_str,'ndt');
end
end


