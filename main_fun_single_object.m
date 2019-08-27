clear all; close all; clc;
%{
用来生成上采样脚本
仅考虑当前帧
object 的帧是不连续的，所以无法进行叠加操作，只有single
%}
data_path = 'H:\kitti\object\training'; %输入path
%save_path='.\save_path\';%输出path
save_path='H:\kitti\object\training\upsample\triangle';

%创建存储目录
%c_save_dir=fullfile(save_path,sprintf('%04d/',seq_idx));
if ~exist(save_path,'dir')
   mkdir(save_path);
end

lidar_dir = fullfile(data_path, 'velodyne');%当前雷达序列path
calib_dir = fullfile(data_path, 'calib');%当前校准文件path
img_dir=fullfile(data_path, 'image_2');%当前图像序列path

nimages = length(dir(fullfile(lidar_dir, '*.bin')));%总图像
fst_frame = 152; nt_frames = nimages-1;%
%fst_frame = 1; nt_frames = nimages;% for icp 叠加，从第二帧开始，用上一帧叠加当前帧

%bias=20;%纵横向偏移，将所有坐标系转为正

interp_method='linear';%nearest linear natural  插值算法，依次速率下降

for frame = fst_frame: 1: nt_frames
    %frame
    bt=cputime;
    %operate calib file
    st=Fun_open_calib(sprintf('/%06d.txt', frame),calib_dir);%获取矫正矩阵
    %boundary parameters--前后左右裁剪参数
    st.x_min= 5;%-20;%left-right
    st.x_max= 80;%60;% kitti 到80米还有障碍物，是否有必要
    st.y_min=-80;%front-back
    st.y_max=80;
    %lidar file
    fd_c = fopen(sprintf('%s/%06d.bin',lidar_dir,frame),'rb');% 当前帧
    %fd_l = fopen(sprintf('%s/%06d.bin',lidar_dir,frame-1),'rb');% 上一帧
    if fd_c < 1 %|| fd_l<1
        fprintf('No LIDAR files !!!\n');
        continue;
        %keyboard
    else
    velo_c = fread(fd_c,[4 inf],'single')';% 读取雷达数据，存为n*4的矩阵
    %velo_l = fread(fd_l,[4 inf],'single')';% 读取雷达数据，存为n*4的矩阵
    
    fclose(fd_c);%fclose(fd_l);
    end
    
    %velo=distanceFilter(velo,st);
    img = imread(sprintf('%s/%06d.png',img_dir,frame));
    
    %%%%%%%%%%%%%%%%%%
    %removeground(velo_c); %需要点云的ring信息，kitti没有
    %%%%%%%%%%%%%%%%%
    
    %前向投影，按视野过滤
    velo_c=velo_preprocess(velo_c,st,size(img));
    
	
    %显示原始点云到原图
    %single_projection(img,velo_c);
    
    up_im=Fun_upsample(velo_c,img,st,interp_method);%上采样+稠密化
       
    time=cputime-bt;
    %imshow(up_im);
    imwrite(up_im,sprintf('%s/%06d.png',save_path,frame));%保存表示深度信息的灰度图
    %信息输出
    point_num=length(velo_c(:,1));
    info_str=sprintf('%06d %d %fs',frame,point_num,time);% 没有rmse
    writelabels(save_path,0,info_str,'single');% seq=0 默认一个
end


