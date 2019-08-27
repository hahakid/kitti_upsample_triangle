clear all; close all; clc;
%{
���������ϲ����ű�
�����ǵ�ǰ֡
%}
data_path = 'H:\kitti\tracking\training'; %����path
%save_path='.\save_path\';%���path
save_path='H:\kitti\tracking\training\LidarROI';

for seq_idx=0:20 %20% 20 %��������0-20
%�����洢Ŀ¼
c_save_dir=fullfile(save_path,sprintf('%04d/',seq_idx));%['F:\jiediaosi\code\kitti\frames\tracking\','labels\']; % �±�ǩ
if ~exist(c_save_dir,'dir')
   mkdir(c_save_dir);
end

lidar_dir = fullfile(data_path, sprintf('velodyne/%04d', seq_idx));%��ǰ�״�����path
calib_dir = fullfile(data_path, 'calib/');%��ǰУ׼�ļ�path
img_dir=fullfile(data_path, sprintf('image_02/%04d', seq_idx));%��ǰͼ������path

st=Fun_open_calib(sprintf('%04d.txt', seq_idx),calib_dir);%��ȡ��������
nimages = length(dir(fullfile(lidar_dir, '*.bin')));%��ͼ��
fst_frame = 0; nt_frames = nimages-1;%
%fst_frame = 1; nt_frames = nimages;% for icp ���ӣ��ӵڶ�֡��ʼ������һ֡���ӵ�ǰ֡

%bias=20;%�ݺ���ƫ�ƣ�����������ϵתΪ��

%boundary parameters--ǰ�����Ҳü�����
st.x_min= 5;%-20;%left-right
st.x_max= 80;%60;% kitti ��80�׻����ϰ���Ƿ��б�Ҫ
st.y_min=-80;%front-back
st.y_max=80;

interp_method='linear';%nearest linear natural  ��ֵ�㷨�����������½�

for frame = fst_frame: 1: nt_frames
    %frame
    bt=cputime;
    fd_c = fopen(sprintf('%s/%06d.bin',lidar_dir,frame),'rb');% ��ǰ֡
    %fd_l = fopen(sprintf('%s/%06d.bin',lidar_dir,frame-1),'rb');% ��һ֡
    if fd_c < 1 %|| fd_l<1
        fprintf('No LIDAR files !!!\n');
        continue;
        %keyboard
    else
    velo_c = fread(fd_c,[4 inf],'single')';% ��ȡ�״����ݣ���Ϊn*4�ľ���
    %velo_l = fread(fd_l,[4 inf],'single')';% ��ȡ�״����ݣ���Ϊn*4�ľ���
    
    fclose(fd_c);%fclose(fd_l);
    end
    
    %velo=distanceFilter(velo,st);
    img = imread(sprintf('%s/%06d.png',img_dir,frame));
    
    %%%%%%%%%%%%%%%%%%
    %removeground(velo_c); %��Ҫ���Ƶ�ring��Ϣ��kittiû��
    %%%%%%%%%%%%%%%%%
    
    %ǰ��ͶӰ������Ұ����
    velo_c=velo_preprocess(velo_c,st,size(img));
    velo_c=velo_c';
    fd=fopen(sprintf('%s/%06d.bin',c_save_dir,frame),'wb');
    fwrite(fd,velo_c,'single');% single= float32, double =float64
    fclose(fd);
    %single_projection(img,velo_c);
    
    %velo_l=velo_preprocess(velo_l,st,size(img));
    %[velo,rmse]=lidar_merge_icp(velo_c,velo_l);
    %pixel=[velo.Location,velo.Intensity];
    
    %velo=lidar_merge1(velo_c,velo_l);
    %pixel1=velo_preprocess2(pixel,st,size(img));% ��Ϊǰ���֡�Ѿ�����ͶӰ�任������Ͳ���Ҫ��ͶӰ�任��,ֻ��Խ���ж�
    %up_im=Fun_upsample(velo_c,img,st,interp_method);%�ϲ���+���ܻ�
       
    time=cputime-bt;
    %imshow(up_im);
    %imwrite(up_im,sprintf('%s%06d.png',c_save_dir,frame));%�����ʾ�����Ϣ�ĻҶ�ͼ
    %��Ϣ���
    %point_num=length(velo_c(:,1));
    %info_str=sprintf('%06d %d %fs',frame,point_num,time);% û��rmse
    %writelabels(save_path,seq_idx,info_str,'single');
end
end


