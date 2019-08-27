clear all; close all; clc;
%{
���������ϲ����ű�
�����ǵ�ǰ֡
object ��֡�ǲ������ģ������޷����е��Ӳ�����ֻ��single
%}
data_path = 'H:\kitti\object\training'; %����path
%save_path='.\save_path\';%���path
save_path='H:\kitti\object\training\upsample\triangle';

%�����洢Ŀ¼
%c_save_dir=fullfile(save_path,sprintf('%04d/',seq_idx));
if ~exist(save_path,'dir')
   mkdir(save_path);
end

lidar_dir = fullfile(data_path, 'velodyne');%��ǰ�״�����path
calib_dir = fullfile(data_path, 'calib');%��ǰУ׼�ļ�path
img_dir=fullfile(data_path, 'image_2');%��ǰͼ������path

nimages = length(dir(fullfile(lidar_dir, '*.bin')));%��ͼ��
fst_frame = 152; nt_frames = nimages-1;%
%fst_frame = 1; nt_frames = nimages;% for icp ���ӣ��ӵڶ�֡��ʼ������һ֡���ӵ�ǰ֡

%bias=20;%�ݺ���ƫ�ƣ�����������ϵתΪ��

interp_method='linear';%nearest linear natural  ��ֵ�㷨�����������½�

for frame = fst_frame: 1: nt_frames
    %frame
    bt=cputime;
    %operate calib file
    st=Fun_open_calib(sprintf('/%06d.txt', frame),calib_dir);%��ȡ��������
    %boundary parameters--ǰ�����Ҳü�����
    st.x_min= 5;%-20;%left-right
    st.x_max= 80;%60;% kitti ��80�׻����ϰ���Ƿ��б�Ҫ
    st.y_min=-80;%front-back
    st.y_max=80;
    %lidar file
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
    
	
    %��ʾԭʼ���Ƶ�ԭͼ
    %single_projection(img,velo_c);
    
    up_im=Fun_upsample(velo_c,img,st,interp_method);%�ϲ���+���ܻ�
       
    time=cputime-bt;
    %imshow(up_im);
    imwrite(up_im,sprintf('%s/%06d.png',save_path,frame));%�����ʾ�����Ϣ�ĻҶ�ͼ
    %��Ϣ���
    point_num=length(velo_c(:,1));
    info_str=sprintf('%06d %d %fs',frame,point_num,time);% û��rmse
    writelabels(save_path,0,info_str,'single');% seq=0 Ĭ��һ��
end


