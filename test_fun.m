clear all; close all; clc;

data_path = 'E:\kitti\tracking\training\';
save_path='E:\data\kitti_tracking\training\save_path\';

gridSize = 0.1;%
mergeSize = 0.015;%

for seq_idx=0:0% 20
c_save_dir=fullfile(save_path,sprintf('%04d/',seq_idx));%['F:\jiediaosi\code\kitti\frames\tracking\','labels\']; % �±�ǩ
if ~exist(c_save_dir,'dir')
   mkdir(c_save_dir);
end

lidar_dir = fullfile(data_path, sprintf('velodyne/%04d', seq_idx));
%label_dir = fullfile(data_path, 'label_02/');
calib_dir = fullfile(data_path, 'calib/');
img_dir=fullfile(data_path, sprintf('image_02/%04d', seq_idx));
%tracklets = readLabels(label_dir, seq_idx);
%P = readCalibration(calib_dir,seq_idx,2);
%sprintf('%04d.txt', seq_idx)
st=Fun_open_calib(sprintf('%04d.txt', seq_idx),calib_dir);
nimages = length(dir(fullfile(lidar_dir, '*.bin')));
%fst_frame = 0; nt_frames = nimages-1;%

fst_frame = 2; nt_frames = nimages-1;% for icp ���ӣ��ӵڶ�֡��ʼ������һ֡���ӵ�ǰ֡

%bias=20;%�ݺ���ƫ�ƣ�����������ϵתΪ��
%boundary parameters
st.x_min= -20;%-20;%left-right
st.x_max= 80;%60;% kitti ��80�׻����ϰ���Ƿ��б�Ҫ
st.y_min=-20;%front-back
st.y_max=20;
st.z_min=-2.5;
st.z_max=0.8;

interp_method='linear';%nearest linear natural

%first frame
fd_l = fopen(sprintf('%s/%06d.bin',lidar_dir,fst_frame-2),'rb');% ��һ֡
velo_l=fread(fd_l,[4 inf],'single')';
fclose(fd_l);
%velo_l=distanceFilter(velo_l,st);
velo_l=pointCloud(velo_l(:,1:3),'Intensity',velo_l(:,4));
fixed = pcdownsample(velo_l, 'gridAverage', gridSize);%��ʼ��

fd_c = fopen(sprintf('%s/%06d.bin',lidar_dir,fst_frame-1),'rb');% ��һ֡
velo_c=fread(fd_c,[4 inf],'single')';
fclose(fd_c);
%velo_c=distanceFilter(velo_c,st);
velo_c=pointCloud(velo_c(:,1:3),'Intensity',velo_c(:,4));
moving = pcdownsample(velo_c, 'gridAverage', gridSize);%����ÿ֡ȥ����

gridSize = 0.1;

tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
ptCloudAligned = pctransform(velo_c,tform);% ��ת
ptCloudScene = pcmerge(velo_l, ptCloudAligned, mergeSize);
accumTform = tform; 

for frame = fst_frame: 1: nt_frames
    %frame
    fd_c = fopen(sprintf('%s/%06d.bin',lidar_dir,frame),'rb');% ��ǰ֡
    if fd_c < 1
        fprintf('No LIDAR files !!!\n');
        continue;
        %keyboard
    else
    velo_c = fread(fd_c,[4 inf],'single')';% ��ȡ�״����ݣ���Ϊn*4�ľ���
    fclose(fd_c);
    end
    %img = imread(sprintf('%s/%06d.png',img_dir,frame));
    %ǰ��ͶӰ������Ұ����
    
    %velo_c=distanceFilter(velo_c,st);
    velo_c=pointCloud(velo_c(:,1:3),'Intensity',velo_c(:,4));
    moving = pcdownsample(velo_c, 'gridAverage', gridSize);
    
    fixed=moving;
    tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
	accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(velo_c, accumTform);
    
    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
    pcshow(ptCloudScene); 
    %velo_c=velo_preprocess(velo_c,st,size(img));
    %velo_l=velo_preprocess(velo_l,st,size(img));
    %velo=lidar_merge(velo_c,velo_l);
    
    
    %up_im=Fun_upsample(velo_c,img,st,interp_method);%�ϲ���+���ܻ�
    %imshow(ImaRange);
    %imwrite(up_im,sprintf('%s%06d.png',c_save_dir,frame));%�����ʾ�����Ϣ�ĻҶ�ͼ
end
end


