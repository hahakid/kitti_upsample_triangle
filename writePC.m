function [] = writePC(point,frame,datapath,dir)
  %save combined pc into 1 bin file
  %��single����ͳһ�洢�״��������
  %savepath=['E:\data\lidartracking\LidarConvert\EV820Radar\bin\Debug\data3\',dir]; 
    
    fd=fopen(sprintf('%s/%d.bin',[datapath,dir],frame),'wb');
    fwrite(fd,point','single');
    fclose(fd);
    
end

