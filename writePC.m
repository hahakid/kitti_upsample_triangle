function [] = writePC(point,frame,datapath,dir)
  %save combined pc into 1 bin file
  %用single精度统一存储雷达点云数据
  %savepath=['E:\data\lidartracking\LidarConvert\EV820Radar\bin\Debug\data3\',dir]; 
    
    fd=fopen(sprintf('%s/%d.bin',[datapath,dir],frame),'wb');
    fwrite(fd,point','single');
    fclose(fd);
    
end

