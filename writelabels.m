function [] = writelabels( path, seq, str1,mode)
    %将按行封装好的字符串写入对应文本
    file=sprintf('%s/%04d_%s.txt',path,seq,mode);
    fid=fopen(file,'a+');
    fprintf(fid,'%s\n',str1);
    fclose(fid);
end

