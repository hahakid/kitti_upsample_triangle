function [] = writelabels( path, seq, str1,mode)
    %�����з�װ�õ��ַ���д���Ӧ�ı�
    file=sprintf('%s/%04d_%s.txt',path,seq,mode);
    fid=fopen(file,'a+');
    fprintf(fid,'%s\n',str1);
    fclose(fid);
end

