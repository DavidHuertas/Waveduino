%Tutorial 1: http://www.youtube.com/watch?v=ymWXCPenNM4

function [s,flag] = setupSerial(comPort)

flag=1;
s = serial (comPort);
set(s,'DataBits',8);
set(s,'StopBits',1);
set(s,'BaudRate',9600);
set(s,'Parity','none');
fopen(s);
a='b';
while (a~='a')
    a=fread(s,1,'uchar');
end
if (a=='a')
    disp('serial read');
end
fprintf(s,'%c', 'a');
mbox = msgbox('Serial Comunication setup.'); uiwait(mbox);
fscanf(s,'%u');
end
