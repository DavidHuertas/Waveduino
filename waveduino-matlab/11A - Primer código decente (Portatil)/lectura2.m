sec=3;
t=10;
i=1;
tic

if (~exist('offset', 'var'))
    [offset, gain] = calibrar (sec,s);
end
    disp('Recibiendo datos')
while toc<t

    valuesmatrix(:,i) = lectIMU(s,offset,gain);
    i=i+1;
    
end

dt=t/length(valuesmatrix(1,:));