sec = 3;

if (~exist('offset', 'var'))
    [offset, gain] = calibrar (sec,s);
end

[valuesmatrix,dt] = lectIMU(s,offset,gain);

Q_angle=0.001;
Q_bias=0.003;
R_angle=0.03;
ax(:)=valuesmatrix(1,1,:);
az(:)=valuesmatrix(1,3,:);
omy(:)=valuesmatrix(2,2,:);
n=min(length(ax),length(az),length(omy));
angle=zeros(1,n);
bias=zeros(1,n);
newangle=zeros(1,n);
z=zeros(1,n);
y=zeros(1,n);
s=0;
K=zeros(2,n);
L=100;
P=zeros(2,2,n);
P(:,:,1)=[L 0 ; 0 L];

for i = 2,n;
    
    angle(i)=angle(i-1)+(omy(i)-bias(i-1))*dt;
    P(1,1,i)=P(1,1,i-1)+(dt*P(2,2,i-1)-P(1,2,i-1)-P(2,1,i-1)+Q_angle)*dt;
    P(1,2,i)=P(1,2,i-1)-dt*P(2,2,i-1);
    P(2,1,i)=P(2,1,i-1)-dt*P(2,2,i-1);
    P(2,2,i)=P(2,2,i-1)+dt*Q_bias;
    z(i)=arctg(az(i)/ax(i));
    y(i)=z(i)-angle(i);
    s=P(1,1)+R_angle;
    K(1,i)=P(1,1,i)/s;
    K(2,i)=P(2,1,i)/s;
    angle(i)=angle(i)+K(1,i)*y(i);
    bias(i)=bias(i)+K(1,i)*y(i);
    P(1,1,i)=P(1,1,i)+K(1,i)*P(1,1,i);
    P(1,2,i)=P(1,2,i)+K(1,i)*P(1,2,i);
    P(2,1,i)=P(2,1,i)+K(2,i)*P(1,1,i);
    P(2,2,i)=P(2,2,i)+K(2,i)*P(1,2,i);   
end
