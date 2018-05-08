f=gz;
n=length(f);
t0=0;
x0=0;
tf=10;
x=zeros(1,n+1);

[t,xrk] =rk_1(f,t0,tf,x0,n);

for i=2:n
    
    x(i)=x(i-1)+(10/n)*f(i);
    
end

subplot(2,1,1)
plot(t,xrk)
subplot(2,1,2)
plot(t,x)