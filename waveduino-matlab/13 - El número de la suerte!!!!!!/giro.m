% GIRO: REPRESENTA EL GIRO REALIZADO EN FUNCIÓN DEL TIEMPO

om_zfilt=x';
theta=zeros(length(om_zfilt),1);

for i=2:length(om_zfilt)
    
    theta(i)=theta(i-1)+om_zfilt(i)*dt-30/1000;
    
end

figure(2);
subplot(2,1,1)
plot(1:length(theta),theta);
grid on;
subplot(2,1,2)
plot(1:length(om_zfilt),om_zfilt);
grid on