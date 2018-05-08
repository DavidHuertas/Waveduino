% GIRO: REPRESENTA EL GIRO REALIZADO EN FUNCIÓN DEL TIEMPO

om_zfilt=x';
theta=zeros(length(om_zfilt),1);

for i=2:length(om_zfilt)
    
    theta(i)=theta(i-1)+om_zfilt(i)*dt;
    
end

figure(2);
subplot(2,1,1)
plot(1:length(theta),theta);
subplot(2,1,2)
plot(1:length(om_zfilt),om_zfilt);