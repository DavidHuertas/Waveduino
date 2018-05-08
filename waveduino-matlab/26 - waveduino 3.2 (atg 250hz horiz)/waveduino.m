function varargout = waveduino(varargin)
% WAVEDUINO MATLAB code for waveduino.fig
%      WAVEDUINO, by itself, creates a new WAVEDUINO or raises the existing
%      singleton*.
%
%      H = WAVEDUINO returns the handle to a new WAVEDUINO or the handle to
%      the existing singleton*.
%
%      WAVEDUINO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in WAVEDUINO.M with the given input arguments.
%
%      WAVEDUINO('Property','Value',...) creates a new WAVEDUINO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before waveduino_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to waveduino_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help waveduino

% Last Modified by GUIDE v2.5 02-Apr-2014 04:03:20

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @waveduino_OpeningFcn, ...
                   'gui_OutputFcn',  @waveduino_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before waveduino is made visible.
function varargout = waveduino_OpeningFcn(hObject, eventdata, handles,...
    varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to waveduino (see VARARGIN)
if ~isempty(instrfind)
    tic
    while (toc<=3)
        fprintf(instrfind,'%c','b');
    end
    fclose(instrfind);
    delete(instrfind);
end
load('calibrado.mat');
set(handles.puerto,'String','COM5');
set(handles.baudrate,'Value',7);
set(handles.tiempocalbar,'Value',5);
set(handles.tiempocal,'Value',5);
set(handles.tiempomedirbar,'Value',30);
set(handles.tiempomedir,'Value',30);
set(handles.gananciaacel,'String', num2str(graw));
set(handles.acelumb,'String',num2str(ac_umbral));
set(handles.diftemp, 'String', num2str(dt));
set(handles.frecmuest, 'String', num2str(1/dt));

% Choose default command line output for waveduino
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes waveduino wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = waveduino_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(instrfind)
    tic
    while (toc<=3)
        fprintf(instrfind,'%c','b');
    end
    fclose(instrfind);
    delete(instrfind);
end
% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in botongrafpos.
function varargout = botongrafpos_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to botongrafpos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Representa la posición frente al tiempo
load('medicion.mat');
figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Posiciones');
hold on;
plot(tiempo, pos(:,1), 'r');
plot(tiempo, pos(:,2), 'g');
plot(tiempo, pos(:,3), 'b');
title('Posiciones');
xlabel('Tiempo (s)');
ylabel('Posición (m)');
legend('X', 'Y', 'Z');
hold off;
guidata(hObject, handles);


% --- Executes on button press in botongrafposfilt.
function varargout = botongrafposfilt_Callback(hObject, eventdata,...
    handles, varargin)
% hObject    handle to botongrafposfilt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Representa la posición filtrada frente al tiempo
load('medicion.mat');
figure('Number', 'off', 'Name', 'Posición filtrada');
hold on;
plot(tiempo, posHP(:,1), 'r');
plot(tiempo, posHP(:,2), 'g');
plot(tiempo, posHP(:,3), 'b');
xlabel('Tiempo (s)');
ylabel('Posicion (m)');
title('Posición filtrada');
legend('X', 'Y', 'Z');
guidata(hObject, handles);

% --- Executes on button press in botongrafdatos.
function varargout = botongrafdatos_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to botongrafdatos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load('medicion.mat');
figure('Position', [9 39 900 600], 'Number', 'off', 'Name',...
    'Datos del Sensor');
sensor(1) = subplot(2,1,1);
    hold on;
    plot(tiempo, gx, 'r');
    plot(tiempo, gy, 'g');
    plot(tiempo, gz, 'b');
    title('Giroscopio');
    xlabel('Tiempo (s)');
    ylabel('Velocidad angular (rad/s)');
    legend('X', 'Y', 'Z');
    hold off;
sensor(2) = subplot(2,1,2);
    hold on;
    plot(tiempo, ax, 'r');
    plot(tiempo, ay, 'g');
    plot(tiempo, az, 'b');
    plot(tiempo, ac_modFilt, ':k');
    plot(tiempo, umbral, 'k', 'LineWidth', 2);
    title('Acelerómetro');
    xlabel('Tiempo (s)');
    ylabel('Aceleración (g)');
    legend('X', 'Y', 'Z', 'Acel. Filtrada', 'Umbral');
    hold off;
linkaxes(sensor,'x');


% --- Executes on button press in botongrafacabs.
function varargout = botongrafacabs_Callback(hObject, eventdata,...
    handles,varargin)
load('medicion.mat');
% hObject    handle to botongrafacabs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Representar aceleraciones frente al tiempo
figure('Position', [9 39 900 300], 'Number', 'off', 'Name',...
    'Aceleraciones absolutas');
hold on;
plot(tiempo, acel(:,1), 'r');
plot(tiempo, acel(:,2), 'g');
plot(tiempo, acel(:,3), 'b');
title('Aceleraciones absolutas');
xlabel('Tiempo (s)');
ylabel('Aceleración (m/s²)');
legend('X', 'Y', 'Z');
hold off;

% --- Executes on button press in botongrafvel.
function varargout = botongrafvel_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to botongrafvel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Representar velocidades frente al tiempo
load('medicion.mat');
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Velocidades');
hold on;
plot(tiempo, vel(:,1), 'r');
plot(tiempo, vel(:,2), 'g');
plot(tiempo, vel(:,3), 'b');
title('Velocidades');
xlabel('Tiempo (s)');
ylabel('Velocidad (m/s)');
legend('X', 'Y', 'Z');
hold off;


% --- Executes on button press in boton3d.
function varargout = boton3d_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to boton3d (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% ANIMACIÓN 3D

load('medicion.mat');

%posPlot = pos;
posPlot = posHP;
quatPlot = quat;

% Retrasa el final de la animación
extraTime = 0;
onesVector = ones(extraTime*(floor(1/dt)), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*...
    onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*...
    onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];

% Crear la animación 3D
SamplePlotFreq = 4;
SixDOFanimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 768],...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)',...
                'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false,...
                'AVIfps', (floor((floor(1/dt))...
                / SamplePlotFreq)));
            
% --- Executes on selection change in puerto.
function varargout = puerto_Callback(hObject, eventdata, handles, varargin)
% hObject    handle to puerto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns puerto contents as cell array
%        contents{get(hObject,'Value')} returns selected item from puerto


% --- Executes during object creation, after setting all properties.
function varargout = puerto_CreateFcn(hObject, eventdata, handles,...
    varargin)
% hObject    handle to puerto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in baudrate.
function varargout = baudrate_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to baudrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns baudrate contents as cell array
%        contents{get(hObject,'Value')} returns selected item from baudrate

% --- Executes during object creation, after setting all properties.
function varargout = baudrate_CreateFcn(hObject, eventdata, handles,...
    varargin)
% hObject    handle to baudrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in botoncal.
function varargout = botoncal_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to botoncal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

puerto=get(handles.puerto,'String');
baudrate = cellstr(get(handles.baudrate,'String'));
baudrate = fix(str2double(baudrate(get(handles.baudrate,'Value'))));
s = serial (puerto,'DataBits',8,'StopBits',1,...
    'BaudRate',baudrate,'Parity','none');
fopen(s);

n=50000;
t=get(handles.tiempocalbar,'Value');

i=1;
j=1;
ax=zeros(n,1);
ay=zeros(n,1);
az=zeros(n,1);
temp=zeros(n,1);
gx=zeros(n,1);
gy=zeros(n,1);
gz=zeros(n,1);

disp('Calibrando, espere cinco segundos...')
pause(1)
tic

fprintf(s,'%c','a');

while (i<=n && toc<=(t))

    ax(i)=str2double(fgetl(s));
    ay(i)=str2double(fgetl(s));
    az(i)=str2double(fgetl(s));
    temp(i)=str2double(fgetl(s));
    gx(i)=str2double(fgetl(s));
    gy(i)=str2double(fgetl(s));
    gz(i)=str2double(fgetl(s));
    i=i+1;

end

disp('Finalizada la recogida de datos')

if i==n
    t=toc;
else
    ax=ax(1:i-1);
    ay=ay(1:i-1);
    az=az(1:i-1);
    gx=gx(1:i-1);
    gy=gy(1:i-1);
    gz=gz(1:i-1);
    temp=temp(1:i-1);
    n=i-1;
end
if ~isempty(instrfind)
    tic
    while (toc<=3)
        fprintf(instrfind,'%c','b');
    end
    fclose(instrfind);
    delete(instrfind);
end
dt=(t)/n;

set(handles.diftemp, 'String', num2str(dt))

set(handles.frecmuest, 'String', num2str(1/dt))

graw=norm([mean(ax) mean(ay) mean(az)]);

set(handles.gananciaacel,'String', num2str(graw));

temp=35-(-13200-temp)./280;

gain_gyr=(14.375+((15.3733-14.375)/(32-25)).*(temp-25));   

offset_gyr=[mean(gx),mean(gy),mean(gz)];

ax=ax/graw;
ay=ay/graw;
az=az/graw;
gx=(gx-offset_gyr(1))./gain_gyr;
gy=(gy-offset_gyr(2))./gain_gyr;
gz=(gz-offset_gyr(3))./gain_gyr;

tiempocal = linspace(1,t,n);
% Cálculo del módulo de la aceleración
ac_mod = sqrt(ax.*ax + ay.*ay + az.*az);

% Filtro Paso Alto a la aceleración
frecCorte = 0.001;
[b, a] = butter(1, (2*frecCorte)/(1/dt), 'high');
ac_modFilt = filtfilt(b, a, ac_mod);

% Cálculo del valor absoluto de la aceleración filtrada
ac_modFilt = abs(ac_modFilt);

% Filtro Paso Bajo a la aceleración
frecCorte = 5;
[b, a] = butter(1, (2*frecCorte)/(1/dt), 'low');
ac_modFilt = filtfilt(b, a, ac_modFilt);

% Detección de los datos por debajo del umbral
ac_umbral = mean(ac_modFilt) + std(ac_modFilt);
set(handles.acelumb,'String',num2str(ac_umbral));
 
quat = zeros(length(tiempocal), 4);
AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);

% Convergencia inicial
for i = 1:n
    AHRSalgorithm.UpdateIMU([0 0 0], [mean(ax) mean(ay) mean(az)]);
end

save('calibrado','tiempocal','ax','ay','az','gx','gy','gz','ac_modFilt',...
'ac_umbral','offset_gyr','graw','dt')

guidata(hObject, handles);


% --- Executes on button press in botonini.
function varargout = botonini_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to botonini (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load('calibrado.mat');
puerto=get(handles.puerto,'String');
baudrate = cellstr(get(handles.baudrate,'String'));
baudrate = fix(str2double(baudrate(get(handles.baudrate,'Value'))));
s = serial (puerto,'DataBits',8,'StopBits',1,...
    'BaudRate',baudrate,'Parity','none');

n=50000;
t=get(handles.tiempomedirbar,'Value');

fopen(s);

i=1;
j=1;
ax=zeros(n,1);
ay=zeros(n,1);
az=zeros(n,1);
temp=zeros(n,1);
gx=zeros(n,1);
gy=zeros(n,1);
gz=zeros(n,1);

disp('Recibiendo datos en')
disp('3')
pause(1)
disp('2')
pause(1)
disp('1')
pause(1)
disp('Recibiendo...')
tic

fprintf(s,'%c','a');

while (i<=n && toc<=(t+2))
    
    ax(i)=str2double(fgetl(s));
    ay(i)=str2double(fgetl(s));
    az(i)=str2double(fgetl(s));
    temp(i)=str2double(fgetl(s));
    gx(i)=str2double(fgetl(s));
    gy(i)=str2double(fgetl(s));
    gz(i)=str2double(fgetl(s));
    i=i+1;

end

disp('Finalizada la recogida de datos')

if i==n
    t=toc;
else
    ax=ax(1:i-1);
    ay=ay(1:i-1);
    az=az(1:i-1);
    gx=gx(1:i-1);
    gy=gy(1:i-1);
    gz=gz(1:i-1);
    temp=temp(1:i-1);
    n=i-1;
end
if ~isempty(instrfind)
    tic
    while (toc<=3)
        fprintf(instrfind,'%c','b');
    end
    fclose(instrfind);
    delete(instrfind);
end
dt=(t+2)/n;

set(handles.diftemp, 'String', num2str(dt))

set(handles.frecmuest, 'String', num2str(1/dt))

temp=35-(-13200-temp)./280;

gain_gyr=14.375+((15.3733-14.375)/(32-25)).*(temp-25);  

gain_ac=str2double(get(handles.gananciaacel,'String'));

ax=ax/gain_ac;
ay=ay/gain_ac;
az=az/gain_ac;
gx=(gx-offset_gyr(1))./gain_gyr;
gy=(gy-offset_gyr(2))./gain_gyr;
gz=(gz-offset_gyr(3))./gain_gyr;

%Eliminar picos en giroscopio:

for i=1:n-1
    if(abs(gx(i+1)-gx(i))>500)
        gx(i+1)=gx(i);
    end
    if(abs(gy(i+1)-gy(i))>500)
        gy(i+1)=gy(i);
    end
    if(abs(gz(i+1)-gz(i))>500)
        gz(i+1)=gz(i);
    end
end
    

%Eliminar picos en acelerómetro:

for i=1:n-1
    if(abs(ax(i+1)-ax(i))>20)
        ax(i+1)=ax(i);
    end
    if(abs(ay(i+1)-ay(i))>20)
        ay(i+1)=ay(i);
    end
    if(abs(az(i+1)-az(i))>20)
        az(i+1)=az(i);
    end
end
    
    tiempo = linspace(1,t,n);
% Cálculo del módulo de la aceleración
ac_mod = sqrt(ax.*ax + ay.*ay + az.*az);

% Filtro Paso Alto a la aceleración
frecCorte = 0.001;
[b, a] = butter(1, (2*frecCorte)/(1/dt), 'high');
ac_modFilt = filtfilt(b, a, ac_mod);

% Cálculo del valor absoluto de la aceleración filtrada
ac_modFilt = abs(ac_modFilt);

% Filtro Paso Bajo a la aceleración
frecCorte = 5;
[b, a] = butter(1, (2*frecCorte)/(1/dt), 'low');
ac_modFilt = filtfilt(b, a, ac_modFilt);

% Detección de los datos por debajo del umbral
umbral = ac_modFilt < ac_umbral;
quat = zeros(length(tiempo), 4);
AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);

% Convergencia inicial
%tConvInicial = 0;
%sel = 1 : find(sign(tiempo-(tiempo(1)+tConvInicial))+1, 1);
%for i = 1:2000
%    AHRSalgorithm.UpdateIMU([0 0 0], [mean(ax(sel)) mean(ay(sel)) mean(az(sel))]);
%end

% Orientación de todos los datos
for t = 1:length(tiempo)
    if(umbral(t))
        AHRSalgorithm.Kp = 0.5;
    else
        AHRSalgorithm.Kp = 0;
    end
    AHRSalgorithm.UpdateIMU(deg2rad([gx(t) gy(t) gz(t)]), [ax(t) ay(t) az(t)]);
    quat(t,:) = AHRSalgorithm.Quaternion;
end
% Aplicar rotación por cuaterniones
acel = quaternRotate([ax ay az], quaternConj(quat));


% Eliminar picos en los valores
for i=1:n-1
    if(abs(acel(i+1,1)-acel(i,1))>20)
        acel(i+1,1)=acel(i,1);
    end
    if(abs(acel(i+1,2)-acel(i,2))>20)
        acel(i+1,2)=acel(i,2);
    end
    if(abs(acel(i+1,3)-acel(i,3))>20)
        acel(i+1,3)=acel(i,3);
    end
end

% Convertir a m/s²
acel = acel * 9.81;

acel(:,3) = acel(:,3) - 9.81; % Sustraer la gravedad de la aceleración en z

% Integrar la aceleración para obtener la velocidad. Iguala a cero los
% valores situados por debajo del umbral de aceleración nula.

vel = zeros(size(acel));
for t = 2:length(vel)
    vel(t,:) = vel(t-1,:) + acel(t,:) * dt;
    if(umbral(t) == 1)
        vel(t,:) = [0 0 0];
    end
end


% Calcular el error de integración de las velocidades tras el umbral

errorVel = zeros(size(vel));
inicioUmbral = find([0; diff(umbral)] == -1);
finUmbral = find([0; diff(umbral)] == 1);

for i = 1:numel(finUmbral)                                                 % FALLO: REVISAR BIEN LO DEL UMBRAL
    errorEstim = vel(finUmbral(i)-1, :) /...
        (finUmbral(i) - inicioUmbral(i));
    enum = 1:(finUmbral(i) - inicioUmbral(i));
    error = [enum'*errorEstim(1) enum'*errorEstim(2) enum'*errorEstim(3)];
    errorVel(inicioUmbral(i):finUmbral(i)-1, :) = error;
end

% Eliminar el error de integración
vel = vel - errorVel;

% Integrar la velocidad para obtener la posición
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * dt;
end
% Aplica un filtro de paso alto para eliminar el error. Ésto hace que
% siempre que se quede quieto se desplace lentamente hacia el origen.

orden = 1;
frecCorte = 0.1;
[b, a] = butter(orden, (2*frecCorte)/(1/dt), 'high');
posHP = filtfilt(b, a, pos);

% Mantengo constantes los últimos segundos de la posición                  % FALLO: REVISAR BIEN ÚLTIMA POSICIÓN DE LOS DATOS
nrf=fix((0.1*t)/dt);

for i=(n-nrf):n
    posHP(i,:)=posHP((n-nrf),:);
end

save('medicion','tiempo','ax','ay','az','gx','gy','gz','ac_modFilt',...
'ac_umbral','vel','pos','posHP','quat','acel','dt','umbral')
guidata(hObject, handles);

function varargout = tiempocal_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to tiempocal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.tiempocal, 'String', fix(str2double(get...
    (handles.tiempocal,'String'))))
tiempocalib=str2double(get(handles.tiempocal,'String'));
if ((tiempocalib>=2) && (tiempocalib<=10))
    set(handles.tiempocalbar,'Value',fix(tiempocalib))
elseif tiempocalib<2
    set(handles.tiempocalbar,'Value',2)
    set(handles.tiempocal,'String',2)
elseif tiempocalib>10
    set(handles.tiempocalbar,'Value',10)
    set(handles.tiempocal,'String',10)
else
    errordlg({'Introduzca un tiempo válido entre 2 y 10 segundos'},...
        'Tiempo Inválido');
    set(handles.tiempocal,'String',get(handles.tiempocalbar,'Value'))
end
% Hints: get(hObject,'String') returns contents of tiempocal as text
%        str2double(get(hObject,'String')) returns contents of tiempocal as a double
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function varargout = tiempocal_CreateFcn(hObject, eventdata, handles,... 
    varargin)
% hObject    handle to tiempocal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function varargout = tiempocalbar_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to tiempocalbar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.tiempocal,'String',get(handles.tiempocalbar,'Value'))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function varargout = tiempocalbar_CreateFcn(hObject, eventdata, handles,...
    varargin)
% hObject    handle to tiempocalbar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function varargout = tiempomedir_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to tiempomedir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.tiempomedir, 'String', fix(str2double(get...
    (handles.tiempomedir,'String'))))
tiempomed=str2double(get(handles.tiempomedir,'String'));
if ((tiempomed>=20) && (tiempomed<=180))
    set(handles.tiempomedirbar,'Value',fix(tiempomed))
elseif tiempomed<20
    set(handles.tiempomedirbar,'Value',20)
    set(handles.tiempomedir,'String',20)
elseif tiempomed>180
    set(handles.tiempomedirbar,'Value',180)
    set(handles.tiempomedir,'String',180)
else
    errordlg({'Introduzca un tiempo válido entre 20 y 180 segundos'},...
        'Tiempo Inválido');
    set(handles.tiempomedir,'String',get(handles.tiempomedirbar,'Value'))
end
% Hints: get(hObject,'String') returns contents of tiempomedir as text
%        str2double(get(hObject,'String')) returns contents of tiempomedir as a double
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function varargout = tiempomedir_CreateFcn(hObject, eventdata, handles,...
    varargin)
% hObject    handle to tiempomedir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function varargout = tiempomedirbar_Callback(hObject, eventdata,...
    handles, varargin)
% hObject    handle to tiempomedirbar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.tiempomedir,'String',get(handles.tiempomedirbar,'Value'))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function varargout = tiempomedirbar_CreateFcn(hObject, eventdata,...
    handles, varargin)
% hObject    handle to tiempomedirbar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in grafcal.
function varargout = grafcal_Callback(hObject, eventdata, handles,...
    varargin)
% hObject    handle to grafcal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load('calibrado.mat')
figure('Position', [9 39 900 600], 'Number', 'off', 'Name',...
    'Calibrado del Sensor');
sensor(1) = subplot(2,1,1);
    hold on;
    plot(tiempocal, gx, 'r');
    plot(tiempocal, gy, 'g');
    plot(tiempocal, gz, 'b');
    title('Giroscopio');
    xlabel('Tiempo (s)');
    ylabel('Velocidad angular (rad/s)');
    legend('X', 'Y', 'Z');
    hold off;
sensor(2) = subplot(2,1,2);
    hold on;
    plot(tiempocal, ax, 'r');
    plot(tiempocal, ay, 'g');
    plot(tiempocal, az, 'b');
    plot(tiempocal, ac_modFilt, ':k');
    plot(tiempocal, ac_umbral, 'k', 'LineWidth', 2);
    title('Acelerómetro');
    xlabel('Tiempo (s)');
    ylabel('Aceleración (g)');
    legend('X', 'Y', 'Z', 'Acel. Filtrada', 'Umbral');
    hold off;
linkaxes(sensor,'x');
guidata(hObject, handles);



function fig = SixDOFanimation(varargin)

    % Create local variables

    % Required arguments
    p = varargin{1};                % position of body
    R = varargin{2};                % rotation matrix of body
    [numSamples dummy] = size(p);

    % Default values of optional arguments
    SamplePlotFreq = 1;
    Trail = 'Off';
    LimitRatio = 1;
    Position = [];
    FullScreen = false;
    View = [30 20];
    AxisLength = 1;
    ShowArrowHead = 'on';
    Xlabel = 'X';
    Ylabel = 'Y';
    Zlabel = 'Z';
    Title = '6DOF Animation';
    ShowLegend = true;
    CreateAVI = false;
    AVIfileName = '6DOF Animation';
    AVIfileNameEnum = true;
    AVIfps = 30;

    for i = 3:2:nargin
        if  strcmp(varargin{i}, 'SamplePlotFreq'), SamplePlotFreq = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Trail')
            Trail = varargin{i+1};
            if(~strcmp(Trail, 'Off') && ~strcmp(Trail, 'DotsOnly') && ~strcmp(Trail, 'All'))
                error('Invalid argument.  Trail must be ''Off'', ''DotsOnly'' or ''All''.');
            end
        elseif  strcmp(varargin{i}, 'LimitRatio'), LimitRatio = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Position'), Position = varargin{i+1};
        elseif  strcmp(varargin{i}, 'FullScreen'), FullScreen = varargin{i+1};
        elseif  strcmp(varargin{i}, 'View'), View = varargin{i+1};
        elseif  strcmp(varargin{i}, 'AxisLength'), AxisLength = varargin{i+1};
        elseif  strcmp(varargin{i}, 'ShowArrowHead'), ShowArrowHead = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Xlabel'), Xlabel = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Ylabel'), Ylabel = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Zlabel'), Zlabel = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Title'), Title = varargin{i+1};
        elseif  strcmp(varargin{i}, 'ShowLegend'), ShowLegend = varargin{i+1};
        elseif  strcmp(varargin{i}, 'CreateAVI'), CreateAVI = varargin{i+1};
        elseif  strcmp(varargin{i}, 'AVIfileName'), AVIfileName = varargin{i+1};
        elseif  strcmp(varargin{i}, 'AVIfileNameEnum'), AVIfileNameEnum = varargin{i+1};
        elseif  strcmp(varargin{i}, 'AVIfps'), AVIfps = varargin{i+1};
        else error('Invalid argument.');
        end
    end;

    % Reduce data to samples to plot only

    p = p(1:SamplePlotFreq:numSamples, :);
    R = R(:, :, 1:SamplePlotFreq:numSamples) * AxisLength;
    if(numel(View) > 2)
        View = View(1:SamplePlotFreq:numSamples, :);
    end
    [numPlotSamples dummy] = size(p);

    % Setup AVI file

    aviobj = [];                                                            	% create null object
    if(CreateAVI)
        fileName = strcat(AVIfileName, '.avi');
        if(exist(fileName, 'file'))
            if(AVIfileNameEnum)                                              	% if file name exists and enum enabled
                i = 0;
                while(exist(fileName, 'file'))                                  % find un-used file name by appending enum
                    fileName = strcat(AVIfileName, sprintf('%i', i), '.avi');
                    i = i + 1;
                end
            else                                                                % else file name exists and enum disabled
                fileName = [];                                                  % file will not be created
            end
        end
        if(isempty(fileName))
            sprintf('AVI file not created as file already exists.')
        else
            aviobj = avifile(fileName, 'fps', AVIfps, 'compression', 'Cinepak', 'quality', 100);
        end
    end

    % Setup figure and plot

    % Create figure
    fig = figure('Number', 'off', 'Name', '6DOF Animation');
    if(FullScreen)
        screenSize = get(0, 'ScreenSize');
        set(fig, 'Position', [0 0 screenSize(3) screenSize(4)]);
    elseif(~isempty(Position))
        set(fig, 'Position', Position);
    end
    set(gca, 'drawmode', 'fast');
    lighting phong;
    set(gcf, 'Renderer', 'zbuffer');
    hold on;
    axis equal;
    grid on;
    view(View(1, 1), View(1, 2));
    title(i);
    xlabel(Xlabel);
    ylabel(Ylabel);
    zlabel(Zlabel);

    % Create plot data arrays
    if(strcmp(Trail, 'DotsOnly') || strcmp(Trail, 'All'))
        x = zeros(numPlotSamples, 1);
        y = zeros(numPlotSamples, 1);
        z = zeros(numPlotSamples, 1);
    end
    if(strcmp(Trail, 'All'))
        ox = zeros(numPlotSamples, 1);
        oy = zeros(numPlotSamples, 1);
        oz = zeros(numPlotSamples, 1);
        ux = zeros(numPlotSamples, 1);
        vx = zeros(numPlotSamples, 1);
        wx = zeros(numPlotSamples, 1);
        uy = zeros(numPlotSamples, 1);
        vy = zeros(numPlotSamples, 1);
        wy = zeros(numPlotSamples, 1);
        uz = zeros(numPlotSamples, 1);
        vz = zeros(numPlotSamples, 1);
        wz = zeros(numPlotSamples, 1);
    end
    x(1) = p(1,1);
    y(1) = p(1,2);
    z(1) = p(1,3);
    ox(1) = x(1);
    oy(1) = y(1);
    oz(1) = z(1);
    ux(1) = R(1,1,1:1);
    vx(1) = R(2,1,1:1);
    wx(1) = R(3,1,1:1);
    uy(1) = R(1,2,1:1);
    vy(1) = R(2,2,1:1);
    wy(1) = R(3,2,1:1);
    uz(1) = R(1,3,1:1);
    vz(1) = R(2,3,1:1);
    wz(1) = R(3,3,1:1);

    % Create graphics handles
    orgHandle = plot3(x, y, z, 'k.');
    if(ShowArrowHead)
        ShowArrowHeadStr = 'on';
    else
        ShowArrowHeadStr = 'off';
    end
    quivXhandle = quiver3(ox, oy, oz, ux, vx, wx,  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivYhandle = quiver3(ox, oy, oz, uy, vy, wy,  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivZhandle = quiver3(ox, ox, oz, uz, vz, wz,  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

    % Create legend
    if(ShowLegend)
        legend('Origin', 'X', 'Y', 'Z');
    end
    
    % Set initial limits
    Xlim = [x(1)-AxisLength x(1)+AxisLength] * LimitRatio;
    Ylim = [y(1)-AxisLength y(1)+AxisLength] * LimitRatio;
    Zlim = [z(1)-AxisLength z(1)+AxisLength] * LimitRatio;
    set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim);
    
    % Set initial view
    view(View(1, :));

    % Plot one sample at a time

    for i = 1:numPlotSamples

        % Update graph title
        if(strcmp(Title, ''))
            titleText = sprintf('Sample %i of %i', 1+((i-1)*SamplePlotFreq), numSamples);
        else
            titleText = strcat(Title, ' (', sprintf('Sample %i of %i', 1+((i-1)*SamplePlotFreq), numSamples), ')');
        end
        title(titleText);

        % Plot body x y z axes
        if(strcmp(Trail, 'DotsOnly') || strcmp(Trail, 'All'))
            x(1:i) = p(1:i,1);
            y(1:i) = p(1:i,2);
            z(1:i) = p(1:i,3);
        else
            x = p(i,1);
            y = p(i,2);
            z = p(i,3);
        end
        if(strcmp(Trail, 'All'))
            ox(1:i) = p(1:i,1);
            oy(1:i) = p(1:i,2);
            oz(1:i) = p(1:i,3);
            ux(1:i) = R(1,1,1:i);
            vx(1:i) = R(2,1,1:i);
            wx(1:i) = R(3,1,1:i);
            uy(1:i) = R(1,2,1:i);
            vy(1:i) = R(2,2,1:i);
            wy(1:i) = R(3,2,1:i);
            uz(1:i) = R(1,3,1:i);
            vz(1:i) = R(2,3,1:i);
            wz(1:i) = R(3,3,1:i);
        else
            ox = p(i,1);
            oy = p(i,2);
            oz = p(i,3);
            ux = R(1,1,i);
            vx = R(2,1,i);
            wx = R(3,1,i);
            uy = R(1,2,i);
            vy = R(2,2,i);
            wy = R(3,2,i);
            uz = R(1,3,i);
            vz = R(2,3,i);
            wz = R(3,3,i);
        end
        set(orgHandle, 'xdata', x, 'ydata', y, 'zdata', z);
        set(quivXhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
        set(quivYhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
        set(quivZhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);

        % Adjust axes for snug fit and draw
        axisLimChanged = false;
        if((p(i,1) - AxisLength) < Xlim(1)), Xlim(1) = p(i,1) - LimitRatio*AxisLength; axisLimChanged = true; end
        if((p(i,2) - AxisLength) < Ylim(1)), Ylim(1) = p(i,2) - LimitRatio*AxisLength; axisLimChanged = true; end
        if((p(i,3) - AxisLength) < Zlim(1)), Zlim(1) = p(i,3) - LimitRatio*AxisLength; axisLimChanged = true; end
        if((p(i,1) + AxisLength) > Xlim(2)), Xlim(2) = p(i,1) + LimitRatio*AxisLength; axisLimChanged = true; end
        if((p(i,2) + AxisLength) > Ylim(2)), Ylim(2) = p(i,2) + LimitRatio*AxisLength; axisLimChanged = true; end
        if((p(i,3) + AxisLength) > Zlim(2)), Zlim(2) = p(i,3) + LimitRatio*AxisLength; axisLimChanged = true; end
        if(axisLimChanged), set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim); end
        drawnow;

        % Adjust view
        if(numel(View) > 2)
            view(View(i, :));
        end

        % Add frame to AVI object
        if(~isempty(aviobj))
            frame = getframe(fig);
            aviobj = addframe(aviobj, frame);
        end

    end

    hold off;

    % Close AVI file
    if(~isempty(aviobj))
        aviobj = close(aviobj);
    end


% --- Executes on button press in cerrarpuerto.
function cerrarpuerto_Callback(hObject, eventdata, handles)
% hObject    handle to cerrarpuerto (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(instrfind)
    tic
    while (toc<=3)
        fprintf(instrfind,'%c','b');
    end
    fclose(instrfind);
    delete(instrfind);
    disp ('SE HA FORZADO EL CIERRE DEL PUERTO SERIE')
end

function q = axisAngle2quatern(axis, angle)
    q0 = cos(angle./2);
    q1 = -axis(:,1)*sin(angle./2);
    q2 = -axis(:,2)*sin(angle./2);
    q3 = -axis(:,3)*sin(angle./2); 
    q = [q0 q1 q2 q3];

function R = axisAngle2rotMat(axis, angle)
    kx = axis(:,1);
    ky = axis(:,2);
    kz = axis(:,3);
    cT = cos(angle);
    sT = sin(angle);
    vT = 1 - cos(angle);
    
    R(1,1,:) = kx.*kx.*vT + cT;
    R(1,2,:) = kx.*ky.*vT - kz.*sT;
    R(1,3,:) = kx.*kz.*vT + ky.*sT;
    
    R(2,1,:) = kx.*ky.*vT + kz.*sT;
    R(2,2,:) = ky.*ky.*vT + cT;
    R(2,3,:) = ky.*kz.*vT - kx.*sT;
    
    R(3,1,:) = kx.*kz.*vT - ky.*sT;
    R(3,2,:) = ky.*kz.*vT + kx.*sT;
    R(3,3,:) = kz.*kz.*vT + cT;

function R = euler2rotMat(phi, theta, psi)
    R(1,1,:) = cos(psi).*cos(theta);
    R(1,2,:) = -sin(psi).*cos(phi) + cos(psi).*sin(theta).*sin(phi);
    R(1,3,:) = sin(psi).*sin(phi) + cos(psi).*sin(theta).*cos(phi);
    
    R(2,1,:) = sin(psi).*cos(theta);
    R(2,2,:) = cos(psi).*cos(phi) + sin(psi).*sin(theta).*sin(phi);
    R(2,3,:) = -cos(psi).*sin(phi) + sin(psi).*sin(theta).*cos(phi);
    
    R(3,1,:) = -sin(theta);
    R(3,2,:) = cos(theta).*sin(phi);
    R(3,3,:) = cos(theta).*cos(phi);

function euler = quatern2euler(q)
    % from paper: "Adaptive Filter for a Miniature MEMS Based Attitude and
    % Heading Reference System" by Wang et al, IEEE.
    
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;    
    
    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );    
    psi = atan2(R(2,1,:), R(1,1,:) );

    euler = [phi(1,:)' theta(1,:)' psi(1,:)']; 

function R = quatern2rotMat(q)
    [rows cols] = size(q);
    R = zeros(3,3, rows);
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(1,2,:) = 2.*(q(:,2).*q(:,3)+q(:,1).*q(:,4));
    R(1,3,:) = 2.*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(2,2,:) = 2.*q(:,1).^2-1+2.*q(:,3).^2;
    R(2,3,:) = 2.*(q(:,3).*q(:,4)+q(:,1).*q(:,2));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;

function qConj = quaternConj(q)
    qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];


function ab = quaternProd(a, b)
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);


function v = quaternRotate(v, q)
    [row col] = size(v);
    v0XYZ = quaternProd(quaternProd(q, [zeros(row, 1) v]), quaternConj(q));
    v = v0XYZ(:, 2:4);


function euler = rotMat2euler(R)
    % from paper: "Adaptive Filter for a Miniature MEMS Based Attitude and
    % Heading Reference System" by Wang et al, IEEE.
    
    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );    
    psi = atan2(R(2,1,:), R(1,1,:) );

    euler = [phi(1,:)' theta(1,:)' psi(1,:)'];  


function q = rotMat2quatern(R)  
    % wiki URL: http://en.wikipeRia.org/wiki/Quaternions_anR_spatial_rotation#Fitting_quaternions
    % paper URL: http://www.aiaa.org/content.cfm?pageiR=406&gTable=japaperimport&gIR=4654 
    [row col numR] = size(R);
    q = zeros(numR, 4);
	K = zeros(4,4);    
    for i = 1:numR
        K(1,1) = (1/3) * (R(1,1,i) - R(2,2,i) - R(3,3,i));
        K(1,2) = (1/3) * (R(2,1,i) + R(1,2,i));
        K(1,3) = (1/3) * (R(3,1,i) + R(1,3,i));
        K(1,4) = (1/3) * (R(2,3,i) - R(3,2,i));  
        K(2,1) = (1/3) * (R(2,1,i) + R(1,2,i));
        K(2,2) = (1/3) * (R(2,2,i) - R(1,1,i) - R(3,3,i));
        K(2,3) = (1/3) * (R(3,2,i) + R(2,3,i));
        K(2,4) = (1/3) * (R(3,1,i) - R(1,3,i));   
        K(3,1) = (1/3) * (R(3,1,i) + R(1,3,i));
        K(3,2) = (1/3) * (R(3,2,i) + R(2,3,i));
        K(3,3) = (1/3) * (R(3,3,i) - R(1,1,i) - R(2,2,i));
        K(3,4) = (1/3) * (R(1,2,i) - R(2,1,i));    
        K(4,1) = (1/3) * (R(2,3,i) - R(3,2,i));
        K(4,2) = (1/3) * (R(3,1,i) - R(1,3,i));
        K(4,3) = (1/3) * (R(1,2,i) - R(2,1,i));
        K(4,4) = (1/3) * (R(1,1,i) + R(2,2,i) + R(3,3,i)); 
        [V,D] = eig(K);
        %p = find(max(D));
        %q = V(:,p)';    
        q(i,:) = V(:,4)';
        q(i,:) = [q(i,4) q(i,1) q(i,2) q(i,3)];
    end
