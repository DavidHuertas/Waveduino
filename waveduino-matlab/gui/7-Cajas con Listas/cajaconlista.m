function varargout = cajaconlista(varargin)
% CAJACONLISTA MATLAB code for cajaconlista.fig
%      CAJACONLISTA, by itself, creates a new CAJACONLISTA or raises the existing
%      singleton*.
%
%      H = CAJACONLISTA returns the handle to a new CAJACONLISTA or the handle to
%      the existing singleton*.
%
%      CAJACONLISTA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CAJACONLISTA.M with the given input arguments.
%
%      CAJACONLISTA('Property','Value',...) creates a new CAJACONLISTA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before cajaconlista_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to cajaconlista_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help cajaconlista

% Last Modified by GUIDE v2.5 28-Mar-2014 19:34:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @cajaconlista_OpeningFcn, ...
                   'gui_OutputFcn',  @cajaconlista_OutputFcn, ...
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


% --- Executes just before cajaconlista is made visible.
function cajaconlista_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to cajaconlista (see VARARGIN)
set(handles.text2,'string','CAJA CON LISTA');
set(handles.txt1,'string',...
['Grupo británico. Inicio su carrera en 1973.',...
' Último álbum: *Angel of Retribution (2005)*.']);
% Choose default command line output for cajaconlista
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes cajaconlista wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = cajaconlista_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1
inf=get(hObject,'Value');
gos=get(hObject,'String');
switch inf
case 1
set(handles.txt1,'string',...
['Grupo británico. Inicio su carrera en 1973.',...
' Último album: *Angel of Retribution (2005)*.']);
case 2
set(handles.txt1,'string',...
['Grupo británico. Inicio su carrera en 1979.',...
' Último album: *Dance of death (2005)*.']);
case 3
set(handles.txt1,'string',...
['Grupo español. Inicio su carrera en 1999.',...
' Último album: *Agotarás (2004)*.']);
case 4
set(handles.txt1,'string',...
['Grupo estadounidense. Inicio su carrera en 1990.',...
' Último album: *Louder than hell (2005)*.']);
case 5
set(handles.txt1,'string',...
['Solista británico. Inicio su carrera en 1971.',...
' Último album: *Ozzfest (2006)*.']);
case 6
set(handles.txt1,'string',...
['Grupo ecuatoriano. Inicio su carrera en 1995.',...
' Último album: *Prisionero del tiempo (2005)*.']);
case 7
set(handles.txt1,'string',...
['Grupo estadounidense. Inicio su carrera en 1995.',...
' Último album: *Prisionero del tiempo (2005)*.']);
case 8
set(handles.txt1,'string',...
['Grupo argentino. Inicio su carrera en 1989.',...
' Último album: *El camino del fuego (1994)*.']);
case 9
set(handles.txt1,'string',...
['Grupo alemán. Inicio su carrera en 1983.',...
' Último album: *Keeper OSK III (2005)*.']);
case 10
set(handles.txt1,'string',...
['Grupo estadounidense. Inicio su carrera en 1983.',...
' Primer album: *Kill them all (1983)*.']);
case 11
set(handles.txt1,'string',...
['Grupo finlandés. Inicio su carrera en 1984.',...
' Último album: *Infinity (2005)*.']);
case 12
set(handles.txt1,'string',...
['Grupo alemán. Inicio su carrera en 1973.',...
' Último album: *Eye by eye (2005)*.']);
end
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
