function varargout = mensajes(varargin)
% MENSAJES MATLAB code for mensajes.fig
%      MENSAJES, by itself, creates a new MENSAJES or raises the existing
%      singleton*.
%
%      H = MENSAJES returns the handle to a new MENSAJES or the handle to
%      the existing singleton*.
%
%      MENSAJES('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MENSAJES.M with the given input arguments.
%
%      MENSAJES('Property','Value',...) creates a new MENSAJES or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mensajes_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mensajes_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mensajes

% Last Modified by GUIDE v2.5 28-Mar-2014 17:42:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mensajes_OpeningFcn, ...
                   'gui_OutputFcn',  @mensajes_OutputFcn, ...
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


% --- Executes just before mensajes is made visible.
function mensajes_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mensajes (see VARARGIN)

% Choose default command line output for mensajes
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mensajes wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mensajes_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in aviso.
function aviso_Callback(hObject, eventdata, handles)
% hObject    handle to aviso (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
warndlg('Esto es un aviso','Aviso');

% --- Executes on button press in error.
function error_Callback(hObject, eventdata, handles)
% hObject    handle to error (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
errordlg({'Error con ejemplo','de salto de línea'},' Error ');

% --- Executes on button press in ayuda.
function ayuda_Callback(hObject, eventdata, handles)
% hObject    handle to ayuda (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
helpdlg('Esto es una ayuda',' Ayuda ');

% --- Executes on button press in informacion.
function informacion_Callback(hObject, eventdata, handles)
% hObject    handle to informacion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
msgbox('Esto es un cuadro de información',' Información ');

% --- Executes on button press in pregunta.
function pregunta_Callback(hObject, eventdata, handles)
% hObject    handle to pregunta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
opc=questdlg('¿Desea salir del programa?','SALIR','Si','No','No');
if strcmp(opc,'No')
return;
end
clear,clc,close all

% --- Executes on button press in dato.
function dato_Callback(hObject, eventdata, handles)
% hObject    handle to dato (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
inputdlg('DATO','Encabezado');