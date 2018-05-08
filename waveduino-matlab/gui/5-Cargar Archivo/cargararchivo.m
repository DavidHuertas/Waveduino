function varargout = cargararchivo(varargin)
% CARGARARCHIVO MATLAB code for cargararchivo.fig
%      CARGARARCHIVO, by itself, creates a new CARGARARCHIVO or raises the existing
%      singleton*.
%
%      H = CARGARARCHIVO returns the handle to a new CARGARARCHIVO or the handle to
%      the existing singleton*.
%
%      CARGARARCHIVO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CARGARARCHIVO.M with the given input arguments.
%
%      CARGARARCHIVO('Property','Value',...) creates a new CARGARARCHIVO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before cargararchivo_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to cargararchivo_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help cargararchivo

% Last Modified by GUIDE v2.5 28-Mar-2014 18:36:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @cargararchivo_OpeningFcn, ...
                   'gui_OutputFcn',  @cargararchivo_OutputFcn, ...
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


% --- Executes just before cargararchivo is made visible.
function cargararchivo_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to cargararchivo (see VARARGIN)

% Choose default command line output for cargararchivo
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes cargararchivo wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = cargararchivo_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in imagen.
function imagen_Callback(hObject, eventdata, handles)
% hObject    handle to imagen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName Path]=uigetfile({'*.jpg;*.bmp'},'Abrir Imagen');
if isequal(FileName,0)
return
else
a=imread(strcat(Path,FileName));
imshow(a);
end
handles.direccion=strcat(Path,FileName);
guidata(hObject,handles)


% --- Executes on button press in doc.
function doc_Callback(hObject, eventdata, handles)
% hObject    handle to doc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName Path]=uigetfile({'*.docx;*.xlsx'},'Abrir documento');
if isequal(FileName,0)
return
else
winopen(strcat(Path,FileName));
end

% --- Executes on button press in programa.
function programa_Callback(hObject, eventdata, handles)
% hObject    handle to programa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName Path]=uigetfile({'*.exe'},'Abrir documento');
if isequal(FileName,0)
return
else
winopen(strcat(Path,FileName));
end
