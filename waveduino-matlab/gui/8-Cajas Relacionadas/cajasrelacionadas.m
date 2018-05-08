function varargout = cajasrelacionadas(varargin)
% CAJASRELACIONADAS MATLAB code for cajasrelacionadas.fig
%      CAJASRELACIONADAS, by itself, creates a new CAJASRELACIONADAS or raises the existing
%      singleton*.
%
%      H = CAJASRELACIONADAS returns the handle to a new CAJASRELACIONADAS or the handle to
%      the existing singleton*.
%
%      CAJASRELACIONADAS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CAJASRELACIONADAS.M with the given input arguments.
%
%      CAJASRELACIONADAS('Property','Value',...) creates a new CAJASRELACIONADAS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before cajasrelacionadas_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to cajasrelacionadas_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help cajasrelacionadas

% Last Modified by GUIDE v2.5 28-Mar-2014 19:57:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @cajasrelacionadas_OpeningFcn, ...
                   'gui_OutputFcn',  @cajasrelacionadas_OutputFcn, ...
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


% --- Executes just before cajasrelacionadas is made visible.
function cajasrelacionadas_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to cajasrelacionadas (see VARARGIN)
set(handles.lista1,'String',[10 20 30 40 50 60]);
set(handles.lista2,'String',[db(10,'Power') db(20,'Power')...
db(30,'Power') db(40,'Power') db(50,'Power') db(60,'Power')]);
% Choose default command line output for cajasrelacionadas
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes cajasrelacionadas wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = cajasrelacionadas_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in lista1.
function lista1_Callback(hObject, eventdata, handles)
% hObject    handle to lista1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns lista1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lista1
a=get(hObject,'Value');
set(handles.lista2,'Value',a);

% --- Executes during object creation, after setting all properties.
function lista1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lista1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in lista2.
function lista2_Callback(hObject, eventdata, handles)
% hObject    handle to lista2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns lista2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lista2


% --- Executes during object creation, after setting all properties.
function lista2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lista2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
