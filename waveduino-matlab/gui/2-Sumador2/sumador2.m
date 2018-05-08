function varargout = sumador2(varargin)
% SUMADOR2 MATLAB code for sumador2.fig
%      SUMADOR2, by itself, creates a new SUMADOR2 or raises the existing
%      singleton*.
%
%      H = SUMADOR2 returns the handle to a new SUMADOR2 or the handle to
%      the existing singleton*.
%
%      SUMADOR2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SUMADOR2.M with the given input arguments.
%
%      SUMADOR2('Property','Value',...) creates a new SUMADOR2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before sumador2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to sumador2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help sumador2

% Last Modified by GUIDE v2.5 28-Mar-2014 16:05:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @sumador2_OpeningFcn, ...
                   'gui_OutputFcn',  @sumador2_OutputFcn, ...
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


% --- Executes just before sumador2 is made visible.
function sumador2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to sumador2 (see VARARGIN)

% Choose default command line output for sumador2
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes sumador2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = sumador2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function sumando1_Callback(hObject, eventdata, handles)
% hObject    handle to sumando1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sumando1 as text
%        str2double(get(hObject,'String')) returns contents of sumando1 as a double
sum1=str2double(get(hObject,'String'));
sum2=str2double(get(handles.sumando2,'String'));
if isnan(sum1)
errordlg('El valor debe ser numérico','ERROR')
set(handles.sumando1,'String',0);
sum1=0;
end
if isnan(sum2)
errordlg('El valor debe ser numérico','ERROR')
set(handles.sumando2,'String',0);
sum2=0;
end
suma=sum1+sum2;
set(handles.resultado,'String',suma);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function sumando1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sumando1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sumando2_Callback(hObject, eventdata, handles)
% hObject    handle to sumando2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sumando2 as text
%        str2double(get(hObject,'String')) returns contents of sumando2 as a double
sum2=str2double(get(hObject,'String'));
sum1=str2double(get(handles.sumando1,'String'));
if isnan(sum1)
errordlg('El valor debe ser numérico','ERROR')
set(handles.sumando1,'String',0);
sum1=0;
end
if isnan(sum2)
errordlg('El valor debe ser numérico','ERROR')
set(handles.sumando2,'String',0);
sum2=0;
end
suma=sum1+sum2;
set(handles.resultado,'String',suma);
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function sumando2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sumando2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
