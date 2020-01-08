function varargout = AMDSBSSB(varargin)
% AMDSBSSB MATLAB code for AMDSBSSB.fig
%      AMDSBSSB, by itself, creates a new AMDSBSSB or raises the existing
%      singleton*.
%
%      H = AMDSBSSB returns the handle to a new AMDSBSSB or the handle to
%      the existing singleton*.
%
%      AMDSBSSB('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in AMDSBSSB.M with the given input arguments.
%
%      AMDSBSSB('Property','Value',...) creates a new AMDSBSSB or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AMDSBSSB_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AMDSBSSB_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AMDSBSSB

% Last Modified by GUIDE v2.5 05-Jan-2020 16:21:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AMDSBSSB_OpeningFcn, ...
                   'gui_OutputFcn',  @AMDSBSSB_OutputFcn, ...
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


% --- Executes just before AMDSBSSB is made visible.
function AMDSBSSB_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AMDSBSSB (see VARARGIN)

% Choose default command line output for AMDSBSSB
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes AMDSBSSB wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = AMDSBSSB_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function fc_Callback(hObject, eventdata, handles)
% hObject    handle to fc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fc as text
%        str2double(get(hObject,'String')) returns contents of fc as a double


% --- Executes during object creation, after setting all properties.
function fc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function A_Callback(hObject, eventdata, handles)
% hObject    handle to A (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of A as text
%        str2double(get(hObject,'String')) returns contents of A as a double


% --- Executes during object creation, after setting all properties.
function A_CreateFcn(hObject, eventdata, handles)
% hObject    handle to A (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear Fc;
clear Ac;
clear m;
clear c;
clear s;
clear y;
clear a1 b1;
Fs = 800000;           %sample freq
tmin = 0;                %initial time
tmax = 0.001;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%amplitude setting
a1=get(handles.fc,'string');
b1= get(handles.A,'string');
Ac= b1 - 48;
Fc= (a1 - 48)*10000;
Am = 1;                  %msg amplitude               %carrier amplitude
Fm = 2000;


%------------------------------------------------------------
%generate msg & carrier & modulated signals
m = Am*cos(2*pi*Fm*t);   %msg
c = Ac*cos(2*pi*Fc*t);   %carrier
s = (1+m/Ac).*c;         %AM modulated signal
%demodulated------------------------------------------------
c1 = (Ac + m).*cos(4*pi*Fc*t)/2
y1 = (s.*c/Ac)-c1;
y = 2*y1-(Ac/2);
%plot-----------------------------------------------------
axes(handles.axes1)
plot(t,y);
xlabel ('time');
ylabel ('amplitude');
title('AM-demodulated signal');
%spectrum----------------------------------------------------
g1 = fft(y);
g2 = abs(g1);
G=(g2.^2)/(0.0005);
%plot-------------------------------------------------------
axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear Fc;
clear Ac;
clear m;
clear c;
clear s;
clear a1 b1;
Fs = 800000;             %sample freq
tmin = 0;                %initial time
tmax = 0.001;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%amplitude setting
a1=get(handles.fc,'string');
b1= get(handles.A,'string');
Ac= b1 - 48;
Fc= (a1 - 48)*10000;
Am = 1;                  %msg amplitude               %carrier amplitude
Fm = 2000;

%------------------------------------------------------------
%generate msg & carrier & modulated signals
m = Am*cos(2*pi*Fm*t);   %msg
c = Ac*cos(2*pi*Fc*t);   %carrier
s = m.*c;                %DSB-SC modulated signal
%------------------------------------------------------------
%demodulation of DSB-SC
x = s.*c;
Wn = Fc/(Fs/2);             % Normalozed cutoff frequency
[b a] = butter(5,Wn,'low'); %5th order low-pass filter
y = filter(b,a,x);
%plot--------------------------------------------------------
axes(handles.axes1)
plot(t,y);
xlabel ('time');
ylabel ('amplitude');
title('DSB-SC demodulated signal');
%spectrum-------------------------------------------------------
g1 = fft(y);
g2 = abs(g1);
G=(g2.^2)/(0.0005);
%plot------------------------------------------------------
axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear Fc;
clear Ac;
clear m;
clear c;
clear s;
clear a1 b1;
Fs = 800000;             %sample freq
tmin = 0;                %initial time
tmax = 0.001;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%amplitude setting
a1=get(handles.fc,'string');
b1= get(handles.A,'string');
Ac= b1 - 48;
Fc= (a1 - 48)*10000;
Am = 1;                  %msg amplitude               %carrier amplitude
Fm = 2000;

%------------------------------------------------------------
%generate msg & carrier & modulated signals
m = Am*cos(2*pi*Fm*t);       %msg
mh = Am*cos(2*pi*Fm*t+90);   %helbert transform of msg
c1 = Ac*cos(2*pi*Fc*t);      %carrier (cos)
c2 = Ac*sin(2*pi*Fc*t);      %carrier (sin)
%LSSB) modulated signal
s1 = m.*c1/Ac; 
s2 = mh.*c2/Ac;
s = s1+s2;
%------------------------------------------------------------
%demodulation of SSB(LSB)
x = c1.*s;
Wn = Fc/(Fs/2);             % Normalozed cutoff frequency
[b a] = butter(5,Wn,'low'); %5th order low-pass filter
y = filter(b,a,x);
%------------------------------------------------------------
axes(handles.axes1)
plot(t,y);
xlabel ('time');
ylabel ('amplitude');
title('LSSB-SC demodulated signal');

g1 = fft(y);
g2 = abs(g1);
G=(g2.^2)/(0.0005);

axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear Fc;
clear Ac;
clear m;
clear c;
clear s;
clear a1 b1;
Fs = 800000;             %sample freq
tmin = 0;                %initial time
tmax = 0.001;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%amplitude setting
a1=get(handles.fc,'string');
b1= get(handles.A,'string');
Ac= b1 - 48;
Fc= (a1 - 48)*10000;
Am = 1;                  %msg amplitude               %carrier amplitude
Fm = 2000;

%------------------------------------------------------------
%generate msg & carrier & modulated signals
m = Am*cos(2*pi*Fm*t);       %msg
mh = Am*cos(2*pi*Fm*t+90);   %helbert transform of msg
c1 = Ac*cos(2*pi*Fc*t);      %carrier (cos)
c2 = Ac*sin(2*pi*Fc*t);      %carrier (sin)
%SSB(USB) modulated signal
s1 = m.*c1/Ac; 
s2 = mh.*c2/Ac;
s = s1-s2;
%------------------------------------------------------------
%demodulation of SSB(USB)
x = c1.*s;
Wn = Fc/(Fs/2);             % Normalozed cutoff frequency
[b a] = butter(5,Wn,'low'); %5th order low-pass filter
y = filter(b,a,x);
%------------------------------------------------------------
axes(handles.axes1)
plot(t,y);
xlabel ('time');
ylabel ('amplitude');
title('USSB-SC demodulated signal');

g1 = fft(y);
g2 = abs(g1);
G=(g2.^2)/(0.0005);

axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear m;
Fs = 800000;           %sample freq
tmin = 0;                %initial time
tmax = 0.01;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%----------------------------------------------------------
Am = 1;                  %msg signal    
Fm = 2000;
%-----------------------------------------------------------
m = Am*cos(2*pi*Fm*t);   %msg
%------------------------------------------------------------
%plot axes1--------------------------------------------------
axes(handles.axes1)
plot(t,m);
xlabel ('time');
ylabel ('amplitude');
title('msg signal');
%----------------------------------------------------------
%spectrum--------------------------------------------------
g1 = fft(m);
g2 = abs(g1);
G=(g2.^2)/(0.0005);
%plot axes2------------------------------------------------
axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear Fc;
clear Ac;
clear c;
clear a1 b1;
Fs = 800000;           %sample freq
tmin = 0;                %initial time
tmax = 0.01;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%amplitude setting
a1=get(handles.fc,'string');
b1= get(handles.A,'string');
Ac=b1-48;
Fc=(a1-48)*1000;
%------------------------------------------------------------
c = Ac*cos(2*pi*Fc*t);   %carrier
%------------------------------------------------------------
%plot axes1------------------------------------------------
axes(handles.axes1)
plot(t,c);
xlabel ('time');
ylabel ('amplitude');
title('carrier signal');
%spectrum------------------------------------------------
g1 = fft(c);
g2 = abs(g1);
G=(g2.^2)/(1/Fc);
%plot axes2------------------------------------------------
axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear Fc;
clear Ac;
clear m;
clear c;
clear s;
clear a1 b1;
Fs = 800000;           %sample freq
tmin = 0;                %initial time
tmax = 0.001;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%amplitude setting
a1=get(handles.fc,'string');
b1= get(handles.A,'string');
Ac= b1 - 48;
Fc= (a1 - 48)*10000;     %carrier amplitudeaa
Am = 1;                  %msg amplitude     
Fm = 2000;
%------------------------------------------------------------
%generate msg & carrier & modulated signals
m = Am*cos(2*pi*Fm*t);   %msg
c = Ac*cos(2*pi*Fc*t);   %carrier
s = (1+m/Ac).*c;         %AM modulated signal
%------------------------------------------------------------
%plot axes1-----------------------------------------------------
axes(handles.axes1)
plot(t,s); hold on;
plot (t,Ac*(1+m/Ac),'r:'); hold on;
plot (t,-Ac*(1+m/Ac),'r:'); hold off;
xlabel ('time');
ylabel ('amplitude');
title('AM-modulated signal');
%spectrum-----------------------------------------------------
g1 = fft(s);
g2 = abs(g1);
G=(g2.^2)/(0.0005);
%plot axes2-----------------------------------------------------
axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');



% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%DSB-SC modulated signal
%time setting
clear Fc;
clear Ac;
clear m;
clear c;
clear s;
clear a1 b1;
Fs = 800000;             %sample freq
tmin = 0;                %initial time
tmax = 0.001;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%amplitude setting
a1=get(handles.fc,'string');
b1= get(handles.A,'string');
Ac= b1 - 48;
Fc= (a1 - 48)*10000;     %carrier amplitude
Am = 1;                  %msg amplitude               
Fm = 2000;
%------------------------------------------------------------
m = Am*cos(2*pi*Fm*t);   %msg
c = Ac*cos(2*pi*Fc*t);   %carrier
s = m.*c/Ac;             %DSB-SC modulated signal
%------------------------------------------------------------
%plot axes1------------------------------------------------------
axes(handles.axes1)
plot(t,s); hold on;
plot (t,m,'r:'); hold on;
plot (t,-m,'r:'); hold off;
xlabel ('time');
ylabel ('amplitude');
title('DSB-SC modulated signal');
%spectrum------------------------------------------------------
g1 = fft(s);
g2 = abs(g1);
G=(g2.^2)/(0.00025);
%plot axes2------------------------------------------------------
axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear Fc;
clear Ac;
clear m;
clear c;
clear s;
clear a1 b1;
Fs = 800000;             %sample freq
tmin = 0;                %initial time
tmax = 0.001;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%amplitude setting
a1=get(handles.fc,'string');
b1= get(handles.A,'string');
Ac= b1 - 48;
Fc= (a1 - 48)*10000;     %carrier amplitude
Am = 1;                  %msg amplitude              
Fm = 2000;

%------------------------------------------------------------
m = Am*cos(2*pi*Fm*t);       %msg
mh = Am*cos(2*pi*Fm*t+90);   %helbert transform of msg
c1 = Ac*cos(2*pi*Fc*t);      %carrier (cos)
c2 = Ac*sin(2*pi*Fc*t);      %carrier (sin)
% modulated signal
s1 = m.*c1/Ac; 
s2 = mh.*c2/Ac;
s = s1+s2;                  %LSSB modulated signal
%------------------------------------------------------------
%plot axes1-----------------------------------------------------
axes(handles.axes1)
plot(t,s); 
xlabel ('time');
ylabel ('amplitude');
title('SSB(LSSB)-modulated signal');
%spectrum-----------------------------------------------------
g1 = fft(s);
g2 = abs(g1);
G=(g2.^2)/(0.00025);
%plot axes2-----------------------------------------------------
axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clear Fc;
clear Ac;
clear m;
clear c;
clear s;
clear a1 b1;
Fs = 800000;             %sample freq
tmin = 0;                %initial time
tmax = 0.001;            %max time
step = 1/Fs;             %sample time
t = tmin:step:tmax;      %time periode of signal
%amplitude setting
a1=get(handles.fc,'string');
b1= get(handles.A,'string');
Ac= b1 - 48;
Fc= (a1 - 48)*10000;
Am = 1;                  %msg amplitude               %carrier amplitude
Fm = 2000;

%------------------------------------------------------------
%generate msg & carrier & modulated signals
m = Am*cos(2*pi*Fm*t);       %msg
mh = Am*cos(2*pi*Fm*t+90);   %helbert transform of msg
c1 = Ac*cos(2*pi*Fc*t);      %carrier (cos)
c2 = Ac*sin(2*pi*Fc*t);      %carrier (sin)
%SSB(USB) modulated signal
s1 = m.*c1/Ac; 
s2 = mh.*c2/Ac;
s = s1-s2;

axes(handles.axes1)
plot(t,s); 
xlabel ('time');
ylabel ('amplitude');
title('SSB(USSB)-modulated signal');

g1 = fft(s);
g2 = abs(g1);
G=(g2.^2)/(0.00025);

axes(handles.axes2)
plot(t,G);
xlabel ('omega');
ylabel ('spectrum');
title('Spectrogram');
