function varargout = Simple_UWB(varargin)
% SIMPLE_UWB MATLAB code for Simple_UWB.fig
%      SIMPLE_UWB, by itself, creates a new SIMPLE_UWB or raises the existing
%      singleton*.
%
%      H = SIMPLE_UWB returns the handle to a new SIMPLE_UWB or the handle to
%      the existing singleton*.
%
%      SIMPLE_UWB('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMPLE_UWB.M with the given input arguments.
%
%      SIMPLE_UWB('Property','Value',...) creates a new SIMPLE_UWB or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Simple_UWB_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Simple_UWB_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Simple_UWB

% Last Modified by GUIDE v2.5 28-Nov-2016 12:28:04

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Simple_UWB_OpeningFcn, ...
                   'gui_OutputFcn',  @Simple_UWB_OutputFcn, ...
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


% --- Executes just before Simple_UWB is made visible.
function Simple_UWB_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Simple_UWB (see VARARGIN)

%What should it do when started?
global t;
global session;
session = 1;
%--------------------------
%@@@@@@@@@@@@@@@@@@@@@@@@@@
%     Init Java commands
%--------------------------

%javaaddpath(fullfile(matlabroot,'work','triateration.jar'))
javaaddpath('Trilateration.jar');
tri = com.lemmingapex.trilateration.TrilaterationTest;
handles.tri = tri;
%@@@@ End of java init @@@@@

% Origin of anchors
% (Needs to be hard coded)
x1 = 0;
y1 = 0;
handles.x1 = x1;
handles.y1 = y1;

x2 = 1;
y2 = 0;
handles.x2 = x2;
handles.y2 = y2;

x3 = 0;
y3 = 1;
handles.x3 = x3;
handles.y3 = y3;


handles.i = 1;
S.fh = figure('units','pixels',...
              'position',[200 200 450 400],...
              'menubar','none',...
              'name','GUI_8',...
              'numbertitle','off',...
              'resize','off');
hold on;

S.fh = plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro');
global result;
result = [0, 0];
handles.result = result;

%Java input, needs the position of the three anchors
positions =[ x1,y1; x2, y2 ;  x3, y3 ];
handles.positions = positions;

%Places all x,y-values to take min/max to lock axis
xtot = [x1,x2,x3];
ytot = [y1,y2,y3];
handles.xtot = xtot;
handles.ytot = ytot;

t = timer('StartDelay',1, 'ExecutionMode',...
          'fixedDelay','Period', 0.5); 
t.TimerFcn = {@timerFcn, hObject, handles};




% Choose default command line output for Simple_UWB
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Simple_UWB wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function timerFcn(obj, event, hObject, handles) 
    hold on;
    
    %Locks the axis
    xlim([(min(handles.xtot)-2) (max(handles.xtot)+2)]);
    ylim([(min(handles.ytot)-2) (max(handles.ytot)+2)]);
    
    %Plots the anchors (Which are stationary)
    
    S.fh = plot(handles.x1,handles.y1,'ro',handles.x2,handles.y2,'ro',handles.x3,handles.y3,'ro');
    
    r1 = rand();
    r2 = rand();
    r3 = rand();
    
    %Java input, takes the real time distances.
    distances = [r1, r2, r3];

    p = javaMethod('trilateration2DInexact1', handles.tri, handles.positions, distances);
    disp(p);

    %Th for plotting of circles.
    %rX for each radius of circle (distances).
    th = 0:pi/50:2*pi;
    xunit = r1 * cos(th) + handles.x1;
    yunit = r1 * sin(th) + handles.y1;
    S.fh = plot(xunit, yunit);
    
    xunit2 = r2 * cos(th) + handles.x2;
    yunit2 = r2 * sin(th) + handles.y2;
    S.fh = plot(xunit2, yunit2);
   
    xunit3 = r3 * cos(th) + handles.x3;
    yunit3 = r3 * sin(th) + handles.y3;
    S.fh = plot(xunit3, yunit3);

    %Plot the estimated position with an error of e-meters around it.
    h = plot(p(1),p(2), 'b*');
    
    e = 0.1;
    punitx = e*cos(th) + p(1);
    punity = e*sin(th) + p(2);
    plot(punitx,punity, 'g-');
    global result;
    result = [result; p(1) p(2)];    
    
    pause(0.499); 
    clf;
    %delete(h);
    hold off;
    
   

    % Choose default command line output for Simple_UWB
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);
    


% --- Outputs from this function are returned to the command line.
function varargout = Simple_UWB_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Run_button.
function Run_button_Callback(hObject, eventdata, handles)
% hObject    handle to Run_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global t;
start(t);
    
    
    
% % What to do when Run is pressed
% 
% port = 'COM3'; %Where 3 is COMport number (usually standard)
% BR = 9600; % BaudRate of port
% handles.obj = serial(port, 'BaudRate', BR); % Creating object to read serial
%                                     % with baudrate 9600
% fopen(handles.obj); %opens object
% 
% while(1)
%    
%     hold on;
%     %Locks the axis
%     xlim([(min(handles.xtot)-2) (max(handles.xtot)+2)]);
%     ylim([(min(handles.ytot)-2) (max(handles.ytot)+2)]);
%     
%     %Plots the anchors (Which are stationary)
%     plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro');
%     
%     %Just put in the string and 
%     %use %d for the values you want.
%     %Reads data from mother node.
%     A = fscanf(handles.obj, ['D1: %d D2: %d D3: %d']) 
%     
%     r1 = A(1,1)/1000;  %divides it down to [m]
%     r2 = A(2,1)/1000;
%     r3 = A(3,1)/1000;
%     
%     %Java input, takes the real time distances.
%     distances = [r1, r2, r3];
% 
%     p = javaMethod('trilateration2DInexact1',handles.tri, handles.positions, distances);
%     disp(p);
%     
%     %Th for plotting of circles.
%     %rX for each radius of circle (distances).
%     th = 0:pi/50:2*pi;
%     xunit = r1 * cos(th) + x1;
%     yunit = r1 * sin(th) + y1;
%     h = plot(xunit, yunit);
%     
%     xunit2 = r2 * cos(th) + x2;
%     yunit2 = r2 * sin(th) + y2;
%     h = plot(xunit2, yunit2);
%    
%     xunit3 = r3 * cos(th) + x3;
%     yunit3 = r3 * sin(th) + y3;
%     h = plot(xunit3, yunit3);
%     
%     %Plot the estimated position with an error of e-meters around it.
%     h = plot(p(1),p(2), 'b*');
%     
%     e = 0.1;
%     punitx = e*cos(th) + p(1);
%     punity = e*sin(th) + p(2);
%     plot(punitx,punity, 'g-');
%     
%     result = [result; punitx punity];
%     handles.result = result;
%     
%     pause(0.1); 
%     clf
%     %delete(h);
%     hold off;
%     
% end

%Testingtesting

% while(handles.i < 20)
%     
%     hold on;
%     
%     %Locks the axis
%     xlim([(min(handles.xtot)-2) (max(handles.xtot)+2)]);
%     ylim([(min(handles.ytot)-2) (max(handles.ytot)+2)]);
%     
%     %Plots the anchors (Which are stationary)
%     
%     S.fh = plot(handles.x1,handles.y1,'ro',handles.x2,handles.y2,'ro',handles.x3,handles.y3,'ro');
%     
%     %Just put in the string and 
%     %use %d for the values you want.
%     %Reads data from mother node.
%     %A = fscanf(obj, ['D1: %d D2: %d D3: %d']) 
%     
%     %r1 = A(1,1)/1000;  %divides it down to [m]
%     %r2 = A(2,1)/1000;
%     %r3 = A(3,1)/1000;
%     
%     r1 = rand();
%     r2 = rand();
%     r3 = rand();
%     
%     %Java input, takes the real time distances.
%     distances = [r1, r2, r3];
% 
%     p = javaMethod('trilateration2DInexact1', handles.tri, handles.positions, distances);
%     disp(p);
%     
%     %Th for plotting of circles.
%     %rX for each radius of circle (distances).
%     th = 0:pi/50:2*pi;
%     xunit = r1 * cos(th) + handles.x1;
%     yunit = r1 * sin(th) + handles.y1;
%     S.fh = plot(xunit, yunit);
%     
%     xunit2 = r2 * cos(th) + handles.x2;
%     yunit2 = r2 * sin(th) + handles.y2;
%     S.fh = plot(xunit2, yunit2);
%    
%     xunit3 = r3 * cos(th) + handles.x3;
%     yunit3 = r3 * sin(th) + handles.y3;
%     S.fh = plot(xunit3, yunit3);
%     
%     %Plot the estimated position with an error of e-meters around it.
%     h = plot(p(1),p(2), 'b*');
%     
%     e = 0.1;
%     punitx = e*cos(th) + p(1);
%     punity = e*sin(th) + p(2);
%     plot(punitx,punity, 'g-');
%     
%     handles.result = [handles.result; p(1) p(2)];    
%     
%     pause(0.1); 
%     clf
%     %delete(h);
%     hold off;
%     handles.i = handles.i + 1;
%     
%     %if ()
%      %   break;
%     %end
%     
    % Choose default command line output for Simple_UWB
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);
%     
% end

% --- Executes on button press in Stop_button.
function Stop_button_Callback(hObject, eventdata, handles)
% hObject    handle to Stop_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% What happens when Stop button is pressed
% closes everything
    global t;
    stop(t);
    clf;
%    handles.i 
%    plot(1,1,'g*');
%    fclose(handles.obj);
%    delete(handles.obj);
%    clear handles.obj;
    % Choose default command line output for Simple_UWB
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);


% --- Executes on button press in Previous_button.
function Previous_button_Callback(hObject, eventdata, handles)
% hObject    handle to Previous_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    global t;
    global result;
    stop(t);
    clf;
    hold on;
    handles.i = 0;
    
    xlim([(min(handles.xtot)-2) (max(handles.xtot)+2)]);
    ylim([(min(handles.ytot)-2) (max(handles.ytot)+2)]);
    
    %Plots the anchors (Which are stationary)
    
    plot(handles.x1,handles.y1,'ro',handles.x2,handles.y2,'ro',handles.x3,handles.y3,'ro'); 
    comet(result(:,1),result(:,2));
    result = [0 0];

    % Choose default command line output for Simple_UWB
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);


% --------------------------------------------------------------------
