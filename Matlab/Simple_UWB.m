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

% Last Modified by GUIDE v2.5 08-Dec-2016 14:28:47

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

%connect to db
db_connect;

% --- Executes just before Simple_UWB is made visible.
function Simple_UWB_OpeningFcn(hObject, eventdata, handles, varargin)
    places = getPlacesDb();
    set(handles.places_listbox, 'String', places);
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Simple_UWB (see VARARGIN)


%Set simulation or not
global sim;
sim = 1;



%What should it do when started?
global t;
global session;
session = 1;
%--------------------------
%@@@@@@@@@@@@@@@@@@@@@@@@@@
%     Init Java commands
%--------------------------

%javaaddpath(fullfile(matlabroot,'work','triateration.jar'))
%javaaddpath('Trilateration.jar');
tri = com.lemmingapex.trilateration.TrilaterationTest;
handles.tri = tri;
%@@@@ End of java init @@@@@

% Origin of anchors
% (Needs to be hard coded)
x1 = 4.45;
y1 = 1.5;
handles.x1 = x1;
handles.y1 = y1;

x2 = 5.75;
y2 = 2.45;
handles.x2 = x2;
handles.y2 = y2;

x3 = 2.35;
y3 = 3.45;
handles.x3 = x3;
handles.y3 = y3;

roomWidth = 6;
roomHeigth = 6;
handles.roomWidth = roomWidth;
handles.roomHeigth = roomHeigth;

global img
img = imread('layout2.jpg');

global x_exact y_exact;
x_exact = [0.9 1.75 2.75 3.75 3.75 4.75];
y_exact = [2.4 1.45 1.45 1.45 2.45 2.45];

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

global average;
average = [0,0; 0,0; 0,0; 0,0; 0,0];

global j;
j = 5;

global result;
result = [0 0];

%Java input, needs the position of the three anchors
positions =[ x1,y1; x2, y2 ;  x3, y3 ];
handles.positions = positions;

%Places all x,y-values to take min/max to lock axis
xtot = [x1,x2,x3];
ytot = [y1,y2,y3];
handles.xtot = xtot;
handles.ytot = ytot;

global iteration;
iteration = 0;
t = timer('StartDelay',0.05, 'ExecutionMode',...
    'fixedDelay','Period', 0.020);
t.TimerFcn = {@timerFcn, hObject, handles};



% Choose default command line output for Simple_UWB
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Simple_UWB wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function timerFcn(object, event, hObject, handles)
global currentPlace placeId placeName placeUrl placeWidth placeHeight placeAx placeAy placeBx placeBy placeCx placeCy;
global session;
firstTic = tic;
global obj;
SECONDTIC = tic;
global average;
global iteration;
global xhat;
global P0;
global sigmaP;
global sigmaA;
global Hk;
global j;
global img;
%hold on;

%     r1 = rand();
%     r2 = rand();
%     r3 = rand();

%Just put in the string and
%use %d for the values you want.
%Reads data from mother node.

%Check if it is a simulation or not
global sim;
if(sim)
    
    simDist1 = placeWidth*rand;
    simDist2 = placeWidth*rand;
    simDist3 = placeWidth*rand;
    A = [simDist1;simDist2;simDist3];
	
else
    A = fscanf(obj, ['D1: %d D2: %d D3: %d']);
end
endFirstTic = toc(firstTic)
iteration = iteration + 1;
if (iteration == 3)
    r1 = A(1,1)/1000;  %divides it down to [m]
    r2 = A(2,1)/1000;
    r3 = A(3,1)/1000;
    
    %Java input, takes the real time distances.
    distances = [r1, r2, r3];
    
    p = javaMethod('trilateration2DInexact1', handles.tri, handles.positions, distances);
    %disp(p);
    endsecondtic = toc(SECONDTIC)
    %Th for plotting of circles.
    %rX for each radius of circle (distances).
    clf;
    
    imagesc([0 placeWidth],[0 placeHeight],flipud(img));
    colormap(gray);
    set(gca,'ydir','normal');

    hold on;
    %Locks the axis
    %xlim([(min(handles.xtot)-2) (max(handles.xtot)+2)]);
    %ylim([(min(handles.ytot)-2) (max(handles.ytot)+2)]);
    %xlim([0 handles.roomWidth]);
    %ylim([0 handles.roomHeigth]);
    
    %Plots the anchors (Which are stationary)
    
    S.fh = plot(placeAx,placeAy,'ro',placeBx,placeBy,'ro',placeCx,placeCy,'ro');
    
    th = 0:pi/50:2*pi;
    xunit = r1 * cos(th) + placeAx;
    yunit = r1 * sin(th) + placeAy;
    S.fh = plot(xunit, yunit);
    
    xunit2 = r2 * cos(th) + placeBx;
    yunit2 = r2 * sin(th) + placeBy;
    S.fh = plot(xunit2, yunit2);
    
    xunit3 = r3 * cos(th) + placeCx;
    yunit3 = r3 * sin(th) + placeCy;
    S.fh = plot(xunit3, yunit3);
    
    %Plot the estimated position with an error of e-meters around it.
    %h = plot(p(1),p(2), 'b*');
    
    
    e = 0.1;
    punitx = e*cos(th) + p(1);
    punity = e*sin(th) + p(2);
    plot(punitx,punity, 'g-');
    
    
    
    %Plot the average of 5 values.
    average = [average; p(1), p(2)];
    j = j+1;
    avgpy = (average(j-5,2)+average(j-4,2)+average(j-3,2)+average(j-2,2)+average(j-1,2))/5;
    avgpx = (average(j-5,1)+average(j-4,1)+average(j-3,1)+average(j-2,1)+average(j-1,1))/5;
    
    plot(avgpx,avgpy,'b*');
    
%     global KalmanResult;
%     global w;
%     ptot = [avgpx, avgpy];
%     r = 1;
%     xhatd = ptot + w;
%     for(r = 1: 1: 10)
%         %Kalman goes here
%         
%         P0n = P0;
%         K = (P0n)/(P0n + sigmaP);
%         xhat = xhatd + K*(Hk*ptot - xhatd);
%         xhatd = xhat + w;
%         P0 = (1-K)*P0n;
%         KalmanResult = [KalmanResult; xhat(1), xhat(2)];
%     
%     end
    
        plot(xhat(1), xhat(2),'r*');
        global x_exact y_exact;
        %plot(x_exact,y_exact, '+k')
    global result;
    result = [result; avgpx avgpy];
    
    %Send to database
    
    sendPositionDb(avgpx,avgpy,session);
    
    
    
    %     pauseTime = 0.1-endFirstTic;
    %     if(pauseTime > 0)
    %         pause(pauseTime);
    %     end
    endsecondtic = toc(SECONDTIC)
    %pause(0.02);
    %clf;
    %delete(h);
    hold off;
    iteration = 0;
end

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
% Set the

global session placeId;

sendSessionDb(placeId,1);
session = getQuery('SELECT MAX(id) FROM sessions');
session = cell2mat(session);
global t;
global obj;
global xhat;
xhat = [0,0];

global P0;
P0 = 1;

global Hk;
Hk = 1;

global sigmaP;
sigmaP = 0.1;

global sigmaA;
sigmaA = 0.1;  %?

global w;
w = wgn(1,2,0.01);

global KalmanResult;
KalmanResult = [0,0];
port = 'COM3'; %Where 3 is COMport number (usually standard)
BR = 9600; % BaudRate of port
obj = serial(port, 'BaudRate', BR); % Creating object to read serial
% with baudrate 9600
global sim;
if(not(sim))
fopen(obj); %opens object
end
start(t);

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
global obj;
fclose(obj);
delete(obj);
clear obj;
stop(t);
clf;

global j;
j = 5;

global average;
average = [0,0; 0,0; 0,0; 0,0; 0,0;];


% Choose default command line output for Simple_UWB
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in Previous_button.
function Previous_button_Callback(hObject, eventdata, handles)
global currentPlace placeId placeName placeUrl placeWidth placeHeight placeAx placeAy placeBx placeBy placeCx placeCy;
% hObject    handle to Previous_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global t;
global result;
global x_exact;
global y_exact;
stop(t);
clf;
hold on;
handles.i = 0;

xlim([(min(handles.xtot)-2) (max(handles.xtot)+2)]);
ylim([(min(handles.ytot)-2) (max(handles.ytot)+2)]);

%Plots the anchors (Which are stationary)

plot(x_exact,y_exact, '+k')
xlabel('X')
ylabel('Y')
plot(placeAx,placeAy,'ro',placeBx,placeBy,'ro',placeCx,placeCy,'ro');
comet(result(:,1),result(:,2));


% Choose default command line output for Simple_UWB
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --------------------------------------------------------------------


% --- Executes on selection change in places_listbox.
function places_listbox_Callback(hObject, eventdata, handles)
global currentPlace placeId placeName placeUrl placeWidth placeHeight placeAx placeAy placeBx placeBy placeCx placeCy;
% hObject    handle to places_listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns places_listbox contents as cell array
%        contents{get(hObject,'Value')} returns selected item from places_listbox

names = get(hObject,'String');
id = get(hObject,'Value');
currName = strjoin(names(id));
query = strcat('SELECT * FROM places WHERE name = "',currName,'"');
place = getQuery(query);
currentPlace = place;
placeId = cell2mat(place(1,1));
placeName = strjoin(place(1,2));
placeUrl = strjoin(place(1,3));
placeWidth = cell2mat(place(1,4));
placeHeight = cell2mat(place(1,5));
placeAx = cell2mat(place(1,6));
placeAy = cell2mat(place(1,7));
placeBx = cell2mat(place(1,8));
placeBy = cell2mat(place(1,9));
placeCx = cell2mat(place(1,10));
placeCy = cell2mat(place(1,11));
placeStr = ['Name: ' place(1,2) ' Url: ' place(1,3) 'Width: ' num2str(cell2mat(place(1,4))) 'Heigth: ' num2str(cell2mat(place(1,5))) 'AX: ' num2str(cell2mat(place(1,6))) 'AY: ' num2str(cell2mat(place(1,7))) 'BX: ' num2str(cell2mat(place(1,8))) 'BY: ' num2str(cell2mat(place(1,9))) 'CX: ' num2str(cell2mat(place(1,10))) 'CY: ' num2str(cell2mat(place(1,11)))];
placeStr = strjoin(placeStr);
set(handles.place_details,'String',placeStr);

% --- Executes during object creation, after setting all properties.
function places_listbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to places_listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
