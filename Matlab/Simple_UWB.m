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

%Connect to database
db_connect();

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
% sim = 1 -> simulate with fake values
global sim;
sim = 1;

%What should it do when started?

%------------------------------------------------
% ------ Global Variables ------- %

global t;
global session;
global img
global result;
global average;
global j;
global iteration;
global last_x last_P noise acc
global A B Q R H G
%------------------------------------------------

%------------------------------------------------
% If exact values (for visual)
%global x_exact y_exact;

%x_exact = [0.9 1.75 2.75 3.75 3.75 4.75];
%y_exact = [2.4 1.45 1.45 1.45 2.45 2.45];

%------------------------------------------------

%------------------------------------------------
% ----- If SIMULATION ----- %

session = 1;
%------------------------------------------------

%------------------------------------------------
% ----- Init Java commands ------ %

%javaaddpath(fullfile(matlabroot,'work','triateration.jar'))
%javaaddpath('Trilateration.jar');
tri = com.lemmingapex.trilateration.TrilaterationTest;
handles.tri = tri;
%------------------------------------------------

%------------------------------------------------
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
%------------------------------------------------

%------------------------------------------------
% ----- Room Width and Heigth ----- %

roomWidth = 6;
roomHeigth = 6;
handles.roomWidth = roomWidth;
handles.roomHeigth = roomHeigth;
%------------------------------------------------

%------------------------------------------------
% ----- Image and Plot window ----- %
img = imread('layout2.jpg');



handles.i = 1;
S.fh = figure('units','pixels',...
    'position',[200 200 450 400],...
    'menubar','none',...
    'name','GUI_8',...
    'numbertitle','off',...
    'resize','off');
hold on;
%------------------------------------------------

%------------------------------------------------
% ----- Setting some standard values ----- %

% If anchor plot on startup 
%S.fh = plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro');

% A result matrix to show the previous run
result = [0, 0];
handles.result = result;

% Average matrix to not have so much jumping around
average = [0,0; 0,0; 0,0; 0,0; 0,0];

% To read the average of the last 5 values.
j = 5;

%Java input, needs the position of the three anchors
positions =[ x1,y1; x2, y2 ;  x3, y3 ];
handles.positions = positions;

%Places all x,y-values to take min/max to lock axis
xtot = [x1,x2,x3];
ytot = [y1,y2,y3];
handles.xtot = xtot;
handles.ytot = ytot;

% The loop only updates every three new reading
iteration = 0;

%------------------------------------------------

%------------------------------------------------
% ----- Init the timer function ----- %
t = timer('StartDelay',0.05, 'ExecutionMode',...
    'fixedDelay','Period', 0.020);
t.TimerFcn = {@timerFcn, hObject, handles};
%------------------------------------------------

%------------------------------------------------
% ----- Kalman fliter matrices init ----- %

delta_T = 0.1;
A = [1, delta_T, 0, 0; 0, 1, 0, 0; 0, 0, 1, delta_T; 0, 0, 0, 1];

B = [1, 0, 0, 0;0, 1, 0, 0;0, 0, 1, 0;0, 0, 0, 1];

H = [1, 0, 0, 0;0, 1, 0, 0;0, 0, 1, 0;0, 0, 0, 1];

Q = [0.01, 0, 0, 0;0, 0.002, 0, 0;0, 0, 0.01, 0;0, 0, 0, 0.002];

G = [(delta_T^2)/2; delta_T; (delta_T^2)/2; delta_T];

R = 0.01*eye(4);%[1, 0, 0, 0;0, 1, 0, 0;0, 0, 0.1, 0;0, 0, 0, 0.1];
x_init = [0 0 0 0];
P = 1*eye(4);
last_x = x_init';
last_P = P;
noise = 0.01;%normrnd(0,0.01)*[0 1 0 1]';
acc = 0.01*(noise/2-rand()*noise);%normrnd(0,0.1);
%------------------------------------------------


% Choose default command line output for Simple_UWB
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Simple_UWB wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function timerFcn(object, event, hObject, handles)
%------------------------------------------------
% ------ Global Variables ------- %

global currentPlace placeId placeName placeUrl placeWidth placeHeight placeAx placeAy placeBx placeBy placeCx placeCy;
global session;
global obj;
global average;
global iteration;
global j;
global img;
global sim;
global last_x last_P noise acc
global A B Q R H G
global result;

%------------------------------------------------

%------------------------------------------------
% ----- Clocks to see time of loop ----- %

firstTic = tic;
SECONDTIC = tic;
%------------------------------------------------

%------------------------------------------------
%Check if it is a simulation or not

if(sim)
    
    simDist1 = placeWidth*rand;
    simDist2 = placeWidth*rand;
    simDist3 = placeWidth*rand;
    Adist = [simDist1;simDist2;simDist3];
	
else
    Adist = fscanf(obj, ['D1: %d D2: %d D3: %d']); %Read from serial port
end
endFirstTic = toc(firstTic) % See how fast the USB-reading is
iteration = iteration + 1;
%------------------------------------------------

%------------------------------------------------
% ----- When iteration == 3 the calculation begins ----- %

if (iteration == 3)
    r1 = Adist(1,1)/1000;  %divides it down to [m]
    r2 = Adist(2,1)/1000;
    r3 = Adist(3,1)/1000;
    
    %Java input, takes the real time distances.
    distances = [r1, r2, r3];
    
    p = javaMethod('trilateration2DInexact1', handles.tri, handles.positions, distances);
    
    endsecondtic = toc(SECONDTIC) %See how long it takes to come here
    
    %Th for plotting of circles.
    %rX for each radius of circle (distances).
    clf;
    
    imagesc([0 placeWidth],[0 placeHeight],flipud(img));
    colormap(gray);
    set(gca,'ydir','normal');

    hold on;
    %Locks the axis
    
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
    e = 0.1;
    punitx = e*cos(th) + p(1);
    punity = e*sin(th) + p(2);
    plot(punitx,punity, 'g-');
      
    %Plot the average of 5 values.
    average = [average; p(1), p(2)];
    j = j+1;
    avgpx = (average(j-5,1)+average(j-4,1)+average(j-3,1)+average(j-2,1)+average(j-1,1))/5;
    avgpy = (average(j-5,2)+average(j-4,2)+average(j-3,2)+average(j-2,2)+average(j-1,2))/5;

    plot(avgpx,avgpy,'b*');
%------------------------------------------------

%------------------------------------------------
        % Kalman filter

    cur_xpos = avgpx;%+noise/2 -noise*rand();
    cur_ypos = avgpy;%+noise/2 -noise*rand();
    disp([avgpx, avgpy])
    
    % Velocity = Distance/Time [m/s]
    velX = (cur_xpos-last_x(1))/0.2;
    velY = (cur_ypos-last_x(3))/0.2;
    disp('velocity')
    disp([velX,velY]);
    
    % Quiver to achieve an velocity arrow
    % that points in the traveling direction
    quiver(cur_xpos,cur_ypos,velX,velY)
    
    
    % Measurement vector
    z = [cur_xpos, velX, cur_ypos, velY];

    % Kalman iteration to achieve future values
    
    kalman_time = tic; % See how long the kalman iteration takes
    for n=1:100
        x_est_m = last_x;
        P = last_P;
        
        x_est_m = A*x_est_m + G*acc;
        P_m = A*P*A'+Q;
        
        K = (P*H')/(H*P*H'+R);
        x_est = x_est_m + K*(z' - H*x_est_m);
        P = (eye(4) - K*H)*P_m;
        
        
        last_x = x_est;
        last_P = P;
    end
    kalman_end = toc(kalman_time) %Displays time
    
    % Plots the future values aquired by the kalman iterations
    
    %plot(x_est(1), x_est(3), 'ro');
    plot(x_est_m(1), x_est_m(3), 'bo');
    disp(last_x)
    disp([avgpx,avgpy])
    %disp([p(1),p(2)])
    %------------------------------------------------
    
    %------------------------------------------------
    %    Result that gets sent to database    %
    result = [result; avgpx avgpy];
    
    %Send to database
    
    sendPositionDb(avgpx,avgpy,session);
    
    
    
    %     pauseTime = 0.1-endFirstTic;
    %     if(pauseTime > 0)
    %         pause(pauseTime);
    %     end
    endsecondtic = toc(SECONDTIC) % How long the whole loop takes
    hold off;
    iteration = 0; %iteration set to 0.
end
%------------------------------------------------
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

%------------------------------------------------
% ------ Global Variables ------- %

global session placeId;
global t;
global obj;
global sim;
%------------------------------------------------
%Connect to db
db_connect();

sendSessionDb(placeId,1);
session = getQuery('SELECT MAX(id) FROM sessions');
session = cell2mat(session);

port = 'COM3'; %Where 3 is COMport number (usually standard)
BR = 9600; % BaudRate of port
obj = serial(port, 'BaudRate', BR); % Creating object to read serial
% with baudrate 9600


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

%------------------------------------------------
% ------ Global Variables ------- %

global t;
global obj;
global j;
global average;
%------------------------------------------------

fclose(obj);
delete(obj);
clear obj;
stop(t);
clf;

%------------------------------------------------
% Resets the average vector

j = 5;

average = [0,0; 0,0; 0,0; 0,0; 0,0;];
%------------------------------------------------


% Choose default command line output for Simple_UWB
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in Previous_button.
function Previous_button_Callback(hObject, eventdata, handles)
% hObject    handle to Previous_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%------------------------------------------------
% ------ Global Variables ------- %
global currentPlace placeId placeName placeUrl placeWidth placeHeight placeAx placeAy placeBx placeBy placeCx placeCy;
global t;
global result;
global x_exact;
global y_exact;
%------------------------------------------------


stop(t);
clf;
hold on;

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
