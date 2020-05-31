function varargout = results_gui(varargin)
% RESULTS_GUI MATLAB code for results_gui.fig
%      RESULTS_GUI, by itself, creates a new RESULTS_GUI or raises the existing
%      singleton*.
%
%      H = RESULTS_GUI returns the handle to a new RESULTS_GUI or the handle to
%      the existing singleton*.
%
%      RESULTS_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RESULTS_GUI.M with the given input arguments.
%
%      RESULTS_GUI('Property','Value',...) creates a new RESULTS_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before results_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to results_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help results_gui

% Last Modified by GUIDE v2.5 01-Sep-2016 14:37:15

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @results_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @results_gui_OutputFcn, ...
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


% --- Executes just before results_gui is made visible.
function results_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to results_gui (see VARARGIN)

% Choose default command line output for results_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

acceleration = evalin('base', 'acceleration');
batt_current = evalin('base', 'batt_current');
batt_ocv = evalin('base', 'batt_ocv');
battery_SOC = evalin('base', 'battery_SOC');
distance_run = evalin('base', 'distance_run');
motor_RPM = evalin('base', 'motor_RPM');
motor_torque = evalin('base', 'motor_torque');
velocity = evalin('base', 'velocity');
velocity = velocity .* 2.23694; % converts to mph
wheel_torque = evalin('base', 'wheel_torque');
vehicle_name = evalin('base', 'vehicle_name');
lap_time = evalin('base', 'lap_time');
i_count = evalin('base', 'i_count');
time_step = evalin('base', 'time_step');

set(handles.vehicle_name_static, 'String', vehicle_name);

top_speed = round(max(velocity), 1);
set(handles.top_velocity_static, 'String', top_speed);

avg_speed = round(mean(velocity), 1);
set(handles.average_velocity_static, 'String', avg_speed);

max_accel = round(max(acceleration), 1);
set(handles.top_acceleration_static, 'String', max_accel);

[~,laps] = size(lap_time);
laps = 1:laps;
lap_time_data = [laps', lap_time'];
set(handles.lap_time_table, 'Data', lap_time_data);

max_motor_torque = round(max(motor_torque), 2);
set(handles.max_motor_torque_static, 'String', max_motor_torque);

max_wheel_torque = round(max(wheel_torque), 2);
set(handles.max_wheel_torque_static, 'String', max_wheel_torque);

max_motor_speed = round(max(motor_RPM), 0);
set(handles.max_motor_speed_static, 'String', max_motor_speed);

max_batt_current = round(max(batt_current), 2);
set(handles.max_batt_current_static, 'String', max_batt_current);

avg_batt_current = round(mean(batt_current), 2);
set(handles.avg_batt_current_static, 'String', avg_batt_current);

rms_current = round(sqrt(mean(batt_current.^2)), 2);
set(handles.rms_batt_current_static, 'String', rms_current);

set(handles.batt_SOC_rem_static, 'String', round(battery_SOC(i_count), 2));

avg_lap_time = mean(lap_time);
set(handles.avg_lap_time, 'String', avg_lap_time);

time = time_step:time_step:(i_count*time_step);
assignin('base', 'time', time);
guidata(hObject, handles);



% UIWAIT makes results_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = results_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in y_axis_popup.
function y_axis_popup_Callback(hObject, eventdata, handles)
% hObject    handle to y_axis_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns y_axis_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from y_axis_popup

        

% --- Executes during object creation, after setting all properties.
function y_axis_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_axis_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in x_axis_popup.
function x_axis_popup_Callback(hObject, eventdata, handles)
% hObject    handle to x_axis_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns x_axis_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from x_axis_popup


% --- Executes during object creation, after setting all properties.
function x_axis_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_axis_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function vehicle_name_static_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vehicle_name_static (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in plot_pshbtn.
function plot_pshbtn_Callback(hObject, eventdata, handles)
% hObject    handle to plot_pshbtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

y_axis_string = get(handles.y_axis_popup, 'String');
y_axis = get(handles.y_axis_popup, 'Value');


switch y_axis_string{y_axis}
    case 'Acceleration [m/s^2]'
        y = evalin('base', 'acceleration');
    case 'Battery Current [A]'
        y = evalin('base', 'batt_current');
    case 'Battery Open Circuit Voltage [V]'
        y = evalin('base', 'batt_ocv');
    case 'Battery SOC [%]'
        y = evalin('base', 'battery_SOC');
    case 'Distance [m]'
        y = evalin('base', 'distance_run');
    case 'Motor Speed [RPM]'
        y = evalin('base', 'motor_RPM');
    case 'Motor Torque [Nm]'
        y = evalin('base', 'motor_torque');
    case 'Time [s]'
        y = evalin('base', 'time');
    case 'Velocity [mph]'
        y = evalin('base', 'velocity') * 2.23694;
    case 'Wheel Torque [Nm]'
        y = evalin('base', 'wheel_torque');
    otherwise
        y = 0;
end

x_axis_string = get(handles.x_axis_popup, 'String');
x_axis = get(handles.x_axis_popup, 'Value');

switch x_axis_string{x_axis}
    case 'Acceleration [m/s^2]'
        x = evalin('base', 'acceleration');
    case 'Battery Current [A]'
        x = evalin('base', 'batt_current');
    case 'Battery Open Circuit Voltage [V]'
        x = evalin('base', 'batt_ocv');
    case 'Battery SOC [%]'
        x = evalin('base', 'battery_SOC');
    case 'Distance [m]'
        x = evalin('base', 'distance_run');
    case 'Motor Speed [RPM]'
        x = evalin('base', 'motor_RPM');
    case 'Motor Torque [Nm]'
        x = evalin('base', 'motor_torque');
    case 'Time [s]'
        x = evalin('base', 'time');
    case 'Velocity [mph]'
        x = evalin('base', 'velocity') * 2.23694;
    case 'Wheel Torque [Nm]'
        x = evalin('base', 'wheel_torque');
    otherwise
        x = 0;
end
subplot(handles.axes1);
plot(x,y);


% --- Executes during object creation, after setting all properties.
function lap_time_table_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lap_time_table (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
