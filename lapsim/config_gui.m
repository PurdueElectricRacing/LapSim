function varargout = config_gui(varargin)
% CONFIG_GUI MATLAB code for config_gui.fig
%      CONFIG_GUI, by itself, creates a new CONFIG_GUI or raises the existing
%      singleton*.
%
%      H = CONFIG_GUI returns the handle to a new CONFIG_GUI or the handle to
%      the existing singleton*.
%
%      CONFIG_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONFIG_GUI.M with the given input arguments.
%
%      CONFIG_GUI('Property','Value',...) creates a new CONFIG_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before config_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to config_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help config_gui

% Last Modified by GUIDE v2.5 11-Sep-2016 19:36:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @config_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @config_gui_OutputFcn, ...
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


% --- Executes just before config_gui is made visible.
function config_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to config_gui (see VARARGIN)

% Choose default command line output for config_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes config_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = config_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function title_static_CreateFcn(hObject, eventdata, handles)
% hObject    handle to title_static (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject, 'String', 'Formula SAE Simulation');



function vehicle_name_edit_Callback(hObject, eventdata, handles)
% hObject    handle to vehicle_name_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vehicle_name_edit as text
%        str2double(get(hObject,'String')) returns contents of vehicle_name_edit as a double


% --- Executes during object creation, after setting all properties.
function vehicle_name_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vehicle_name_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function vehicle_name_static_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vehicle_name_static (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in run_pshbtn.
function run_pshbtn_Callback(hObject, eventdata, handles)
% hObject    handle to run_pshbtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vehicle_name = get(handles.vehicle_name_edit, 'String');
assignin('base', 'vehicle_name', vehicle_name);

time_step = get(handles.time_step_edit, 'String');
time_step = str2double(time_step);
assignin('base', 'time_step', time_step);

mass = get(handles.mass_edit, 'String');
mass = str2double(mass);
assignin('base', 'mass', mass);

wheel_base = get(handles.wheel_base_edit, 'String');
wheel_base = str2double(wheel_base);
assignin('base', 'wheel_base', wheel_base);

Cg_z = get(handles.Cg_z_edit, 'String');
Cg_z = str2double(Cg_z);
assignin('base', 'Cg_z', Cg_z);

power_limit = get(handles.power_limit_edit, 'String');
power_limit = str2double(power_limit);
assignin('base', 'power_limit', power_limit);

Cd = get(handles.Cd_edit, 'String');
Cd = str2double(Cd);
assignin('base', 'Cd', Cd);

Af = get(handles.Af_edit, 'String');
Af = str2double(Af);
assignin('base', 'Af', Af);

Cl = get(handles.Cl_edit, 'String');
Cl = str2double(Cl);
assignin('base', 'Cl', Cl);

Al = get(handles.Al_edit, 'String');
Al = str2double(Al);
assignin('base', 'Al', Al);

Cr = get(handles.Cr_edit, 'String');
Cr = str2double(Cr);
assignin('base', 'Cr', Cr);

tire_radius = get(handles.tire_radius_edit, 'String');
tire_radius = str2double(tire_radius);
assignin('base', 'tire_radius', tire_radius);

lateral_g = get(handles.lateral_g_edit, 'String');
lateral_g = str2double(lateral_g);
assignin('base', 'lateral_g', lateral_g);

max_braking_force = get(handles.max_braking_force_edit, 'String');
max_braking_force = str2double(max_braking_force);
assignin('base', 'max_braking_force', max_braking_force);

coeff_f = get(handles.coeff_f_edit, 'String');
coeff_f = str2double(coeff_f);
assignin('base', 'coeff_f', coeff_f);

fdr = get(handles.fdr_edit, 'String');
fdr = str2double(fdr);
assignin('base', 'fdr', fdr);

driveline_eff = get(handles.driveline_eff_edit, 'String');
driveline_eff = str2double(driveline_eff);
assignin('base', 'driveline_eff', driveline_eff);

Ah = get(handles.Ah_edit, 'String');
Ah = str2double(Ah);
assignin('base', 'Ah', Ah);

batt_V_max = get(handles.batt_V_max_edit, 'String');
batt_V_max = str2double(batt_V_max);
assignin('base', 'batt_V_max', batt_V_max);

R_int = get(handles.R_int_edit, 'String');
R_int = str2double(R_int);
assignin('base', 'R_int', R_int);

cell_s = get(handles.cell_s_edit, 'String');
cell_s = str2double(cell_s);
assignin('base', 'cell_s', cell_s);

cell_p = get(handles.cell_p_edit, 'String');
cell_p = str2double(cell_p);
assignin('base', 'cell_p', cell_p);

batt_current_limit = get(handles.batt_current_limit_edit, 'String');
batt_current_limit = str2double(batt_current_limit);
assignin('base', 'batt_current_limit', batt_current_limit);

pack_R = (R_int * cell_s)/cell_p; %pack internal resistance
assignin('base', 'pack_R', pack_R);

battery_table = get(handles.battery_table_edit, 'Data');
assignin('base', 'battery_table', battery_table);

max_motor_power = get(handles.max_motor_power_edit, 'String');
max_motor_power = str2double(max_motor_power);
assignin('base', 'max_motor_power', max_motor_power);

motor_eff = get(handles.motor_eff_edit, 'String');
motor_eff = str2double(motor_eff);
assignin('base', 'motor_eff', motor_eff);

max_motor_current = get(handles.max_motor_current_edit, 'String');
max_motor_current = str2double(max_motor_current);
assignin('base', 'max_motor_current', max_motor_current);

motor_table = get(handles.motor_table, 'Data');
motor_table = motor_table';
assignin('base', 'motor_table', motor_table);

acceleration_value = get(handles.acceleration_radbtn, 'Value');
skidpad_value = get(handles.skidpad_radbtn, 'Value');
autocross_value = get(handles.autocross_radbtn, 'Value');
endurance_value = get(handles.endurance_radbtn, 'Value');
quarter_mile_value = get(handles.quarter_mile_radbtn, 'Value');

if acceleration_value == 1
    acceleration_run;
elseif skidpad_value == 1
    skidpad;
elseif autocross_value == 1
    autox;
elseif endurance_value == 1
    endurance;
elseif quarter_mile_value == 1
    quarter_mile;
end
run_track;

assignin('base', 'batt_current', batt_current);
assignin('base', 'batt_ocv', batt_ocv);
assignin('base', 'battery_SOC', battery_SOC);
assignin('base', 'acceleration', acceleration);
assignin('base', 'distance_run', distance_run);
assignin('base', 'lap_time', lap_time);
assignin('base', 'motor_RPM', motor_RPM);
assignin('base', 'motor_torque', motor_torque);
assignin('base', 'i_count', i_count);
assignin('base', 'velocity', velocity);
assignin('base', 'wheel_torque', wheel_torque);
assignin('base', 'track_table', track_table);
results_gui;
close(config_gui);













% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over run_pshbtn.
function run_pshbtn_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to run_pshbtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function time_step_edit_Callback(hObject, eventdata, handles)
% hObject    handle to time_step_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of time_step_edit as text
%        str2double(get(hObject,'String')) returns contents of time_step_edit as a double


% --- Executes during object creation, after setting all properties.
function time_step_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time_step_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function track_CreateFcn(hObject, eventdata, handles)
% hObject    handle to track (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called\
bg = uibuttongroup('Visible','off',...
                  'Position',[0 0 .2 1],...
                  'SelectionChangedFcn',@bselection);

r_acceleration = uicontrol(bg,'Style','radiobutton',...
                  'String','Acceleration',...
                  'HandleVisibility','off');



function mass_edit_Callback(hObject, eventdata, handles)
% hObject    handle to mass_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mass_edit as text
%        str2double(get(hObject,'String')) returns contents of mass_edit as a double


% --- Executes during object creation, after setting all properties.
function mass_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mass_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cd_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Cd_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cd_edit as text
%        str2double(get(hObject,'String')) returns contents of Cd_edit as a double


% --- Executes during object creation, after setting all properties.
function Cd_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cd_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Af_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Af_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Af_edit as text
%        str2double(get(hObject,'String')) returns contents of Af_edit as a double


% --- Executes during object creation, after setting all properties.
function Af_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Af_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cr_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Cr_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cr_edit as text
%        str2double(get(hObject,'String')) returns contents of Cr_edit as a double


% --- Executes during object creation, after setting all properties.
function Cr_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cr_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tire_radius_edit_Callback(hObject, eventdata, handles)
% hObject    handle to tire_radius_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tire_radius_edit as text
%        str2double(get(hObject,'String')) returns contents of tire_radius_edit as a double


% --- Executes during object creation, after setting all properties.
function tire_radius_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tire_radius_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function lateral_g_edit_Callback(hObject, eventdata, handles)
% hObject    handle to lateral_g_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of lateral_g_edit as text
%        str2double(get(hObject,'String')) returns contents of lateral_g_edit as a double


% --- Executes during object creation, after setting all properties.
function lateral_g_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lateral_g_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function max_braking_force_edit_Callback(hObject, eventdata, handles)
% hObject    handle to max_braking_force_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of max_braking_force_edit as text
%        str2double(get(hObject,'String')) returns contents of max_braking_force_edit as a double


% --- Executes during object creation, after setting all properties.
function max_braking_force_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to max_braking_force_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fdr_edit_Callback(hObject, eventdata, handles)
% hObject    handle to fdr_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fdr_edit as text
%        str2double(get(hObject,'String')) returns contents of fdr_edit as a double


% --- Executes during object creation, after setting all properties.
function fdr_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fdr_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function driveline_eff_edit_Callback(hObject, eventdata, handles)
% hObject    handle to driveline_eff_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of driveline_eff_edit as text
%        str2double(get(hObject,'String')) returns contents of driveline_eff_edit as a double


% --- Executes during object creation, after setting all properties.
function driveline_eff_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to driveline_eff_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor_eff_edit_Callback(hObject, eventdata, handles)
% hObject    handle to motor_eff_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor_eff_edit as text
%        str2double(get(hObject,'String')) returns contents of motor_eff_edit as a double


% --- Executes during object creation, after setting all properties.
function motor_eff_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor_eff_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function kt_edit_Callback(hObject, eventdata, handles)
% hObject    handle to kt_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of kt_edit as text
%        str2double(get(hObject,'String')) returns contents of kt_edit as a double


% --- Executes during object creation, after setting all properties.
function kt_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to kt_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function KV_edit_Callback(hObject, eventdata, handles)
% hObject    handle to KV_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of KV_edit as text
%        str2double(get(hObject,'String')) returns contents of KV_edit as a double


% --- Executes during object creation, after setting all properties.
function KV_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to KV_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function motor_R_Callback(hObject, eventdata, handles)
% hObject    handle to motor_R (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of motor_R as text
%        str2double(get(hObject,'String')) returns contents of motor_R as a double


% --- Executes during object creation, after setting all properties.
function motor_R_CreateFcn(hObject, eventdata, handles)
% hObject    handle to motor_R (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ah_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Ah_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ah_edit as text
%        str2double(get(hObject,'String')) returns contents of Ah_edit as a double


% --- Executes during object creation, after setting all properties.
function Ah_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ah_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function batt_V_max_edit_Callback(hObject, eventdata, handles)
% hObject    handle to batt_V_max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of batt_V_max_edit as text
%        str2double(get(hObject,'String')) returns contents of batt_V_max_edit as a double


% --- Executes during object creation, after setting all properties.
function batt_V_max_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to batt_V_max_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in load_pshbtn.
function load_pshbtn_Callback(hObject, eventdata, handles)
% hObject    handle to load_pshbtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filename = get(handles.vehicle_name_load_edit, 'String');
filename = char(strcat('FSAE Vehicle Model\vehicle_models\', filename,'.mat'));
load(filename);

set(handles.vehicle_name_edit, 'String', vehicle_name);

set(handles.time_step_edit, 'String', time_step);

set(handles.mass_edit, 'String', mass);

set(handles.wheel_base_edit, 'String', wheel_base);

set(handles.Cg_z_edit, 'String', Cg_z);

set(handles.power_limit_edit, 'String', power_limit);

set(handles.Cd_edit, 'String', Cd);

set(handles.Cl_edit, 'String', Cl);

set(handles.Af_edit, 'String', Af);

set(handles.Al_edit, 'String', Al);

set(handles.Cr_edit, 'String', Cr);

set(handles.tire_radius_edit, 'String', tire_radius);

set(handles.lateral_g_edit, 'String', lateral_g);

set(handles.max_braking_force_edit, 'String', max_braking_force);

set(handles.coeff_f_edit, 'String', coeff_f);

set(handles.fdr_edit, 'String', fdr);

set(handles.driveline_eff_edit, 'String', driveline_eff);

set(handles.Ah_edit, 'String', Ah);

set(handles.batt_V_max_edit, 'String', batt_V_max);

set(handles.R_int_edit, 'String', R_int);

set(handles.cell_s_edit, 'String', cell_s);

set(handles.cell_p_edit, 'String', cell_p);

set(handles.batt_current_limit_edit, 'String', batt_current_limit);

set(handles.battery_table_edit, 'Data', battery_table);

set(handles.max_motor_power_edit, 'String', max_motor_power);

set(handles.motor_eff_edit, 'String', motor_eff);

set(handles.max_motor_current_edit, 'String', max_motor_current);

set(handles.motor_table, 'Data', motor_table);



% --- Executes on button press in save_pshbtn.
function save_pshbtn_Callback(hObject, eventdata, handles)
% hObject    handle to save_pshbtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vehicle_name = get(handles.vehicle_name_edit, 'String');

time_step = get(handles.time_step_edit, 'String');

mass = get(handles.mass_edit, 'String');

wheel_base = get(handles.wheel_base_edit, 'String');

Cg_z = get(handles.Cg_z_edit, 'String');

power_limit = get(handles.power_limit_edit, 'String');

Cd = get(handles.Cd_edit, 'String');

Cl = get(handles.Cl_edit, 'String');

Af = get(handles.Af_edit, 'String');

Al = get(handles.Al_edit, 'String');

Cr = get(handles.Cr_edit, 'String');

tire_radius = get(handles.tire_radius_edit, 'String');

lateral_g = get(handles.lateral_g_edit, 'String');

max_braking_force = get(handles.max_braking_force_edit, 'String');

coeff_f = get(handles.coeff_f_edit, 'String');

fdr = get(handles.fdr_edit, 'String');

driveline_eff = get(handles.driveline_eff_edit, 'String');

Ah = get(handles.Ah_edit, 'String');

batt_V_max = get(handles.batt_V_max_edit, 'String');

R_int = get(handles.R_int_edit, 'String');

cell_s = get(handles.cell_s_edit, 'String');

cell_p = get(handles.cell_p_edit, 'String');

battery_table = get(handles.battery_table_edit, 'Data');

batt_current_limit = get(handles.batt_current_limit_edit, 'String');

max_motor_power = get(handles.max_motor_power_edit, 'String');

motor_eff = get(handles.motor_eff_edit, 'String');

max_motor_current = get(handles.max_motor_current_edit, 'String');

motor_table = get(handles.motor_table, 'Data');

vehicle_name = get(handles.vehicle_name_edit, 'String');
file = char(strcat('FSAE Vehicle Model\vehicle_models\', vehicle_name, '.mat'));
save(file, 'vehicle_name', 'time_step', 'mass', 'wheel_base', 'Cg_z', 'power_limit', 'Cd', 'Cl', 'Af', 'Al', 'Cr', 'tire_radius', 'lateral_g', 'max_braking_force', 'coeff_f', 'fdr', 'driveline_eff', 'Ah', 'batt_V_max', 'cell_s', 'cell_p', 'R_int', 'battery_table', 'batt_current_limit', 'max_motor_power', 'motor_eff', 'max_motor_current', 'motor_table');



function R_int_edit_Callback(hObject, eventdata, handles)
% hObject    handle to R_int_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R_int_edit as text
%        str2double(get(hObject,'String')) returns contents of R_int_edit as a double


% --- Executes during object creation, after setting all properties.
function R_int_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to R_int_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cell_s_edit_Callback(hObject, eventdata, handles)
% hObject    handle to cell_s_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cell_s_edit as text
%        str2double(get(hObject,'String')) returns contents of cell_s_edit as a double


% --- Executes during object creation, after setting all properties.
function cell_s_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cell_s_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cell_p_edit_Callback(hObject, eventdata, handles)
% hObject    handle to cell_p_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cell_p_edit as text
%        str2double(get(hObject,'String')) returns contents of cell_p_edit as a double


% --- Executes during object creation, after setting all properties.
function cell_p_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cell_p_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vehicle_name_load_edit_Callback(hObject, eventdata, handles)
% hObject    handle to vehicle_name_load_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vehicle_name_load_edit as text
%        str2double(get(hObject,'String')) returns contents of vehicle_name_load_edit as a double


% --- Executes during object creation, after setting all properties.
function vehicle_name_load_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vehicle_name_load_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cl_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Cl_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cl_edit as text
%        str2double(get(hObject,'String')) returns contents of Cl_edit as a double


% --- Executes during object creation, after setting all properties.
function Cl_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cl_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function wheel_base_edit_Callback(hObject, eventdata, handles)
% hObject    handle to wheel_base_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of wheel_base_edit as text
%        str2double(get(hObject,'String')) returns contents of wheel_base_edit as a double


% --- Executes during object creation, after setting all properties.
function wheel_base_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wheel_base_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function track_width_edit_Callback(hObject, eventdata, handles)
% hObject    handle to track_width_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of track_width_edit as text
%        str2double(get(hObject,'String')) returns contents of track_width_edit as a double


% --- Executes during object creation, after setting all properties.
function track_width_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to track_width_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cg_z_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Cg_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cg_z_edit as text
%        str2double(get(hObject,'String')) returns contents of Cg_z_edit as a double


% --- Executes during object creation, after setting all properties.
function Cg_z_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cg_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function max_motor_current_edit_Callback(hObject, eventdata, handles)
% hObject    handle to max_motor_current_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of max_motor_current_edit as text
%        str2double(get(hObject,'String')) returns contents of max_motor_current_edit as a double


% --- Executes during object creation, after setting all properties.
function max_motor_current_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to max_motor_current_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function power_limit_edit_Callback(hObject, eventdata, handles)
% hObject    handle to power_limit_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of power_limit_edit as text
%        str2double(get(hObject,'String')) returns contents of power_limit_edit as a double


% --- Executes during object creation, after setting all properties.
function power_limit_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to power_limit_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function batt_current_limit_edit_Callback(hObject, eventdata, handles)
% hObject    handle to batt_current_limit_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of batt_current_limit_edit as text
%        str2double(get(hObject,'String')) returns contents of batt_current_limit_edit as a double


% --- Executes during object creation, after setting all properties.
function batt_current_limit_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to batt_current_limit_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Al_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Al_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Al_edit as text
%        str2double(get(hObject,'String')) returns contents of Al_edit as a double


% --- Executes during object creation, after setting all properties.
function Al_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Al_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function coeff_f_edit_Callback(hObject, eventdata, handles)
% hObject    handle to coeff_f_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of coeff_f_edit as text
%        str2double(get(hObject,'String')) returns contents of coeff_f_edit as a double


% --- Executes during object creation, after setting all properties.
function coeff_f_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to coeff_f_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function max_motor_power_edit_Callback(hObject, eventdata, handles)
% hObject    handle to max_motor_power_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of max_motor_power_edit as text
%        str2double(get(hObject,'String')) returns contents of max_motor_power_edit as a double


% --- Executes during object creation, after setting all properties.
function max_motor_power_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to max_motor_power_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
