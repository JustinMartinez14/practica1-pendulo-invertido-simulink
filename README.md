Práctica 1 - Péndulo Invertido en Simulink

Materia: Seminario de Modelado y Simulación de Sistemas
Institución: Universidad de Guadalajara - CUCEI
Programa: Ingeniería en Robótica

Descripción

Implementación en Simulink de las ecuaciones diferenciales del sistema de péndulo invertido mediante representación a bloques.

Ecuaciones Implementadas
Ecuación para xc_ddot (aceleración del carro):
xc_ddot = [(Ip + Mp*lp²)*Fc + Mp²*lp²*g*cos(α)*sin(α) - (Ip + Mp*lp²)*Beq*xc_dot] / denominador

Ecuación para alpha_ddot (aceleración angular):
alpha_ddot = [(Mc + Mp)*Mp*g*lp*sin(α) - (Mc + Mp)*Bp*alpha_dot] / denominador

Denominador común:
denominador = (Mc + Mp)*Ip + Mc*Mp*lp² + Mp²*lp²*sin²(α)

Parámetros del Sistema

Momento de inercia (Ip): 0.0079 kg·m²

Masa del carro (Mc): 0.7031 kg

Masa del péndulo (Mp): 0.23 kg

Longitud del péndulo (lp): 0.3302 m

Amortiguamiento equivalente (Beq): 4.3 N·s/m

Amortiguamiento del péndulo (Bp): 0.0024 N·m·s/rad

Condiciones Iniciales

Ángulo inicial: 1°

Posición inicial: 0 m

Velocidades iniciales: 0

Código MATLAB Completo
%% PRÁCTICA 1 - REPRESENTACIÓN A BLOQUES EN SIMULINK
%% Sistema de Péndulo Invertido - VERSIÓN FINAL CORREGIDA
% Seminario de Modelado y Simulación de Sistemas
% Universidad de Guadalajara - CUCEI

clc; clear all; close all;

%% Definición de parámetros del sistema
Ip = 0.0079;    % Momento de inercia del péndulo [kg*m²]
Mc = 0.7031;    % Masa del carro [kg]
lp = 0.3302;    % Distancia del pivot al centro de gravedad [m]
Mp = 0.23;      % Masa del péndulo [kg]
Fc = 0;         % Fuerza aplicada al carro [N]
Beq = 4.3;      % Coeficiente de amortiguamiento equivalente [N*s/m]
g = 9.81;       % Constante gravitacional [m/s²]
Bp = 0.0024;    % Coeficiente de amortiguamiento viscoso del péndulo [N*m*s/rad]

%% Condiciones iniciales
alpha_0 = 1 * pi/180;  % Ángulo inicial [rad] - convertido de grados
xc_0 = 0;              % Posición inicial del carro [m]
alpha_dot_0 = 0;       % Velocidad angular inicial [rad/s]
xc_dot_0 = 0;          % Velocidad inicial del carro [m/s]

%% Crear nuevo modelo de Simulink
model_name = 'pendulo_invertido_bloques';

% Cerrar modelo si existe
try
    close_system(model_name, 0);
catch
    % Si no existe, continuar
end

% Crear nuevo sistema
new_system(model_name);
open_system(model_name);

%% Configurar parámetros de simulación
set_param(model_name, 'Solver', 'ode45');
set_param(model_name, 'StopTime', '10');

%% =======================================================================
%% SECCIÓN 1: BLOQUES DE ENTRADA Y CONSTANTES
%% =======================================================================

% Bloque de entrada Fc (fuerza aplicada)
add_block('simulink/Sources/Constant', [model_name '/Fc_input'], ...
    'Position', [50, 200, 100, 230], 'Value', num2str(Fc));

%% =======================================================================
%% SECCIÓN 2: INTEGRADORES (PRIMERO PARA EVITAR PROBLEMAS DE CONEXIÓN)
%% =======================================================================

% Integradores para xc (posición del carro)
add_block('simulink/Continuous/Integrator', [model_name '/Int_xc_dot'], ...
    'Position', [600, 180, 630, 210], 'InitialCondition', num2str(xc_dot_0));

add_block('simulink/Continuous/Integrator', [model_name '/Int_xc'], ...
    'Position', [650, 180, 680, 210], 'InitialCondition', num2str(xc_0));

% Integradores para alpha (ángulo del péndulo)
add_block('simulink/Continuous/Integrator', [model_name '/Int_alpha_dot'], ...
    'Position', [600, 350, 630, 380], 'InitialCondition', num2str(alpha_dot_0));

add_block('simulink/Continuous/Integrator', [model_name '/Int_alpha'], ...
    'Position', [650, 350, 680, 380], 'InitialCondition', num2str(alpha_0));

%% =======================================================================
%% SECCIÓN 3: BLOQUES TRIGONOMÉTRICOS
%% =======================================================================

% sin(α)
add_block('simulink/Math Operations/Trigonometric Function', [model_name '/Sin_alpha'], ...
    'Position', [200, 350, 230, 380], 'Function', 'sin');

% cos(α)
add_block('simulink/Math Operations/Trigonometric Function', [model_name '/Cos_alpha'], ...
    'Position', [200, 300, 230, 330], 'Function', 'cos');

% sin²(α)
add_block('simulink/Math Operations/Math Function', [model_name '/Sin2_alpha'], ...
    'Position', [280, 350, 310, 380], 'Function', 'square');

%% =======================================================================
%% SECCIÓN 4: CÁLCULO DEL DENOMINADOR COMÚN
%% =======================================================================

% Denominador = (Mc + Mp)*Ip + Mc*Mp*lp² + Mp²*lp²*sin²(α)
den_const = (Mc + Mp)*Ip + Mc*Mp*lp^2;

add_block('simulink/Sources/Constant', [model_name '/Den_constant'], ...
    'Position', [350, 480, 400, 510], 'Value', num2str(den_const));

add_block('simulink/Math Operations/Gain', [model_name '/Gain_sin2_den'], ...
    'Position', [350, 350, 380, 380], 'Gain', num2str(Mp^2*lp^2));

add_block('simulink/Math Operations/Add', [model_name '/Sum_denominator'], ...
    'Position', [450, 400, 480, 430], 'Inputs', '++');

%% =======================================================================
%% SECCIÓN 5: NUMERADOR DE LA ECUACIÓN xc_ddot
%% =======================================================================

% Término 1: (Ip + Mp*lp²)*Fc
add_block('simulink/Math Operations/Gain', [model_name '/Term1_xc'], ...
    'Position', [150, 180, 180, 210], 'Gain', num2str(Ip + Mp*lp^2));

% Término 2: Mp²*lp²*g*cos(α)*sin(α)
add_block('simulink/Math Operations/Product', [model_name '/CosSin_product'], ...
    'Position', [280, 300, 310, 330], 'Inputs', '**');

add_block('simulink/Math Operations/Gain', [model_name '/Term2_xc'], ...
    'Position', [350, 300, 380, 330], 'Gain', num2str(Mp^2*lp^2*g));

% Término 3: -(Ip + Mp*lp²)*Beq*xc_dot
add_block('simulink/Math Operations/Gain', [model_name '/Term3_xc'], ...
    'Position', [350, 180, 380, 210], 'Gain', num2str(-(Ip + Mp*lp^2)*Beq));

% Suma del numerador xc
add_block('simulink/Math Operations/Add', [model_name '/Sum_num_xc'], ...
    'Position', [450, 200, 480, 250], 'Inputs', '+++');

%% =======================================================================
%% SECCIÓN 6: NUMERADOR DE LA ECUACIÓN alpha_ddot
%% =======================================================================

% Término 1: (Mc + Mp)*Mp*g*lp*sin(α)
add_block('simulink/Math Operations/Gain', [model_name '/Term1_alpha'], ...
    'Position', [280, 400, 330, 430], 'Gain', num2str((Mc + Mp)*Mp*g*lp));

% Término 2: -(Mc + Mp)*Bp*α_dot
add_block('simulink/Math Operations/Gain', [model_name '/Term2_alpha'], ...
    'Position', [350, 400, 380, 430], 'Gain', num2str(-(Mc + Mp)*Bp));

% Suma del numerador alpha (versión simplificada)
add_block('simulink/Math Operations/Add', [model_name '/Sum_num_alpha'], ...
    'Position', [450, 350, 480, 400], 'Inputs', '++');

%% =======================================================================
%% SECCIÓN 7: DIVISORES PARA OBTENER LAS ACELERACIONES
%% =======================================================================

% División para xc_ddot
add_block('simulink/Math Operations/Divide', [model_name '/Div_xc'], ...
    'Position', [520, 200, 550, 230]);

% División para alpha_ddot
add_block('simulink/Math Operations/Divide', [model_name '/Div_alpha'], ...
    'Position', [520, 360, 550, 390]);

%% =======================================================================
%% SECCIÓN 8: ELEMENTOS DE VISUALIZACIÓN
%% =======================================================================

% Scopes para visualización
add_block('simulink/Sinks/Scope', [model_name '/Scope_Posicion'], ...
    'Position', [720, 180, 750, 210]);

add_block('simulink/Sinks/Scope', [model_name '/Scope_Angulo'], ...
    'Position', [720, 350, 750, 380]);

% To Workspace para análisis posterior
add_block('simulink/Sinks/To Workspace', [model_name '/ToWS_xc'], ...
    'Position', [720, 220, 770, 250], 'VariableName', 'xc_out', 'SaveFormat', 'Array');

add_block('simulink/Sinks/To Workspace', [model_name '/ToWS_alpha'], ...
    'Position', [720, 390, 770, 420], 'VariableName', 'alpha_out', 'SaveFormat', 'Array');

%% =======================================================================
%% SECCIÓN 9: CONEXIONES DE SEÑALES
%% =======================================================================

% Conexiones de integradores (cadena principal)
add_line(model_name, 'Div_xc/1', 'Int_xc_dot/1');
add_line(model_name, 'Int_xc_dot/1', 'Int_xc/1');
add_line(model_name, 'Div_alpha/1', 'Int_alpha_dot/1');
add_line(model_name, 'Int_alpha_dot/1', 'Int_alpha/1');

% Conexiones trigonométricas desde alpha
add_line(model_name, 'Int_alpha/1', 'Sin_alpha/1');
add_line(model_name, 'Int_alpha/1', 'Cos_alpha/1');
add_line(model_name, 'Sin_alpha/1', 'Sin2_alpha/1');

% Conexiones para cos*sin
add_line(model_name, 'Cos_alpha/1', 'CosSin_product/1');
add_line(model_name, 'Sin_alpha/1', 'CosSin_product/2');

% Construcción del denominador
add_line(model_name, 'Sin2_alpha/1', 'Gain_sin2_den/1');
add_line(model_name, 'Gain_sin2_den/1', 'Sum_denominator/1');
add_line(model_name, 'Den_constant/1', 'Sum_denominator/2');

% Construcción del numerador xc
add_line(model_name, 'Fc_input/1', 'Term1_xc/1');
add_line(model_name, 'Term1_xc/1', 'Sum_num_xc/1');
add_line(model_name, 'CosSin_product/1', 'Term2_xc/1');
add_line(model_name, 'Term2_xc/1', 'Sum_num_xc/2');
add_line(model_name, 'Int_xc_dot/1', 'Term3_xc/1');
add_line(model_name, 'Term3_xc/1', 'Sum_num_xc/3');

% Construcción del numerador alpha
add_line(model_name, 'Sin_alpha/1', 'Term1_alpha/1');
add_line(model_name, 'Term1_alpha/1', 'Sum_num_alpha/1');
add_line(model_name, 'Int_alpha_dot/1', 'Term2_alpha/1');
add_line(model_name, 'Term2_alpha/1', 'Sum_num_alpha/2');

% Divisiones finales
add_line(model_name, 'Sum_num_xc/1', 'Div_xc/1');
add_line(model_name, 'Sum_denominator/1', 'Div_xc/2');
add_line(model_name, 'Sum_num_alpha/1', 'Div_alpha/1');
add_line(model_name, 'Sum_denominator/1', 'Div_alpha/2');

% Conexiones a visualización
add_line(model_name, 'Int_xc/1', 'Scope_Posicion/1');
add_line(model_name, 'Int_alpha/1', 'Scope_Angulo/1');
add_line(model_name, 'Int_xc/1', 'ToWS_xc/1');
add_line(model_name, 'Int_alpha/1', 'ToWS_alpha/1');

%% Organizar bloques automáticamente
try
    Simulink.BlockDiagram.arrangeSystem(model_name);
catch
    fprintf('Nota: No se pudo organizar automáticamente el diagrama\n');
end

%% Guardar el modelo
save_system(model_name);


Autor: Justin Axel Martinez Rocha
Fecha: 15/08/2025
Universidad de Guadalajara - CUCEI
