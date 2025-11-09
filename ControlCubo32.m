%% Conexión al Robotat % CUBO 21 ROBOT 10
robotat = robotat_connect();
% Conexión al agente Pololu 3Pi 
robot_no10 = 14; % Seleccionar el agente específico a emplear
robot_no10_2=10;
% Offsets de los markers (medidos para que el yaw sea cero cuando cada
% robot esté alineado al eje +x del Robotat)
marker_offsets = [-93.7618, -89.7361, -89.8177, -81.9006, -88.8106, ...
    -89.9653, -94.0866, -89.9872, -94.2587, 33.6573]; 

% Se establece la conexión con el robot y se extrae el
% offset respectivo
robot10 = robotat_3pi_connect(robot_no10);


%offset14 = -82.7264;
%offset10 =  31.8544; 
offset10 =  -82.7264;
%%
marker_offsets = [30, robot_no10,32]; 
robotat_trvisualize(robotat, marker_offsets);

%% 
robotat_3pi_set_wheel_velocities(robot10, 40, 40); 
%% % === Control del gripper ===
opcionV = 1;  % Cambia a 1 para abrir, 2 para cerrar, o 0 para no hacer nada
if opcionV == 1   
    robotat_3pi_gripper_open(robot10);
    fprintf('Garra abierta.\n');
elseif opcionV == 2
    robotat_3pi_gripper_close(robot10);
    fprintf('Garra cerrada.\n');
else
    fprintf('Sin acción en la garra.\n');
end

%% CAPTURAR POSICIONES 
poseMarker32 = robotat_get_pose(robotat, 32, 'eulzyx');
%% Visualizar markers 
marker_offsets = [30, robot_no10,21]; 
robotat_trvisualize(robotat, marker_offsets);
%% 
%% ******************CONTROL PARA IR A TRAER EL OBJETO ******************************************
%% Parametros de control
% Pose del robot
xi10 = robotat_get_pose(robotat, robot_no10, 'eulzyx');
xpos10 = xi10(1) * 1000; % en mm
ypos10 = xi10(2) * 1000; % en mm
theta10 = atan2d(sind(xi10(4) - offset10), cosd(xi10(4) - offset10));
xi10PID1 = xi10; % pose inicial del recorrido del PID 

thetag10 = 0; % en rad
ePC3 = inf;
% dimensiones del Pololu
l = 48; % largo en mm
r = 16; % radio ruedas en mm

% PID posición
kpP = 1;
kiP = 0.0001; 
kdP = 0.6;
EP = 0;
eP_1 = 0;

% PID orientación
kiO10 = 0.0;    % menos acumulación
kdO10 = 0.7;    % mejora la amortiguación 
kpO10 = 8;
EO3 = 0;
eO_1_3 = 0;

% Inicialización de variables pa controlador Acercamiento exponencial
alpha10 = 0.000005;    % más sensible al error → desacelera más al acercarse
%v0 = 2000;      % velocidad máxima
v010 = 1500;

% === Configuración de la figura === DESCOMENTAR  FUNCIONA (TENGO FE EN DIOSITO)
figure(1);
hold on;
axis equal; 
xlabel('X [mm]');
ylabel('Y [mm]');
xlim([-1500,800]);
ylim([-1000,1500]);
title('Seguimiento de robot Pololu 3Pi+ para ir a traer el objeto');
grid on;

% Reflejar eje Y
set(gca, 'YDir', 'reverse');

% Dibujar meta inicial
poseMarker32 = robotat_get_pose(robotat, 21, 'eulzyx');
xg32 =  poseMarker32(1)*1000;
yg32 =  poseMarker32(2)*1000;
h_goal32 = plot(xg32, yg32, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
poseMarker30PID1 = poseMarker32; % posicion de pose cubo para ir a traer 

% Dibujar robot inicial (cuerpo circular)
radio_robot = 48; % radio aproximado del cuerpo
h_robot10 = rectangle('Position',[xpos10-radio_robot, ypos10-radio_robot, 2*radio_robot, 2*radio_robot], ...
                    'Curvature',[1,1], 'EdgeColor','b', 'LineWidth',1.5);

% Dibujar ruedas (dos círculos pequeños)
wheel_radius = 16;
offset_wheel = l;
h_wheelL10 = rectangle('Position',[xpos10-offset_wheel-wheel_radius, ypos10-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);
h_wheelR10 = rectangle('Position',[xpos10+offset_wheel-wheel_radius, ypos10-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);

% Dibujar trayectoria
h_trajPID10_1 = animatedline('Color','m','LineWidth',1.5); % Trayectoria robot 14 pero es del primer PID

%% Control  para qué se dirija al objeto 
maxIterPID10_1 = 5000; %DESCOMENTAR SI FUNCIONA
trayectoria10PID_1 = zeros(maxIterPID10_1,2);
idx10_1 = 1;
disp('Comienzo para ir a traer el objeto');
while (ePC3 > 130)
    % PID punto-a-punto con acercamiento exponencial
    disp('Ciclo dentro del WHILE de PID');
    % Pose inicial
    try
        xi10 = robotat_get_pose(robotat, robot_no10, 'eulzyx');
    catch
        disp('Error al obtener la pose del Pololu');
    end

    try
        % meta deseada
        poseMarker32 = robotat_get_pose(robotat, 21, 'eulzyx');
        % xg32 = poseMarker32(1)*1000; % en mm
        % yg32 = poseMarker32(2)*1000; % en mm
    catch
        disp('Error al obtener la pose del Pololu');
        robotat_3pi_force_stop(robot10);
    end

    %posiciónd de meta en mm
    xg32 =poseMarker32(1)*1000;
    yg32 =poseMarker32(2)*1000;

    %posición actual de robot en mm
    x10 = xi10(1) * 1000; % en mm
    y10 = xi10(2) * 1000; % en mm
    theta10 = deg2rad(atan2d(sind(xi10(4) - offset10), cosd(xi10(4) - offset10)));
    
    %errores
    e10 = [xg32 - x10; yg32 - y10];
    thetag10 = atan2(e10(2), e10(1));   
    ePC3 = norm(e10);
    ePpdi1_10 = ePC3;
    eO3 = thetag10 - theta10;
    eO3 = atan2(sin(eO3), cos(eO3));
        
    % Control de velocidad lineal
    kP10 = v010 * (1-exp(-alpha10*ePC3^2)) / ePC3;
    v10 = (kP10*ePC3);
        
    % Control de velocidad angular
    eO_D3 = eO3 - eO_1_3;
    EO3 = EO3 + eO3;
    w10 = kpO10*eO3 + kiO*EO3 + kdO10*eO_D3;
    eO_1_3 = eO3;
        
    % Velocidades para las ruedas
    phi_R_ctrl10 = (v10 + l*w10)/r;
    phi_L_ctrl10 = (v10 - l*w10)/r;

    % Envío de velocidades al robot
    robotat_3pi_set_wheel_velocities(robot10, phi_L_ctrl10, phi_R_ctrl10);

    % === Actualizar visualización === DESCOMENTAR SI FUNCIONA
    addpoints(h_trajPID10_1, x10, y10);
    % === Guardar punto actual de la trayectoria ===
    %trayectoria14PID1 = [trayectoria14PID1; x14, y14];
    trayectoria10PID_1(idx10_1, :) = [x10, y10];
    idx10_1 = idx10_1 + 1;
    % Actualizar cuerpo del robot
    set(h_robot10, 'Position', [x10-radio_robot, y10-radio_robot, 2*radio_robot, 2*radio_robot]);

    title(sprintf('Seguimiento de robot Pololu 3Pi+ para recoger el objeto. \nError P = %.1f mm x = %.2fmm y =%.2fmm',ePC3, x10,y10));
    % Actualizar ruedas según orientación
    xL10 = x10 - offset_wheel*cos(theta10 + pi/2);
    yL10 = y10 - offset_wheel*sin(theta10 + pi/2);
    xR10 = x10 + offset_wheel*cos(theta10 + pi/2);
    yR10 = y10 + offset_wheel*sin(theta10 + pi/2);
    set(h_wheelL10, 'Position', [xL10-wheel_radius, yL10-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
    set(h_wheelR10, 'Position', [xR10-wheel_radius, yR10-wheel_radius, 2*wheel_radius, 2*wheel_radius]);

    % Actualizar goal (en caso de moverse)
    set(h_goal32, 'XData', xg32, 'YData', yg32); 
    drawnow;
end
% Si ya está suficientemente cerca, detener el robot y salir del loop
disp('Meta alcanzada. Deteniendo robot...');
robotat_3pi_force_stop(robot10);
robotat_3pi_gripper_close(robot10);
%%
robotat_3pi_force_stop(robot10);

%% ***********CONTROL PARA EVACIÓN DE OBSTÁCULOS***********************
%% % === Control del gripper ===
opcionV = 1;  % Cambia a 1 para abrir, 2 para cerrar, o 0 para no hacer nada

if opcionV == 1
    robotat_3pi_gripper_open(robot10);
    fprintf('Garra abierta.\n');
elseif opcionV == 2
    robotat_3pi_gripper_close(robot10);
    fprintf('Garra cerrada.\n');
else
    fprintf('Sin acción en la garra.\n');
end


%% TOMAR GOAL PARA PLANIFICACIÓN Y PARA PUNTO INTERMEDIO ALINEACIÓN
%meta para planificacion 
meta_id=21;
try
    goal32=robotat_get_pose(robotat, meta_id, 'eulxyz');
catch 
end 
%goalCubo = goal;
%goal=[0, 0];
goal32=goal32(1:2)*100; %cm
goal32(1)=-goal32(1)+200;
goal32(2)=goal32(2)+250;
%goal=[200,250];  % Goal ejemplo
goal32=round(goal32,0);

%% GOAL PUNTO INTERMEDIO
metaInter32=21;
try
    goalInterM32=robotat_get_pose(robotat, metaInter32, 'eulxyz');
catch 
end 
goalInter32 = goalInterM32;
%TOMAR POISICION COMO SE VA A JUNTAR LE MARKER 

%% GOAL FORMACION
metaFormacion32=21;
try
    goalFormacion32=robotat_get_pose(robotat, metaFormacion32, 'eulxyz');
catch  
end 
goalCubo32 = goalFormacion32;


%% Definición de obstáculos y creación del mapa
% Se definen uno por uno los obstáculos, en caso de querer deshabilitar
% alguno igualar a un vector vacío
ids3 = [71 64 65 66 75 61 72 74 68 70  63 73  67 69];
%obs0=[0 0];
% Inicialización del array de obstáculos
%obs = [obs0];
obs3=[];
% Se recorre cada ID y se obtiene su posición con robotat_get_pose
for i3 = 1:length(ids3)
    pm3 = robotat_get_pose(robotat, ids3(i3), 'eulxyz');
    
    % Validación por si no se puede obtener la posición
    if ~isempty(pm3)
        % Se agrega la posición (x, y) al array de obstáculos
        obs3(end+1, :) = pm3(1:2);
    end
end

% Se emplea el array de obstáculos para crear y visualizar el mapa (bordes
% de plataforma + obstáculos) como un occupancy grid
map3 = genmap(obs3);
figure(2);
imshow(map3);
title('Mapa actual Robotat', ...
    'FontSize', 14, 'Interpreter', 'latex');
xlabel('$x$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');
ylabel('$y$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');

%legend();
axis on;

%% PLANIFICACIÓN DE MOVIMIENTO
start10=robotat_get_pose(robotat, robot_no10, 'eulzyx');
start10=start10(1:2);
start10(1)=-start10(1)*100+200;
start10(2)=start10(2)*100+250;
start10=round(start10,0);
%start=[50,50]; 
% Crear objeto para planificación
dx3 = Dstar(map3,'inflate', 8);
%dx = DXform(map);
% Establecer meta
dx3.plan(goal32);

% Consultar ruta desde el inicio (convertido también si es necesario)
%start_cell = place(start);
%dx.query(start_cell, 'animate');

figure(3);  % ← Figura exclusiva para animación de D*
set(gcf, 'Name', sprintf('D* Planificación - Robot %d', robot_no10));
p3=dx3.query(start10, 'animate');
%input("Presiona enter para continuar")

%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot10);


%% Parametros control:CICLO DE CONTROL (No lineal) MÁS EJECUCIÓN DE PLANIFICACIÓN

% --- Parámetros del Pololu
R = 32;      % radio de rueda en mm
L = (96-6.8)/2;        % distancia entre ejes en mm
% Campo vectorial del sistema dinámico
f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

% Pure pursuit
ePtol3 = 40; %mm
eOtol3=0.00174533*100; %0.1*n grado xd
eP3=1;
eO3=pi;

%No lineal
% Control no lineal de pose
k_rho3 = 6;
k_alpha3 = 40;
k_beta3 = -4;

% --- Configuración pure pursuit ---

i3     = 1;      % índice inicial en p_world
lookahead_dist3 = 100;   % mm de look-ahead (ajusta según tu dinámica)
idx_goal3      = 1;
n_pts3         = size(p3,1);
align_tol3  = eOtol3*1.5;   % si el error angular > 10°
puntosA3 = floor(n_pts3 *0.9);  % por ejemplo 70% del camino

%% Crear figura para visualización
% close all;
% figure(4); % comentar cuando funcione lo otro
% axis equal; grid on;
% xlabel('X [mm]'); ylabel('Y [mm]');
% plot(10*(-p3(:,1)+200), 10*(p3(:,2)-250));
% xlim([-2000,2000]);
% ylim([-3000,3000]);
% hold on;
% % Preparar interrupción por teclad
% disp('Presiona ENTER en la consola para detener el robot manualmente.')
% stop_flag = false;
% listener = @(~,~) assignin('base', 'stop_flag', true);
% set(gcf, 'KeyPressFcn', listener);
% 
% 
% % Usamos animatedline para la trayectoria
% h_traj3 = animatedline('Color','b','LineWidth',1.5);
% h_meta3 = animatedline('Marker','*','Color','r');

figure(4);
axis equal; grid on;
xlabel('X [mm]'); ylabel('Y [mm]');

% === TRANSFORMACIÓN Y REFLEJO ===
p3_plot = [10*(-p3(:,1) + 200), 10*(p3(:,2) - 250)];
p3_plot(:,1) = p3_plot(:,1);  % ✅ reflejar en X

% === GRAFICAR ===
plot(p3_plot(:,1), p3_plot(:,2), 'k'); hold on;

% === AJUSTES DE EJES ===
set(gca, 'YDir', 'reverse');   % ✅ eje Y positivo hacia abajo
xlim([-3000,500]);
ylim([-3000,800]);

% === INTERACTIVIDAD ===
disp('Presiona ENTER en la consola para detener el robot manualmente.');
stop_flag = false;
listener = @(~,~) assignin('base', 'stop_flag', true);
set(gcf, 'KeyPressFcn', listener);

% === ELEMENTOS ANIMADOS ===
h_traj10 = animatedline('Color','b','LineWidth',1.5);
h_meta10 = animatedline('Marker','*','Color','r');

try
xi10   = robotat_get_pose(robotat, robot_no10, 'eulzyx');
xpos10 = xi10(1)*1000;
ypos10 = xi10(2)*1000;
theta10 = deg2rad(xi10(4) - offset10);       % yaw en rad
catch
end 

flag_control_cubo3 = false; % antes del while true

trayectoria10 = [];
maxIterNonLin10 = 5000;
trayectoria10NonLin10 = zeros(maxIterNonLin10,2);
idx10_2 = 1;
while true
    try
        % 1) pose objetivo
        %pm = robotat_get_pose(robotat, id, 'eulxyz');
        %pm=[0,0];
        %pm ahora debería de ser cada punto de la planificación xd
        % ==== Selección dinámica de índice por pure-pursuit ====
        % Avanza idx_goal hasta que el punto esté a ≥ lookahead_dist
        while idx_goal3 < n_pts3
            % calcula posición candidata en mm
            pm_tmp3 = p3(idx_goal3,:);
            pm_tmp3(1) = -pm_tmp3(1) + 200;
            pm_tmp3(2) =  pm_tmp3(2) - 250;
            pm_tmp3    = 10 * pm_tmp3;     % pasa a mm
            dx3 = pm_tmp3(1) - xpos10;
            dy3 = pm_tmp3(2) - ypos10;
            if hypot(dx3,dy3) >= lookahead_dist3
                break;
            end
            idx_goal3 = idx_goal3 + 1;
        end
        % Ahora sí, usamos idx_goal como i
        i3 = idx_goal3;
        % Recupera tu pm igual que antes
        pm3 = p3(i3,:);
        pm3(1) = -pm3(1) + 200;
        pm3(2) =  pm3(2) - 250;
        pm3    = 10 * pm3;      % mm
        xg32 = pm3(1);
        yg32 = pm3(2);
        % ==== fin selección pura ====
        
        xg32 = pm3(1);
        yg32 = pm3(2);

        % 2) pose actual
        try 
            xi10   = robotat_get_pose(robotat, robot_no10, 'eulzyx');
            xpos10 = xi10(1)*1000;
            ypos10 = xi10(2)*1000;
            theta10 = deg2rad(xi10(4) - offset10);       % yaw en rad
        catch 
        end 
        % 3) errores
        eNL3  = [xg32-xpos10; yg32-ypos10];
        ePNL3 = norm(eNL3);
        thetagNL3 = atan2(eNL3(2), eNL3(1));
        eONL3 = thetagNL3 - theta10;
        eONL3 = atan2(sin(eONL3), cos(eONL3));
        
        % 4) Control No lineal

        beta3 = -theta10 - eONL3;
        beta3 = atan2(sin(beta3), cos(beta3));

        v10 = k_rho3*ePNL3;
        w10 = k_alpha3*eONL3 + k_beta3*(beta3 + thetagNL3);  
        
        % 6) bloqueo de avance si no estás casi alineado
        max_rpm10 = 1200;  % límite de tus motores
        min_rpm10=100;
        if abs(eONL3) > align_tol3
            % solo gira para alinearse
            v10 = 0;
            min_rpm10=0; 
        end
        if (abs(ePNL3)<=ePtol3)
            v10=0;
            min_rpm10=0;
        end
        if (abs(eONL3)<=eOtol3)
            w10=0;
            min_rpm10=0;
        end
        u=[v10;-w10];
        % 7) ruedas (rad/s → RPM)
        omega_r10 = (u(1) + (L/2)*u(2))/R;
        omega_l10 = (u(1) - (L/2)*u(2))/R;
        rpm_r10 = omega_r10;
        rpm_l10 = omega_l10;

        % Rueda derecha
        rpm_r10 = sign(rpm_r10) * min( max(abs(rpm_r10), min_rpm10), max_rpm10 );
        rpm_l10 = sign(rpm_l10) * min( max(abs(rpm_l10), min_rpm10), max_rpm10 );

        % 9) enviar velocidades
        robotat_3pi_set_wheel_velocities(robot10, rpm_r10, rpm_l10);
        robotat_3pi_gripper_close(robot10);

        % % 10) añadir punto a la trayectoria COMENTAR 10-11 FUNCIONANDO
        % addpoints(h_traj3, xpos10, ypos10);
        % 
        % % 11) dibujar meta y robot (sin borrar la trayectoria)
        % % Nota: al estar 'hold on', trazamos encima de h_traj
        % clearpoints(h_meta3);           % borra el punto anterior
        % addpoints(h_meta3, xg32, yg32);
        % % Guardar el punto en la variable (historial)
        % % trayectoria10(end+1, :) = [xpos10, ypos10];
        % % title(sprintf('Error P=%.1f mm, E_O=%.2f rad', ePNL3, eONL3));
        % % drawnow limitrate;

        % 10) añadir punto a la trayectoria DESCOMENTAR FUNCIONANDO 
        % addpoints(h_traj14, xpos14, ypos14);
        % === Transformar y reflejar la posición real del robot (en mm) ===
        x_plot10 = xpos10; %+ 2000;   % equivalente a 10*(-x + 200) pero ya en mm
        y_plot10 =  ypos10;% - 2500;   % equivalente a 10*(y - 250)
        %x_plot10 = -x_plot10;          % ✅ reflejar en X  
        % === Agregar punto transformado ===
        addpoints(h_traj10, x_plot10, y_plot10);

        % % 11) dibujar meta y robot (sin borrar la trayectoria)
        % Meta actual reflejada
        xg_plot10 = xg32;   % p1 está en metros → *1000 a mm
        yg_plot10 = yg32;
        %xg_plot10 = -xg_plot10;                   % ✅ reflejar en X

        % === GRAFICAR ===
        clearpoints(h_meta10);
        addpoints(h_meta10, xg_plot10, yg_plot10);

        % === GUARDAR TRAYECTORIA TRANSFORMADA ===
        %trayectoria14NonLin(end+1,:) = [x_plot, y_plot];
        trayectoria10NonLin10(idx10_2, :) = [x_plot10, y_plot10];
        idx10_2 = idx10_2 + 1;

       title(sprintf('Trayectoria actual del robot hacia punto final de planificación.\nError P = %.1f mm, E_O = %.2f rad', ePNL3, eONL3), ...
      'Interpreter','tex', 'HorizontalAlignment','center');
        drawnow limitrate;
     

        % 12) Parada automática o manual       
        % ... dentro del while true, reemplaza la parte de parada automática por:
        if (abs(eONL3) <= eOtol3) || stop_flag
            if stop_flag
                disp('Detenido manualmente con ENTER');
                robotat_3pi_force_stop(robot10);
                robotat_3pi_gripper_open(robot10);
                break;
            elseif (idx_goal3 > puntosA3) && ~flag_control_cubo3
                % Llegaste al tramo final y aún no has pasado a control del cubo.
                robotat_3pi_force_stop(robot10);
                robotat_3pi_gripper_close(robot10);
                disp('Objetivo alcanzado, cambiando a control del cubo...');
                pause(0.5);
                disp('Comienzo');
                flag_control_cubo3 = true; % marcar para que no vuelva a entrar
        
                % Actualiza pose antes de entrar al while
                try
                    xi10   = robotat_get_pose(robotat, robot_no10, 'eulzyx');
                    xpos10 = xi10(1) * 1000;
                    ypos10 = xi10(2) * 1000;
                catch
                end

                ePC3 = inf; 

                % % === Inicializar visualización y almacenamiento de
                % trayectoria === DESCOMENTAR FUNCIONANDO 
                figure(5);
                clf; hold on; grid on; axis equal;
                xlabel('X [mm]');
                ylabel('Y [mm]');
                set(gca, 'YDir', 'reverse');
                title('Trayectoria a punto intermedio y punto de formación final');
                xlim([-2000 500]);
                ylim([-3000 2000]);

                % Dibujar goals
                plot(goalInter32(1)*1000, goalInter32(2)*1000, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g','LineWidth', 1.5, 'DisplayName', 'Goal Intermedio');
                plot(goalCubo32(1)*1000, goalCubo32(2)*1000, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g','LineWidth', 1.5, 'DisplayName', 'Goal Cubo');

                % === Inicializar robot ===
                radio_robot = 48;
                wheel_radius = 16;
                offset_wheel = l;
                h_robot10 = rectangle('Position',[xpos10-radio_robot, ypos10-radio_robot, 2*radio_robot, 2*radio_robot], ...
                                    'Curvature',[1,1], 'EdgeColor','b', 'LineWidth',1.5);
                h_wheelL10 = rectangle('Position',[xpos10-offset_wheel-wheel_radius, ypos10-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);
                h_wheelR10 = rectangle('Position',[xpos10+offset_wheel-wheel_radius, ypos10-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);

                % animated line (misma para ambas trayectorias para continuidad visual)
                h_trajPID10_2_3 = animatedline('Color','m','LineWidth',1.5, 'DisplayName', 'Trayectorias PID2+PID3');

                % prealoca PID2:
                maxIterPID10 = 8000;
                trayectoria10PID_2 = zeros(maxIterPID10,2);
                idx10_3 = 1;
                % prealocar PID3 
                trayectoria10PID_3 = zeros(maxIterPID10,2);
                idx10_4 = 1;

                % bucle de acercamiento al goal (actualizamos pose dentro)

                while (ePC3> 125)
                    % PID punto-a-punto con acercamiento exponencial
                    disp('Ciclo dentro del WHILE');
                    % Pose inicial
                    try
                        xi10 = robotat_get_pose(robotat, robot_no10, 'eulzyx');
                    catch
                        disp('Error al obtener la pose del Pololu');
                    end
                    xi10PID2 = xi10; % Para guardar donde inicia la posicion en el segundo PID
                   
                    xg32 = goalInter32(1)*1000; % en mm
                    yg32 = goalInter32(2)*1000; % en mm
                    x10 = xi10(1) * 1000; % en mm
                    y10 = xi10(2) * 1000; % en mm
                    theta10 = deg2rad(atan2d(sind(xi10(4) - offset10), cosd(xi10(4) - offset10)));
                    
                    %errores
                    e3 = [xg32 - x10; yg32 - y10];
                    thetag3 = atan2(e3(2), e3(1));   
                    ePC3 = norm(e3);
                    ePpid2_10 = ePC3;
                    eO3 = thetag3 - theta10;
                    eO3 = atan2(sin(eO3), cos(eO3));
                        
                    % Control de velocidad lineal
                    kP10 = v010 * (1-exp(-alpha10*ePC3^2)) / ePC3;
                    v10 = (kP10*ePC3);
                        
                    % Control de velocidad angular
                    eO_D3 = eO3 - eO_1_3;
                    EO3 = EO3 + eO3;
                    w10 = kpO10*eO3 + kiO*EO3 + kdO10*eO_D3;
                    eO_1_3 = eO3;
                        
                    % Velocidades para las ruedas
                    phi_R_ctrl10 = (v10 + l*w10)/r;
                    phi_L_ctrl10 = (v10 - l*w10)/r;
                
                    % Envío de velocidades al robot
                    robotat_3pi_set_wheel_velocities(robot10, phi_L_ctrl10, phi_R_ctrl10); 

                    % === Actualizar dibujo === DESCOMENTAR FUNCIONANDO
                    %actualizar trayectoria robot
                    addpoints(h_trajPID10_2_3, x10, y10);
                    
                    %Guardar trayectroia PID3
                    if idx10_3 <= maxIterPID10 %Antes de guardar un nuevo punto, se verifica que aún haya espacio en el vector.
                        trayectoria10PID_2(idx10_3 , :) = [x10, y10];
                        idx10_3  = idx10_3  + 1;
                    end

                    % actualizar dibujo del robot 
                    set(h_robot10, 'Position', [x10-radio_robot, y10-radio_robot, 2*radio_robot, 2*radio_robot]);
                    xL10 = x10 - offset_wheel*cos(theta10 + pi/2);
                    yL10 = y10 - offset_wheel*sin(theta10 + pi/2);
                    xR10 = x10 + offset_wheel*cos(theta10 + pi/2);
                    yR10 = y10 + offset_wheel*sin(theta10 + pi/2);
                    set(h_wheelL10, 'Position', [xL10-wheel_radius, yL10-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
                    set(h_wheelR10, 'Position', [xR10-wheel_radius, yR10-wheel_radius, 2*wheel_radius, 2*wheel_radius]);

                    drawnow limitrate;

                end

                 % recortar trayectoria PID2 DESCOMENTAR FUNCIONANDO
                if idx10_3>1
                    trayectoria10PID_2 = trayectoria10PID_2(1:idx10_3-1, :);
                else
                    trayectoria10PID_2 = zeros(0,2);
                end

                % Si ya está suficientemente cerca, detener el robot y salir del loop
                disp('Meta alcanzada :D, punto intermedio');
                robotat_3pi_force_stop(robot10);
                robotat_3pi_gripper_close(robot10);
                pause(0.5);

                % CAMINO HACIA FORMACIÓN 
                % Actualiza pose antes de entrar al while
                try
                    xi10   = robotat_get_pose(robotat, robot_no10, 'eulzyx');
                    xpos10 = xi10(1) * 1000;
                    ypos10 = xi10(2) * 1000;
                catch
                end
                xi10PID3 = xi10; % Para guardar donde inicia la posicion en el tercer PID

                ePC3 = inf; 
                alpha10_1 = 0.000003;
                % bucle de acercamiento al fromacion (actualizamos pose dentro)
                while (ePC3 > 130)
                    % PID punto-a-punto con acercamiento exponencial
                    disp('Ciclo while para fomración');
                    % Pose inicial
                    try
                        xi10 = robotat_get_pose(robotat, robot_no10, 'eulzyx');
                    catch
                        disp('Error al obtener la pose del Pololu');
                    end

                    xg32 = goalCubo32(1)*1000; % en mm
                    yg32 = goalCubo32(2)*1000; % en mm
                    x10 = xi10(1) * 1000; % en mm
                    y10 = xi10(2) * 1000; % en mm
                    theta10 = deg2rad(atan2d(sind(xi10(4) - offset10), cosd(xi10(4) - offset10)));
                    % errores
                    e3 = [xg32 - x10; yg32 - y10];
                    thetag3 = atan2(e3(2), e3(1));   
                    ePC3 = norm(e3);
                    ePpid3_10 = ePC3;
                    eO3 = thetag3 - theta10;
                    eO3 = atan2(sin(eO3), cos(eO3));
                        
                    % Control de velocidad lineal
                    kP10 = v010 * (1-exp(-alpha10_1*ePC3^2)) / ePC3;
                    v10 = (kP10*ePC3);
                        
                    % Control de velocidad angular
                    eO_D3 = eO3 - eO_1_3;
                    EO3 = EO3 + eO3;
                    w10 = kpO10*eO3 + kiO*EO3 + kdO10*eO_D3;
                    eO_1_3 = eO3;
                        
                    % Velocidades para las ruedas
                    phi_R_ctrl10 = (v10 + l*w10)/r;
                    phi_L_ctrl10 = (v10 - l*w10)/r;
                
                    % Envío de velocidades al robot
                    robotat_3pi_set_wheel_velocities(robot10, phi_L_ctrl10, phi_R_ctrl10);

                    % === Actualizar dibujo de trayectoria === DESCOMENTAR FUNCIONANDO
                    addpoints(h_trajPID10_2_3, x10, y10);

                    %Guardar trayectoria PID3
                    if idx10_4 <= maxIterPID10 %Antes de guardar un nuevo punto, se verifica que aún haya espacio en el vector.
                        trayectoria10PID_3(idx10_4 , :) = [x10, y10];
                        idx10_4  = idx10_4  + 1;
                    end

                    % actualizar dibujo del robot 
                    set(h_robot10, 'Position', [x10-radio_robot, y10-radio_robot, 2*radio_robot, 2*radio_robot]);
                    xL10 = x14 - offset_wheel*cos(theta10 + pi/2);
                    yL10 = y14 - offset_wheel*sin(theta10 + pi/2);
                    xR10 = x14 + offset_wheel*cos(theta10 + pi/2);
                    yR10 = y14 + offset_wheel*sin(theta10 + pi/2);
                    set(h_wheelL10, 'Position', [xL10-wheel_radius, yL10-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
                    set(h_wheelR10, 'Position', [xR10-wheel_radius, yR10-wheel_radius, 2*wheel_radius, 2*wheel_radius]);

                    drawnow limitrate;


                end
                % recortar trayectoria PID3 DESCOMENTAR SI FUNCIONA
                if idx10_4>1
                    trayectoria10PID_3 = trayectoria10PID_3(1:idx10_4-1, :);
                else
                    trayectoria10PID_3 = zeros(0,2);
                end

                % Si ya está suficientemente cerca, detener el robot y salir del loop
                disp('Meta alcanzada en formación. Deteniendo robot ');
                robotat_3pi_force_stop(robot10);
                robotat_3pi_gripper_open(robot10);

                pause(0.5);
                disp('Retrocediendo...');
                robotat_3pi_set_wheel_velocities(robot10, -40, -40);
                pause(3);
                robotat_3pi_force_stop(robot10);

                pause(0.05);
                ErrorF3=[eP3;eO_1_3];
                rpmNL3=[rpm_r10;rpm_l10];
                disp("Error Posición y Angular control PID final")
                disp(ErrorF3)
                disp("Velocidades del PID final")
                disp(rpmNL3)
                break;
            end
        end


        pause(0.05);
        ErrorNL=[ePNL3;eONL3];
        rpmNL3=[rpm_r10;rpm_l10];
        disp("Error Posición y Angular")
        disp(ErrorNL)
        disp("Velocidades")
        disp(rpmNL3)

    catch ME
        % Esto se ejecuta si dentro del try ocurre cualquier error
        warning(ME.identifier, "Ocurrió un error, deteniendo el robot: %s", ME.message);
        robotat_3pi_set_wheel_velocities(robot10, 0, 0);
        %input("Presiona enter para continuar"
       
    end


end
    % === GUARDAR FIGURAS COMO .FIG ===
try
    % Crear carpeta si no existe
    folder_name = sprintf('resultados_robotFinales_14 pero debería ser %d', robot_no10_2);
    if ~exist(folder_name, 'dir')
        mkdir(folder_name);
    end

    % Crear subcarpeta para las variables dentro de resultados
    folder_vars = fullfile(folder_name, "VariablesFinales10");
    if ~exist(folder_vars, 'dir')
        mkdir(folder_vars);
    end
    
    % Crear timestamp para identificar ejecuciones distintas
    timestamp = string(datetime("now", "Format", "yyyy-MM-dd_HH-mm-ss"));

    % Guardar figuras
    figure(1);
    savefig(fullfile(folder_name, sprintf('TrayectoriaPrimerPID_%d_%s10.fig', robot_no10_2, timestamp)));

    figure(2);
    savefig(fullfile(folder_name, sprintf('mapa_robot_%d_%s10.fig', robot_no10_2, timestamp)));

    figure(3);
    savefig(fullfile(folder_name, sprintf('planificacion_robot_10%d_%s.fig', robot_no10_2, timestamp)));

    figure(4);
    savefig(fullfile(folder_name, sprintf('trayectoria_robot_10%d_%s.fig', robot_no10_2, timestamp)));
    
    figure(5); %DESCOMENTAR SI FUNCIONA
    savefig(fullfile(folder_name, sprintf('trayectoria_robot_PID2_3_%d_%s.fig', robot_no10_2, timestamp)));
    
    % Guardar variables dentro de la subcarpeta "Variables"
    save(fullfile(folder_vars, sprintf('Variables10%d_%s.mat', robot_no10, timestamp)));

    fprintf('✅ Figuras .fig guardadas correctamente en la carpeta10 "%s".\n', folder_name);

catch ME
    warning(ME.identifier,'⚠️ No se pudieron guardar las figuras10 .fig: %s', ME.message);
end
% Limpieza del flag y función
clear stop_flag
set(gcf, 'KeyPressFcn', '');

%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot10);


%% Desconexión del Robotat
robotat_disconnect(robotat);

%% Desconexión del Pololu 3Pi
robotat_3pi_disconnect(robot10);

