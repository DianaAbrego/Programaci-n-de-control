%% Conexión al Robotat  %% CUBO 20 ROBOT 14
robotat = robotat_connect();

% Conexión al agente Pololu 3Pi 
robot_no14 = 14; % Seleccionar el agente específico a emplear

% Se establece la conexión con el robot y se extrae el offset respectivo
robot14 = robotat_3pi_connect(robot_no14); 
offset14 = -82.7264;

%% 
robotat_3pi_set_wheel_velocities(robot14, 40, 40); 
%% % === Control del gripper ===
opcionV =1;  % Cambia a 1 para abrir, 2 para cerrar, o 0 para no hacer nada
if opcionV == 1   
    robotat_3pi_gripper_open(robot14);
    fprintf('Garra abierta.\n');
elseif opcionV == 2
    robotat_3pi_gripper_close(robot14);
    fprintf('Garra cerrada.\n');
else
    fprintf('Sin acción en la garra.\n');
end

%% CAPTURAR POSICIONES 
%poseMarker = robotat_get_pose(robotat, 30, 'eulzyx');
%% Visualizar markers 
marker_offsets = [20, robot_no14]; 
robotat_trvisualize(robotat, marker_offsets);
%% 
%% ******************CONTROL PARA IR A TRAER EL OBJETO ******************************************
%% Parametros de control
% Pose del robot
xi14 = robotat_get_pose(robotat, robot_no14, 'eulzyx');
xpos14 = xi14(1) * 1000; % en mm
ypos14 = xi14(2) * 1000; % en mm
theta14 = atan2d(sind(xi14(4) - offset14), cosd(xi14(4) - offset14));
xi14PID1 = xi14; % pose inicial del recorrido del PID 

thetag1 = 0; % en rad 
ePC1 = inf;
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
kiO = 0.0;    % menos acumulación
kdO = 0.7;    % mejora la amortiguación 
kpO = 8;
EO1 = 0;
eO_1 = 0;

% Inicialización de variables pa controlador Acercamiento exponencial
alpha14 = 0.000005;    % más sensible al error → desacelera más al acercarse
%v0 = 2000;      % velocidad máxima
v014 = 1500;
% === Configuración de la figura ===
figure(1);
hold on;
axis equal; 
xlabel('X [mm]');
ylabel('Y [mm]');
xlim([-800,800]);
ylim([-800,900]);

grid on;

% Reflejar eje Y
set(gca, 'YDir', 'reverse');

% Dibujar meta inicial
poseMarker20 = robotat_get_pose(robotat, 20, 'eulzyx');
xg20 = poseMarker20(1)*1000;
yg20 = poseMarker20(2)*1000;
h_goal20 = plot(xg20, yg20, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
poseMarker20PID1 = poseMarker20; % posicion de pose cubo para ir a traer 

% Dibujar robot inicial (cuerpo circular)
radio_robot = 48; % radio aproximado del cuerpo
h_robot14 = rectangle('Position',[xpos14-radio_robot, ypos14-radio_robot, 2*radio_robot, 2*radio_robot], ...
                    'Curvature',[1,1], 'EdgeColor','b', 'LineWidth',1.5);

% Dibujar ruedas (dos círculos pequeños)
wheel_radius = 16;
offset_wheel = l;
h_wheelL14 = rectangle('Position',[xpos14-offset_wheel-wheel_radius, ypos14-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);
h_wheelR14 = rectangle('Position',[xpos14+offset_wheel-wheel_radius, ypos14-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);

% Dibujar trayectoria
h_trajPID14_1 = animatedline('Color','m','LineWidth',1.5); % Trayectoria robot 14 pero es del primer PID






%% Control  para qué se dirija al objeto 

maxIterPID14_1 = 5000;
trayectoria14PID_1 = zeros(maxIterPID14_1,2);
idx14_1 = 1;
subtitleObj = subtitle(' ', 'Interpreter', 'latex', 'FontSize', 12);
disp('Comienzo para ir a recoger el objeto');
while (ePC1 > 135)
    % PID punto-a-punto con acercamiento exponencial
    disp('Ciclo dentro del WHILE de PID');
    % Pose inicial
    try
        xi14 = robotat_get_pose(robotat, robot_no14, 'eulzyx');
    catch
        disp('Error al obtener la pose del Pololu');
    end

    try
        % meta deseada
        poseMarker20 = robotat_get_pose(robotat, 20, 'eulzyx');
        % xg20 = poseMarker20(1)*1000; % en mm
        % yg20 = poseMarker20(2)*1000; % en mm
    catch
        disp('Error al obtener la pose del Marcador');
        robotat_3pi_force_stop(robot14);
    end

  
    %posiciónd de meta en mm
    xg20 =poseMarker20(1)*1000;
    yg20 =poseMarker20(2)*1000;

    %posición actual de robot en mm
    x14 = xi14(1) * 1000; % en mm
    y14 = xi14(2) * 1000; % en mm
    theta14 = deg2rad(atan2d(sind(xi14(4) - offset14), cosd(xi14(4) - offset14)));

    e1 = [xg20 - x14; yg20 - y14];
    thetag1 = atan2(e1(2), e1(1));   
    ePC1 = norm(e1);
    ePpid1_14 = ePC1;  % Para guardar el error 
    eO1 = thetag1 - theta14;
    eO1 = atan2(sin(eO1), cos(eO1));
        
    % Control de velocidad lineal
    kP14 = v014 * (1-exp(-alpha14*ePC1^2)) / ePC1;
    v14 = (kP14*ePC1);
        
    % Control de velocidad angular
    eO_D1_1 = eO1 - eO_1;
    EO1 = EO1 + eO1;
    w14 = kpO*eO1 + kiO*EO1 + kdO*eO_D1_1;
    eO_1 = eO1;
        
    % Velocidades para las ruedas
    phi_R_ctrl14 = (v14 + l*w14)/r;
    phi_L_ctrl14 = (v14 - l*w14)/r;

    % === 📋 IMPRIMIR VALORES DE INTERÉS ===
    fprintf('Error lineal: %.2f mm | Error angular: %.2f rad | v: %.2f mm/s | w: %.2f rad/s | phiL: %.2f | phiR: %.2f\n', ...
            ePC1, eO1, v14, w14, phi_L_ctrl14, phi_R_ctrl14);
  
    % Envío de velocidades al robot
    robotat_3pi_set_wheel_velocities(robot14, phi_L_ctrl14, phi_R_ctrl14);

    % === Actualizar visualización ===
    addpoints(h_trajPID14_1, x14, y14);
    % === Guardar punto actual de la trayectoria ===
    trayectoria14PID_1(idx14_1, :) = [x14, y14];
    idx14_1 = idx14_1 + 1;
    % Actualizar cuerpo del robot
    set(h_robot14, 'Position', [x14-radio_robot, y14-radio_robot, 2*radio_robot, 2*radio_robot]);
    
    title(sprintf('Seguimiento de robot Pololu 3Pi+ para ir a traer el objeto. \nError P = %.1f mm x = %.2fmm y =%.2fmm',ePC1, x14,y14));

    % Actualizar ruedas según orientación
    xL14 = x14 - offset_wheel*cos(theta14 + pi/2);
    yL14 = y14 - offset_wheel*sin(theta14 + pi/2);
    xR14 = x14 + offset_wheel*cos(theta14 + pi/2);
    yR14 = y14 + offset_wheel*sin(theta14 + pi/2);
    set(h_wheelL14, 'Position', [xL14-wheel_radius, yL14-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
    set(h_wheelR14, 'Position', [xR14-wheel_radius, yR14-wheel_radius, 2*wheel_radius, 2*wheel_radius]);

    % Actualizar goal (en caso de moverse)
    set(h_goal20, 'XData', xg20, 'YData', yg20); 
    drawnow;
    %disp error y velcoidades phi 
    
end
% recortar trayectoria PID1 al tamaño usado
if exist('trayectoria14PID_1','var')
    trayectoria14PID_1 = trayectoria14PID_1(1:idx14_1-1, :);
end
% Si ya está suficientemente cerca, detener el robot y salir del loop
disp('Meta alcanzada. Deteniendo robot...');
robotat_3pi_force_stop(robot14);
robotat_3pi_gripper_close(robot14);
%%
robotat_3pi_force_stop(robot14);

%% ***********CONTROL PARA EVACIÓN DE OBSTÁCULOS***********************
%% % === Control del gripper ===
opcionV = 1;  % Cambia a 1 para abrir, 2 para cerrar, o 0 para no hacer nada

if opcionV == 1
    robotat_3pi_gripper_open(robot14);
    fprintf('Garra abierta.\n');
elseif opcionV == 2
    robotat_3pi_gripper_close(robot14);
    fprintf('Garra cerrada.\n');
else
    fprintf('Sin acción en la garra.\n');
end


%% TOMAR GOAL PARA PLANIFICACIÓN Y PARA PUNTO INTERMEDIO ALINEACIÓN
%meta para planificacion 
meta_id=20;
try
    goal20=robotat_get_pose(robotat, meta_id, 'eulxyz');
catch 
end 
%goalCubo = goal;
%goal=[0, 0]; % si se quiere probar goal como origen del robotat
goal20=goal20(1:2)*100; %cm
goal20(1)=-goal20(1)+200;
goal20(2)=goal20(2)+250;
%goal=[200,250];  % Goal ejemplo
goal20=round(goal20,0);

%% GOAL PUNTO INTERMEDIO
metaInter20=20;
try
    goalInterC20=robotat_get_pose(robotat, metaInter20, 'eulxyz');
catch 

end 
goalInter20 = goalInterC20;

%% GOAL FORMACION
metaFormacion20=20;
try
    goalFormacion20=robotat_get_pose(robotat, metaFormacion20, 'eulxyz');
catch 
end 
goalCubo20 = goalFormacion20;


%% Definición de obstáculos y creación del mapa
% Se definen uno por uno los obstáculos, en caso de querer deshabilitar
% alguno igualar a un vector vacío
ids1 = [71 64 65 66 75 61 72 74 68 70  63 73  67 69 ];
% Inicialización del array de obstáculos
obs1=[];
% Se recorre cada ID y se obtiene su posición con robotat_get_pose
for i1 = 1:length(ids1)
    pm1 = robotat_get_pose(robotat, ids1(i1), 'eulxyz');
    
    % Validación por si no se puede obtener la posición
    if ~isempty(pm1)
        % Se agrega la posición (x, y) al array de obstáculos
        obs1(end+1, :) = pm1(1:2);
    end
end

% Se emplea el array de obstáculos para crear y visualizar el mapa (bordes
% de plataforma + obstáculos) como un occupancy grid
map1 = genmap(obs1);
figure(2);
imshow(map1);
title('Mapa actual Robotat', ...
    'FontSize', 14, 'Interpreter', 'latex');
xlabel('$x$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');
ylabel('$y$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');

%legend();
axis on;

%% PLANIFICACIÓN DE MOVIMIENTO
start14=robotat_get_pose(robotat, robot_no14, 'eulzyx');
start14=start14(1:2);
start14(1)=-start14(1)*100+200;
start14(2)=start14(2)*100+250;
start14=round(start14,0);
%start=[50,50]; % si se quiere iniciar desde un punto en específico
% Crear objeto para planificación
dx1 = Dstar(map1,'inflate', 8);
%dx = DXform(map);
dx1.plan(goal20); % Establecer meta
%p1=dx1.query(start14, 'animate');

figure(3);  % ← Figura exclusiva para animación de D*
set(gcf, 'Name', sprintf('D* Planificación - Robot %d', robot_no14));
p1 = dx1.query(start14, 'animate');
%fliplr para invertirla de izquierda a derecha (reflejo sobre el eje Y) o la función flipud para invertirla de arriba a abajo (reflejo sobre el eje X)

%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot14);

%% Parametros control:CICLO DE CONTROL (No lineal) MÁS EJECUCIÓN DE PLANIFICACIÓN

% --- Parámetros del Pololu
R = 32;      % radio de rueda en mm
L = (96-6.8)/2;        % distancia entre ejes en mm
% Campo vectorial del sistema dinámico
f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

% Pure pursuit
ePtol1 = 40; %mm
eOtol1=0.00174533*100; %0.1*n grado 
eP1=1;
eO1=pi;

%No lineal
% Control no lineal de pose
k_rho1 = 6;
k_alpha1 = 40;
k_beta1 = -4;

% --- Configuración pure pursuit ---
i1 = 1;      % índice inicial en p_world
lookahead_dist1 = 100;   % mm de look-ahead (ajusta según tu dinámica)
idx_goal1      = 1;
n_pts1         = size(p1,1);
align_tol1  = eOtol1*1.5;   % si el error angular > 10°
puntosA1 = floor(n_pts1 * 0.85);  % por ejemplo 85% del camino

%% Crear figura para visualización
figure(4);
axis equal; 
xlabel('X [mm]'); ylabel('Y [mm]');

% === TRANSFORMACIÓN Y REFLEJO ===
p1_plot = [10*(-p1(:,1) + 200), 10*(p1(:,2) - 250)];
p1_plot(:,1) = p1_plot(:,1);  % ✅ reflejar en X

% === GRAFICAR ===
plot(p1_plot(:,1), p1_plot(:,2), 'k'); hold on;

% === AJUSTES DE EJES ===
set(gca, 'YDir', 'reverse');   % ✅ eje Y positivo hacia abajo
xlim([-100,1000]);
ylim([-4000,4000]);
grid on;
% === INTERACTIVIDAD ===
disp('Presiona ENTER en la consola para detener el robot manualmente.');
stop_flag = false;
listener = @(~,~) assignin('base', 'stop_flag', true);
set(gcf, 'KeyPressFcn', listener);

% === ELEMENTOS ANIMADOS ===
h_traj14 = animatedline('Color','b','LineWidth',1.5);
h_meta14 = animatedline('Marker','*','Color','r');

try
xi14   = robotat_get_pose(robotat, robot_no14, 'eulzyx');
xpos14 = xi14(1)*1000;
ypos14 = xi14(2)*1000;
theta14 = deg2rad(xi14(4) - offset14);       % yaw en rad
catch
end 

flag_control_cubo1 = false; % antes del while true
maxIterNonLin14 = 5000;
trayectoria14NonLin14 = zeros(maxIterNonLin14,2);
idx14_2 = 1;
while true
    try
        % 1) pose objetivo
        %pm = robotat_get_pose(robotat, id, 'eulxyz');
        %pm=[0,0];
        %pm ahora debería de ser cada punto de la planificación xd
        % ==== Selección dinámica de índice por pure-pursuit ====
        % Avanza idx_goal hasta que el punto esté a ≥ lookahead_dist
        while idx_goal1 < n_pts1
            % calcula posición candidata en mm
            pm_tmp1 = p1(idx_goal1,:);
            pm_tmp1(1) = -pm_tmp1(1) + 200;
            pm_tmp1(2) =  pm_tmp1(2) - 250;
            pm_tmp1    = 10 * pm_tmp1;     % pasa a mm
            dx1 = pm_tmp1(1) - xpos14;
            dy1 = pm_tmp1(2) - ypos14;

            if hypot(dx1,dy1) >= lookahead_dist1
                break;
            end
            idx_goal1 = idx_goal1 + 1;
        end
        % Ahora sí, usamos idx_goal como i
        i1 = idx_goal1;
        % Recupera tu pm igual que antes
        pm1 = p1(i1,:);
        pm1(1) = -pm1(1) + 200;
        pm1(2) =  pm1(2) - 250;
        pm1    = 10 * pm1;      % mm
        xg20 = pm1(1);
        yg20 = pm1(2);
        % ==== fin selección pura ====


        % 2) pose actual
        try 
            xi14   = robotat_get_pose(robotat, robot_no14, 'eulzyx');
            xpos14 = xi14(1)*1000;
            ypos14 = xi14(2)*1000;
            theta14 = deg2rad(xi14(4) - offset14);       % yaw en rad
        catch 
        end 

        % 3) errores control No Lineal
        eNL1  = [xg20-xpos14; yg20-ypos14];
        ePNL1 = norm(eNL1);
        thetagNL1 = atan2(eNL1(2), eNL1(1));
        eONL1 = thetagNL1 - theta14;
        eONL1 = atan2(sin(eONL1), cos(eONL1));
        
        % 4) Control No lineal
        %eO=eO;
        % eONL1 = -theta14 + atan2(eNL1(2), eNL1(1));
        % eONL1 = atan2(sin(eONL1), cos(eONL1));
        beta1 = -theta14 - eONL1;
        beta1 = atan2(sin(beta1), cos(beta1));

        v14 = k_rho1*ePNL1;
        w14 = k_alpha1*eONL1 + k_beta1*(beta1 + thetagNL1);  
        
        % 6) bloqueo de avance si no estás casi alineado
        max_rpm14 = 1200;  % límite de tus motores
        min_rpm14=100;
        if abs(eONL1) > align_tol1
            % solo gira para alinearse
            v14 = 0;
            min_rpm14=0; 
        end
        if (abs(ePNL1)<=ePtol1)
            v14=0;
            min_rpm14=0;
        end
        if (abs(eONL1)<=eOtol1)
            w14=0;
            min_rpm14=0;
        end
        u14=[v14;-w14];
        % 7) ruedas (rad/s → RPM)
        omega_r14 = (u14(1) + (L/2)*u14(2))/R;
        omega_l14 = (u14(1) - (L/2)*u14(2))/R;
        rpm_r14 = omega_r14;
        rpm_l14 = omega_l14;

        % Rueda derecha
        rpm_r14 = sign(rpm_r14) * min( max(abs(rpm_r14), min_rpm14), max_rpm14 );
        rpm_l14 = sign(rpm_l14) * min( max(abs(rpm_l14), min_rpm14), max_rpm14 );

        % 9) enviar velocidades
        robotat_3pi_set_wheel_velocities(robot14, rpm_r14, rpm_l14);
        robotat_3pi_gripper_close(robot14);
        
        % 10) añadir punto a la trayectoria
        % === Transformar y reflejar la posición real del robot (en mm) ===
        x_plot14 = -xpos14; %+ 2000;   % equivalente a 10*(-x + 200) pero ya en mm
        y_plot14 =  ypos14; % 2500;   % equivalente a 10*(y - 250)
        x_plot14 = -x_plot14;          % ✅ reflejar en X  
        % === Agregar punto transformado ===
        addpoints(h_traj14, x_plot14, y_plot14);

        % % 11) dibujar meta y robot (sin borrar la trayectoria)
        % Meta actual reflejada
        xg_plot14 = xg20;%   % p1 está en metros → *1000 a mm
        yg_plot14 =  yg20;
        %xg_plot14 = -xg_plot14;                   % ✅ reflejar en X
        
        % === GRAFICAR ===
        clearpoints(h_meta14);
        addpoints(h_meta14, xg_plot14, yg_plot14);

        % === GUARDAR TRAYECTORIA TRANSFORMADA ===
        trayectoria14NonLin14(idx14_2, :) = [x_plot14, y_plot14];
        idx14_2 = idx14_2 + 1;

       title(sprintf('Trayectoria actual del robot hacia punto final de planificación.\nError P = %.1f mm, E_O = %.2f rad', ePNL1, eONL1), ...
      'Interpreter','tex', 'HorizontalAlignment','center');
        drawnow limitrate;

        %abs(eP)<=ePtol && //170

        % 12) Parada automática o manual       
        % ... dentro del while true, reemplaza la parte de parada automática por:
        if (abs(eONL1) <= eOtol1) || stop_flag
            if stop_flag
                disp('Detenido manualmente con ENTER');
                robotat_3pi_force_stop(robot14);
                robotat_3pi_gripper_open(robot14);
                break;
            elseif (idx_goal1 > puntosA1) && ~flag_control_cubo1
                % Llegaste al tramo final y aún no has pasado a control del cubo.
                robotat_3pi_force_stop(robot14);
                robotat_3pi_gripper_close(robot14);
                disp('Objetivo alcanzado, cambiando a control del cubo...');
                pause(0.5);
                disp('Comienzo');
                flag_control_cubo1 = true; % marcar para que no vuelva a entrar
        
                % Actualiza pose antes de entrar al while
                try
                    xi14   = robotat_get_pose(robotat, robot_no14, 'eulzyx');
                    xpos14 = xi14(1) * 1000;
                    ypos14 = xi14(2) * 1000;
                catch
                end
                

                ePC1 = inf; 

                % % === Inicializar visualización y almacenamiento de trayectoria ===
                figure(5);
                clf; hold on; grid on; axis equal;
                xlabel('X [mm]');
                ylabel('Y [mm]');
                set(gca, 'YDir', 'reverse');
                title('Trayectoria a punto intermedio y punto de formación final');
                xlim([-900 900]);
                ylim([-1800 500]);
                
                % Dibujar goals
                plot(goalInter20(1)*1000, goalInter20(2)*1000, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', '[0.85,0.325,0.9080]','LineWidth', 1.5, 'DisplayName', 'Goal Intermedio');
                plot(goalCubo20(1)*1000, goalCubo20(2)*1000, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', '[0.85,0.325,0.9080]','LineWidth', 1.5, 'DisplayName', 'Goal Cubo');

                % === Inicializar robot ===
                radio_robot = 48;
                wheel_radius = 16;
                offset_wheel = l;
                h_robot14 = rectangle('Position',[xpos14-radio_robot, ypos14-radio_robot, 2*radio_robot, 2*radio_robot], ...
                                    'Curvature',[1,1], 'EdgeColor','b', 'LineWidth',1.5);
                h_wheelL14 = rectangle('Position',[xpos14-offset_wheel-wheel_radius, ypos14-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);
                h_wheelR14 = rectangle('Position',[xpos14+offset_wheel-wheel_radius, ypos14-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);
               
                % animated line (misma para ambas trayectorias para continuidad visual)
                h_trajPID14_2_3 = animatedline('Color','m','LineWidth',1.5, 'DisplayName', 'Trayectorias PID2+PID3');

                % Si no quieres warnings, prealoca (opcional):
                maxIterPID14 = 8000;
                trayectoria14PID_2 = zeros(maxIterPID14,2);
                idx14_3 = 1;
                % prealocar PID3 (opcional)
                trayectoria14PID_3 = zeros(maxIterPID14,2);
                idx14_4 = 1;

        
                % bucle de acercamiento al goal (actualizamos pose dentro)
                while (ePC1 > 130)
                    % PID punto-a-punto con acercamiento exponencial
                    disp('Ciclo dentro del WHILE');
                    % Pose inicial
                    try
                        xi14 = robotat_get_pose(robotat, robot_no14, 'eulzyx');
                    catch
                        disp('Error al obtener la pose del Pololu');
                        robotat_3pi_force_stop(robot14);
                    end
                    xi14PID2 = xi14; % Para guardar donde inicia la posicion en el segundo PID
                   

                    xg20 = goalInter20(1)*1000; % en mm
                    yg20 = goalInter20(2)*1000; % en mm
                    x14 = xi14(1) * 1000; % en mm
                    y14 = xi14(2) * 1000; % en mm
                    theta14 = deg2rad(atan2d(sind(xi14(4) - offset14), cosd(xi14(4) - offset14)));
                    %errores
                    e1 = [xg20 - x14; yg20 - y14];
                    thetag1 = atan2(e1(2), e1(1));   
                    ePC1 = norm(e1);
                    ePpid2_14 = ePC1;
                    eO1 = thetag1 - theta14;
                    eO1 = atan2(sin(eO1), cos(eO1));
                        
                    % Control de velocidad lineal
                    kP14 = v014 * (1-exp(-alpha14*ePC1^2)) / ePC1;
                    v14 = (kP14*ePC1);
                        
                    % Control de velocidad angular
                    eO_D1_2 = eO1 - eO_1;
                    EO1 = EO1 + eO1;
                    w14 = kpO*eO1 + kiO*EO1 + kdO*eO_D1_2;
                    eO_1_2 = eO1;
                        
                    % Velocidades para las ruedas
                    phi_R_ctrl14 = (v14 + l*w14)/r;
                    phi_L_ctrl14 = (v14 - l*w14)/r;
                
                    % Envío de velocidades al robot
                    robotat_3pi_set_wheel_velocities(robot14, phi_L_ctrl14, phi_R_ctrl14);

                    % === Actualizar dibujo ===
                    %actualizar trayectoria robot
                    addpoints(h_trajPID14_2_3, x14, y14);
                    if idx14_3 <= maxIterPID14 %Antes de guardar un nuevo punto, se verifica que aún haya espacio en el vector.
                        trayectoria14PID_2(idx14_3 , :) = [x14, y14];
                        idx14_3  = idx14_3  + 1;
                    end
                   
                    
                    % actualizar dibujo del robot 
                    set(h_robot14, 'Position', [x14-radio_robot, y14-radio_robot, 2*radio_robot, 2*radio_robot]);
                    xL14 = x14 - offset_wheel*cos(theta14 + pi/2);
                    yL14 = y14 - offset_wheel*sin(theta14 + pi/2);
                    xR14 = x14 + offset_wheel*cos(theta14 + pi/2);
                    yR14 = y14 + offset_wheel*sin(theta14 + pi/2);
                    set(h_wheelL14, 'Position', [xL14-wheel_radius, yL14-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
                    set(h_wheelR14, 'Position', [xR14-wheel_radius, yR14-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
                
                    drawnow limitrate;
                end
                % recortar trayectoria PID2
                if idx14_3>1
                    trayectoria14PID_2 = trayectoria14PID_2(1:idx14_3-1, :);
                else
                    trayectoria14PID_2 = zeros(0,2);
                end

                % Si ya está suficientemente cerca, detener el robot y salir del loop
                disp('Meta alcanzada :D, punto intermedio');
                robotat_3pi_force_stop(robot14);
                robotat_3pi_gripper_close(robot14);
                pause(0.5);

                % CAMINO HACIA FORMACIÓN 
                % Actualiza pose antes de entrar al while
                try
                    xi14   = robotat_get_pose(robotat, robot_no14, 'eulzyx');
                    xpos14 = xi14(1) * 1000;
                    ypos14 = xi14(2) * 1000;
                catch
                end
                xi14PID3 = xi14; % Para guardar donde inicia la posicion en el tercer PID
                ePC1 = inf; 

    
                % Bucle de acercamiento a formación 
                while (ePC1 > 150)
                    % PID punto-a-punto con acercamiento exponencial
                    disp('Ciclo while para fomración');
                    % Pose inicial
                    try
                        xi14 = robotat_get_pose(robotat, robot_no14, 'eulzyx');
                    catch
                        disp('Error al obtener la pose del Pololu');
                        robotat_3pi_force_stop(robot14);
                    end

                    xg20 = goalCubo20(1)*1000; % en mm
                    yg20 = goalCubo20(2)*1000; % en mm
                  
                
                    x14 = xi14(1) * 1000; % en mm
                    y14 = xi14(2) * 1000; % en mm
                    theta14 = deg2rad(atan2d(sind(xi14(4) - offset14), cosd(xi14(4) - offset14)));
                    e1 = [xg20 - x14; yg20 - y14];
                    thetag1 = atan2(e1(2), e1(1));   
                    ePC1 = norm(e1);
                    ePpid3_14 = ePC1; % para guardar el valor del error 
                    eO1 = thetag1 - theta14;
                    eO1 = atan2(sin(eO1), cos(eO1));
                        
                    % Control de velocidad lineal
                    kP14 = v014 * (1-exp(-alpha14*ePC1^2)) / ePC1;
                    v14 = (kP14*ePC1);
                        
                    % Control de velocidad angular
                    eO_D1_3 = eO1 - eO_1;
                    EO1 = EO1 + eO1;
                    w14 = kpO*eO1 + kiO*EO1 + kdO*eO_D1_3;
                    eO_1 = eO1;
                        
                    % Velocidades para las ruedas
                    phi_R_ctrl14 = (v14 + l*w14)/r;
                    phi_L_ctrl14 = (v14 - l*w14)/r;
                
                    % Envío de velocidades al robot
                    robotat_3pi_set_wheel_velocities(robot14, phi_L_ctrl14, phi_R_ctrl14);  
                    
                    % === Actualizar dibujo (misma animatedline para continuidad) ===
                    addpoints(h_trajPID14_2_3, x14, y14);
                
                    % Guardar trayectoria PID3
                    if idx14_4 <= maxIterPID14 %Antes de guardar un nuevo punto, se verifica que aún haya espacio en el vector.
                        trayectoria14PID_3(idx14_4, :) = [x14, y14];
                        idx14_4 = idx14_4 + 1;
                    end
                    % actualizar robot y ruedas
                    set(h_robot14, 'Position', [x14-radio_robot, y14-radio_robot, 2*radio_robot, 2*radio_robot]);
                
                    xL14 = x14 - offset_wheel*cos(theta14 + pi/2);
                    yL14 = y14 - offset_wheel*sin(theta14 + pi/2);
                    xR14 = x14 + offset_wheel*cos(theta14 + pi/2);
                    yR14 = y14 + offset_wheel*sin(theta14 + pi/2);
                    set(h_wheelL14, 'Position', [xL14-wheel_radius, yL14-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
                    set(h_wheelR14, 'Position', [xR14-wheel_radius, yR14-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
                
                    drawnow limitrate;
                end

                % recortar trayectoria PID3
                if idx14_4>1
                    trayectoria14PID_3 = trayectoria14PID_3(1:idx14_4-1, :);
                else
                    trayectoria14PID_3 = zeros(0,2);
                end
                % Si ya está suficientemente cerca, detener el robot y salir del loop
                disp('Meta alcanzada en formación de robot no 14. Deteniendo robot ');
                robotat_3pi_force_stop(robot14);
                robotat_3pi_gripper_open(robot14);                
  
  
                pause(0.5);
                disp('Retrocediendo...');
                robotat_3pi_set_wheel_velocities(robot14, -40, -40);
                pause(3);
                robotat_3pi_force_stop(robot14);
                pause(0.3);
                
                ErrorF1=[eP1;eO_1];
                rpmF1=[rpm_r14;rpm_l14];
                disp("Error Posición y Angular control PID final")
                disp(ErrorF1)
                disp("Velocidades del PID final")
                disp(rpmF1)
                break;
            end
        end


        pause(0.05);
        ErrorNL=[ePNL1;eONL1];
        rpmNL=[rpm_r14;rpm_l14];
        disp("Error Posición y Angular control No Lineal")
        disp(ErrorNL)
        disp("Velocidades del control No lineal")
        disp(rpmNL)

    catch ME
        % Esto se ejecuta si dentro del try ocurre cualquier error
        warning(ME.identifier, "Ocurrió un error, deteniendo el robot: %s", ME.message);
        robotat_3pi_force_stop(robot14);
        %input("Presiona enter para continuar"
       
    end



end
  % recortar trayectoria PID2 al tamaño usado
if exist('trayectoria14NonLin','var')
    trayectoria14NonLin14 = trayectoria14NonLin14(1:idx14_2-1, :);
end
% === GUARDAR FIGURAS COMO .FIG ===
try
    % Crear carpeta si no existe
    folder_name = sprintf('resultadosFinales_robot_%d', robot_no14);
    if ~exist(folder_name, 'dir')
        mkdir(folder_name);
    end

    % Crear subcarpeta para las variables dentro de resultados
    folder_vars = fullfile(folder_name, "VariablesFinales14");
    if ~exist(folder_vars, 'dir')
        mkdir(folder_vars);
    end

    % Crear timestamp para identificar ejecuciones distintas
    timestamp = string(datetime("now", "Format", "yyyy-MM-dd_HH-mm-ss"));

    % Guardar figuras
    figure(1);
    savefig(fullfile(folder_name, sprintf('TrayectoriaPrimerPID%d_%s.fig', robot_no14, timestamp)));

    figure(2);
    savefig(fullfile(folder_name, sprintf('mapa_robot_%d_%s.fig', robot_no14, timestamp)));

    figure(3);
    savefig(fullfile(folder_name, sprintf('planificacion_robot_%d_%s.fig', robot_no14, timestamp)));

    figure(4);
    savefig(fullfile(folder_name, sprintf('trayectoria_robot_%d_%s.fig', robot_no14, timestamp)));
    
    figure(5);
    savefig(fullfile(folder_name, sprintf('trayectoria_robot_PID2_3%d_%s.fig', robot_no14, timestamp)));
    fprintf('✅ Figuras .fig guardadas correctamente en la carpeta "%s".\n', folder_name);

    
    % Guardar variables dentro de la subcarpeta "Variables"
    save(fullfile(folder_vars, sprintf('Variables14%d_%s.mat', robot_no14, timestamp)));

catch ME
    warning(ME.identifier,'⚠️ No se pudieron guardar las figuras .fig: %s', ME.message);
end

% Limpieza del flag y función
clear stop_flag
set(gcf, 'KeyPressFcn', '');

%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot14);


%% Desconexión del Robotat
robotat_disconnect(robotat);

%% Desconexión del Pololu 3Pi
robotat_3pi_disconnect(robot14);