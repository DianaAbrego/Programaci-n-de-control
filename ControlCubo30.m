%% Conexión al Robotat  % CUBO 30 ROBOT 2
robotat = robotat_connect();

% Conexión al agente Pololu 3Pi 
robot_no2 = 2; % Seleccionar el agente específico a emplear

% Se establece la conexión con el robot y se extrae el offset respectivo
robot2 = robotat_3pi_connect(robot_no2); 
offset2 =  262.9864; 
%xi = robotat_get_pose(robotat, robot_no, 'eulzyx');
%offset2 =  31.8544 ;
%%
marker_offsets = [30, robot_no2,21]; 
robotat_trvisualize(robotat, marker_offsets);
%% 
robotat_3pi_set_wheel_velocities(robot2, 0, 0); 
%% % === Control del gripper ===
opcionV = 1;  % Cambia a 1 para abrir, 2 para cerrar, o 0 para no hacer nada
if opcionV == 1   
    robotat_3pi_gripper_open(robot2);
    fprintf('Garra abierta.\n');
elseif opcionV == 2
    robotat_3pi_gripper_close(robot2);
    fprintf('Garra cerrada.\n');
else
    fprintf('Sin acción en la garra.\n');
end

%% CAPTURAR POSICIONES 
%poseMarker30 = robotat_get_pose(robotat, 30, 'eulzyx');
%% Visualizar markers 
marker_offsets = [30, robot_no2,32]; 
robotat_trvisualize(robotat, marker_offsets);
%% 
%% ******************CONTROL PARA IR A TRAER EL OBJETO ******************************************
%% Parametros de control
% Pose del robot
xi2 = robotat_get_pose(robotat, robot_no2, 'eulzyx');
xpos2 = xi2(1) * 1000; % en mm
ypos2 = xi2(2) * 1000; % en mm
theta2 = atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2));
xi2PID1 = xi2; % pose inicial del recorrido del PID 

thetag2 = 0; % en rad
ePC2 = inf;
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
kiO2 = 0.0;    % menos acumulación
kdO2 = 0.7;    % mejora la amortiguación 
kpO2 = 8;
EO2 = 0;
eO_1_2 = 0;

% Inicialización de variables pa controlador Acercamiento exponencial
alpha2 = 0.000005;    % más sensible al error → desacelera más al acercarse
%v0 = 2000;      % velocidad máxima
v02 = 1500; %mm/s
% === Configuración de la figura === DESCOMENTAR FUNCIONANDO
figure(1);
hold on;
axis equal; 
xlabel('X [mm]');
ylabel('Y [mm]');
xlim([-800,800]);
ylim([-500,900]);
title('Seguimiento de robot Pololu 3Pi+ para ir a traer el objeto');
grid on;

% Reflejar eje Y
set(gca, 'YDir', 'reverse');

% Dibujar meta inicial
poseMarker30 = robotat_get_pose(robotat, 30, 'eulzyx');
xg30 = poseMarker30(1)*1000;
yg30 = poseMarker30(2)*1000;
h_goal30 = plot(xg30, yg30, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 1.5);
poseMarker30PID1 = poseMarker30; % posicion de pose cubo para ir a traer 

% Dibujar robot inicial (cuerpo circular)
radio_robot = 48; % radio aproximado del cuerpo
h_robot2 = rectangle('Position',[xpos2-radio_robot, ypos2-radio_robot, 2*radio_robot, 2*radio_robot], ...
                    'Curvature',[1,1], 'EdgeColor','b', 'LineWidth',1.5);

% Dibujar ruedas (dos círculos pequeños)
wheel_radius = 16;
offset_wheel = l;
h_wheelL2 = rectangle('Position',[xpos2-offset_wheel-wheel_radius, ypos2-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);
h_wheelR2 = rectangle('Position',[xpos2+offset_wheel-wheel_radius, ypos2-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);

% Dibujar trayectoria
h_trajPID2_1 = animatedline('Color','m','LineWidth',1.5); % Trayectoria robot 14 pero es del primer PID
%% Control  para qué se dirija al objeto 
maxIterPID10_1 = 5000; % DESCOMENTAR AL FUNCIONAR
trayectoria2PID_1 = zeros(maxIterPID10_1,2);
idx2_1 = 1;

disp('Comienzo para ir a traer el objeto');
while (ePC2 > 135)
    % PID punto-a-punto con acercamiento exponencial
    disp('Ciclo dentro del WHILE de PID');
    % Pose inicial
    try
        xi2 = robotat_get_pose(robotat, robot_no2, 'eulzyx');
    catch
        disp('Error al obtener la pose del Pololu');
    end

    try
        % meta deseada
        poseMarker30 = robotat_get_pose(robotat, 30, 'eulzyx');
        % xg30 = poseMarker30(1)*1000; % en mm
        % yg30 = poseMarker30(2)*1000; % en mm
    catch
        disp('Error al obtener la pose del Pololu');
        robotat_3pi_force_stop(robot2);
    end

    %posiciónd de meta en mm
    xg30 =poseMarker30(1)*1000;
    yg30 =poseMarker30(2)*1000;

    %posición actual de robot en mm
    x2 = xi2(1) * 1000; % en mm
    y2 = xi2(2) * 1000; % en mm
    theta2 = deg2rad(atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2)));
    
    %errores
    e2 = [xg30 - x2; yg30 - y2];
    thetag2 = atan2(e2(2), e2(1));   
    ePC2 = norm(e2);
    ePpid1_2 = ePC2;
    eO2 = thetag2 - theta2;
    eO2 = atan2(sin(eO2), cos(eO2));
        
    % Control de velocidad lineal
    kP2 = v02 * (1-exp(-alpha2*ePC2^2)) / ePC2;
    v2 = (kP2*ePC2);
        
    % Control de velocidad angular
    eO_D2 = eO2 - eO_1_2;
    EO2 = EO2 + eO2;
    w2 = kpO2*eO2 + kiO2*EO2 + kdO2*eO_D2;
    eO_1_2 = eO2;
        
    % Velocidades para las ruedas
    phi_R_ctrl2 = (v2 + l*w2)/r;
    phi_L_ctrl2 = (v2 - l*w2)/r;

    % Envío de velocidades al robot
    robotat_3pi_set_wheel_velocities(robot2, phi_L_ctrl2, phi_R_ctrl2);

    %=== Actualizar visualización === DESCOMENTAR FUNCIONANDO
    addpoints(h_trajPID2_1, x2, y2);

    % === Guardar punto actual de la trayectoria ===
    trayectoria2PID_1(idx2_1, :) = [x2, y2];
    idx2_1 = idx2_1 + 1;
    % Actualizar cuerpo del robot
    set(h_robot2, 'Position', [x2-radio_robot, y2-radio_robot, 2*radio_robot, 2*radio_robot]);
    title(sprintf('Seguimiento de robot Pololu 3Pi+ para ir a traer el objeto. \nError P = %.1f mm x = %.2fmm y =%.2fmm',ePC2, x2,y2));

    % Actualizar ruedas según orientación
    xL2 = x2 - offset_wheel*cos(theta2 + pi/2);
    yL2 = y2 - offset_wheel*sin(theta2 + pi/2);
    xR2 = x2 + offset_wheel*cos(theta2 + pi/2);
    yR2 = y2 + offset_wheel*sin(theta2 + pi/2);
    set(h_wheelL2, 'Position', [xL2-wheel_radius, yL2-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
    set(h_wheelR2, 'Position', [xR2-wheel_radius, yR2-wheel_radius, 2*wheel_radius, 2*wheel_radius]);

    % Actualizar goal (en caso de moverse)
    set(h_goal30, 'XData', xg30, 'YData', yg30); 
    drawnow;
end
% Si ya está suficientemente cerca, detener el robot y salir del loop
disp('Meta alcanzada. Deteniendo robot...');
robotat_3pi_force_stop(robot2);
robotat_3pi_gripper_close(robot2);
%%
robotat_3pi_force_stop(robot2);

%% ***********CONTROL PARA EVACIÓN DE OBSTÁCULOS***********************
%% % === Control del gripper ===
opcionV = 2;  % Cambia a 1 para abrir, 2 para cerrar, o 0 para no hacer nada

if opcionV == 1
    robotat_3pi_gripper_open(robot2);
    fprintf('Garra abierta.\n');
elseif opcionV == 2
    robotat_3pi_gripper_close(robot2);
    fprintf('Garra cerrada.\n');
else
    fprintf('Sin acción en la garra.\n');
end


%% TOMAR GOAL PARA PLANIFICACIÓN Y PARA PUNTO INTERMEDIO ALINEACIÓN
%meta para planificacion 
meta_id=30;
try
    goal30=robotat_get_pose(robotat, meta_id, 'eulxyz');
catch 
end 
%goalCubo = goal;
%goal=[0, 0];
goal30=goal30(1:2)*100; %cm
goal30(1)=-goal30(1)+200;
goal30(2)=goal30(2)+250;
%goal=[200,250];  % Goal ejemplo
goal30=round(goal30,0);

%% GOAL PUNTO INTERMEDIO
metaInter30=30;
try
    goalInterC30=robotat_get_pose(robotat, metaInter30, 'eulxyz');
catch 

end 
goalInter30 = goalInterC30;

%% GOAL FORMACION
metaFormacion30=30;
try
    goalFormacion30=robotat_get_pose(robotat, metaFormacion30, 'eulxyz');
catch 
end 
goalCubo30 = goalFormacion30;


%% Definición de obstáculos y creación del mapa
% Se definen uno por uno los obstáculos, en caso de querer deshabilitar
% alguno igualar a un vector vacío
ids2 = [ 71 64 65 66 75 61 72 74 68 70  63 73  67 69 ];
%obs0=[0 0];
% Inicialización del array de obstáculos
%obs = [obs0];
obs2=[];
% Se recorre cada ID y se obtiene su posición con robotat_get_pose
for i2 = 1:length(ids2)
    pm2 = robotat_get_pose(robotat, ids2(i2), 'eulxyz');
    
    % Validación por si no se puede obtener la posición
    if ~isempty(pm2)
        % Se agrega la posición (x, y) al array de obstáculos
        obs2(end+1, :) = pm2(1:2);
    end
end

% Se emplea el array de obstáculos para crear y visualizar el mapa (bordes
% de plataforma + obstáculos) como un occupancy grid
map2 = genmap(obs2);
figure(2);
imshow(map2);
title('Mapa actual Robotat', ...
    'FontSize', 14, 'Interpreter', 'latex');
xlabel('$x$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');
ylabel('$y$ (en cm)', 'FontSize', 14, 'Interpreter', 'latex');

%legend();
axis on;

%% PLANIFICACIÓN DE MOVIMIENTO

start2=robotat_get_pose(robotat, robot_no2, 'eulzyx');
start2=start2(1:2);
start2(1)=-start2(1)*100+200;
start2(2)=start2(2)*100+250;
start2=round(start2,0);
%start=[50,50]; 
% Crear objeto para planificación
dx2 = Dstar(map2,'inflate', 9);
%dx = DXform(map);
% Establecer meta
dx2.plan(goal30);

figure(3);  % ← Figura exclusiva para animación de D*
set(gcf, 'Name', sprintf('D* Planificación - Robot %d', robot_no2));
p2 = dx2.query(start2, 'animate');


%input("Presiona enter para continuar")

%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot2);


%% Parametros control:CICLO DE CONTROL (No lineal) MÁS EJECUCIÓN DE PLANIFICACIÓN

% --- Parámetros del Pololu
R = 32;      % radio de rueda en mm
L = (96-6.8)/2;        % distancia entre ejes en mm
% Campo vectorial del sistema dinámico
f = @(xi,u) [u(1)*cos(xi(3)); u(1)*sin(xi(3)); u(2)];

% Pure pursuit
ePtol2 = 40; %mm
eOtol2=0.00174533*100; %0.1*n grado x
% d
eP2=1;
eO2=pi;

%No lineal
% Control no lineal de pose
k_rho2 = 6;
k_alpha2 = 40;
k_beta2 = -4;

% --- Configuración pure pursuit ---

i2     = 1;      % índice inicial en p_world
lookahead_dist2 = 100;   % mm de look-ahead (ajusta según tu dinámica)
idx_goal2      = 1;
n_pts2         = size(p2,1);
align_tol2  = eOtol2*1.5;   % si el error angular > 10°
puntosA2 = floor(n_pts2 *0.9);  % por ejemplo 70% del camino

%% Crear figura para visualización
% figure(4);
% axis equal; grid on;
% xlabel('X [mm]'); ylabel('Y [mm]');
% plot(10*(-p2(:,1)+200), 10*(p2(:,2)-250));
% xlim([-2000,2000]);
% 
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
% h_traj2 = animatedline('Color','b','LineWidth',1.5);
% h_meta2 = animatedline('Marker','*','Color','r');


figure(4); % DESCOMENTAR FUNCIONANDO
axis equal; 
xlabel('X [mm]'); ylabel('Y [mm]');

% === TRANSFORMACIÓN Y REFLEJO ===
p2_plot = [10*(-p2(:,1) + 200), 10*(p2(:,2) - 250)];
p2_plot(:,1) = p2_plot(:,1);  % ✅ reflejar en X

% === GRAFICAR ===
plot(p2_plot(:,1), p2_plot(:,2), 'k'); hold on;

% === AJUSTES DE EJES ===
set(gca, 'YDir', 'reverse');   % ✅ eje Y positivo hacia abajo
xlim([-2000,2000]);
ylim([-2000,1000]);
grid on;  
% === INTERACTIVIDAD ===
disp('Presiona ENTER en la consola para detener el robot manualmente.');
stop_flag = false;
listener = @(~,~) assignin('base', 'stop_flag', true);
set(gcf, 'KeyPressFcn', listener);

% === ELEMENTOS ANIMADOS ===
h_traj2 = animatedline('Color','b','LineWidth',1.5);
h_meta2 = animatedline('Marker','*','Color','r');

try
xi2   = robotat_get_pose(robotat, robot_no2, 'eulzyx');
xpos2 = xi2(1)*1000;
ypos2 = xi2(2)*1000;
theta2 = deg2rad(xi2(4) - offset2);       % yaw en rad
catch
end 

flag_control_cubo2 = false; % antes del while true
maxIterNonLin2 = 10000;
trayectoria2NonLin2 = zeros(maxIterNonLin2,2);
idx2_2 = 1;
while true
    try
        % 1) pose objetivo
        %pm = robotat_get_pose(robotat, id, 'eulxyz');
        %pm=[0,0];
        %pm ahora debería de ser cada punto de la planificación xd
        % ==== Selección dinámica de índice por pure-pursuit ====
        % Avanza idx_goal hasta que el punto esté a ≥ lookahead_dist
        while idx_goal2 < n_pts2
            % calcula posición candidata en mm
            pm_tmp2 = p2(idx_goal2,:);
            pm_tmp2(1) = -pm_tmp2(1) + 200;
            pm_tmp2(2) =  pm_tmp2(2) - 250;
            pm_tmp2    = 10 * pm_tmp2;     % pasa a mm
            dx2 = pm_tmp2(1) - xpos2;
            dy2 = pm_tmp2(2) - ypos2;
            if hypot(dx2,dy2) >= lookahead_dist2
                break;
            end
            idx_goal2 = idx_goal2 + 1;
        end
        % Ahora sí, usamos idx_goal como i
        i2 = idx_goal2;
        % Recupera tu pm igual que antes
        pm2 = p2(i2,:);
        pm2(1) = -pm2(1) + 200;
        pm2(2) =  pm2(2) - 250;
        pm2    = 10 * pm2;      % mm
        xg30 = pm2(1);
        yg30 = pm2(2);
        % ==== fin selección pura ====
        
        xg30 = pm2(1);
        yg30 = pm2(2);

        % 2) pose actual
        try 
            xi2   = robotat_get_pose(robotat, robot_no2, 'eulzyx');
            xpos2 = xi2(1)*1000;
            ypos2 = xi2(2)*1000;
            theta2 = deg2rad(xi2(4) - offset2);       % yaw en rad
        catch 
        end 

        % 3) errores
        eNL2  = [xg30-xpos2; yg30-ypos2];
        ePNL2 = norm(eNL2);
        thetagNL2 = atan2(eNL2(2), eNL2(1));
        eONL2 = thetagNL2 - theta2;
        eONL2 = atan2(sin(eONL2), cos(eONL2));
        
        % 4) Control No lineal
 
        beta2 = -theta2 - eONL2;
        beta2 = atan2(sin(beta2), cos(beta2));

        v2 = k_rho2*ePNL2;
        w2 = k_alpha2*eONL2 + k_beta2*(beta2 + thetagNL2);  
        
        % 6) bloqueo de avance si no estás casi alineado
        max_rpm2 = 1200;  % límite de tus motores
        min_rpm2=100;
        if abs(eONL2) > align_tol2
            % solo gira para alinearse
            v2 = 0;
            min_rpm2=0; 
        end
        if (abs(ePNL2)<=ePtol2)
            v2=0;
            min_rpm2=0;
        end
        if (abs(eONL2)<=eOtol2)
            w2=0;
            min_rpm2=0;
        end
        u2=[v2;-w2];
        % 7) ruedas (rad/s → RPM)
        omega_r2 = (u2(1) + (L/2)*u2(2))/R;
        omega_l2 = (u2(1) - (L/2)*u2(2))/R;
        rpm_r2 = omega_r2;
        rpm_l2 = omega_l2;

        % Rueda derecha
        rpm_r2 = sign(rpm_r2) * min( max(abs(rpm_r2), min_rpm2), max_rpm2 );
        rpm_l2 = sign(rpm_l2) * min( max(abs(rpm_l2), min_rpm2), max_rpm2 );

        % 9) enviar velocidades
        robotat_3pi_set_wheel_velocities(robot2, rpm_r2, rpm_l2);
        robotat_3pi_gripper_close(robot2);
        % % 10) añadir punto a la trayectoria
        % addpoints(h_traj2, xpos2, ypos2);
        % 
        % % 11) dibujar meta y robot (sin borrar la trayectoria)
        % % Nota: al estar 'hold on', trazamos encima de h_traj
        % clearpoints(h_meta2);           % borra el punto anterior
        % addpoints(h_meta2, xg30, yg30);
        % 
        % title(sprintf('Error P=%.1f mm, E_O=%.2f rad', ePNL2, eONL2));
        % drawnow limitrate;
      % 
         % 10) añadir punto a la trayectoria DESCOMENTAR FUNCINANDO
        % === Transformar y reflejar la posición real del robot (en mm) ===
        x_plot2 = xpos2;% + 2000;   % equivalente a 10*(-x + 200) pero ya en mm
        y_plot2 =  ypos2;% - 2500;   % equivalente a 10*(y - 250)
        % x_plot2 = -x_plot2;          % ✅ reflejar en X  
        % === Agregar punto transformado ===
        addpoints(h_traj2, x_plot2, y_plot2);

        % % 11) dibujar meta y robot (sin borrar la trayectoria)
        % Meta actual reflejada
        xg_plot2 = xg30;   % p1 está en cm → *1000 a mm
        yg_plot2 = yg30;
        %xg_plot2 = -xg_plot2;                   % ✅ reflejar en X

        % === GRAFICAR ===
        clearpoints(h_meta2);
        addpoints(h_meta2, xg_plot2, yg_plot2);

        % === GUARDAR TRAYECTORIA TRANSFORMADA ===
        trayectoria2NonLin2(idx2_2, :) = [x_plot2, y_plot2];
        idx2_2 = idx2_2 + 1;

       title(sprintf('Trayectoria actual del robot hacia punto final de planificación.\nError P = %.1f mm, E_O = %.2f rad', ePNL2, eONL2), ...
      'Interpreter','tex', 'HorizontalAlignment','center');
        drawnow limitrate;


        %abs(eP)<=ePtol && //170

        % 12) Parada automática o manual       
        % ... dentro del while true, reemplaza la parte de parada automática por:
        if (abs(eONL2) <= eOtol2) || stop_flag
            if stop_flag
                disp('Detenido manualmente con ENTER');
                robotat_3pi_force_stop(robot2);
                robotat_3pi_gripper_open(robot2);
                break;
            elseif (idx_goal2 > puntosA2) && ~flag_control_cubo2
                % Llegaste al tramo final y aún no has pasado a control del cubo.
                robotat_3pi_force_stop(robot2);
                robotat_3pi_gripper_close(robot2);
                disp('Objetivo alcanzado, cambiando a control del cubo...');
                pause(0.5);
                disp('Comienzo');
                flag_control_cubo2 = true; % marcar para que no vuelva a entrar
        
                % Actualiza pose antes de entrar al while
                try
                    xi2   = robotat_get_pose(robotat, robot_no2, 'eulzyx');
                    xpos2 = xi2(1) * 1000;
                    ypos2 = xi2(2) * 1000;
                catch
                end

                ePC2 = inf; 
                % % === Inicializar visualización y almacenamiento de
                % trayectoria === DESCOMENTAR FUNCIONANDO
                figure(5);
                clf; hold on; grid on; axis equal;
                xlabel('X [mm]');
                ylabel('Y [mm]');
                set(gca, 'YDir', 'reverse');
                title('Trayectoria a punto intermedio y punto de formación final');
                xlim([-2500 1000]);
                ylim([-2500 1000]);

                % Dibujar goals
                plot(goalInter30(1)*1000, goalInter30(2)*1000, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'y','LineWidth', 1.5, 'DisplayName', 'Goal Intermedio');
                plot(goalCubo30(1)*1000, goalCubo30(2)*1000, 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'y','LineWidth', 1.5, 'DisplayName', 'Goal Cubo');

                % === Inicializar robot ===
                radio_robot = 48;
                wheel_radius = 16;
                offset_wheel = l;
                h_robot2= rectangle('Position',[xpos2-radio_robot, ypos2-radio_robot, 2*radio_robot, 2*radio_robot], ...
                                    'Curvature',[1,1], 'EdgeColor','b', 'LineWidth',1.5);
                h_wheelL2 = rectangle('Position',[xpos2-offset_wheel-wheel_radius, ypos2-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);
                h_wheelR2 = rectangle('Position',[xpos2+offset_wheel-wheel_radius, ypos2-wheel_radius, 2*wheel_radius, 2*wheel_radius], ...
                                     'Curvature',[1,1], 'EdgeColor','k', 'LineWidth',1);

                % animated line (misma para ambas trayectorias para continuidad visual)
                h_trajPID2_2_3 = animatedline('Color','m','LineWidth',1.5, 'DisplayName', 'Trayectorias PID2+PID3');

                % Si no quieres warnings, prealoca (opcional):
                maxIterPID2 = 10000;
                trayectoria2PID_2 = zeros(maxIterPID2,2);
                idx2_3 = 1;
                % prealocar PID3 (opcional)
                trayectoria2PID_3 = zeros(maxIterPID2,2);
                idx2_4 = 1;

                % bucle de acercamiento al goal (actualizamos pose dentro)
                while (ePC2 > 125)
                    % PID punto-a-punto con acercamiento exponencial
                    disp('Ciclo dentro del WHILE de punto intermedio');
                    % Pose inicial
                    try
                        xi2 = robotat_get_pose(robotat, robot_no2, 'eulzyx');
                    catch
                        disp('Error al obtener la pose del Pololu');
                    end
                    xi2PID2 = xi2; % Para guardar donde inicia la posicion en el segundo PID
                   
                    xg30 = goalInter30(1)*1000; % en mm
                    yg30 = goalInter30(2)*1000; % en mm                               
                    x2 = xi2(1) * 1000; % en mm
                    y2 = xi2(2) * 1000; % en mm
                    theta2 = deg2rad(atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2)));
                    
                    % errores
                    e2 = [xg30 - x2; yg30 - y2];
                    thetag2 = atan2(e2(2), e2(1));   
                    ePC2 = norm(e2);
                    ePpid2_2 = ePC2;
                    eO2 = thetag2 - theta2;
                    eO2 = atan2(sin(eO2), cos(eO2));
                        
                    % Control de velocidad lineal
                    kP2 = v02 * (1-exp(-alpha2*ePC2^2)) / ePC2;
                    v2 = (kP2*ePC2);
                        
                    % Control de velocidad angular
                    eO_D2 = eO2 - eO_1_2;
                    EO2 = EO2 + eO2;
                    w2 = kpO2*eO2 + kiO2*EO2 + kdO2*eO_D2;
                    eO_1_2 = eO2;
                        
                    % Velocidades para las ruedas
                    phi_R_ctrl2 = (v2 + l*w2)/r;
                    phi_L_ctrl2 = (v2 - l*w2)/r;
                
                    % Envío de velocidades al robot
                    robotat_3pi_set_wheel_velocities(robot2, phi_L_ctrl2, phi_R_ctrl2);

                    % === Actualizar dibujo === DESCOMENTAR FUNCIONANDO
                    %actualizar trayectoria robot
                    addpoints(h_trajPID2_2_3, x2, y2);
                   
                    if idx2_3 <= maxIterPID2 %Antes de guardar un nuevo punto, se verifica que aún haya espacio en el vector.
                        trayectoria2PID_2(idx2_3 , :) = [x2, y2];
                        idx2_3  = idx2_3  + 1;
                    end


                    % actualizar dibujo del robot 
                    set(h_robot2, 'Position', [x2-radio_robot, y2-radio_robot, 2*radio_robot, 2*radio_robot]);
                    xL2 = x2 - offset_wheel*cos(theta2 + pi/2);
                    yL2 = y2 - offset_wheel*sin(theta2 + pi/2);
                    xR2 = x2 + offset_wheel*cos(theta2 + pi/2);
                    yR2 = y2 + offset_wheel*sin(theta2 + pi/2);
                    set(h_wheelL2, 'Position', [xL2-wheel_radius, yL2-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
                    set(h_wheelR2, 'Position', [xR2-wheel_radius, yR2-wheel_radius, 2*wheel_radius, 2*wheel_radius]);

                    drawnow limitrate;

                end

                % recortar trayectoria PID2 DESCOMENTAR FUNCIONANDO
                if idx2_3>1
                    trayectoria2PID_2 = trayectoria2PID_2(1:idx2_3-1, :);
                else
                    trayectoria2PID_2 = zeros(0,2);
                end


                % Si ya está suficientemente cerca, detener el robot y salir del loop
                disp('Meta alcanzada :D, punto intermedio');
                robotat_3pi_force_stop(robot2);
                robotat_3pi_gripper_close(robot2);

                pause(0.5);
                % CAMINO HACIA FORMACIÓN 
                % Actualiza pose antes de entrar al while
                try
                    xi2   = robotat_get_pose(robotat, robot_no2, 'eulzyx');
                    xpos2 = xi2(1) * 1000;
                    ypos2 = xi2(2) * 1000;
                catch
                end
                xi2PID3 = xi2; % Para guardar donde inicia la posicion en el tercer PID
                
                ePC2 = inf; 
                alpha2_1 = 0.000004;    % más sensible al error → desacelera más al acercarse
                % bucle de acercamiento al goal (actualizamos pose dentro)
                while (ePC2 > 150)
                    % PID punto-a-punto con acercamiento exponencial
                    disp('Ciclo while para fomración');
                    % Pose inicial
                    try
                        xi2 = robotat_get_pose(robotat, robot_no2, 'eulzyx');
                    catch
                        disp('Error al obtener la pose del Pololu');
                    end

                    xg30 = goalCubo30(1)*1000; % en mm
                    yg30 = goalCubo30(2)*1000; % en mm
                    x2 = xi2(1) * 1000; % en mm
                    y2 = xi2(2) * 1000; % en mm
                    theta2 = deg2rad(atan2d(sind(xi2(4) - offset2), cosd(xi2(4) - offset2)));
                    % errores
                    e2 = [xg30 - x2; yg30 - y2];
                    thetag2 = atan2(e2(2), e2(1));   
                    ePC2 = norm(e2);
                    ePpid3_2 = ePC2;
                    eO2 = thetag2 - theta2;
                    eO2 = atan2(sin(eO2), cos(eO2));
                        
                    % Control de velocidad lineal
                    kP2 = v02 * (1-exp(-alpha2_1*ePC2^2)) / ePC2;
                    v2 = (kP2*ePC2);
                        
                    % Control de velocidad angular
                    eO_D2 = eO2 - eO_1_2;
                    EO2 = EO2 + eO2;
                    w2 = kpO2*eO2 + kiO2*EO2 + kdO2*eO_D2;
                    eO_1_2 = eO2;
                        
                    % Velocidades para las ruedas
                    phi_R_ctrl2 = (v2 + l*w2)/r;
                    phi_L_ctrl2 = (v2 - l*w2)/r;
                
                    % Envío de velocidades al robot
                    robotat_3pi_set_wheel_velocities(robot2, phi_L_ctrl2, phi_R_ctrl2);  
                    
                    % === Actualizar dibujo === DESCOMENTAR FUNCIONAND
                    addpoints(h_trajPID2_2_3, x2, y2);

                    %Guardar trayectroia PID3 
                    if idx2_4 <= maxIterPID2 %Antes de guardar un nuevo punto, se verifica que aún haya espacio en el vector.
                        trayectoria2PID_2(idx2_4 , :) = [x2, y2];
                        idx2_4  = idx2_4  + 1;
                    end


                    % actualizar dibujo del robot 
                    set(h_robot2, 'Position', [x2-radio_robot, y2-radio_robot, 2*radio_robot, 2*radio_robot]);
                    xL2 = x2 - offset_wheel*cos(theta2 + pi/2);
                    yL2 = y2 - offset_wheel*sin(theta2 + pi/2);
                    xR2 = x2 + offset_wheel*cos(theta2 + pi/2);
                    yR2 = y2 + offset_wheel*sin(theta2 + pi/2);
                    set(h_wheelL2, 'Position', [xL2-wheel_radius, yL2-wheel_radius, 2*wheel_radius, 2*wheel_radius]);
                    set(h_wheelR2, 'Position', [xR2-wheel_radius, yR2-wheel_radius, 2*wheel_radius, 2*wheel_radius]);

                    drawnow limitrate;

                end
                 % recortar trayectoria PID3
                if idx2_4>1
                    trayectoria2PID_3 = trayectoria2PID_3(1:idx2_4-1, :);
                else
                    trayectoria2PID_3 = zeros(0,2);
                end

                % Si ya está suficientemente cerca, detener el robot y salir del loop
                disp('Meta alcanzada en formación de robot no 2. Deteniendo robot ');
                robotat_3pi_force_stop(robot2);
                robotat_3pi_gripper_open(robot2);
                drawnow limitrate;
                pause(0.5);
                disp('Retrocediendo...');
                robotat_3pi_set_wheel_velocities(robot2, -40, -40);
                pause(3);
                robotat_3pi_force_stop(robot2);
                pause(0.3);
                
                disp('Girando 90 grados...');
                robotat_3pi_set_wheel_velocities(robot2, 0, 40 );
                pause(1.8); % ajustar según tu calibración
                robotat_3pi_force_stop(robot2);
                pause(0.3);
                
                disp('Retrocediendo nuevamente...');
                robotat_3pi_set_wheel_velocities(robot2, -40, -40);
                pause(5);
                robotat_3pi_force_stop(robot2);
                
                disp('Maniobra completada.');
                ErrorF2=[eP2;eO_1_2];
                rpmF2=[rpm_r2;rpm_l2];
                disp("Error Posición y Angular control PID final")
                disp(ErrorF2)
                disp("Velocidades del PID final")
                disp(rpmF2)
                break;
            end
        end


        pause(0.05);
        ErrorNL2=[ePNL2;eONL2];
        rpmNL=[rpm_r2;rpm_l2];
        disp("Error Posición y Angular control No Lineal")
        disp(ErrorNL2)
        disp("Velocidades del control No lineal")
        disp(rpmNL)

    catch ME
        % Esto se ejecuta si dentro del try ocurre cualquier error
        warning(ME.identifier, "Ocurrió un error, deteniendo el robot: %s", ME.message);
        robotat_3pi_set_wheel_velocities(robot2, 0, 0);
        %input("Presiona enter para continuar"
       
    end


end
% === GUARDAR FIGURAS COMO .FIG ===
try
    % Crear carpeta si no existe
    folder_name = sprintf('resultadosFinales_robot_%d', robot_no2);
    if ~exist(folder_name, 'dir')
        mkdir(folder_name);
    end
    
    % Crear subcarpeta para las variables dentro de resultados
    folder_vars = fullfile(folder_name, "VariablesFinales2");
    if ~exist(folder_vars, 'dir')
        mkdir(folder_vars);
    end


    % Crear timestamp para identificar ejecuciones distintas
    timestamp = string(datetime("now", "Format", "yyyy-MM-dd_HH-mm-ss"));

    % Guardar figuras
    figure(1);
    savefig(fullfile(folder_name, sprintf('TrayectoriaPrimerPID_%d_%s.fig', robot_no2, timestamp)));

    figure(2);
    savefig(fullfile(folder_name, sprintf('mapa_robot_%d_%s.fig', robot_no2, timestamp)));

    figure(3);
    savefig(fullfile(folder_name, sprintf('planificacion_robot_%d_%s.fig', robot_no2, timestamp)));

    figure(4);
    savefig(fullfile(folder_name, sprintf('trayectoria_robot_%d_%s.fig', robot_no2, timestamp)));

    figure(5);
    savefig(fullfile(folder_name, sprintf('trayectoria_robot_PID2_3_%d_%s.fig', robot_no2, timestamp)));
    
    % Guardar variables dentro de la subcarpeta "Variables"
    save(fullfile(folder_vars, sprintf('Variables2%d_%s.mat', robot_no2, timestamp)));
    
    fprintf('✅ Figuras .fig guardadas correctamente en la carpeta "%s".\n', folder_name);

catch ME
    warning(ME.identifier,'⚠️ No se pudieron guardar las figuras .fig: %s', ME.message);
end

% Limpieza del flag y función
clear stop_flag
set(gcf, 'KeyPressFcn', '');


%% Freno de emergencia para el robot (importante tener a la mano)
robotat_3pi_force_stop(robot2);


%% Desconexión del Robotat
robotat_disconnect(robotat);

%% Desconexión del Pololu 3Pi
robotat_3pi_disconnect(robot2);