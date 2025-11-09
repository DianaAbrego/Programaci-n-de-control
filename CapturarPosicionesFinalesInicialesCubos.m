%% Conexión al Robotat 

robotat = robotat_connect();
%%
% tomar posiciones iniciales de formacion
metaFormacion30=30;
try
    goalFormacion30Inicial=robotat_get_pose(robotat, metaFormacion30, 'eulxyz');
catch 
end 


metaFormacion20=20; 
try
    goalFormacion20Inicial=robotat_get_pose(robotat, metaFormacion20, 'eulxyz');
catch 
end 


metaFormacion32=21;
try
    goalFormacion32Inicial=robotat_get_pose(robotat, metaFormacion32, 'eulxyz');
catch 
end 

%% tomar posiciones final de formacion 
metaFormacion30=30;
try
    goalFormacion30Final=robotat_get_pose(robotat, metaFormacion30, 'eulxyz');
catch 
end 


metaFormacion20=20;
try
    goalFormacion20Final=robotat_get_pose(robotat, metaFormacion20, 'eulxyz');
catch 
end 


metaFormacion32=21;
try
    goalFormacion32Final=robotat_get_pose(robotat, metaFormacion32, 'eulxyz');
catch 
end 





%% === GUARDAR POSICIONES INICIALES Y FINALES ===
try
    % Crear carpeta principal si no existe
    folder_name = "PosicionesFinalesEInicialesFINALES";
    if ~exist(folder_name, 'dir')
        mkdir(folder_name);
    end

    % Crear timestamp para identificar ejecuciones distintas
    timestamp = string(datetime("now", "Format", "yyyy-MM-dd_HH-mm-ss"));

    % Nombre del archivo .mat con fecha y hora
    file_name = sprintf('Posiciones_%s.mat', timestamp);

    % Guardar todas las variables relevantes
    save(fullfile(folder_name, file_name), ...
        'goalFormacion30Inicial', 'goalFormacion20Inicial', 'goalFormacion32Inicial', ...
        'goalFormacion30Final', 'goalFormacion20Final', 'goalFormacion32Final');

    fprintf('✅ Archivo guardado correctamente: "%s"\n', fullfile(folder_name, file_name));

catch ME
    warning(ME.identifier,'⚠️ No se pudieron guardar las posiciones: %s', ME.message);
end
%%


