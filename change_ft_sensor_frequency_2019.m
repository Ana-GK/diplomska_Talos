function [] = change_ft_sensor_frequency_2019()

    % stop force_torque_sensor_controller
    [switch_controller_client,switch_controller_msg]=rossvcclient("/controller_manager/switch_controller");
    switch_controller_msg.StopControllers = {'force_torque_sensor_controller'};
    status = call(switch_controller_client,switch_controller_msg,"Timeout",5);
    if ~status.Ok
        error("Service /controller_manager/switch_controller not available on network")
    end
%     pause(0.5)

    % unload force_torque_sensor_controller
    [unload_controller_client,unload_controller_msg]=rossvcclient("/controller_manager/unload_controller");
    unload_controller_msg.Name = 'force_torque_sensor_controller';
    status = call(unload_controller_client,unload_controller_msg,"Timeout",5);
    if ~status.Ok
        error("Service /controller_manager/unload_controller not available on network")
    end
%     pause(0.5)

    % CHANGE PUBLISH RATE
    rosparam("set","/force_torque_sensor_controller/publish_rate",500);
%     pause(0.5)
    
    % load force_torque_sensor_controller
    [load_controller_client,load_controller_msg]=rossvcclient("/controller_manager/load_controller");
    load_controller_msg.Name = 'force_torque_sensor_controller';
    status = call(load_controller_client,load_controller_msg,"Timeout",5);
    if ~status.Ok
        error("Service /controller_manager/load_controller not available on network")
    end
%     pause(0.5)

    % start force_torque_sensor_controller
    [switch_controller_client,switch_controller_msg]=rossvcclient("/controller_manager/switch_controller");
    switch_controller_msg.StartControllers = {'force_torque_sensor_controller'};
    status = call(switch_controller_client,switch_controller_msg,"Timeout",5);
    if ~status.Ok
        error("Service /controller_manager/switch_controller not available on network")
    end
%     pause(0.5)
