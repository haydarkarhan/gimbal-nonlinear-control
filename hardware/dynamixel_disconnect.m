function dynamixel_disconnect(dynamixel)
% Disconnect from dynamixel hardware 

syncReadClearParamDxl(dynamixel.lib_name, dynamixel.group_num_read);

% Disable torque
torqueDxl(dynamixel.lib_name, dynamixel.port_num, dynamixel.protocol_version, dynamixel.motor_IDs, dynamixel.motor_models, ...
    'Disable');

% Check error
checkErrorDxl(dynamixel.lib_name, dynamixel.port_num, dynamixel.protocol_version)

% Close port 
closePortDxl(dynamixel.lib_name, dynamixel.port_num);