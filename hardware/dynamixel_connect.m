function dynamixel = dynamixel_connect(comport)
% Connect to Dynamixel motor hardware

% Set baudrate
baud_rate = 4000000;

% Set protocol Version
protocol_version = 2;

% Initialize library and port number
[lib_name, port_num] = initDxl(comport);

% Open port
openPortDxl(lib_name, port_num);

% Set Baudrate
setBaudDxl(lib_name, port_num, baud_rate);

% Find dynamixels
[motor_IDs, motor_models] = findDxl(lib_name, port_num, protocol_version);

% Operating modes
operatingModeDxl(lib_name, port_num, protocol_version, motor_IDs, ...
    motor_models, 'Current Control');

% Drive modes
driveModeDxl(lib_name, port_num, protocol_version, motor_IDs, ...
    motor_models, 'Velocity-based Profile');

% Indirect addresses are accessible only when the torque is disabled
torqueDxl(lib_name, port_num, protocol_version, motor_IDs, ...
    motor_models, 'Disable');

% Define the read and write quantities
select_sync_write = "Goal Current";
select_sync_read = ["Present Position", "Present Velocity", ... 
    "Present Current"];

% Set indirect addresses for fast sync write
[start_address_write_indirect, data_length_write_indirect] = ...
    setIndirectWriteDxl(lib_name, port_num, protocol_version, motor_IDs, ...
    motor_models, select_sync_write);

% Set indirect addresses for fast sync read
start_address_read_indirect = start_address_write_indirect + ...
    data_length_write_indirect;
[start_address_read_indirect, data_length_read_indirect] = ...
    setIndirectReadDxl(lib_name, port_num, protocol_version, motor_IDs, ...
    motor_models, start_address_read_indirect, select_sync_read);

% Enable torque
torqueDxl(lib_name, port_num, protocol_version, motor_IDs, motor_models, ...
    'Enable');
% Check error
checkErrorDxl(lib_name, port_num, protocol_version)

% Initialize sync write
group_num_write = initSyncWriteIndirectDxl(lib_name, port_num, protocol_version,...
    motor_models, start_address_write_indirect, data_length_write_indirect);

% Initialize sync read
group_num_read = initSyncReadIndirectDxl(lib_name, port_num, protocol_version,...
    motor_models, start_address_read_indirect, data_length_read_indirect);

% Add parameters for the sync read
syncReadAddParamDxl(lib_name, group_num_read, motor_IDs);

dynamixel.port_num = port_num;
dynamixel.protocol_version = protocol_version;
dynamixel.lib_name = lib_name;
dynamixel.motor_IDs = motor_IDs;
dynamixel.motor_models = motor_models;
dynamixel.group_num_read = group_num_read;
dynamixel.group_num_write = group_num_write;
dynamixel.select_sync_read = select_sync_read;
dynamixel.select_sync_write = select_sync_write;
dynamixel.start_address_read_indirect = start_address_read_indirect;