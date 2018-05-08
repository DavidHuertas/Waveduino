function msfun_arduino_io_setup(block)
% Help for Writing Level-2 M-File S-Functions:
%   web([docroot '/toolbox/simulink/sfg/f7-67622.html']

    %   Copyright 2011 The MathWorks, Inc.
      
% define instance variables 
myArduino = [];

setup(block);
      
%% ---------------------------------------------------------

    function setup(block)
        % Register the number of ports.
        block.NumInputPorts  = 0;
        block.NumOutputPorts = 0;
        
        % Set up the states
        block.NumContStates = 0;
        block.NumDworks = 0;
        
        % Register the parameters.
        block.NumDialogPrms     = 2; % arduino var, COM port
        block.DialogPrmsTunable = {'Nontunable', 'Nontunable'};
        
        % Block is fixed in minor time step, i.e., it is only executed on major
        % time steps. Withg a fixed-step solver, the block runs at the fastest
        % discrete rate.
        block.SampleTimes = [0 1];
        
        block.SetAccelRunOnTLC(false); % run block in interpreted mode even w/ Acceleration
        block.SimStateCompliance = 'DefaultSimState';
        
        % the ArduinoIO block uses the Start method to initialize the arduino
        % connection; by using InitConditions, we ensure that we don't access
        % the variable before it is created
        
        block.RegBlockMethod('CheckParameters', @CheckPrms); % called during update diagram
        block.RegBlockMethod('Start', @Start); % called first
        % block.RegBlockMethod('InitializeConditions', @InitConditions); % called second
        block.RegBlockMethod('Terminate', @Terminate);
    end

%%
    function CheckPrms(block)        
        try
             validateattributes(block.DialogPrm(1).Data, {'char'}, {'nonempty'}); % Name of arduino instance
             validateattributes(block.DialogPrm(2).Data, {'char'}, {'nonempty'}); % serial port  
        catch %#ok<CTCH>
            error('Simulink:ArduinoIO:invalidParameter', 'Invalid parameter for Arduino IO block');
        end        
    end                

%%
    function Start(block) 
        % fprintf('%s: Start\n', getfullname(block.BlockHandle));
        if strcmpi(block.DialogPrm(2).Data, 'demo')
            delete(instrfind('PORT', 'DEMO')); 
        end        
        myArduino = arduino(block.DialogPrm(2).Data); % create the arduino object
        myArduino.chkp=false; % avoid checking arguments since we know how functions are called
        customData = containers.Map('UniformValues', false);
        customData('arduinoUnit') = block.DialogPrm(1).Data; % name of arduino instance
        customData('arduinoHandle') = myArduino;
        set(block.BlockHandle, 'UserData', customData, 'UserDataPersistent', 'off');        
    end        

%%
    function Terminate(block) 
        % fprintf('%s: Terminate\n', getfullname(block.BlockHandle));
        customData = get(block.BlockHandle, 'UserData');        
        arduinoCleanup(customData); % this function will delete the arduino object
        delete(customData); 
        fprintf('Connection to ''%s'' (COM port ''%s'') successfully closed\n', ...
            block.DialogPrm(1).Data, block.DialogPrm(2).Data);        
    end

end

