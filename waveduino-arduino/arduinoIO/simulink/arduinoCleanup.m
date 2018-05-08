% arduinoCleanup(customData)
%  Performs cleanup for the Arduino object (corresponding to Arduino IO Block, 
%  specified by ioBlockHandle) This is called from the Terminate function 
%  in msfun_arduino_io_setup

% Note: This function should be used for ALL Arduino-related cleanup. I.e., 
% don't use the Terminate functions of the individual blocks. The reason: we 
% cannot guarantee that the Terminate functions for individual blocks will be
% called BEFORE the Terminate function for msfun_arduino_io, so to have a 
% coherent teardown, we need to do it all in one place.

    %   Copyright 2011 The MathWorks, Inc.
 
function arduinoCleanup(customData)

assert(isa(customData,'containers.Map') && customData.isKey('arduinoHandle'));

arduinoObj = customData('arduinoHandle');
if ~isvalid(arduinoObj)
    warning('arduinoCleanup:InvalidObject', 'Arduino object is not valid');
   return;
end

for servoNum = [1 2]
    servoVar = sprintf('servo%d', servoNum);
    if customData.isKey(servoVar)
        arduinoObj.servoDetach(servoNum);
        customData.remove(servoVar);
    end
end

delete(arduinoObj);
