classdef ImuInput
    %IMU_INPUT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        omega
        f
    end
    
    methods
        function obj = ImuInput(omega,f)
            obj.omega = omega(:);
            obj.f = f(:);
        end
    end
end

