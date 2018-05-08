classdef AHRS < handle
    %% Propiedades públicas
    properties (Access = public)
        SamplePeriod = 1/256;
        Quaternion = [1 0 0 0];     % Cuaternión de salida que describe el 
                                    % sensor relativo a la Tierra
        Kp = 2;                     % Ganancia proporcional
        Ki = 0;                     % Ganancia de integración
        KpInit = 200;               % Ganancia proporcional de 
                                    % inicialización
        InitPeriod = 5;             % Tiempo de inicialización (segundos)
    end

    %% Propiedades privadas
    properties (Access = private)
        q = [1 0 0 0];              % Cuaternión interno que describe 
                                    % la Tierra relativa al sensor
        IntError = [0 0 0]';        % Error integral
        KpRamped;                   % Ganancia proporcional interna usada
                                    % para descender durante la
                                    % inicialización
    end

    %% Métodos Públicos
    methods (Access = public)
        function obj = AHRS(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'),...
                        obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion')
                    obj.Quaternion = varargin{i+1};
                    obj.q = quaternConj(obj.Quaternion);
                elseif  strcmp(varargin{i}, 'Kp'), obj.Kp = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Ki'), obj.Ki = varargin{i+1};
                elseif  strcmp(varargin{i}, 'KpInit'), obj.KpInit =...
                        varargin{i+1};
                elseif  strcmp(varargin{i}, 'InitPeriod'), ...
                        obj.InitPeriod = varargin{i+1};                    
                else error('Error en la variable de entrada');
                end
                obj.KpRamped = obj.KpInit;
            end;
        end
        
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer)

            % Normalizar los datos del acelerómetro
            if(norm(Accelerometer) == 0)
                warning(0,...
  'Los valores del acelerómetro son nulos. El algoritmo será abortado.');
                return;
            else
                Accelerometer = Accelerometer / norm(Accelerometer);
                % Normalizar las medidas del acelerómetro
            end

            % Procesar el error entre la dirección de la gravedad estimada 
            % y la medida
            v = [2*(obj.q(2)*obj.q(4) - obj.q(1)*obj.q(3))
                2*(obj.q(1)*obj.q(2) + obj.q(3)*obj.q(4))
                obj.q(1)^2 - obj.q(2)^2 - obj.q(3)^2 + obj.q(4)^2];
            error = cross(v, Accelerometer');
            
            % Procesar los términos del error de integración
            
            obj.IntError = obj.IntError + error;

            % Aplicar dichos términos
            Ref = Gyroscope - (obj.Kp*error + obj.Ki*obj.IntError)';

            % Procesar la velocidad de cambio del cuaternión:
            pDot = 0.5 * obj.quaternProd(obj.q, [0 Ref(1) Ref(2) Ref(3)]);
            
            % Integrar la velocidad de cambio del cuaternión:
            obj.q = obj.q + pDot * obj.SamplePeriod;
            
            % Normalizar el cuaternión:
            obj.q = obj.q / norm(obj.q);

            % Almacenar el cuaternión conjugado
            obj.Quaternion = obj.quaternConj(obj.q);
        end
        
        function obj = Reset(obj)
            obj.KpRamped = obj.KpInit;      % reinicia Kp
            obj.IntError = [0 0 0]';      	% inicializa los términos de 
                                            % integración
            obj.q = [1 0 0 0];           	% alinea el cuaternión a la 
                                            % vertical	
        end   
    end
    
    %% Métodos get/set
    methods
        function obj = set.Quaternion(obj, value)
            if(norm(value) == 0)
                error('El valor del cuaternión no puede ser cero.');
            end
            value = value / norm(value);
            obj.Quaternion = value;
            obj.q = obj.quaternConj(value);
        end        
    end

    %% Métodos privados
    methods (Access = private)
        function ab = quaternProd(obj, a, b)
            ab(:,1) = a(:,1).*b(:,1)-a(:,2).*...
                b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
            ab(:,2) = a(:,1).*b(:,2)+a(:,2).*...
                b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
            ab(:,3) = a(:,1).*b(:,3)-a(:,2).*...
                b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
            ab(:,4) = a(:,1).*b(:,4)+a(:,2).*...
                b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
        end
        function qConj = quaternConj(obj, q)
            qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
        end
    end
end