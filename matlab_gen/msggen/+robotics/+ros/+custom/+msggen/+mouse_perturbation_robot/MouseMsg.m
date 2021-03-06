classdef MouseMsg < robotics.ros.Message
    %MouseMsg MATLAB implementation of mouse_perturbation_robot/MouseMsg
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2019 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'mouse_perturbation_robot/MouseMsg' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = 'c8b256345673229922e013ff820cd8d0' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant)
        MNONE = uint8(0)
        MBTNA = uint8(1)
        MBTNB = uint8(2)
        MLEFTCLICK = uint8(3)
        MRIGHTCLICK = uint8(4)
        MWHEEL = uint8(5)
        MCURSOR = uint8(6)
    end
    
    properties (Dependent)
        Event
        ButtonState
        RelX
        RelY
        RelZ
        RelWheel
        FilteredRelX
        FilteredRelY
        FilteredRelZ
    end
    
    properties (Constant, Hidden)
        PropertyList = {'ButtonState', 'Event', 'FilteredRelX', 'FilteredRelY', 'FilteredRelZ', 'RelWheel', 'RelX', 'RelY', 'RelZ'} % List of non-constant message properties
        ROSPropertyList = {'buttonState', 'event', 'filteredRelX', 'filteredRelY', 'filteredRelZ', 'relWheel', 'relX', 'relY', 'relZ'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = MouseMsg(msg)
            %MouseMsg Construct the message object MouseMsg
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function event = get.Event(obj)
            %get.Event Get the value for property Event
            event = typecast(int8(obj.JavaMessage.getEvent), 'uint8');
        end
        
        function set.Event(obj, event)
            %set.Event Set the value for property Event
            validateattributes(event, {'numeric'}, {'nonempty', 'scalar'}, 'MouseMsg', 'Event');
            
            obj.JavaMessage.setEvent(event);
        end
        
        function buttonstate = get.ButtonState(obj)
            %get.ButtonState Get the value for property ButtonState
            buttonstate = int32(obj.JavaMessage.getButtonState);
        end
        
        function set.ButtonState(obj, buttonstate)
            %set.ButtonState Set the value for property ButtonState
            validateattributes(buttonstate, {'numeric'}, {'nonempty', 'scalar'}, 'MouseMsg', 'ButtonState');
            
            obj.JavaMessage.setButtonState(buttonstate);
        end
        
        function relx = get.RelX(obj)
            %get.RelX Get the value for property RelX
            relx = int32(obj.JavaMessage.getRelX);
        end
        
        function set.RelX(obj, relx)
            %set.RelX Set the value for property RelX
            validateattributes(relx, {'numeric'}, {'nonempty', 'scalar'}, 'MouseMsg', 'RelX');
            
            obj.JavaMessage.setRelX(relx);
        end
        
        function rely = get.RelY(obj)
            %get.RelY Get the value for property RelY
            rely = int32(obj.JavaMessage.getRelY);
        end
        
        function set.RelY(obj, rely)
            %set.RelY Set the value for property RelY
            validateattributes(rely, {'numeric'}, {'nonempty', 'scalar'}, 'MouseMsg', 'RelY');
            
            obj.JavaMessage.setRelY(rely);
        end
        
        function relz = get.RelZ(obj)
            %get.RelZ Get the value for property RelZ
            relz = int32(obj.JavaMessage.getRelZ);
        end
        
        function set.RelZ(obj, relz)
            %set.RelZ Set the value for property RelZ
            validateattributes(relz, {'numeric'}, {'nonempty', 'scalar'}, 'MouseMsg', 'RelZ');
            
            obj.JavaMessage.setRelZ(relz);
        end
        
        function relwheel = get.RelWheel(obj)
            %get.RelWheel Get the value for property RelWheel
            relwheel = int32(obj.JavaMessage.getRelWheel);
        end
        
        function set.RelWheel(obj, relwheel)
            %set.RelWheel Set the value for property RelWheel
            validateattributes(relwheel, {'numeric'}, {'nonempty', 'scalar'}, 'MouseMsg', 'RelWheel');
            
            obj.JavaMessage.setRelWheel(relwheel);
        end
        
        function filteredrelx = get.FilteredRelX(obj)
            %get.FilteredRelX Get the value for property FilteredRelX
            filteredrelx = single(obj.JavaMessage.getFilteredRelX);
        end
        
        function set.FilteredRelX(obj, filteredrelx)
            %set.FilteredRelX Set the value for property FilteredRelX
            validateattributes(filteredrelx, {'numeric'}, {'nonempty', 'scalar'}, 'MouseMsg', 'FilteredRelX');
            
            obj.JavaMessage.setFilteredRelX(filteredrelx);
        end
        
        function filteredrely = get.FilteredRelY(obj)
            %get.FilteredRelY Get the value for property FilteredRelY
            filteredrely = single(obj.JavaMessage.getFilteredRelY);
        end
        
        function set.FilteredRelY(obj, filteredrely)
            %set.FilteredRelY Set the value for property FilteredRelY
            validateattributes(filteredrely, {'numeric'}, {'nonempty', 'scalar'}, 'MouseMsg', 'FilteredRelY');
            
            obj.JavaMessage.setFilteredRelY(filteredrely);
        end
        
        function filteredrelz = get.FilteredRelZ(obj)
            %get.FilteredRelZ Get the value for property FilteredRelZ
            filteredrelz = single(obj.JavaMessage.getFilteredRelZ);
        end
        
        function set.FilteredRelZ(obj, filteredrelz)
            %set.FilteredRelZ Set the value for property FilteredRelZ
            validateattributes(filteredrelz, {'numeric'}, {'nonempty', 'scalar'}, 'MouseMsg', 'FilteredRelZ');
            
            obj.JavaMessage.setFilteredRelZ(filteredrelz);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.Event = obj.Event;
            cpObj.ButtonState = obj.ButtonState;
            cpObj.RelX = obj.RelX;
            cpObj.RelY = obj.RelY;
            cpObj.RelZ = obj.RelZ;
            cpObj.RelWheel = obj.RelWheel;
            cpObj.FilteredRelX = obj.FilteredRelX;
            cpObj.FilteredRelY = obj.FilteredRelY;
            cpObj.FilteredRelZ = obj.FilteredRelZ;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.Event = strObj.Event;
            obj.ButtonState = strObj.ButtonState;
            obj.RelX = strObj.RelX;
            obj.RelY = strObj.RelY;
            obj.RelZ = strObj.RelZ;
            obj.RelWheel = strObj.RelWheel;
            obj.FilteredRelX = strObj.FilteredRelX;
            obj.FilteredRelY = strObj.FilteredRelY;
            obj.FilteredRelZ = strObj.FilteredRelZ;
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.Event = obj.Event;
            strObj.ButtonState = obj.ButtonState;
            strObj.RelX = obj.RelX;
            strObj.RelY = obj.RelY;
            strObj.RelZ = obj.RelZ;
            strObj.RelWheel = obj.RelWheel;
            strObj.FilteredRelX = obj.FilteredRelX;
            strObj.FilteredRelY = obj.FilteredRelY;
            strObj.FilteredRelZ = obj.FilteredRelZ;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.mouse_perturbation_robot.MouseMsg.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.mouse_perturbation_robot.MouseMsg;
            obj.reload(strObj);
        end
    end
end
