function [d_fk,d_ik]=error_codes()

%     global d_fk;
%     global d_ik;
    
    mes = rosmessage('moveit_msgs/GetPositionFKResponse');
    val = mes.ErrorCode;
    
    d_fk = containers.Map;
    
    props = properties(val);
    
    for prop = 4:length(props)-1
        thisprop = props{prop};
        value = thisprop;
    %     disp(value)
        key = string(val.(thisprop));
    %     disp(key)
        d_fk(key) = value;
    end
    
    mes = rosmessage('moveit_msgs/GetPositionIKResponse');
    val = mes.ErrorCode;
    
    d_ik = containers.Map;
    
    props = properties(val);
    
    for prop = 4:length(props)-1
        thisprop = props{prop};
        value = thisprop;
    %     disp(value)
        key = string(val.(thisprop));
    %     disp(key)
        d_ik(key) = value;
    end
end