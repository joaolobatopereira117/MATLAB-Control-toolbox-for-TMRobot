%Control head Light
%Inputs: m- Object for the Modbus client;
%        state- 0(Turn Off) or 1(Turn On);

function setHeadlight(m,state)
    write(m,"coils",7207,state);
end