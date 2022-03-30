%Support function to solve the checksum of the package acording to the external script protocol 

function checksum=checksum(trama)

for i=1:length(trama)
    char_hex = dec2hex(trama(i)); 
    char_binary = hexToBinaryVector(char_hex,8);

    if(i~=1)
        for j=1:8
            binary_result(j) = xor(binary_result_ant(j),char_binary(j));
        end
    else
       binary_result = char_binary;
    end
    
    binary_result_ant = binary_result;
    
end

checksum = binaryVectorToHex(binary_result);

end