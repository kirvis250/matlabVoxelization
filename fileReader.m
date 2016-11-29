function [F,V] = fileReader(fileName)

[pathstr,name,ext] = fileparts(fileName);


if(strcmpi(ext,'.stl') == 1)
    [F,V] = stlRead(fileName);
end

if(strcmpi(ext,'.obj') == 1)
     [F,V] = objRead(fileName);
end

end