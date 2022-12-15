function S = fillDefaults(S,Sdefault)
% finds fields in struct Sdefault that aren't found in struct S, and adds
% those fields and values to S

fieldsS = fieldnames(S);
fieldsSD = fieldnames(Sdefault);

fieldsSDnotinS = setdiff(fieldsSD,fieldsS);

for i = 1:length(fieldsSDnotinS)
   curField = fieldsSDnotinS{i};
   S.(curField) = Sdefault.(curField);
end
