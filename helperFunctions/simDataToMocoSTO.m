function simDataToMocoSTO(fileName,simData,header)
% if adding multipliers and slacks, add them as fields to simData.data, and their
% names to simData.columnLabels 

[fid, msg] = fopen(fileName, 'w');
if fid < 1 
    error('could not write output file because "%s"', msg);
end
fwrite(fid, strjoin(header, '\n'));
fprintf(fid,'\nendheader\n');

fieldsData = fieldnames(simData.data);
fieldsLabels = fieldnames(simData.columnLabels);
isValue = find(endsWith(fieldsLabels,'value'));
isSpeed = find(endsWith(fieldsLabels,'speed'));
isLambda = find(startsWith(fieldsLabels,'lambda')); % multipliers
isGamma = find(startsWith(fieldsLabels,'gamma')); % slacks

[columnLabelsValue,columnLabelsSpeed,columnLabelsLambda,columnLabelsGamma] = deal({});
nRows = length(simData.data.time);
dataMatrix = NaN(nRows,length(fieldsData));
dataMatrix(:,1) = simData.data.time;

for i = 1:length(isValue)
    curField = fieldsLabels{isValue(i)};
    columnLabelsValue{i} = simData.columnLabels.(curField); 
    dataMatrix(:,i+1) = simData.data.(curField);
end
nValue = i;
for i = 1:length(isSpeed)
    curField = fieldsLabels{isSpeed(i)};
    columnLabelsSpeed{i} = simData.columnLabels.(curField);
    dataMatrix(:,i+nValue+1) = simData.data.(curField);
end
nValue = i+nValue;
for i = 1:length(isLambda)
    curField = fieldsLabels{isLambda(i)};
    columnLabelsLambda{i} = simData.columnLabels.(curField);
    dataMatrix(:,i+nValue+1) = simData.data.(curField);
end
nValue = i+nValue;
for i  = 1:length(isGamma)
    curField = fieldsLabels{isGamma(i)};
    columnLabelsGamma{i} = simData.columnLabels.(curField);
    dataMatrix(:,i+nValue+1) = simData.data.(curField);
end

columnLabels = [{'time'},columnLabelsValue,columnLabelsSpeed,columnLabelsLambda,columnLabelsGamma];
fprintf(fid,[strjoin(columnLabels,'\t'),'\n']);

for i = 1:nRows
   	fprintf(fid, '%20.8f\t', dataMatrix(i,:));
	fprintf(fid, '\n');
end

fclose(fid);