function rewriteLine(fname,newText,lineNum)
% based on code by Walter Roberson at https://uk.mathworks.com/matlabcentral/answers/62986-how-to-change-a-specific-line-in-a-text-file


S = readlines(fname);
S(lineNum) = newText;
[fid, msg] = fopen(fname, 'w');
if fid < 1 
    error('could not write output file because "%s"', msg);
end
fwrite(fid, strjoin(S, '\n'));
fclose(fid);
