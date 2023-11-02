% write symbolically generated term in mfile
%
% varargin: a list of symbolic terms 
%
% Note: it must start like "write_symbolic_term_to_mfile(q,dq,param,...)"
%
% by Alireza Ramezani - 11-8-2018
function write_symbolic_term_to_mfile(varargin)

% extract the name of the vars
argName={};
for i=1:length(varargin)
    argName{i}=inputname(i);
end

% create a name by combining the name of all of the args
fcn_name='';
for i=4:length(varargin)
    if i<length(varargin)
        fcn_name=[fcn_name,argName{i},'_'];
    else
        fcn_name=[fcn_name,argName{i}];
    end
end

fcn_name=['func_compute_',fcn_name]; % Function name.

fid=fopen(['autogen/',fcn_name,'.m'],'w+'); % Open m.file

% write function header
argoutStr='[';
for i=4:length(varargin)
    if i<length(varargin)
        argoutStr=[argoutStr,argName{i},','];
    else
        argoutStr=[argoutStr,argName{i}];
    end
end
argoutStr=[argoutStr,']'];

fprintf(fid,['function ',argoutStr,'=' ...
    ' %s(q,dq,param)\n'],fcn_name);   % Write header.

% some header comments.
fprintf(fid,'%s','%%%%');
fprintf(fid,'%%%%%s\n',['  ',fcn_name,'.m']);
fprintf(fid,'%%%%%s',['%%  ',datestr(now,2)]);
fprintf(fid,'\n%s','%%%%');
fprintf(fid,'\n%s','%%%%');
fprintf(fid,'\n%s','%%%%');

% input header
fprintf(fid,'\n%s','%Inputs');
tmpVar=varargin{1};
for i=1:length(tmpVar)
    fprintf(fid,'\n%s',[char(tmpVar(i)),'=','q(',num2str(i),');']);
end

fprintf(fid,'\n%s','%%%%');
fprintf(fid,'\n%s','%%%%');

tmpVar=varargin{2};
for i=1:length(tmpVar)
    fprintf(fid,'\n%s',[char(tmpVar(i)),'=','dq(',num2str(i),');']);
end


fprintf(fid,'\n%s','%%%%');
fprintf(fid,'\n%s','%%%%');

tmpVar=varargin{3};
for i=1:length(tmpVar)
    fprintf(fid,'\n%s',[char(tmpVar(i)),'=','param(',num2str(i),');']);
end

fprintf(fid,'\n%s','%%%%');
fprintf(fid,'\n%s','%%%%');

% output header
for k=4:length(varargin)
    tmpVar=varargin{k};
    [m,n]=size(tmpVar);
    
    fprintf(fid,'\n%s','%%%%');
    fprintf(fid,'\n%s','%%%%');
    
    fprintf(fid,'\n%s', [argName{k},'=zeros(',num2str(m),',',num2str(n),');']);
    
    for i=1:m
        for j=1:n
            fprintf(fid,'\n%s',[argName{k},'(',num2str(i),',',num2str(j),') = ',char(tmpVar(i,j)),';']);
        end
    end
end

fprintf(fid,'\n%s','%%%%');
fprintf(fid,'\n%s','%%%%');
fprintf(fid,'\n%s','%%End of code');
        
% Close m.file
fclose(fid);

end

% end of code.