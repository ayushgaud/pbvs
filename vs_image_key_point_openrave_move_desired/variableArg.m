function []=variableArg(varargin)
persistent varNum

c=0;
if(nargin==1)
    c=varargin;
    varNum=1001;
    display(varNum);
elseif (nargin==2)
    display(varNum);
    c=varargin(2);
    varNum=2001;
    display(varNum);
elseif (nargin==3)
    c=varargin(3);
    varNum=3001;

end
        