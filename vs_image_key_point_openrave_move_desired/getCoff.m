function[coff,A,b]=getCoff(waypoints)
   n=size(waypoints,1)-1;
   A=zeros(8*n+1,8*n);
   % b=zeros(1,8*n);  //original
   b=zeros(3,8*n);
   for i=1:n
       b(:,i)=waypoints(:,i);
       b(:,i+n)=waypoints(:,i+1);
   end
   
   %constraint1 Pi(0)=Wi for all i=1...n
   row=1;
   for i=1:n
       A(row,((i-1)*8)+1:i*8)=polyT(8,0,0);
       row=row+1;
   end
    %constraint2 Pi(1)=Wi+1 for all i=1...n
   for i=1:n
       A(row,((i-1)*8)+1:i*8)=polyT(8,0,1);
       row=row+1;
   end
   %constraint3 P1(k)(0)=0 for all 1<=k<=3
   for i=1:n
       for k=1:3
           A(row,((i-1)*8)+1:i*8)=polyT(8,k,0);
           row=row+1;
       end
   end
    %constraint4 Pn(k)(1)=0 for all 1<=k<=3
%   for n=1:4
%        for k=1:3
%            A(row,((n-1)*8)+1:n*8)=polyT(8,k,1);
%            row=row+1;
%        end
%   end
  for i=1:n
       for k=1:3
           A(row,((i-1)*8)+1:i*8)=polyT(8,k,1);
           row=row+1;
       end
  end
   
  
   %constraint5 Pi-1(k)(1)=Pi(k)(0) for all 1<=k<=3
   for i=2:n
       for k=1:6
           A(row,((i-1)*16)+1:i*16)=[polyT(8,k,1)-polyT(8,k,0)];          
       end
   end
   coff=pinv(A)*b';
end
