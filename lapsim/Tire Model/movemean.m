function [val] = movemean(x, y, step)
%%
% Summary: filters a nonuniform-density dataset by averaging ranges of data
% into new datapoints a fixed x-distance apart.
% Inputs:
%  x - x data
%  y - y data
%  step - the size of the ranges, or the x-distance between filtered data
% Outputs:
%  val - filtered (x,y) data
%%
k = [x,y];
[~,x_sort] = sort(k(:,1));
k = k(x_sort,:);
y_sort = k(:,2);
x_sort = k(:,1);

%plot(x_sort,y_sort,'.')
a = 1;
i = 1;
%val = [ ];

%fprintf('k = %d\n', k);
% fprintf('x sort = %d\n', x_sort);
% fprintf('y sort = %d\n', y_sort);

%for(i=1:length(x_sort))
while(i<=length(x_sort))
    %fprintf("i=%d at beginning 1st loop\n",i);
    val(a,1) = [x_sort(i)];
    val(a,2) = [y_sort(i)];
    n=1; %while loop counter
    w=i; %next index comparative counter
    if(i==length(x_sort))
%          a = a+1;
%          val(a,1) = x_sort(i);
%          val(a,2) = y_sort(i);
         % fprintf("i break\n");
          %fprintf("a=%d\n",a);
          %fprintf("length of x = %d\n",length(x_sort))
          %fprintf("i = %d\n",i);
         break;
     end
    while((x_sort(w+1) <= x_sort(i)+step)&&(x_sort(w+1) >= x_sort(i)-step))
       val(a,2)= val(a,2) + y_sort(w);
        %fprintf("a =%d  w=%d  i=%d  y_sort(w+1)=%d  y_sort(i)=%d  diff=%d\n",a,w,i,y_sort(w+1),y_sort(i),y_sort(w+1)-y_sort(i));
       w=w+1; %increment comparative counter
       n=n+1;
       if(w>=length(x_sort))
           %fprintf("w break\n");
           %fprintf("w = %d\n",w);
           break;
       end
    end
    val(a,2) = val(a,2)/n;
    i=w; %update for loop index based on values compared

    a = a+1; %for loop counter and final val index 
    %fprintf("i=%d at end 1st loop\n",i);
    i = i+1;
    
end 
%plot(val(:,1),val(:,2),'.')
end