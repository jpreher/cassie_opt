
list = dir('params/stepping');

iter = zeros(length(list)-3,1);
time = zeros(length(list)-3,1);
objc = zeros(length(list)-3,1);

for i = 3:length(list)
    
    f = fopen(strcat(pwd,'/params/stepping/', list(i).name, '/log.txt'));
    
    j = 1;
    txt = cell(1,1);
    txt{j} = fgetl(f);
    while ischar(txt{j})
        j = j+1;
        txt{j} = fgetl(f);
    end
    time(i-2, 1) = sscanf(txt{end-1}, 'Elapsed time is %f minutes.');
    
    tmp = sscanf(txt{end-68}, 'Number of Iterations....: %d');
    if isempty(tmp)
        disp('There was a restoration failed or did not converge!');
        iter(i-2, 1) = sscanf(txt{end-69}, 'Number of Iterations....: %d');
        objc(i-2, 1) = sscanf(txt{end-6}, 'f(x*) = %f');
    else
        iter(i-2, 1) = sscanf(txt{end-68}, 'Number of Iterations....: %d');
        objc(i-2, 1) = sscanf(txt{end-5}, 'f(x*) = %f');
    end
    
end