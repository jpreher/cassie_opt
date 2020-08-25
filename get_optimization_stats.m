paths = '.';
list = dir(strcat(paths,'/params/stepping'));

t = zeros(1,length(list)-2);
iter = zeros(1,length(list)-2);
for i = 3:length(list)
    text = fileread(strcat(pwd,'/params/stepping/', list(i).name, '/log.txt'));
    textAsCells = textscan(text, '%s', 'Delimiter', '\n');
    
     temp = regexp(textAsCells{1}{end},'\d.\d*','Match');
     t(i-2) = str2num(temp{1});
     
     temp = regexp(textAsCells{1}{end-21}, '\d*', 'Match');
     iter(i-2) = str2num(temp{1});
end