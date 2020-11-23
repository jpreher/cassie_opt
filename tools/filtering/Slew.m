function [value, rate] = Slew(dt, lb, ub, prev, cur)

rate = (cur - prev) / dt;

value = cur;

for i = 1:length(lb)
    if rate(i) < lb(i)
        rate(i) = lb(i);
    elseif rate(i) > ub(i)
        rate(i) = ub(i);
    end
    
    value(i) = prev(i) + rate(i) * dt;
end

end

