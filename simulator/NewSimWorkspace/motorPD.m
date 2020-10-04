function u = motorPD(err, derr)

Kp = [500, 500, 500, 750, 300, ...
      500, 500, 500, 750, 300]';
Kd = [25, 25, 25, 30, 5, ...
      25, 25, 25, 30, 5]';
  
u = - Kp .* err - Kd .* derr;

end

