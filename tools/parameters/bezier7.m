function ret = bezier7(a, x)

a1 = a(1);
a2 = a(2);
a3 = a(3);
a4 = a(4);
a5 = a(5);
a6 = a(6);
a7 = a(7);

ret =    a1*(1-x)^6 + 6*a2*(1-x)^5 * x +  15*a3*(1-x)^4 * x^2 +  20*a4*(1-x)^3 * x^3 +  15*a5*(1-x)^2 * x^4 +  6*a6*(1-x)   * x^5 +  a7           * x^6;
          
end