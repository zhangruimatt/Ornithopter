% z = panelint2d(nu,mu,b,c,L)
% Two-dimensional panel integral:
%


%
function z = panelint2d(nu,mu,b,c,L)
d = 4*c-b^2;          % discriminant of integrand's denominator
t = (2*mu-b*nu);    % convenience variable
z = nu/2*log(abs((L*L+b*L+c)/c));
if (d > 0)
    sd = sqrt(d);
    z = z + t/sd*(atan((2*L+b)/sd)-atan(b/sd));
elseif (d == 0)
    z = z - t*(1/(2*L+b) - 1/b); % (4c != 0 and d == 0) implies b != 0
else % d < 0
    sd = sqrt(-d);
    z = z - t/sd*(atanh((2*L+b)/sd)-atanh(b/sd));
end