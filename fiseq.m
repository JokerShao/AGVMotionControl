function iseq=fiseq(a, b)

tol=1e-9;
iseq = abs(a-b)<tol;
end
