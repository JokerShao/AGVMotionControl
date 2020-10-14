% a>=b, a is greater than or equal to b
function isgtoeq=fisgtoeq(a,b,tol)
isgt = (a-b)>tol;
iseq = abs(a-b)<=tol;
isgtoeq = isgt | iseq;
end
