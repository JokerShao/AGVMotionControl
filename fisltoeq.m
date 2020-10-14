% a<=b, a is less than or equal to b
function isltoeq=fisltoeq(a,b,tol)
islt = (b-a)>tol;
iseq = abs(a-b)<=tol;
isltoeq = islt | iseq;
end
