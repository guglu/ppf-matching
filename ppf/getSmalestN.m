function [elems, indxs] = getSmalestN(A, n)
% get smallest N element from array
     [ASorted, AIdx] = sort(A);
     elems = ASorted(1:n);
     indxs = AIdx(1:n);
end