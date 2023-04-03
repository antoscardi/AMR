function dsens = fsensitivity(fq,fp,fu,hq,hxhi)
syms sens(t) [3 2], syms sensxhi(t) [3 2]
fq
dsens = fq*sens + fp + fu*(hq*sens + hxhi*sens);
dsens = simplify(dsens)
dsens = matlabFunction(dsens)

end