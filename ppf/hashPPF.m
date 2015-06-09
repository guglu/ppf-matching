function hash = hashPPF(f,angleRadians,distanceStep)
% quantize and hash point pair feature

    key=floor([f(1:3)./(angleRadians) f(4)./distanceStep]);
    
    hout=MurmurHash3(uint32(key),16,42);

    hash=hout(1);
end