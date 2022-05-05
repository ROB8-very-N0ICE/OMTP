function [qi, xi] = ikin_Panda(x,R,q)

[xi,Ri,Ji]=fkin_Panda(q);
Qr = quaternion(R);
qi = q(:);
K = 0.8;
count = 0;
while (norm(x(:)-xi(:)) > 0.05) || (norm(R-Ri) > 0.1)
    
    [xi,Ri,Ji]=fkin_Panda(qi);
    %position controller
    xc = K*(x(:)-xi(:));
    %orientation controller
    Qi = quaternion(Ri);
    Qc = Qr*inv(Qi);
    realQcv=real(Qc.v);
    qdi = pinv(Ji)*K*[xc(:);realQcv(:)];
    %integration
    qi = qi + qdi;
    
    %check
    count = count + 1;
    if count > 1000
        throw(MException('','ikin diverges'));
    end
end