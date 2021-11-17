function [T, para, cost_old] = ScalingAlgorithmRG_totalRG(p1, p2, C, cost_old, para)
    [I,~] = size(p1);
    [J,~] = size(p2);

    b = ones(J,1);
    K = exp(-C./para.epsilon );

    tolerance = 1e-6;
    IterMax =500;
    z = 1;
    for i = 1:IterMax
        s1 = z * (K * b + eps(1)) ;   
        a = min(para.beta*p1,max(para.alpha*p1,s1)) ./s1;
        s2 = z * (K'*a  + eps(1)) ;
        b = min(para.beta*p2,max(para.alpha*p2,s2)) ./s2;
        s3 = a' * K *b +eps ;  
        z = min(para.beta_totalmass*1,max(para.alpha_totalmass*1,s3)) /s3;
        if mod(i,20) == 1 
            T =  bsxfun(@times,b(:)',(bsxfun(@times,a(:),K))); 
            T = z * T;
            cost = sum(sum(T.*C));
            if abs(cost - cost_old) < tolerance
                break;
            end
            cost_old = cost;
        end
    end

end



