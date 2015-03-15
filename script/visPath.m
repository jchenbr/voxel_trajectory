function [ret] = visPath(data,P,T)
    M = floor(size(data,1)*0.5);
    N = 6;

    p_s = data(1,1:3);
    p_t = data(1,4:6);

    B   = data(2:M+1,:);
    E   = data(M+2:end,:);

    plot3(p_s(1),p_s(2),p_s(3),'bo');
    hold on;
    plot3(p_t(1),p_t(2),p_t(3),'bo');

    for k = 1 : M
        for i = 1 : 2
            for j = 1 : 2
                l = [B(k,1) B(k,2+i) B(k,4+j);
                     B(k,2) B(k,2+i) B(k,4+j)];
                line(l(:,1),l(:,2),l(:,3),'color','r','linestyle',':'); 

                l = [B(k,i) B(k,2+1) B(k,4+j);
                     B(k,i) B(k,2+2) B(k,4+j)];
                line(l(:,1),l(:,2),l(:,3),'color','r','linestyle',':'); 

                l = [B(k,i) B(k,2+j) B(k,4+1);
                     B(k,i) B(k,2+j) B(k,4+2)];
                line(l(:,1),l(:,2),l(:,3),'color','r','linestyle',':'); 
            end;
        end
    end

    for k = 1 : M-1
        for i = 1 : 2
            for j = 1 : 2
                l = [E(k,1) E(k,2+i) E(k,4+j);
                     E(k,2) E(k,2+i) E(k,4+j)];
                line(l(:,1),l(:,2),l(:,3),'color','g','linestyle','-'); 

                l = [E(k,i) E(k,2+1) E(k,4+j);
                     E(k,i) E(k,2+2) E(k,4+j)];
                line(l(:,1),l(:,2),l(:,3),'color','g','linestyle','-'); 

                l = [E(k,i) E(k,2+j) E(k,4+1);
                     E(k,i) E(k,2+j) E(k,4+2)];
                line(l(:,1),l(:,2),l(:,3),'color','g','linestyle','-'); 
            end;
        end
    end

    X = [];
    Y = [];
    Z = [];
    for k = 1: M
        for t = 0 : 0.1 : T(k)
            t_  = t.^[0:N-1];
            X(end+1)    = t_ * P(k*N-N+1:k*N,1);
            Y(end+1)    = t_ * P(k*N-N+1:k*N,2);
            Z(end+1)    = t_ * P(k*N-N+1:k*N,3);
        end
    end
    line(X,Y,Z,'color','b','linestyle','-');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    
    hold off;
end
