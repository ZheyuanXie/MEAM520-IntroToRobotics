[a,b]=view();
map = loadmap('map/map_2.txt');
lynxStart('Frame','off','Gripper','on');plotmap(map);hold on;
dense = 15;
range = 400;
[X,Y,Z] = meshgrid(-range:dense:range,-range:dense:range,-range:dense:range);
zeta = 1;
p_goal = [250,-200,222.25];
Fatt_x = -zeta * (X-p_goal(1));
Fatt_y = -zeta * (Y-p_goal(2));
Fatt_z = -zeta * (Z-p_goal(3));

eta = 10e5; rho0 = 100;
Frep_x = X; Frep_y = Y; Frep_z = Z;
for i = 1:size(X,1)
    for j = 1:size(Y,1)
        for k = 1:size(Z,1)
        rho_min = rho0;
        Frep_x(i,j,k) = 0;
        Frep_y(i,j,k) = 0;
        Frep_z(i,j,k) = 0;
        for ob_idx=1:size(map.obstacles,1)
                box = map.obstacles(ob_idx,:);
                [rho, b] = distPointToBox([X(i,j,k),Y(i,j,k),Z(i,j,k)],box);
                if rho < rho_min %eta*(1/rho - 1/rho0)*(1/rho^2)*
                    Frep_x(i,j,k) = eta*(1/rho - 1/rho0)*(1/rho^2)*(X(i,j,k)-b(1));
                    Frep_y(i,j,k) = eta*(1/rho - 1/rho0)*(1/rho^2)*(Y(i,j,k)-b(2));
                    Frep_z(i,j,k) = eta*(1/rho - 1/rho0)*(1/rho^2)*(Z(i,j,k)-b(3));
                    rho_min = rho;
                end
        end
        end
    end
end
thre = 2000;
Frep_x(Frep_x>thre)=thre;Frep_x(Frep_x<-thre)=-thre;
Frep_y(Frep_y>thre)=thre;Frep_y(Frep_y<-thre)=-thre;
Frep_z(Frep_z>thre)=thre;Frep_z(Frep_z<-thre)=-thre;
% a=quiver3(X,Y,Z,Fatt_x,Fatt_y,Fatt_z,'Color',[0,0.5,0],'LineWidth',2);
% b=quiver3(X,Y,Z,Frep_x,Frep_y,Frep_z,'Color',[0.7,0,0],'LineWidth',2);
quiver3(X,Y,Z,Fatt_x+Frep_x,Fatt_y+Frep_y,Fatt_z+Frep_z,'LineWidth',2);
axis([000,400,-300,300,0,410])
view(-90,0)
% legend([a,b],{"Attractive Field","Repulsive Field"})