clear all
clc

syms a1 a2 a3 a4 a5 t1 t2 t3 t4
syms nx ox ax px ny oy ay py nz oz az pz

T1 = Trans(0,0,a1)*Rot('z', t1)*Trans(0,0,a2);
T2 = Rot('y',t2)*Trans(0,0,a3);
T3 = Rot('y',t3)*Trans(0,0,a4);
T4 = Rot('z',t4)*Trans(0,0,a5);

TF = simplify(T1*T2*T3*T4)

a1=55;
a2=30;
a3=200;
a4=200;
a5=90;
t1 = 0;
t2 = 0;
t3 = 0;
t4 = 0;

TFNum = eval(subs(TF)) %Matriz de transformacion final dadas las medidas

%CINEMATICA INVERSA (por metodo geometrico)(resultado en radianes)
syms px py pz
px = 100;
py = 100;
pz = 100;
%ecuaciones de cinematica inversa
theta1 = atan(py/px);
theta2 = atan(pz/px) + acos((a3^2+(px^2+pz^2)-(a4+a5)^2)/(2*a3*(sqrt(px^2+pz^2))));
theta3 = acos((a3^2+(a4+a5)^2-(sqrt(px^2+pz^2))^2)/(2*a3*(a4+a5))) - pi;

%angulos conseguidos para un px py pz
rad2deg(theta1)
rad2deg(theta2)
rad2deg(theta3)

%funciones auxiliares
function [RotMatrix] = Rot(axis, val)

    if axis == 'x'
        RotMatrix = [1,     0,     0,     0;
                     0,cos(val),-sin(val),0;
                     0,sin(val), cos(val),0;
                     0,   0,        0,    1];
    elseif axis == 'y'
        RotMatrix = [ cos(val),0,sin(val),0;
                         0,    1,    0,   0;
                     -sin(val),0,cos(val),0;
                         0,    0,    0,   1];
    elseif axis == 'z'
        RotMatrix = [ cos(val),-sin(val),0,0;
                      sin(val), cos(val),0,0;
                         0,        0,    1,0;
                         0,        0,    0,1];
    end
end

function [TrasMatrix] = Trans(x,y,z)
    TrasMatrix=[1,0,0,x;
                0,1,0,y;
                0,0,1,z;
                0,0,0,1];
end