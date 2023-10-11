clear;
clc;

data = [100,0;
    74.9623,63.1241;
    19.0442,110.3690;
    -52.1027,91.1609;
    -92.0077,33.7429;
    -105.2723,-38.2328;
    -52.3889,-90.9967;
    17.3038,-96.4602;
    86.1478,-71.5721]; % 没有选取（0，0）点

% 随机生成测试数据
data_std = zeros(9,2);
for i = 0:8
    ang = 40*i;
    data_std(i+1,:) = [100*cosd(ang) 100*sind(ang)];
end

dddata = [];
for learning_rate = 0.9:0.1:1.6
    sums = 0;
    for j = 1:100
        % 随机生成初始数据点
        data_test = zeros(9,2);
        data_test(1,:) = [100 0];
        for i = 2:9
            theta = 40*(i-1)+(randn(1)-0.5)*3;
            r = 100+randn(1)*2;
            data_test(i,:) = [r*cosd(theta) r*sind(theta)];
        end
        data = data_test;
        [~,steps,~] = adjust_of_drone(data,data_std,learning_rate);
        sums = sums+steps;
    end
    sums = sums/100;
    dddata = [dddata;learning_rate sums];
end
plot(dddata(:,1),dddata(:,2),'-',LineWidth=1)
xlabel('learning rate')
ylabel('average iteration steps')

% learning_rate = 0.9;
% 
% % 作图
% sz = 15;
% c1 = [0 0.4470 0.7410];
% c2 = [0.8500 0.3250 0.0980];
% [xc0,yc0,R0] = fit_circular(data);
% alpha0=0:pi/30:2*pi;
% x0=R0*cos(alpha0)+xc0;
% y0=R0*sin(alpha0)+yc0;
% plot(x0,y0,'-',LineWidth=1)
% axis equal
% hold on
% 
% learning_rate = 1;
% [data_adjusted,steps,los] = adjust_of_drone(data,data_std,learning_rate);
% [xc,yc,R] = fit_circular(data_adjusted);
% alpha=0:pi/30:2*pi;
% x=R*cos(alpha)+xc;
% y=R*sin(alpha)+yc;
% plot(x,y,'-',LineWidth=1)
% hold on
% 
% scatter(data(:,1),data(:,2),sz,c1,'filled');
% scatter(data_adjusted(:,1),data_adjusted(:,2),sz,c2,'filled');
% legend('originally fitted circle','eventually fitted circle','original data','adjusted data')
% xlabel(['iteration steps=',num2str(steps),' learning rate=',num2str(learning_rate),' residual=',num2str(los)])

%函数
function indexes = choice_of_drone(data,data_std)
% 随机共三架无人机得到的角度坐标，用100m的数据得到估计坐标
% 再拟合成圆，最后计算估计值距离圆最近的三个无人机作为发射源
sample = randperm(9);
index = sample(1:2);
index = sort(index,"ascend");

% 所有飞机和当前选到的两架圆弧上的无人机以及圆心无人机的角
[alpha,clockwise] = get_alpha(data,index);
% 通过式子解出所有的坐标
coordinates = get_coordinates(index,alpha,clockwise,data_std);
% 拟合成圆形
coordinates_exclude = [];
for i = 1:9
    if i == index(1) || i == index(2)
        continue
    end
    coordinates_exclude = [coordinates_exclude;coordinates(i,:) i];
end
[xc,yc,R] = fit_circular(coordinates_exclude); % coordinates中不包含了发射的那几架飞机，但是这些所谓的坐标是data_std，但是其实都是假设的错误坐标

sum = 0;
for i = 1:7
    tmp = coordinates_exclude(i,:)-[xc,yc,0];
    sum = sum+abs(norm(tmp(:,1:2))-R);
end
if sum <= 10
    indexes = sort(datasample(coordinates_exclude(:,3),3));
    % 或者求距离最小的三个飞机
    % error = [];
    % for i = 1:9
    %     error = [error;abs(norm(coordinates_exclude(i,1:2)-[xc,yc])-R)];
    % end
    % m=zeros(1,3);
    % error_min=zeros(1,3);
    % m(1)=find(error==min(error));
    % error_min(1)=error(m(1));
    % error(m(1))=max(error);
    % m(2)=find(error==min(error));
    % error_min(2)=error(m(2));
    % error(m(2))=max(error);
    % m(3)=find(error==min(error));
    % error_min(3)=error(m(3));
    % error(m(3))=max(error);
    %
    % % 发射信号飞机的编号
    % indexes = sort([m(1);m(2);m(3)],"ascend");
else
    indexes = sort([index datasample(coordinates_exclude(:,3),1)]);
end


end

function [data_adjusted,steps,los] = adjust_of_drone(data,data_std,learning_rate)
% 每次调整，通过三/四架飞机来确定当前飞机的位置坐标
% 再用估计坐标调整到标准位置上
data_adjusted = data;
% indexes = choice_of_drone(data,data_std);
indexes = sort(randperm(9));
steps = 0;
while loss(data_adjusted,data_std)>=1e-2
% for i = 1:20000
    steps = steps+1;
    [alpha_case12,clockwise12] = get_alpha(data_adjusted,[indexes(1),indexes(2)]);
    [alpha_case13,clockwise13] = get_alpha(data_adjusted,[indexes(1),indexes(3)]);
    coordinates12 = get_coordinates([indexes(1),indexes(2)],alpha_case12,clockwise12,data_std);
    coordinates13 = get_coordinates([indexes(1),indexes(3)],alpha_case13,clockwise13,data_std);
    coordinates = (coordinates13+coordinates12)/2;
    data_adjusted = data_adjusted-learning_rate*(coordinates-data_std);
    %     indexes = choice_of_drone(data_adjusted,data_std);
    indexes = sort(randperm(9));
end
los = loss(data_adjusted,data_std);
end

function l = loss(data_adjusted,data_std)
tmp = (data_adjusted-data_std).^2;
l = sum(sqrt(tmp(:,1)+tmp(:,2)));
end

function [xc,yc,R] = fit_circular(coordinates)
% 通过参数拟合成圆形，得到圆心和半径值
x = coordinates(:,1); y = coordinates(:,2);
n=length(x); xx=x.*x; yy=y.*y; xy=x.*y;
A=[sum(x) sum(y) n;sum(xy) sum(yy)...
    sum(y);sum(xx) sum(xy) sum(x)];
B=[-sum(xx+yy) ; -sum(xx.*y+yy.*y) ; -sum(xx.*x+xy.*y)];
a=A\B;
xc = -.5*a(1);
yc = -.5*a(2);
R = sqrt((a(1)^2+a(2)^2)/4-a(3));
end

function [angle,clockwise] = get_angle(coordinates)
% 通过三点坐标得到夹角角度大小
% 输入三点坐标，输出13与23的夹角
sub13 = coordinates(1,:)-coordinates(3,:);
k1 = sub13(2)/sub13(1);
sub23 = coordinates(2,:)-coordinates(3,:);
k2 = sub23(2)/sub23(1);
if sub13(1) == 0
    if sub13(2)>=0 && sub23(1)>0 || sub13(2)<=0 && sub23(1)<0
        clockwise = 1;
    else
        clockwise = -1;
    end
    angle = 90-atan(abs(k2))/pi*180;
elseif sub23(1) == 0
    if sub13(1)>0
        clockwise = -1;
    else
        clockwise = 1;
    end
    angle = 90-atan(abs(k1))/pi*180;
else
    tmp = (k1-k2)/(1+k1*k2);
    % 顺逆时针判断：13到23是顺时针，clockwise=1，否则为-1
    if tmp>=0
        clockwise = 1;
    else
        clockwise = -1;
    end
    angle = (atan(abs(tmp)))/pi*180;
end
end

function [alpha,clockwise] = get_alpha(data,index)
% 所有飞机和当前选到的两架圆弧上的无人机以及圆心无人机的夹角，第i，j项是第i架飞机的alpha_j
% index是被选飞机
% clockwise只看相对于飞机1的顺逆时针就可以直接判断，index1和index2分别代表第一二驾飞机的编号
alpha = []; clockwise = [];
for i = 1:9
    if i == index(1)
        [alpha2,clockwise23] = get_angle([0 0;data(index(2),:);data(i,:)]);
        alpha = [alpha;0 alpha2 0];
        clockwise13 = 1;
        clockwise = [clockwise;clockwise13 clockwise23];
    elseif i == index(2)
        [alpha1,clockwise13] = get_angle([0 0;data(index(1),:);data(i,:)]);
        alpha = [alpha;alpha1 0 0];
        clockwise23 = 1;
        clockwise = [clockwise;clockwise13 clockwise23];
    else
        [alpha1,clockwise13] = get_angle([0 0;data(index(1),:);data(i,:)]); % 原点和第一架
        [alpha2,clockwise23] = get_angle([0 0;data(index(2),:);data(i,:)]); % 原点和第二架
        [alpha3,~] = get_angle([data(index(1),:);data(index(2),:);data(i,:)]); % 两架圆弧
        alpha = [alpha;alpha1 alpha2 alpha3];
        clockwise = [clockwise;clockwise13 clockwise23];
    end
end
end

function coordinates = get_coordinates(index,alpha,clockwise,data_std)
% 通过输入角度求出坐标
i = -clockwise;
coordinates = [];
% 第k个飞机的两个alpha，以及对应的角度
for k = 1:9
    if k == index(1) || k == index(2)
        coordinates = [coordinates;data_std(k,:)];
    else
        theta = (index(2)-index(1))*40;
        tan_theta_hat = ((sind(alpha(k,1))*i(k,2)*sind(i(k,2)*alpha(k,2)-theta)-sind(alpha(k,2))*sind(alpha(k,1)))/ ...
            (sind(alpha(k,2))*i(k,1)*cosd(alpha(k,1))-sind(alpha(k,1))*i(k,2)*cosd(i(k,2)*alpha(k,2)-theta)));
        if i(k,1) >= 0
            if tan_theta_hat>=0
                theta_hat = atan(tan_theta_hat)/pi*180;
            else
                theta_hat = atan(tan_theta_hat)/pi*180+180;
            end
        else
            if tan_theta_hat>=0
                theta_hat = atan(tan_theta_hat)/pi*180+180;
            else
                theta_hat = 360+atan(tan_theta_hat)/pi*180;
            end
        end
        theta_hat = theta_hat+(index(1)-1)*40;
        r = 100*i(k,1)*sind(theta_hat+i(k,1)*alpha(k,1))/sind(alpha(k,1));
        coordinates = [coordinates;r*cosd(theta_hat) r*sind(theta_hat)];
    end
end
end