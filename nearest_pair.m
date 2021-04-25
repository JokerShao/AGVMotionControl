sensor_array = csvread('wheel_steer.txt');
ideal_twist = csvread('ideal_twist.txt');
ideal_twist(:,1) = ideal_twist(:,1)*1000;

n = 8000;% size(sensor_array,1);
paired = zeros(n, 9+4);

for i=1:n
    i
    min_offset = 1e10;
    paired(i,1:9) = sensor_array(i,:);
    for j=1:n
        if abs(sensor_array(i,1)-ideal_twist(j,1)) < min_offset
            min_offset = abs(sensor_array(i,1)-ideal_twist(j,1));
            paired(i,10:9+4) = ideal_twist(j,:);
        end
    end
end


save('paired.mat', 'paired')
