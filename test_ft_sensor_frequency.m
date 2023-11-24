
change_ft_sensor_frequency()
disp("Changed frequency")

sub_r = rossubscriber('/right_wrist_ft');
sub_l = rossubscriber('/left_wrist_ft');

% tic_toc_table = zeros(2,100);
% 
% for i = 1:100
%     if i == 1
%         t_init = tic;
%         msg_r = receive(sub_r, 1);
%         t1 = toc(t_init);
%         msg_l = receive(sub_l, 1);
%         t2 = toc(t_init);
%     else
%         t_init = tic;
%         msg_r = receive(sub_r, 0.005);
%         t1 = toc(t_init);
%         msg_l = receive(sub_l, 0.005);
%         t2 = toc(t_init);
%     end
%     tic_toc_table(1,i)=t1;
%     tic_toc_table(2,i)=t2-t1;
% end


msg_r = receive(sub_r, 10);
msg_l = receive(sub_l, 10);

disp("Start")

t_init = tic;
for i = 1:1000
    msg_l = sub_l.LatestMessage;
    msg_r = sub_r.LatestMessage;
end
t_end = toc(t_init);

disp("Stop")

avg = (t_end)/1000/2;
disp(avg)
% plot(1:100,tic_toc_table)