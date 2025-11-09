function robotat_3pi_gripper_open(robot)
    % encode to send to ESP32
    msg = uint8(zeros(1, 2));
    msg(1) = 50; % gripper
    msg(2) = 10; % open
    write(robot.tcpsock, msg);
end