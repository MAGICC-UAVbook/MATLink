function y = mavlinkReceiveControls(u)

 %message_output_type = u(1)
 %output_time = u(2)
 delta_a = u(3);
 delta_e = u(4);
 delta_r = u(5);
 delta_t = u(6);
 h_c = 0;
 Va_c = 0;
 phi_c = 0;
 theta_c = 0;
 chi_c = 0;
 
 
 % control outputs
 delta = [delta_e; delta_a; delta_r; delta_t];
 % commanded (desired) states
 x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        theta_c;%*P.K_theta_DC;... % theta
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];

 y = [delta; x_command];

end