function V_out = FeedbackControl(Tse_curr_act, Tse_curr, Tse_next, kp, ki, del_t)

%test







se3_v_d = (1/del_t)*MatrixLog6((inv(Tse_curr))*Tse_next);
V_s_d = se3ToVec(se3_v_d);
%disp(V_s_d);

V_d = Adjoint((inv(Tse_curr_act))*(Tse_curr))*V_s_d;
%disp('V_d');
%disp(V_d);

X_err_se3 = MatrixLog6((inv(Tse_curr_act))*Tse_curr);
X_err = se3ToVec(X_err_se3);

%disp(X_err);
global intg_X_err;
intg_X_err = intg_X_err + (X_err*del_t);

V_out = V_d + (kp*X_err) + (ki*intg_X_err);
%disp(V_out);
end





