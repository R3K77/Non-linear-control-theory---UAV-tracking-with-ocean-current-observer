function tau = backstepping_controller(state)
    % Wejœcia:
    % state - aktualny stan AUV [x, y, z, u, v, w, phi, theta, psi, p, q, r]
    % ref - trajektoria referencyjna [x_ref, y_ref, z_ref]
    
    % Parametry AUV z artyku³u
    k_pos = 0.5;
    k1 = 0.3; k2 = 1; k3 = 0.5;
    k4 = 0.3; k5 = 0.5; k6 = 0.2;
    k7 = 0.5; k8 = 5;
    
    % Rozpakowanie stanu AUV
    x = state(1); y = state(2); z = state(3);
    u = state(4); v = state(5); w = state(6);
    theta = state(7); psi = state(8);
    q = state(9); r = state(10);
    
    % Rozpakowanie trajektorii referencyjnej
    x_ref = state(11); y_ref = state(12); z_ref = state(13);
    
    % Wyliczenie prêdkoœci referencyjnych na podstawie trajektorii
    vx_ref = (x_ref - x) * k_pos;
    vy_ref = (y_ref - y) * k_pos;
    vz_ref = (z_ref - z) * k_pos;
    
    % Obliczenie docelowych k¹tów na podstawie kierunku ruchu
    psi_ref = atan2(vy_ref, vx_ref);
    theta_ref = -atan2(vz_ref, sqrt(vx_ref^2 + vy_ref^2));
    
    % B³êdy œledzenia pozycji
    ex = x - x_ref;
    ey = y - y_ref;
    ez = z - z_ref;
    
    % B³êdy prêdkoœci
    eu = u - vx_ref;
    ev = v - vy_ref;
    ew = w - vz_ref;
    
    % B³êdy k¹towe
    e_theta = theta - theta_ref;
    e_psi = psi - psi_ref;
    
    % Sterowania backstepping
    tau_u = -k1 * ex - k2 * eu;
    tau_q = -k3 * e_theta - k4 * q;
    tau_r = -k5 * e_psi - k6 * r;
    
    % Wartoœci steruj¹ce
    tau = [tau_u; tau_q; tau_r];
    
end
