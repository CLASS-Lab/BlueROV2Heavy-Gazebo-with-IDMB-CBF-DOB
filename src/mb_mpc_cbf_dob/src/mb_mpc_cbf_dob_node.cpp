#include "mb_mpc_cbf_dob/mb_mpc_cbf_dob_node.h"

namespace mb_mpc_cbf_dob
{
    MbMpcCbfDobNode::MbMpcCbfDobNode(ros::NodeHandle &nh, double tc) :  nh_("~"),
                                                                    bluerov2_states({35.5, -9, 93.3,
                                                                                    0.0, 0.0, 0.0,
                                                                                    0.0, 0.0, 0.0,
                                                                                    0.0, 0.0, 0.0}),
                                                                   bluerov2_input({0, 0, 0, 0, 0, 0}),
                                                                   last_tau_out({0, 0, 0, 0, 0, 0}),
                                                                    target_state_value({20,-9,94,
                                                                                       0.0, 0.0, 0.0,
                                                                                       0.0, 0.0, 0.0,
                                                                                       0.0, 0.0, 0.0}),
                                                                    d_hat({0,0,0,0,0,0}),
                                                                    z_hat({0,0,0,0,0,0})
    {
        nh_.getParam("horizon", horizon_);
        std::string blocking_index_str;
        nh_.getParam("blocking_index", blocking_index_str);
        std::stringstream ss(blocking_index_str);
        std::string item;
        while (std::getline(ss, item, ',')) {
            blocking_indexes.push_back(std::stoi(item));
        }
        // std::cout<<"blocking_index: "<<blocking_index_str<<std::endl;
        // std::cout<<"blocking_index size : "<<blocking_indexes.size()<<std::endl;
        nh_.getParam("k_alpha_1", k_alpha_1);
        nh_.getParam("k_alpha_2", k_alpha_2);
        nh_.getParam("omega_1", omega_1);
        nh_.getParam("beta", beta);
        nh_.getParam("v1", v1);
        nh_.getParam("alpha", alpha);
        nh_.getParam("l_factor", l_factor);
        //  the eq is \lambda^2 + k_alpha_2*\lambda + k_alpha_1 = 0
        // since the state feedback is [k_alpha_1, k_alpha_2]
        //  it's basic knowledge of modern control theory
        p2 = (-k_alpha_2 - std::sqrt(k_alpha_2*k_alpha_2 - 4*k_alpha_1)) / 2.0;
        std::cout<<"p2: "<<p2<<std::endl;
        std::cout<<"k_alpha_1: "<<k_alpha_1<<std::endl;
        std::cout<<"k_alpha_2: "<<k_alpha_2<<std::endl; 
        std::cout<<"omega_1: "<<omega_1<<std::endl;
        std::cout<<"beta: "<<beta<<std::endl;
        std::cout<<"v1: "<<v1<<std::endl;
        std::cout<<"alpha: "<<alpha<<std::endl;
        std::cout<<"l_factor: "<<l_factor<<std::endl;

        tc_ = tc;
        // we init this to 0, so that we can pass the first update of DOB with incomplete time
        last_time = ros::Time(0);
        /******************************************************/
        /*casadi modeling, create solver*/
        // state error
        casadi::SX Q = casadi::SX::diag(
            casadi::SX::vertcat({4000, 4000, 2000,
                                 200, 200, 80,
                                 30, 30, 10,
                                 30, 30, 10}));
        // input
        casadi::SX R = casadi::SX::diag(
            casadi::SX::vertcat({10, 10, 1, 100, 100, 5}));
        // terminal state error
        casadi::SX P = casadi::SX::diag(
            casadi::SX::vertcat({5000, 5000, 4000,
                                 200, 200, 150,
                                 200, 200, 120,
                                 100, 100, 60}));
        // 2*1
        casadi::SX k_alpha =
            casadi::SX::horzcat({k_alpha_1, k_alpha_2});

        // M = MRB + MA
        // std::vector<double> m1 = {m, m, m, Ix, Iy, Iz};
        // std::vector<double> m2 = {Xu, Yv, Zw, Kp, Mq, Nr};
        // casadi::SX M = casadi::SX::diag(m1) + casadi::SX::diag(m2);

        std::vector<double> M_diag_vec(M_diag_data, M_diag_data + 6);

        casadi::SX M = casadi::SX::diag({M_diag_vec});
        // std::cout<<"M: "<<M<<std::endl;
        
        std::vector<double> line = {M_params[EnumMParams::Xu], M_params[EnumMParams::Yv], M_params[EnumMParams::Zw], M_params[EnumMParams::Kp], M_params[EnumMParams::Mq], M_params[EnumMParams::Nr]};
        casadi::SX D_linear = casadi::SX::diag(line);

        /* *******************************************/
        // cost and constraint
        casadi::SX cost = casadi::SX::zeros(1, 1);
        casadi::SX constraints = casadi::SX::vertcat({});

        /* ******************************************* */
        // basic define!! and modeling

        casadi::Slice all;
        // input
        int block_idx = 0;
        n_block = blocking_indexes.size();
        casadi::SX tau = casadi::SX::sym("tau", 6, n_block);
        casadi::SX tau_d = casadi::SX::sym("tau_d",6,1);
        // sys state
        casadi::SX sym_state = casadi::SX::sym("sym_state",12, horizon_+1);
        // Extract x as the first row of the state matrix
        casadi::SX initial_state = casadi::SX::sym("state0",12, 1);


        casadi::SX target_state= casadi::SX::sym("target_state", 12, 1);

        sym_state(all, 0) = initial_state;

        casadi::SX f_fcn  = casadi::SX::zeros(12,1),g_fcn = casadi::SX::zeros(12,6);
        // std::cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!n_block: "<<n_block<<std::endl;
        for (int i = 0; i < horizon_; i++)
        {
            // firstly we check if u needs to be updated
            if ((block_idx != n_block - 1) && ((i == 0) || (i >= blocking_indexes[block_idx + 1]))) 
            {
                // std::cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!i: "<<i<<std::endl;
                if(i != 0)
                {
                    // update u
                    block_idx++;     
                    // std::cout<< "!!!!!!!!!!!!!!!!!!!!!!!!!!!!block_idx: "<<block_idx<<std::endl;     
                }

                //  we only add constr and update system dynamics when u is updated

                casadi::SX sym_x,sym_y ,sym_z ,sym_psi ,sym_theta ,sym_phi ,sym_u ,sym_v ,sym_w ,sym_p ,sym_q ,sym_r, sym_nu ,eta_linear ,nu_linear;

            
                sym_x = sym_state(0, i);
                sym_y = sym_state(1, i);
                sym_z = sym_state(2, i);

                sym_psi = sym_state(5, i);
                sym_theta = sym_state(4, i);
                sym_phi = sym_state(3, i);

                sym_u = sym_state(6, i);
                sym_v = sym_state(7, i);
                sym_w = sym_state(8, i);

                sym_p = sym_state(9, i);
                sym_q = sym_state(10, i);
                sym_r = sym_state(11, i);
            //   slice does not include the stop!
                sym_nu = sym_state(casadi::Slice(6, 12), i);
                eta_linear = sym_state(casadi::Slice(0, 3), i);
                nu_linear = sym_state(casadi::Slice(6, 9), i);

                // Kinematic Jacobians (J1 and J2)
                casadi::SX J1 = vertcat(
                    horzcat(cos(sym_psi) * cos(sym_theta),
                            -sin(sym_psi) * cos(sym_phi) + cos(sym_psi) * sin(sym_theta) * sin(sym_phi),
                            sin(sym_psi) * sin(sym_phi) + cos(sym_psi) * cos(sym_phi) * sin(sym_theta)),
                    horzcat(sin(sym_psi) * cos(sym_theta),
                            cos(sym_psi) * cos(sym_phi) + sin(sym_phi) * sin(sym_theta) * sin(sym_psi),
                            -cos(sym_psi) * sin(sym_phi) + sin(sym_theta) * sin(sym_psi) * cos(sym_phi)),
                    horzcat(-sin(sym_theta), cos(sym_theta) * sin(sym_phi), cos(sym_theta) * cos(sym_phi)));

                casadi::SX J2 = vertcat(
                    horzcat(casadi::SX::ones(1, 1), sin(sym_phi) * tan(sym_theta), cos(sym_phi) * tan(sym_theta)),
                    horzcat(casadi::SX::zeros(1, 1), cos(sym_phi), -sin(sym_phi)),
                    horzcat(casadi::SX::zeros(1, 1), sin(sym_phi) / cos(sym_theta), cos(sym_phi) / cos(sym_theta)));

                casadi::SX J = vertcat(horzcat(J1, casadi::SX::zeros(3, 3)), horzcat(casadi::SX::zeros(3, 3), J2));

                casadi::SX C_RB = casadi::SX::zeros(6, 6);
                C_RB(0, 4) = m * sym_w;
                C_RB(0, 5) = -m * sym_v;
                C_RB(1, 3) = -m * sym_w;
                C_RB(1, 5) = m * sym_u;
                C_RB(2, 3) = m * sym_v;
                C_RB(2, 4) = -m * sym_u;
                C_RB(3, 1) = m * sym_w;
                C_RB(3, 2) = -m * sym_v;
                C_RB(3, 4) = M_params[EnumMParams::Iz] * sym_r;
                C_RB(3, 5) = -M_params[EnumMParams::Iy] * sym_q;
                C_RB(4, 0) = -m * sym_w;
                C_RB(4, 2) = m * sym_u;
                C_RB(4, 3) = -M_params[EnumMParams::Iz] * sym_r;
                C_RB(4, 5) = M_params[EnumMParams::Ix] * sym_p;
                C_RB(5, 0) = m * sym_v;
                C_RB(5, 1) = -m * sym_u;
                C_RB(5, 3) = M_params[EnumMParams::Iy] * sym_q;
                C_RB(5, 4) = -M_params[EnumMParams::Ix] * sym_p;

                casadi::SX C_A = casadi::SX::zeros(6, 6);
                C_A(0, 4) = Zwd * sym_w;
                C_A(0, 5) = Yvd * sym_v;
                C_A(1, 3) = -Zwd * sym_w;
                C_A(1, 5) = -Xud * sym_u;
                C_A(2, 3) = -Yvd * sym_v;
                C_A(2, 4) = Xud * sym_u;
                C_A(3, 1) = -Zwd * sym_w;
                C_A(3, 2) = Yvd * sym_v;
                C_A(3, 4) = -Nrd * sym_r;
                C_A(3, 5) = Mqd * sym_q;
                C_A(4, 0) = Zwd * sym_w;
                C_A(4, 2) = -Xud * sym_u;
                C_A(4, 3) = Nrd * sym_r;
                C_A(4, 5) = -Kpd * sym_p;
                C_A(5, 0) = -Yvd * sym_v;
                C_A(5, 1) = Xud * sym_u;
                C_A(5, 3) = -Mqd * sym_q;
                C_A(5, 4) = Kpd * sym_p;

                casadi::SX D_nonlinear = casadi::SX::zeros(6, 6);
                D_nonlinear(0, 0) = Xuu * fabs(sym_u);
                D_nonlinear(1, 1) = Yvv * fabs(sym_v);
                D_nonlinear(2, 2) = Zww * fabs(sym_w);
                D_nonlinear(3, 3) = Kpp * fabs(sym_p);
                D_nonlinear(4, 4) = Mqq * fabs(sym_q);
                D_nonlinear(5, 5) = Nrr * fabs(sym_r);



                casadi::SX g_eta = casadi::SX::zeros(6, 1);
                g_eta(0) = (W - B) * sin(sym_theta);
                g_eta(1) = -(W - B) * cos(sym_theta) * sin(sym_phi);
                g_eta(2) = -(W - B) * cos(sym_theta) * cos(sym_phi);
                g_eta(3) = zb * B * cos(sym_theta) * sin(sym_phi);
                g_eta(4) = zb * B * sin(sym_theta);

                casadi::SX eta_dot = mtimes(J, sym_nu);
                casadi::SX nu_dot =
                    mtimes(inv(M),
                        tau(all, block_idx)+ tau_d - mtimes((C_RB + C_A), sym_nu) - mtimes((D_linear + D_nonlinear), sym_nu) - g_eta);
                casadi::SX state_dot = vertcat(eta_dot, nu_dot);
                g_fcn = vertcat(casadi::SX::zeros(6, 6), inv(M));
                // casadi::SX f_fcn = state_dot - mtimes(g_fcn, tau(all, i));
                f_fcn = vertcat(eta_dot,  
                                mtimes(inv(M),
                                - mtimes((C_RB + C_A), sym_nu) - mtimes((D_linear + D_nonlinear), sym_nu) - g_eta));
                casadi::SX Lfh;
                casadi::SX h_fcn;


                //  we only apply new cbf constr when u is updated
                for (int j = 0; j < N_OB; j++)
                {
                    std::vector<double> ob_c(obstacles[j], obstacles[j] + 8);
                    casadi::SX ob_current = casadi::SX(ob_c);
                    // std::cout <<"ob!!!!!!!!!!!!!!!!: "<< ob_c <<std::endl;
                    // std::cout<< "ob done!!!"<<std::endl;
                    // std::cout <<"ob slice!!!!!!!!!!!!!!!: "<< ob_current(casadi::Slice(1, 4)) <<std::endl;
                    // tested both ok!
                    casadi::SX partial_Lfh = casadi::SX::zeros(12, 1);
                    if (obstacles[j][0] == 0)
                    {
                        casadi::SX tmp = casadi::SX::eye(3);
                        tmp(2, 2) = 0;
                        Lfh = dot(
                            2 * mtimes(tmp, (eta_linear - ob_current(casadi::Slice(1, 4)))),
                            mtimes(J1, nu_linear));
                        h_fcn = pow((sym_x - ob_current(1)), 2) + pow((sym_y - ob_current(2)), 2) - pow(ob_current(7), 2);

                        partial_Lfh(0) = 2 * sym_w * (sin(sym_phi) * sin(sym_psi) + cos(sym_phi) * cos(sym_psi) * sin(sym_theta)) - 2 * sym_v * (cos(sym_phi) * sin(sym_psi) - cos(sym_psi) * sin(sym_phi) * sin(sym_theta)) + 2 * sym_u * cos(sym_psi) * cos(sym_theta);
                        partial_Lfh(1) = 2 * sym_v * (cos(sym_phi) * cos(sym_psi) + sin(sym_phi) * sin(sym_psi) * sin(sym_theta)) - 2 * sym_w * (cos(sym_psi) * sin(sym_phi) - cos(sym_phi) * sin(sym_psi) * sin(sym_theta)) + 2 * sym_u * cos(sym_theta) * sin(sym_psi);
                        partial_Lfh(3) = (2 * obstacles[j][2] - 2 * sym_y) * (sym_v * (cos(sym_psi) * sin(sym_phi) - cos(sym_phi) * sin(sym_psi) * sin(sym_theta)) + sym_w * (cos(sym_phi) * cos(sym_psi) + sin(sym_phi) * sin(sym_psi) * sin(sym_theta))) - (2 * obstacles[j][1] - 2 * sym_x) * (sym_v * (sin(sym_phi) * sin(sym_psi) + cos(sym_phi) * cos(sym_psi) * sin(sym_theta)) + sym_w * (cos(sym_phi) * sin(sym_psi) - cos(sym_psi) * sin(sym_phi) * sin(sym_theta)));
                        partial_Lfh(4) = -2 * (sym_w * cos(sym_phi) * cos(sym_theta) - sym_u * sin(sym_theta) + sym_v * cos(sym_theta) * sin(sym_phi)) * (obstacles[j][1] * cos(sym_psi) - sym_x * cos(sym_psi) + obstacles[j][2] * sin(sym_psi) - sym_y * sin(sym_psi)),
                        partial_Lfh(5) = (2 * obstacles[j][1] - 2 * sym_x) * (sym_v * (cos(sym_phi) * cos(sym_psi) + sin(sym_phi) * sin(sym_psi) * sin(sym_theta)) - sym_w * (cos(sym_psi) * sin(sym_phi) - cos(sym_phi) * sin(sym_psi) * sin(sym_theta)) + sym_u * cos(sym_theta) * sin(sym_psi)) - (2 * obstacles[j][2] - 2 * sym_y) * (sym_w * (sin(sym_phi) * sin(sym_psi) + cos(sym_phi) * cos(sym_psi) * sin(sym_theta)) - sym_v * (cos(sym_phi) * sin(sym_psi) - cos(sym_psi) * sin(sym_phi) * sin(sym_theta)) + sym_u * cos(sym_psi) * cos(sym_theta)),
                        partial_Lfh(6) = -cos(sym_psi) * cos(sym_theta) * (2 * obstacles[j][1] - 2 * sym_x) - cos(sym_theta) * sin(sym_psi) * (2 * obstacles[j][2] - 2 * sym_y);
                        partial_Lfh(7) = (2 * obstacles[j][1] - 2 * sym_x) * (cos(sym_phi) * sin(sym_psi) - cos(sym_psi) * sin(sym_phi) * sin(sym_theta)) - (2 * obstacles[j][2] - 2 * sym_y) * (cos(sym_phi) * cos(sym_psi) + sin(sym_phi) * sin(sym_psi) * sin(sym_theta));
                        partial_Lfh(8) = (2 * obstacles[j][2] - 2 * sym_y) * (cos(sym_psi) * sin(sym_phi) - cos(sym_phi) * sin(sym_psi) * sin(sym_theta)) - (2 * obstacles[j][1] - 2 * sym_x) * (sin(sym_phi) * sin(sym_psi) + cos(sym_phi) * cos(sym_psi) * sin(sym_theta));
                    }
                    else
                    {
                        // we defaultly use sphere obstacles,not cylinder!
                        Lfh = dot(
                            2 * (eta_linear - ob_current(casadi::Slice(1, 4))),
                            mtimes(J1, nu_linear));

                        h_fcn = pow((sym_x - ob_current(1)), 2) + pow((sym_y - ob_current(2)), 2) + pow((sym_z - ob_current(3)), 2) - pow(ob_current(7), 2);
                        partial_Lfh(0) = 2 * sym_w * (sin(sym_phi) * sin(sym_psi) + cos(sym_phi) * cos(sym_psi) * sin(sym_theta)) - 2 * sym_v * (cos(sym_phi) * sin(sym_psi) - cos(sym_psi) * sin(sym_phi) * sin(sym_theta)) + 2 * sym_u * cos(sym_psi) * cos(sym_theta);
                        partial_Lfh(1) = 2 * sym_v * (cos(sym_phi) * cos(sym_psi) + sin(sym_phi) * sin(sym_psi) * sin(sym_theta)) - 2 * sym_w * (cos(sym_psi) * sin(sym_phi) - cos(sym_phi) * sin(sym_psi) * sin(sym_theta)) + 2 * sym_u * cos(sym_theta) * sin(sym_psi);
                        partial_Lfh(3) = (2 * obstacles[j][2] - 2 * sym_y) * (sym_v * (cos(sym_psi) * sin(sym_phi) - cos(sym_phi) * sin(sym_psi) * sin(sym_theta)) + sym_w * (cos(sym_phi) * cos(sym_psi) + sin(sym_phi) * sin(sym_psi) * sin(sym_theta))) - (2 * obstacles[j][1] - 2 * sym_x) * (sym_v * (sin(sym_phi) * sin(sym_psi) + cos(sym_phi) * cos(sym_psi) * sin(sym_theta)) + sym_w * (cos(sym_phi) * sin(sym_psi) - cos(sym_psi) * sin(sym_phi) * sin(sym_theta))) - (2 * obstacles[j][3] - 2 * sym_z) * (sym_v * cos(sym_phi) * cos(sym_theta) - sym_w * cos(sym_theta) * sin(sym_phi));
                        partial_Lfh(4) = (2 * obstacles[j][3] - 2 * sym_z) * (sym_u * cos(sym_theta) + sym_w * cos(sym_phi) * sin(sym_theta) + sym_v * sin(sym_phi) * sin(sym_theta)) - cos(sym_psi) * (2 * obstacles[j][1] - 2 * sym_x) * (sym_w * cos(sym_phi) * cos(sym_theta) - sym_u * sin(sym_theta) + sym_v * cos(sym_theta) * sin(sym_phi)) - sin(sym_psi) * (2 * obstacles[j][2] - 2 * sym_y) * (sym_w * cos(sym_phi) * cos(sym_theta) - sym_u * sin(sym_theta) + sym_v * cos(sym_theta) * sin(sym_phi));
                        partial_Lfh(5) = (2 * obstacles[j][1] - 2 * sym_x) * (sym_v * (cos(sym_phi) * cos(sym_psi) + sin(sym_phi) * sin(sym_psi) * sin(sym_theta)) - sym_w * (cos(sym_psi) * sin(sym_phi) - cos(sym_phi) * sin(sym_psi) * sin(sym_theta)) + sym_u * cos(sym_theta) * sin(sym_psi)) - (2 * obstacles[j][2] - 2 * sym_y) * (sym_w * (sin(sym_phi) * sin(sym_psi) + cos(sym_phi) * cos(sym_psi) * sin(sym_theta)) - sym_v * (cos(sym_phi) * sin(sym_psi) - cos(sym_psi) * sin(sym_phi) * sin(sym_theta)) + sym_u * cos(sym_psi) * cos(sym_theta));
                        partial_Lfh(6) = sin(sym_theta) * (2 * obstacles[j][3] - 2 * sym_z) - cos(sym_psi) * cos(sym_theta) * (2 * obstacles[j][1] - 2 * sym_x) - cos(sym_theta) * sin(sym_psi) * (2 * obstacles[j][2] - 2 * sym_y);
                        partial_Lfh(7) = (2 * obstacles[j][1] - 2 * sym_x) * (cos(sym_phi) * sin(sym_psi) - cos(sym_psi) * sin(sym_phi) * sin(sym_theta)) - (2 * obstacles[j][2] - 2 * sym_y) * (cos(sym_phi) * cos(sym_psi) + sin(sym_phi) * sin(sym_psi) * sin(sym_theta)) - cos(sym_theta) * sin(sym_phi) * (2 * obstacles[j][3] - 2 * sym_z);
                        partial_Lfh(8) = (2 * obstacles[j][2] - 2 * sym_y) * (cos(sym_psi) * sin(sym_phi) - cos(sym_phi) * sin(sym_psi) * sin(sym_theta)) - (2 * obstacles[j][1] - 2 * sym_x) * (sin(sym_phi) * sin(sym_psi) + cos(sym_phi) * cos(sym_psi) * sin(sym_theta)) - cos(sym_phi) * cos(sym_theta) * (2 * obstacles[j][3] - 2 * sym_z);
                        // }
                    }
                    auto Lf2h = mtimes(transpose(partial_Lfh), f_fcn);
                    auto Lglfh = mtimes(transpose(partial_Lfh), g_fcn);

                    constraints = vertcat(
                        constraints,
                        //  in paper, minus lambda_2 , which is the -root, so we add root
                        Lf2h + mtimes(Lglfh, (tau(all, block_idx)+tau_d)) + k_alpha(0) * h_fcn + k_alpha(1) * Lfh -(omega_1*omega_1)/(2*v1*beta)- beta *casadi::SX::mtimes(Lglfh,Lglfh.T())/(4*alpha-2*v1+2*p2)
                        // Lf2h+ mtimes(Lglfh, tau(all,i))+  mtimes(k_alpha, vertcat(h_fcn, Lfh))
                        //
                    );
                    // std::cout<< "constr new: " << Lf2h + mtimes(Lglfh, tau(all, i)) + k_alpha(0) * h_fcn + k_alpha(1) * Lfh << std::endl;
                }
                
                sym_state(all, i + 1) = sym_state(all, i) + tc_ * state_dot;
            }
            else
            {
                // just update the state
                sym_state(all, i + 1) = sym_state(all, i) + tc_ * (f_fcn + mtimes(g_fcn, tau(all, block_idx)));
            }




            auto state_error = target_state- sym_state(all, i) ;
            cost = cost + casadi::SX::mtimes({state_error.T(), Q, state_error}) + casadi::SX::mtimes({tau(all, block_idx).T(), R, tau(all, block_idx)});
            // std::cout<< "cost new: " << casadi::SX::mtimes({state_error.T(), Q, state_error}) + casadi::SX::mtimes({tau(all, i).T(), R, tau(all, i)}) << std::endl;
        }
        auto state_error = target_state - sym_state(all, horizon_) ;
        cost = cost + casadi::SX::mtimes({state_error.T(), P, state_error});

        // checked correct
        // std::cout<< "!!!!!!!!!!!!!!!!!!constr size: " << constraints.size() << std::endl;

        //  right!
        // blocking_index: 0,1,3
        // blocking_index size : 3
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!n_block: 3
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!i: 0
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!i: 1
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!block_idx: 1
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!i: 3
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!block_idx: 2
        // !!!!!!!!!!!!!!!!!!constr size: [18,1]




        // std::cout << "important : " << casadi::SX::reshape(initial_state, -1, 1) << std::endl;
        //  every row is a unit, with 12+12  *1
        casadi::SX opt_params = casadi::SX::vertcat(
            {initial_state,
             target_state,
             tau_d});
        casadi::SXDict nlp =
            {{"x", casadi::SX::reshape(tau, -1, 1)},
             {"f", cost},
             {"g", constraints},
             {"p", opt_params}};
        casadi::Dict nlp_opts;
        nlp_opts["expand"] = true;
        nlp_opts["ipopt.max_iter"] = 1000;
        nlp_opts["ipopt.print_level"] = 5;
        nlp_opts["ipopt.sb"] = "yes";
        nlp_opts["print_time"] = 0;
        nlp_opts["ipopt.acceptable_tol"] = 1e-6;
        // nlp_opts["ipopt.acceptable_object_change_tol"] = 1e-4;

        this->mpc_cbf_solver = casadi::nlpsol("solver", "ipopt", nlp, nlp_opts);
        std::cout << "build success!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

        // Initialize the lower and upper bound vectors for the entire horizon_
        std::vector<double> lower_bounds = {-85, -85, -120, -26, -14, -22};
        std::vector<double> upper_bounds = {85, 85, 120, 26, 14, 22};
        // Repeat the bounds for each time step in the horizon_
        // int n_block = blocking_indexes.size();
        for (int i = 0; i < n_block; i++)
        {
            lbu.insert(lbu.end(), lower_bounds.begin(), lower_bounds.end());
            ubu.insert(ubu.end(), upper_bounds.begin(), upper_bounds.end());
        }
        /*create solver done*/
        /******************************************************/
        solve_time.data = 0;
        solve_time_pub = nh.advertise<std_msgs::Float64>("/bluerov2/solve_time", 20);


        geometry_msgs::Vector3 zero_vec;
        zero_vec.x = zero_vec.y = zero_vec.z = 0.0;
        std_eta.linear = std_eta.angular = zero_vec;
        cylinder_h = zero_vec;
        sphere_h = zero_vec;

        control_input_std.force = zero_vec;
        control_input_std.torque = zero_vec;
        control_std_pub = nh.advertise<geometry_msgs::Wrench>("/bluerov2/thruster_manager/input",20);

        // topic
        linear_control_pub = nh.advertise<geometry_msgs::Vector3>("/bluerov2/control/linear", 20);
        angular_control_pub = nh.advertise<geometry_msgs::Vector3>("/bluerov2/control/angular", 20);
        eta_pub = nh.advertise<geometry_msgs::Twist>("/bluerov2/std_eta", 20);
        cylinder_pub = nh.advertise<geometry_msgs::Vector3>("/bluerov2/cylinder_h", 20);
        sphere_pub = nh.advertise<geometry_msgs::Vector3>("/bluerov2/sphere_h", 20);

        d_hat_msg.linear = d_hat_msg.angular = zero_vec;
        d_hat_pub = nh.advertise<geometry_msgs::Twist>("/bluerov2/d_hat",20);
        boost::shared_ptr<nav_msgs::Odometry const> shared_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/bluerov2/pose_gt", nh, ros::Duration(100));
        if (shared_msg != nullptr)
        {
            ROS_INFO("pubisher ready, sarting subscribe");
        }
        else
            exit(1);

        br2_state_sub = nh.subscribe("/bluerov2/pose_gt", 10, &MbMpcCbfDobNode::pose_gt_cb, this);

        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.;
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.scale.z = 0.2;
        marker.color.r = 1;
        marker.color.g = 1;
        marker.color.b = 1;
        marker.color.a = 1;
        freq_marker_pub = nh.advertise<visualization_msgs::Marker>("TEXT_VIEW_FACING", 20);
    }

    void MbMpcCbfDobNode::pose_gt_cb(const nav_msgs::Odometry &msg)
    {
        // update std_eta
        std_eta.linear.x = msg.pose.pose.position.x;
        std_eta.linear.y = msg.pose.pose.position.y;
        std_eta.linear.z = msg.pose.pose.position.z;
        




        tf2::Quaternion tf_quat;
        tf2::convert(msg.pose.pose.orientation, tf_quat);
        tf2::Matrix3x3 quat_m(tf_quat);
        quat_m.getRPY(std_eta.angular.x, std_eta.angular.y, std_eta.angular.z);

        bluerov2_states[EnumStates::x] = msg.pose.pose.position.x;
        bluerov2_states[EnumStates::y] = -msg.pose.pose.position.y;
        bluerov2_states[EnumStates::z] = -msg.pose.pose.position.z;
        bluerov2_states[EnumStates::phi] = std_eta.angular.x;
        bluerov2_states[EnumStates::theta] = -std_eta.angular.y;
        bluerov2_states[EnumStates::psi] = -std_eta.angular.z;
        bluerov2_states[EnumStates::u] = msg.twist.twist.linear.x;
        bluerov2_states[EnumStates::v] = -msg.twist.twist.linear.y;
        bluerov2_states[EnumStates::w] = -msg.twist.twist.linear.z;
        bluerov2_states[EnumStates::p] = msg.twist.twist.angular.x;
        bluerov2_states[EnumStates::q] = -msg.twist.twist.angular.y;
        bluerov2_states[EnumStates::r] = -msg.twist.twist.angular.z;

        // the state is in ned frame
        // std::cout<< "state: "  << bluerov2_states << std::endl;

        if (last_time.is_zero())
            last_time = ros::Time::now();
        else
        {
            double dt = (ros::Time::now() - last_time).toSec();
            last_time = ros::Time::now();
            dob_euler(dt);
        }
    }

    void MbMpcCbfDobNode::control_pub(void)
    {
        eta_pub.publish(std_eta);
        // https://stackoverflow.com/questions/43907601/how-to-copy-data-from-c-array-to-eigen-matrix-or-vector
        Eigen::VectorXd control_input = Eigen::Map<Eigen::VectorXd>(bluerov2_input.data(), bluerov2_input.size());        
        
        // make this  into std frame
        control_input[1] *= -1;
        control_input[2] *= -1;
        control_input[4] *= -1;
        control_input[5] *= -1;
        linear_control.x = control_input[0];
        linear_control.y = control_input[1];
        linear_control.z = control_input[2];
        angular_control.x = control_input[3];
        angular_control.y = control_input[4];
        angular_control.z = control_input[5];
        linear_control_pub.publish(linear_control);
        angular_control_pub.publish(angular_control);
        std::cout << "-------------------------control input in std frame-----------------------------------------------------------------------" << std::endl;
        std::cout << "u0:     " << control_input[0] << "\tu1:     " << control_input[1] << "\tu2:     " << control_input[2] << std::endl;
        std::cout << "u3:        " << control_input[3] << "\tu4:       " << control_input[4] << "\tu5:     " << control_input[5] << std::endl;

        // thruster manager is able to trnasform control input
        // u into the value that thruster needs to give!
        // and it is indeed the right value!
        control_input_std.force.x = control_input[0];
        control_input_std.force.y = control_input[1];
        control_input_std.force.z = control_input[2];
        control_input_std.torque.x = control_input[3];
        control_input_std.torque.y = control_input[4];
        control_input_std.torque.z = control_input[5];
        control_std_pub.publish(control_input_std);

        double h;
        for (int i = 0; i < N_OB; i++)
        {
            if (obstacles[i][0] == 0)
            {
                // it's cylinder
                h = std::pow(obstacles_std[i][1] - std_eta.linear.x, 2) +
                    std::pow(obstacles_std[i][2] - std_eta.linear.y, 2) -
                    std::pow(obstacles_std[i][7], 2);
                if (i == 0)
                    cylinder_h.x = h;
                else if (i == 1)
                    cylinder_h.y = h;
                else
                    cylinder_h.z = h;
            }
            else
            {
                // it's sphere
                h = std::pow(obstacles_std[i][1] - std_eta.linear.x, 2) +
                    std::pow(obstacles_std[i][2] - std_eta.linear.y, 2) +
                    std::pow(obstacles_std[i][3] - std_eta.linear.z, 2) -
                    std::pow(obstacles_std[i][7], 2);
                if (i == 0)
                    sphere_h.x = h;
                else if (i == 1)
                    sphere_h.y = h;
                else
                    sphere_h.z = h;
            }
        }
        cylinder_pub.publish(cylinder_h);
        sphere_pub.publish(sphere_h);

        std::ostringstream str;
        str << "frequency: " << int(1 / cpu_time) << " Hz";
        marker.text = str.str();
        marker.pose.position.x = std_eta.linear.x;
        marker.pose.position.y = std_eta.linear.y;
        marker.pose.position.z = std_eta.linear.z + 0.8;

        freq_marker_pub.publish(marker);

        solve_time.data = cpu_time;
        solve_time_pub.publish(solve_time);
    }

    void MbMpcCbfDobNode::run(void)
    {

        // std::pair<int,int> size_ = target_state_value.size();
        // std::cout<< size_.first <<"," << size_.second <<std::endl;

        // size_ = transpose(bluerov2_states).size();
        // std::cout<<"state' "<< size_.first <<"," << size_.second <<std::endl;

        // 1e7*casadi::DM::ones(horizon_ * N_OB)
        // it's a single loop

        std::vector<double> params = bluerov2_states;
        params.insert(params.end(), target_state_value.begin(), target_state_value.end());
        // std::vector<double> d_hat_vec(d_hat, d_hat + 6);
        params.insert(params.end(),d_hat.begin(),d_hat.end());

        // std::cout << "params::::::::::::::::" << params <<std::endl;
        // vertcat(transpose(casadi::DM(bluerov2_states)),
        //                   transpose(casadi::DM(target_state_value)))

        // {"lbx",  vertcat(casadi::DM(lbu), -1*casadi::DM::inf((horizon_+1)*12,1))  },
        // {"ubx", vertcat(casadi::DM(ubu),-1*casadi::DM::inf((horizon_+1)*12,1))  },
        casadi::DMDict args;
        args = {
            {"lbx", lbu},
            {"ubx", ubu},
            {"lbg", casadi::DM::zeros(n_block * N_OB, 1)},
            {"ubg", casadi::DM::inf(n_block * N_OB, 1)},
            {"p", params}};
        // std::cout << "right***********************************" << std::endl;
        // std::cout << "lbu " << lbu << std::endl;
        std::stringstream buffer;
        // save old cout buffer
        std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

        // store solve results
        std::map<std::string, casadi::DM> result = mpc_cbf_solver(args);
        auto stats = mpc_cbf_solver.stats();
        std::string return_status = stats["return_status"];

        std::cout.rdbuf(old);

        std::string line;
        while (std::getline(buffer, line)) {
            if (line.find("Total seconds in IPOPT") != std::string::npos) {
                size_t pos = line.find("=");
                if (pos != std::string::npos) {
                    std::string time_str = line.substr(pos + 1);
                    cpu_time = std::stod(time_str);
                    break;
                }
            }
        }
        if (return_status != "Solve_Succeeded")
        {
            std::cout << "Optimization failed. Status: " << return_status << std::endl;
            ROS_ERROR("mpc casadi solve failed!!!!!!!!!");

            std::cout << "input initial state:  " << bluerov2_states << std::endl;
            std::cout << "u res:  " << result.at("x") << std::endl;
            std::cout << "constraint res:  " << result.at("g") << std::endl;
            exit(1);
        }


        std::vector<double> res(result.at("x"));
        // get sys input and solve time
        std::copy(res.begin(), res.begin() + N_U, bluerov2_input.begin());
        std::copy(bluerov2_input.begin() , bluerov2_input.end(), last_tau_out.begin());

      

        std::cout << cpu_time << "  time!!!!!!!!!!!!!!!" << std::endl;
        std::cout << "solve once done **********************************************  " << std::endl;
        // publish control input and other msgs
        control_pub();
        std::cout << "pub once done **********************************************  " << std::endl;
    }

    void MbMpcCbfDobNode::dob_euler(double dt)
    {
        Eigen::VectorXd dz(6), 
        z_hat_vec = Eigen::Map<Eigen::VectorXd>(z_hat.data(), 6),
        d_hat_vec = Eigen::Map<Eigen::VectorXd>(d_hat.data(),6);
        
        dz = dob_state_equation(d_hat_vec);


        z_hat_vec = z_hat_vec + dz*dt;
        double *tmp = z_hat_vec.data();
        std::copy(tmp, tmp + N_U, z_hat.begin());
        

        // calculate d_hat
        Eigen::VectorXd p = Eigen::Map<Eigen::VectorXd>(p_data, 6, 1);
        d_hat_vec = z_hat_vec+alpha*p;
        tmp = d_hat_vec.data();
        std::copy(tmp, tmp + N_U, d_hat.begin());
        d_hat_msg.linear.x = d_hat[0];
        d_hat_msg.linear.y = d_hat[1];
        d_hat_msg.linear.z = d_hat[2];
        d_hat_msg.angular.x = d_hat[3];
        d_hat_msg.angular.y = d_hat[4];
        d_hat_msg.angular.z = d_hat[5];
        d_hat_pub.publish(d_hat_msg);
        // std::cout << "***********d-hat***********"<<std::endl;
        // std::cout << d_hat_vec <<std::endl;
    }

    Eigen::VectorXd MbMpcCbfDobNode::dob_state_equation(Eigen::VectorXd d_hat_tmp)
    {
        // Eigen::DiagonalMatrix<double, N_U> M_mat;
        Eigen::Map<Eigen::VectorXd>(M_mat.diagonal().data(), N_U) = Eigen::Map<const Eigen::VectorXd>(M_diag_data, N_U);


        Eigen::MatrixXd L(N_U, N_X);
        Eigen::MatrixXd zero_mat_6 = Eigen::MatrixXd::Zero(N_U, N_U);
        // if want to horcat, need to use toDenseMatrix
        L << zero_mat_6, l_factor*M_mat.toDenseMatrix();

        // correct
        // std::cout << "*****************L************" << std::endl;
        // std::cout << L << std::endl;

        for (int i = 0; i < N_U; i++)
            p_data[i] = M_diag_data[i]*l_factor*bluerov2_states[6 + i];

        // Eigen::VectorXd p = Eigen::Map<Eigen::VectorXd>(p_data, 6, 1);
        // correct
        // std::cout <<"*****************p************"<< std::endl;
        // std::cout << p << std::endl;

        // and we also need bluerov2's state
        /***************we calculate f_cn***************/
        // tf2::Quaternion quat;
        // quat.setRPY(bluerov2_states[3],bluerov2_states[4],bluerov2_states[5]);
        // tf2::Matrix3x3 quat_mat(quat);
        // Eigen::Matrix3d J1;
        // tf2::convert(quat_mat,J1);

        Eigen::AngleAxisd roll_angle(bluerov2_states[EnumStates::phi], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(bluerov2_states[EnumStates::theta], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_angle(bluerov2_states[EnumStates::psi], Eigen::Vector3d::UnitZ());

        Eigen::Matrix3d J1;
        J1 = roll_angle * pitch_angle * yaw_angle;

        // to be verified,correct
        // std::cout << "angles: " << bluerov2_states[3] << "," << bluerov2_states[4] << "," << bluerov2_states[5] << std::endl;

        // std::cout << "*****************J1************" << std::endl;
        // std::cout << J1 << std::endl;

        double sin_phi = std::sin(bluerov2_states[EnumStates::phi]), cos_phi = std::cos(bluerov2_states[EnumStates::phi]),
               cos_theta = std::cos(bluerov2_states[EnumStates::theta]), sin_theta = std::sin(bluerov2_states[EnumStates::theta]);

        Eigen::Matrix3d J2;
        J2 << 1, sin_phi * std::tan(bluerov2_states[EnumStates::theta]), cos_phi * std::tan(bluerov2_states[EnumStates::theta]),
            0, cos_phi, -sin_phi,
            0, sin_phi / cos_theta, cos_phi / cos_theta;

        Eigen::VectorXd g_eta(N_U);
        g_eta << 
            (W - B) * sin_theta,
            -(W - B) * cos_theta * sin_phi,
            -(W - B) * cos_theta * cos_phi,
            zb * B * cos_theta * sin_phi,
            zb * B * sin_theta,
            0;

        Eigen::MatrixXd C_RB(N_U, N_U);
        C_RB << 0, 0, 0, 0, M_params[EnumMParams::mass] * bluerov2_states[EnumStates::w], -M_params[EnumMParams::mass] * bluerov2_states[EnumStates::v],
            0, 0, 0, -M_params[EnumMParams::mass] * bluerov2_states[EnumStates::w], 0, M_params[EnumMParams::mass] * bluerov2_states[EnumStates::u],
            0, 0, 0, M_params[EnumMParams::mass] * bluerov2_states[EnumStates::v], -M_params[EnumMParams::mass] * bluerov2_states[EnumStates::u], 0,
            0, M_params[EnumMParams::mass] * bluerov2_states[EnumStates::w], -M_params[EnumMParams::mass] * bluerov2_states[EnumStates::v], 0, M_params[EnumMParams::Iz] * bluerov2_states[EnumStates::r], -M_params[EnumMParams::Iy] * bluerov2_states[EnumStates::q],
            -M_params[EnumMParams::mass] * bluerov2_states[EnumStates::w], 0, M_params[EnumMParams::mass] * bluerov2_states[EnumStates::u], -M_params[EnumMParams::Iz] * bluerov2_states[EnumStates::r], 0, M_params[EnumMParams::Ix] * bluerov2_states[EnumStates::p],
            M_params[EnumMParams::mass] * bluerov2_states[EnumStates::v], -M_params[EnumMParams::mass] * bluerov2_states[EnumStates::u], 0, M_params[EnumMParams::Iy] * bluerov2_states[EnumStates::q], -M_params[EnumMParams::Ix] * bluerov2_states[EnumStates::p], 0;

        Eigen::MatrixXd C_A(N_U, N_U);
        C_A << 
                0, 0, 0, 0, Zwd * bluerov2_states[EnumStates::w], Yvd * bluerov2_states[EnumStates::v],
                0, 0, 0, -Zwd * bluerov2_states[EnumStates::w], 0, -Xud * bluerov2_states[EnumStates::u],
                0, 0, 0, -Yvd * bluerov2_states[EnumStates::v], Xud * bluerov2_states[EnumStates::u], 0,
                0, -Zwd * bluerov2_states[EnumStates::w], Yvd * bluerov2_states[EnumStates::v], 0, -Nrd * bluerov2_states[EnumStates::r], Mqd * bluerov2_states[EnumStates::q],
                Zwd * bluerov2_states[EnumStates::w], 0, -Xud * bluerov2_states[EnumStates::u], Nrd * bluerov2_states[EnumStates::r], 0, -Kpd * bluerov2_states[EnumStates::p],
                -Yvd * bluerov2_states[EnumStates::v], Xud * bluerov2_states[EnumStates::u], 0, -Mqd * bluerov2_states[EnumStates::q], Kpd * bluerov2_states[EnumStates::p], 0;
        Eigen::DiagonalMatrix<double, N_U> D_mat;
        D_mat.diagonal() << 
        M_params[EnumMParams::Xu] + Xuu * std::fabs(bluerov2_states[EnumStates::u]), 
        M_params[EnumMParams::Yv] + Yvv * std::fabs(bluerov2_states[EnumStates::v]), 
        M_params[EnumMParams::Zw] + Zww * std::fabs(bluerov2_states[EnumStates::w]),
        M_params[EnumMParams::Kp] + Kpp * std::fabs(bluerov2_states[EnumStates::p]),
        M_params[EnumMParams::Mq] + Mqq * std::fabs(bluerov2_states[EnumStates::q]), 
        M_params[EnumMParams::Nr] + Nrr * std::fabs(bluerov2_states[EnumStates::r]);

        // Eigen::VectorXd nu_l = Eigen::Map<Eigen::VectorXd>(bluerov2_states+6,3,1);



        // Eigen::VectorXd nu_a = Eigen::Map<Eigen::VectorXd>(bluerov2_states+9,3,1);
        // Eigen::VectorXd nu_vec = Eigen::Map<Eigen::VectorXd>(bluerov2_states+6,6,1);
        Eigen::Vector3d nu_l = Eigen::Vector3d::Map(&bluerov2_states[6]);
        Eigen::Vector3d nu_a = Eigen::Vector3d::Map(&bluerov2_states[9]);
        Eigen::VectorXd nu_vec = Eigen::VectorXd::Map(&bluerov2_states[6], 6);
        
        // std::cout<<"nu_l: "<<nu_l<<std::endl;
        // if(last_tau_out[0] != acados_out.bluerov_input[0])
        //     std::cout << "control not the same!!!!!!!!!!!!!!!"<<std::endl;
        // else
        //     std::cout << "control same!!!!!!!!!!!!!!!"<<std::endl;
        // Eigen::VectorXd last_tau_vec = Eigen::Map<Eigen::VectorXd>(last_tau_out,N_U,1);

        Eigen::VectorXd last_tau_vec = Eigen::VectorXd::Map(&last_tau_out[0], N_U);

        Eigen::VectorXd dz(N_X);
        dz <<  
        J1 * nu_l,
        J2 * nu_a,
        M_mat.inverse()*(last_tau_vec+d_hat_tmp-g_eta-(C_A+C_RB+D_mat.toDenseMatrix())*nu_vec);
        
        // std::cout << "dz !!!!!!!!!!!!!!!"<<std::endl;
        // std::cout<< -alpha*L*dz <<std::endl;

        // std::cout << "ap!!!!!!!!!!!!!!!!!!"<<std::endl;
        // std::cout<< -alpha*p <<std::endl;

        // this returns dz
        return -alpha*L*dz;
    }
}

int main(int argc, char **argv)
{
    int control_rate = 20;
    ros::init(argc, argv, "mb_mpc_cbf_dob_node");
    ros::NodeHandle nh;
    mb_mpc_cbf_dob::MbMpcCbfDobNode mb_mpc_cbf_dob_node(nh, 1. / control_rate);
    ros::Rate rate(control_rate);
    ROS_INFO_ONCE("**********************mb_mpc_cbf_dob_node_start************************");
    while (ros::ok())
    {
        ros::spinOnce();
        mb_mpc_cbf_dob_node.run();
        rate.sleep();
    }
    return 0;
}