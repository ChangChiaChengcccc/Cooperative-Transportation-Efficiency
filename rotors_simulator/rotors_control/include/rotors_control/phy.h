#pragma once
#include <Eigen/Eigen>
class PhysicsParameters
{
public:
        double x_q1 = 0.6; 
        double y_q1 = 0;
        double z_q1 = 0.35;

        double x_q2 = -0.6;
        double y_q2 = 0;
        double z_q2 = 0.35;

        double x_p = 0;
        double y_p = 0;
        double z_p = 0.125;

        double x_ft_sensor1 = 0.6;
        double y_ft_sensor1 = 0;
        double z_ft_sensor1 = 0.130;

        double x_ft_sensor2 = -0.6;
        double y_ft_sensor2 = 0;
        double z_ft_sensor2 = 0.130;

        double Ixx_q = 0.03;
        double Iyy_q = 0.05;
        double Izz_q = 0.1;

        double Ixx_p = 0.0031;
        double Iyy_p = 0.0656;
        double Izz_p = 0.0656;

        double Ixx_ft_sensor1 = 0.00000841666;
        double Iyy_ft_sensor1 = 0.00000841666;
        double Izz_ft_sensor1 = 0.00001666666;

        double Ixx_ft_sensor2 = 0.00000841666;
        double Iyy_ft_sensor2 = 0.00000841666;
        double Izz_ft_sensor2 = 0.00001666666;

        double m_q1 = 1.55;
        double m_q2 = 1.55;
        double m_p = 0.3;
        double m_ft_sensor1 = 0.01;
        double m_ft_sensor2 = 0.01;
        double m_system = m_q1 + m_q2 + m_p + m_ft_sensor1 + m_ft_sensor2;
        
        double x_com = (x_q1*m_q1 + x_q2*m_q2 + x_p *m_p + m_ft_sensor1*x_ft_sensor1 + m_ft_sensor2*x_ft_sensor2) / m_system;
        double y_com = (y_q1*m_q1 + y_q2*m_q2 + y_p *m_p + m_ft_sensor1*y_ft_sensor1 + m_ft_sensor2*y_ft_sensor2) / m_system;
        double z_com = (z_q1*m_q1 + z_q2*m_q2 + z_p *m_p + m_ft_sensor1*z_ft_sensor1 + m_ft_sensor2*z_ft_sensor2) / m_system;
        double z_offset = z_com - z_p;
        double Ixx_system = (Ixx_q + m_q1*(pow((z_q1 - z_com),2) +pow((y_q1 - y_com),2))) + (Ixx_q + m_q2*(pow((z_q2 - z_com),2) + pow((y_q2 - y_com),2))) + (Ixx_p + m_p*(pow((z_p - z_com),2) + pow((y_p - y_com),2)));
        double Iyy_system = (Iyy_q + m_q1*(pow((z_q1 - z_com),2) +pow((x_q1 - x_com),2))) + (Iyy_q + m_q2*(pow((z_q2 - z_com),2) + pow((x_q2 - x_com),2))) + (Iyy_p + m_p*(pow((x_p - x_com),2) + pow((z_p - z_com),2)));
        double Izz_system = (Izz_q + m_q1*(pow((y_q1 - y_com),2) +pow((x_q1 - x_com),2))) + (Izz_q + m_q2*(pow((y_q2 - y_com),2) + pow((x_q2 - x_com),2))) + (Izz_p + m_p*(pow((x_p - x_com),2) + pow((y_p - y_com),2)));
        Eigen::Matrix3d I_system;             
};
