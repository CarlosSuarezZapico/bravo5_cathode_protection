/**
 *    @file  utils.cpp
 *    @brief UDP interface for bravo in cpp
 *
 *    @author  Carlos Suarez Zapico, carlossuarezzapico@gmail.com
 *
 *    @internal
 *    Project UNITE
 *    Created  17-Nov-2023
 *    Modification 24-Oct-2023
 *    Revision  ---
 *    Company  Heriot-Watt University / National Robotarium
 * ====
 */
#include "bravo_compliance/bravo_cpp/utils/utils.h"

namespace bravo_utils{

    void GetEulerAngles(Eigen::Quaterniond q, double& yaw, double& pitch, double& roll){
        const double w2 = q.w()*q.w();
        const double x2 = q.x()*q.x();
        const double y2 = q.y()*q.y();
        const double z2 = q.z()*q.z();
        const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
        const double abcd = q.w()*q.x() + q.y()*q.z();
        const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
        const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
        if (abcd > (0.5-eps)*unitLength)
        {
            yaw = 2 * std::atan2(q.y(), q.w());
            pitch = pi;
            roll = 0;
        }
        else if (abcd < (-0.5+eps)*unitLength)
        {
            yaw = -2 * std::atan2(q.y(), q.w());
            pitch = -pi;
            roll = 0;
        }
        else
        {
            const double adbc = q.w()*q.z() - q.x()*q.y();
            const double acbd = q.w()*q.y() - q.x()*q.z();
            yaw = std::atan2(2*adbc, 1 - 2*(z2+x2));
            pitch = std::asin(2*abcd/unitLength);
            roll = std::atan2(2*acbd, 1 - 2*(y2+x2));
        }
    }
}