#ifndef visp_TYPES_HPP
#define visp_TYPES_HPP

#include "base/samples/RigidBodyState.hpp"
/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace visp 
{
    /* List of the three availables visual servoing architecures
     * IBVS: Image-based visual servoing
     * PBVS: Position-based visual servoing
     * HBS: Hybrid Visual Servoing (2 1/2D Visual Servoing)
     */
    enum architecture_t {IBVS, PBVS, HVS};

    struct architecture
    {
        architecture_t mode;
    };

    struct pose
    {
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;
    };

    struct adaptiveGains
    {
        float gain_at_zero;
        float gain_at_infinity;
        float slope_at_zero;
    };

    /** Group relevant information about the controller
     *
     *  error        - the norm-2 of the error vector
     *  residual     - value expressed in meter that represents the
                       "quality" of the measurement"
     *  desired_pose - the current setpoint
     *  current_pose - the current pose
     *  timestamp    - time when the last visual feature was received
     */
    struct controllerState
    {
        double error;
        double residual;
        base::samples::RigidBodyState desired_pose;
        base::samples::RigidBodyState current_pose;
        base::Time timestamp;
    };
    
    struct expectedInputs
    {
        bool linear[3];
        bool angular[3];
    };

    /** Saturation values for the controller
     *  represents the maximum absolut output value
     */
    struct saturationValues
    {
        base::Vector3d linear;
        base::Vector3d angular;
    };

}

#endif
