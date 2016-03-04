#ifndef visp_TYPES_HPP
#define visp_TYPES_HPP

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
}

#endif
