/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */


#include "Task.hpp"
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpFeatureBuilder.h>

using namespace visp;

Task::Task(std::string const& name)
    : TaskBase(name), tu(vpFeatureThetaU::cdRc), t(vpFeatureTranslation::cdMc)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine), tu(vpFeatureThetaU::cdRc), t(vpFeatureTranslation::cdMc)
{
}

Task::~Task()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    //Set the target_list and the desired target
    target_list = _target_list.get();
    desired_target = _desired_target.get();

    // Initialize camera parameters.
    frame_helper::CameraCalibration cal = _camera_parameters.get();
    double scaling = _scaling.get();
    cal.fx = scaling * cal.fx;
    cal.fy = scaling * cal.fy;
    cal.cx = scaling * cal.cx;
    cal.cy = scaling * cal.cy;
    cal.height = scaling * cal.height;
    cal.width = scaling * cal.width;
    // The library desconsider the tangencial values, that's why only the
    // radial values are informed.
    cam_cal = vpCameraParameters(cal.fx, cal.fy,cal.cx,cal.cy);

    //Center of the target (reference point)
    P.setWorldCoordinates(0, 0, 0);

    // Define the task
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);

    switch(_architecture.get().mode)
    {
        case IBVS:
            task.addFeature(p,pd) ;
            break;
        case PBVS:
            task.addFeature(t);
            task.addFeature(tu);
            break;
        case HVS:
            task.addFeature(p,pd) ;
            task.addFeature(depth) ;
            task.addFeature(tu) ;
    }

    expected_inputs = _expected_inputs.get();
    // Set the Jacobian (expressed in the end-effector frame)
    // it depends of the desired control dof
    eJe = jacobianFromExpectedInputs(expected_inputs);
    task.set_eJe(eJe);

    // Set the pose transformation between the camera and the robot frame
    Eigen::Affine3d body2camera;
    _body2camera.get(base::Time::now(), body2camera);

    // Computes the Velocity Twist Matrix from the spatial
    // transformation between the camera and the robot
    vpTranslationVector ctb(body2camera.translation()[0],
                            body2camera.translation()[1],
                            body2camera.translation()[2]);
    base::Quaterniond q(body2camera.rotation());
    vpRotationMatrix cRb(vpQuaternionVector(q.x(), q.y(), q.z(), q.w()));

    //build the homogeneous matrix of the body frame wrt the camera
    cMb.buildFrom(ctb, cRb);
    //build and set the velocity twist matrix
    cVb.buildFrom(cMb);
    task.set_cVe(cVb);

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    //read input ports
    Points corners;

    //set the state CONTROLLING only if there is setpoint and detected corners
    if (!readCorners(corners, desired_target))
    {
        if (state() != WAITING_CORNERS)
            state(WAITING_CORNERS);
    }
    else if (!readSetpoint(setpoint))
    {
        if (state() != WAITING_SETPOINT)
            state(WAITING_SETPOINT);
    }
    else if (state() != CONTROLLING)
    {
        state(CONTROLLING);
    }

    if (state() != CONTROLLING)
    {
        // write a vector of zeros in the output port
        // if the state is not controlling
        vpColVector v(6,0);
        writeVelocities(v);
        return;
    }

    // transforms the setpoint from the body frame to the camera frame and
    // build the Homogeneus Matrix of the desired object pose in the camera
    // frame
    vpHomogeneousMatrix cdMo = updateDesiredPose(setpoint);

    // update the target attributes according to the desired target and update
    // the visual features given the detected corners and the desired pose.
    updateFeaturesHVS(corners, cdMo);

    // update eJe (required for each loop)
    task.set_eJe(eJe);

    // Set the gain
    setGain(_gain.get(), _adaptive_gains.get(), _use_adaptive_gain.get());

    // Compute the visual servoing skew vector
    vpColVector v;
    task.set_cVe(cVb);
    v = task.computeControlLaw() ;

    // Compute the norm-2 of the error vector
    ctrl_state.error = vpMath::sqr((task.getError()).sumSquare());

    writeVelocities(v);
    _controller_state.write(ctrl_state);
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    task.kill();
}

void Task::updateFeaturesHVS(Points const &corners, vpHomogeneousMatrix const &cdMo)
{
    //  Assign the corners to the vpPose
    vpPose pose;
    for (int i=0 ; i < 4 ; i++)
    {
        double x,y;
        //vpImagePoint is instanciated using (v,u) instead of (u,v)
        vpImagePoint a(corners.points[i][1], corners.points[i][0]);
        vpPixelMeterConversion::convertPoint(cam_cal,a,x,y);
        point[i].set_x(x);
        point[i].set_y(y);
        pose.addPoint(point[i]) ;
    }

    //compute the pose of the object in the camera frame
    vpHomogeneousMatrix cMo;
    pose.computePose(vpPose::LAGRANGE, cMo);
    pose.computePose(vpPose::VIRTUAL_VS, cMo);
    //compute the "quality" of the result
    ctrl_state.residual = pose.computeResidual(cMo);

    // apply a rotation around the z on the camera frame
    vpHomogeneousMatrix rot(0, 0, 0, 0, 0, vpMath::rad(rotation_around_z));
    cMo = cMo * rot;

    /**
     *  Update Feature 1 - p
     */
    // Sets the current position of the visual feature
    P.track(cMo);

    ctrl_state.current_pose = convertToRbs(cMo);
    vpFeatureBuilder::create(p, P);

    /**
     *  Update Feature 2 - depth
     */

    //Takes the features, calculate the intersection of the diagonals of the square and
    //given the camera parameters, retrieve the coordinates in meters.
    vpImagePoint refP;
    vpImagePoint cog[4];

    for (int i=0; i< 4; i++)
    {
        //according to vpImagePoint documentation (i,j) is equivalent to (v,u)
        cog[i].set_ij(corners.points[i][1], corners.points[i][0]);
    }

    // Compute the coordinates of the 2D reference point from the
    // intersection of the diagonals of the square
    double a1 = (cog[0].get_i() - cog[2].get_i()) /
                (cog[0].get_j() - cog[2].get_j());
    double a2 = (cog[1].get_i() - cog[3].get_i()) /
                (cog[1].get_j() - cog[3].get_j());
    double b1 = cog[0].get_i() - a1*cog[0].get_j();
    double b2 = cog[1].get_i() - a2*cog[1].get_j();

    refP.set_j((b1-b2) / (a2-a1));
    refP.set_i(a1*refP.get_j() + b1);
    double x,y;
    vpPixelMeterConversion::convertPoint(cam_cal, refP, x, y);

    // Update the 2D ref point visual feature
    double Z;
    Z = P.get_Z();
    p.set_xyZ(x, y, Z);
    depth.buildFrom(P.get_x(), P.get_y(), Z, log(Z/Zd));

    /**
     *  Update Feature 3 - t, tu
     */
    vpHomogeneousMatrix cdMc = cdMo*cMo.inverse();
    t.buildFrom(cdMc);
    tu.buildFrom(cdMc);
}

vpHomogeneousMatrix Task::updateDesiredPose(base::LinearAngular6DCommand setpoint)
{
    // Set the input command to their default values when they are not expected.
    // This is required to create a valid vpHomogeneousMatrix, since non-expected
    // inputs are NaN.
    // Given that the object frame's z-axis points to the oposite side of the body
    // frame's x-axis. The object's x and y axis are in such way that in order
    // to have a vehicle stable in "d" meters in front of the object, the desired
    // posistion has to be: {x=d, y=0, z=0, roll=-pi/2, pitch=-pi/2, yaw=0}.
    static const double arr[6] = {0, 0, 0, -M_PI_2, -M_PI_2, 0};
    std::vector<double> default_linear_values(&arr[0], &arr[0]+3);
    std::vector<double> default_angular_values(&arr[3], &arr[3]+3);

    for (int i = 0; i < 3; ++i)
    {
        if (!expected_inputs.linear[i])
            setpoint.linear[i] = default_linear_values[i];

        if (!expected_inputs.angular[i])
            setpoint.angular[i] = default_angular_values[i];
    }

    //create a vpHomogeneousMatrix of the object desired position wrt body (bdMo)
    vpTranslationVector bdto(setpoint.linear[0],
                             setpoint.linear[1],
                             setpoint.linear[2]);
    vpRotationMatrix bdRo(vpRxyzVector(setpoint.angular[0],
                                       setpoint.angular[1],
                                       setpoint.angular[2]));
    vpHomogeneousMatrix bdMo(bdto, bdRo);

    //bdMo is the desired pose of the object in the body frame
    //cdMo is the desired pose of the object in the camera frame
    //cMb is the transformation matrix of the body in the camera frame
    vpHomogeneousMatrix cdMo = cMb * bdMo;
    ctrl_state.desired_pose = convertToRbs(cdMo);

    //update the desired features Zd and pd
    P.track(cdMo);
    vpFeatureBuilder::create(pd, P);
    Zd = P.get_Z();

    return cdMo;
}

void Task::setGain(double gain, visp::adaptiveGains const &adaptive_gains, bool use_adaptive)
{
    if (!use_adaptive)
    {
        task.setLambda(gain);
    }
    else
    {
        vpAdaptiveGain lambda;
        lambda.initStandard(adaptive_gains.gain_at_zero,
                            adaptive_gains.gain_at_infinity,
                            adaptive_gains.slope_at_zero);
        task.setLambda(lambda);
    }
}

void Task::writeVelocities(vpColVector const &v)
{
    base::LinearAngular6DCommand vel;
    visp::saturationValues sat = _saturation.get();

    for (int i=0; i < 3; ++i)
    {
        //check min and max saturation values
        vel.linear[i] = std::min(std::max(v[i], sat.linear_min[i]), sat.linear_max[i]);
        vel.angular[i] = std::min(std::max(v[i+3], sat.angular_min[i]), sat.angular_max[i]);

        // write NaN on the non-desirable outputs in order to
        // keep the consistence with the other controllers
        if (!expected_inputs.linear[i])
            vel.linear[i] = base::NaN<double>();

        if (!expected_inputs.angular[i])
            vel.angular[i] = base::NaN<double>();
    }

    vel.time = ctrl_state.timestamp;
    _cmd_out.write(vel);
}

base::samples::RigidBodyState Task::convertToRbs(vpHomogeneousMatrix pose) const
{
    base::samples::RigidBodyState rbs;

    rbs.position[0] = pose.getTranslationVector()[0];
    rbs.position[1] = pose.getTranslationVector()[1];
    rbs.position[2] = pose.getTranslationVector()[2];

    vpRotationMatrix R;
    pose.extract(R);
    vpQuaternionVector q(R);
    rbs.orientation = base::Orientation(q.w(), q.x(), q.y(), q.z());

    return rbs;
}

vpMatrix Task::jacobianFromExpectedInputs(visp::expectedInputs const &expected_inputs) const
{
    vpMatrix eJe;
    eJe.eye(6);

    for(int i = 0; i < 3; ++i)
    {
        if (!expected_inputs.linear[i])
            eJe[i][i] = 0;

        if (!expected_inputs.angular[i])
            eJe[i+3][i+3] = 0;
    }

    return eJe;
}

bool Task::readSetpoint(base::LinearAngular6DCommand& setpoint)
{
    if (_cmd_in.read(setpoint) == RTT::NoData)
        return false;

    return true;
}

bool Task::readCorners(Points &corners, std::string const &desired_target)
{
    PointsVector corners_vector;
    if (_marker_corners.read(corners_vector) != RTT::NewData)
        return false;

    for (int i = 0; i < corners_vector.size(); ++i)
    {
        if (corners_vector[i].identifier == desired_target)
        {
            // update desired target parameters parameters.
            // It will throw an exeption if the desired target
            // is not in the target list.
            updateTargetParameters(desired_target);

            // return the corners of the desired target
            corners = corners_vector[i];
            ctrl_state.timestamp = corners_vector[i].time;
            return true;
        }
    }

    // Return false if the corners_vector is zero or the desired target
    // is not in the corners vector (not detected).
    return false;
}

void Task::updateTargetParameters(std::string const &desired_target)
{
    //sweep the target list until match the target identifier
    for (int i = 0; i < target_list.size(); ++i)
    {
        if(target_list[i].identifier == desired_target)
        {
            //update the size of the object
            setObjectSize(target_list[i].width, target_list[i].height);
            //update the rotation
            rotation_around_z = target_list[i].rotation_around_z;
            return;
        }
    }

    throw std::invalid_argument("There is no target identifier"
                                "in the informed target list");
}

void Task::setObjectSize(double object_width, double object_height)
{
    double w = object_width/2;
    double h = object_height/2;

    point[0].setWorldCoordinates(-w,  h, 0);
    point[1].setWorldCoordinates( w,  h, 0);
    point[2].setWorldCoordinates( w, -h, 0);
    point[3].setWorldCoordinates(-w, -h, 0);
}

bool Task::setDesired_target(std::string const &value)
{
    this->desired_target = value;
    return true;
}
