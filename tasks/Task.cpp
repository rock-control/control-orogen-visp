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

    // Initialize camera parameters.
    // The library desconsider the tangencial values, that's why only the
    // radial values are informed. 
    frame_helper::CameraCalibration cal = _camera_parameters.get();
    cam_cal = vpCameraParameters(cal.fx, cal.fy,cal.cx,cal.cy);

    // Set the position of the square target in a frame which origin is
    // centered in the middle of the square
    double L = _marker_size.get()/2; // 2L is the square size
    point[0].setWorldCoordinates(-L,  L, 0);
    point[1].setWorldCoordinates( L,  L, 0);
    point[2].setWorldCoordinates( L, -L, 0);
    point[3].setWorldCoordinates(-L, -L, 0);

    //Center of the target (reference point)
    P.setWorldCoordinates(0, 0, 0);

    // Define the task
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
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
    eJe = getJacobianFromExpectedInputs(expected_inputs);
    task.set_eJe(eJe);

    // Set the pose transformation between the camera and the robot frame
    Eigen::Affine3d body2camera;
    _body2camera.get(base::Time::now(), body2camera);

    //Computes the Velocity Twist Matrix from the spatial transformation between the camera and the robot
    //translation
    vpTranslationVector cto(body2camera.translation()[0],
            body2camera.translation()[1],
            body2camera.translation()[2]);
    //rotation
    base::Quaterniond q(body2camera.rotation());
    vpRotationMatrix cRo(vpQuaternionVector(q.x(), q.y(), q.z(), q.w()));

    //build the homogeneous matrix
    cMe.buildFrom(cto, cRo); 
    //build and set the velocity twist matrix
    cVe.buildFrom(cMe);
    task.set_cVe(cVe);

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
    std::vector<base::Vector2d> corners;
    base::LinearAngular6DCommand setpoint;
    bool no_corners = (_marker_corners.read(corners) == RTT::NoData);
    bool no_setpoint = (_cmd_in.read(setpoint) == RTT::NoData);
    ctrl_state.timestamp = base::Time::now();

    //set the state CONTROLLING only if there is setpoint and detected corners
    if (no_corners) 
    {
        state(WAITING_CORNERS);
    }
    else if (no_setpoint)
    {
        state(WAITING_SETPOINT);
    }
    else if (state() != CONTROLLING)
    {
        state(CONTROLLING);
    }

    if (state() == CONTROLLING)
    {
        // transforms the setpoint in resepect with the body
        // to the visp convenion, camera frame.
        setpoint = transformInput(setpoint);

        // Sets the desired position of the visual feature (reference point)
        updateDesiredPose(setpoint);
        updateFeatures(corners);
   
        // Set the gain
        setGain();

        // Compute the visual servoing skew vector
        vpColVector v;
        task.set_cVe(cVe);
        task.print();
        v = task.computeControlLaw() ;

        // Compute the norm-2 of the error vector
        ctrl_state.error = vpMath::sqr((task.getError()).sumSquare());

        writeVelocities(v);
        _controller_state.write(ctrl_state);
    }
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

void Task::updateFeatures(std::vector<base::Vector2d> corners)
{
    //  Compute the initial pose
    pose.clearPoint();

    for (int i=0 ; i < 4 ; i++)
    {
        base::Vector2d aux;
        aux = corners[i];
        //vpImagePoint is instanciated using (v,u) instead of (u,v)
        vpImagePoint a(aux[1],aux[0]);
        double x,y;
        vpPixelMeterConversion::convertPoint(cam_cal,a,x,y);
        point[i].set_x(x);
        point[i].set_y(y);
        pose.addPoint(point[i]) ;
    }

    pose.computePose(vpPose::LAGRANGE, cMo);
    pose.computePose(vpPose::VIRTUAL_VS, cMo);
    //compute the "quality" of the result
    ctrl_state.residual = pose.computeResidual(cMo);

    // Update Feature 1 - p
    // Sets the current position of the visual feature
    P.track(cMo);
    ctrl_state.current_pose = convertToRbs(cMo);
    vpFeatureBuilder::create(p, P);

    //////////////////////////////
    // Update Feature 2 - depth //
    //////////////////////////////

    //Takes the features, calculate the intersection of the diagonals of the square and
    //given the camera parameters, retrieve the coordinates in meters.
    vpImagePoint refP;
    //double x,y;
    vpImagePoint cog[4];

    for (int i=0; i< 4; i++)
    {
        base::Vector2d aux;
        aux = corners[i];
        //according to vpImagePoint documentation (i,j) is equivalent to (v,u)
        cog[i].set_ij(aux[1], aux[0]);
    }

    // Compute the coordinates of the 2D reference point from the
    // intersection of the diagonals of the square
    double a1 = (cog[0].get_i()- cog[2].get_i()) /
        (cog[0].get_j()- cog[2].get_j());
    double a2 = (cog[1].get_i()- cog[3].get_i()) /
        (cog[1].get_j()- cog[3].get_j());
    double b1 = cog[0].get_i() - a1*cog[0].get_j();
    double b2 = cog[1].get_i() - a2*cog[1].get_j();

    refP.set_j( (b1-b2) / (a2-a1) );
    refP.set_i( a1*refP.get_j() + b1 );
    double x,y;
    vpPixelMeterConversion::convertPoint(cam_cal, refP, x, y);

    // Update the 2D ref point visual feature
    double Z;
    Z = P.get_Z();
    p.set_xyZ(x, y, Z);
    depth.buildFrom(P.get_x(), P.get_y(), Z, log(Z/Zd));

    //////////////////////////////
    // Update Feature 3 - t, tu //
    //////////////////////////////
    cdMc = cdMo*cMo.inverse();
    t.buildFrom(cdMc);
    tu.buildFrom(cdMc) ;

}

bool Task::updateDesiredPose(base::LinearAngular6DCommand setpoint)
{
    vpTranslationVector cdto(setpoint.linear[0], setpoint.linear[1], setpoint.linear[2]);
    vpRxyzVector cdro(setpoint.angular[0], setpoint.angular[1], setpoint.angular[2]);

    //Build the rotation and homogeneus matrix according to desired pose
    vpRotationMatrix cdRo(cdro); // Build the rotation matrix
    cdMo.buildFrom(cdto, cdRo); // Build the homogeneous matrix
    ctrl_state.desired_pose = convertToRbs(cdMo);

    P.track(cdMo);
    vpFeatureBuilder::create(pd, P);
    Zd = P.get_Z();

    return true;
}

void Task::setGain()
{
    if (_use_adaptive_gain.get())
    {
        vpAdaptiveGain  lambda;
        lambda.initStandard(_adaptive_gains.get().gain_at_zero,
                _adaptive_gains.get().gain_at_infinity,
                _adaptive_gains.get().slope_at_zero);

        task.setLambda(lambda) ;
    }
    else
    {
        task.setLambda(_gain.get());
    }
}

void Task::writeVelocities(vpColVector v)
{
    base::LinearAngular6DCommand vel;

    for (int i=0; i < 3; ++i)
    {
        // write NaN on the non-desirable outputs in order to 
        // keep the consistence with the other controllers 
        // linear
        if (!expected_inputs.linear[i]) {v[i] = base::NaN<double>();}
        vel.linear[i] = v[i];
        //angular
        if (!expected_inputs.angular[i]) {v[i+3] = base::NaN<double>();}
        vel.angular[i] = v[i+3];
    }

    vel.time = ctrl_state.timestamp;
    _cmd_out.write(vel);
}

base::samples::RigidBodyState Task::convertToRbs(vpHomogeneousMatrix pose)
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

base::LinearAngular6DCommand Task::transformInput(base::LinearAngular6DCommand cmd_in_body)
{
     //initialize setpoint on the body frame
     vpColVector body_sp(6);
     for (int i = 0; i < 3; ++i)
     {
         //if there is not expected input, set it to zero, so the 
         //visp do not crashs when try to build cdMo matrix
         if (!expected_inputs.linear[i]) {cmd_in_body.linear[i] = 0;}
         body_sp[i] = cmd_in_body.linear[i];
         if (!expected_inputs.angular[i]) {cmd_in_body.angular[i] = 3.1415;}
         body_sp[i+3] = cmd_in_body.angular[i];
     }

     // c_sp = cVb * b_sp (here e = body)
     vpColVector camera_sp(6);
     camera_sp = cVe*body_sp;

     //convert the vpColVector to LinearAngular6DCommand
     base::LinearAngular6DCommand cmd_in_camera;
     for (int i = 0; i < 3; ++i)
     {
         cmd_in_camera.linear[i] = camera_sp[i];
         cmd_in_camera.angular[i] = camera_sp[i+3];
     }

    return cmd_in_camera;
}

vpMatrix Task::getJacobianFromExpectedInputs(visp::expectedInputs expected_inputs)
{
    vpMatrix eJe;
    eJe.eye(6);
    
    for(int i = 0; i < 3; ++i)
    {
        if (!expected_inputs.linear[i])  {eJe[i][i] = 0;} 
        if (!expected_inputs.angular[i]) {eJe[i+3][i+3] = 0;}
    }

    return eJe;
}
