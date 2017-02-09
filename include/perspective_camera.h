#ifndef PERSPECTIVE_CAMERA_H
#define PERSPECTIVE_CAMERA_H

#include <generic_camera.h>

/*
 * This class inherits from GenericCamera and implements the classical perspective camera model
 * Methods to be filled up are:
 * - project
 * - computeJacobianIntrinsic
 * - computeJacobianExtrinsic
 */


namespace covis
{


// xi = (px, py, u0, v0)
class PerspectiveCamera : public GenericCamera
{
public:

    vpFeaturePoint3D p_;
    vpMatrix dPdX_;

    //define the intrinsic paramters of the PerspectiveCamera
    PerspectiveCamera(const double &_px, const double &_py, const double &_u0, const double &_v0, const bool &_calibrated=false)
    {
        xi_.resize(4);
        xi_[0] = _px;
        xi_[1] = _py;
        xi_[2] = _u0;
        xi_[3] = _v0;
    }


    // check that the parameters stay meaningfull in case of wrong update
    void updateIntrinsic(const vpColVector &_dxi)
    {
        // update
        xi_ += _dxi;
        // all parameters should be positive
        for(unsigned int i=0;i<4;++i)
            if(xi_[i] < 0)
                xi_[i] = 0;

    }


    // compute pixel coordinates of a 3D point
    // we assume the point is already projected in the camera frame
    void project(const vpPoint &_P, double &_u, double &_v)
    {

        _u = xi_[0]*((_P.get_X())/(_P.get_Z())) + xi_[2];    // u = px.X/nu + u0
        _v = xi_[1]*((_P.get_Y())/(_P.get_Z())) + xi_[3];    // v = py.Y/nu + v0
    }


    // write the Jacobian corresponding to the intrinsic parameters
    // Jacobian_intrinsic should be 2x4
    // we assume the point is already projected in the camera frame
    // how u and v change when we change fx,fy,u0,v0
    void computeJacobianIntrinsic(const vpPoint &_P, vpMatrix &_J)
    {
        _J.resize(2,4);
        _J[0][0] = (_P.get_X())/(_P.get_Z());                                   // du/dpx
        _J[1][1] = (_P.get_Y())/(_P.get_Z());                                   // dv/dpy
        _J[0][2] = _J[1][3] = 1;                                                // du/du0, dv/dv0
        _J[0][3] =_J[1][0]=_J[0][1]=_J[1][2]=0;
    }


    // write the Jacobian wrt extrinsic parameters
    // J_extrinsic should be 2x6
    // we assume the point is already projected in the camera frame
    // how u and v change when we change pose of camera.
    void computeJacobianExtrinsic(const vpPoint &_P, vpMatrix &_J)
    {
        _J.resize(2,6);
        _J[0][0] = -xi_[0]/_P.get_Z();                                                   // -px/Z
        _J[1][1] = -xi_[1]/_P.get_Z();                                                   // -py/Z
        _J[1][0] = _J[0][1] = 0;
        _J[0][2] =  xi_[0]*((_P.get_X())/(_P.get_Z()*_P.get_Z()));                       // px.x/Z
        _J[0][3] =  xi_[0]*((_P.get_X()*_P.get_Y())/(_P.get_Z()*_P.get_Z()));            // -px.xy
        _J[0][4] =  -xi_[0]*(1+((_P.get_X()*_P.get_X())/(_P.get_Z()*_P.get_Z())));       //-px(1+x*x)
        _J[0][5] =  xi_[0]*((_P.get_Y())/(_P.get_Z()));                                  //px.y
        _J[1][2] =  xi_[1]*((_P.get_Y())/(_P.get_Z()*_P.get_Z()));                       //-py.y/Z
        _J[1][3] =  xi_[1]*(1+((_P.get_Y()*_P.get_Y())/(_P.get_Z()*_P.get_Z())));        //py(1+y*y)
        _J[1][4] =  -xi_[1]*((_P.get_X()*_P.get_Y())/(_P.get_Z()*_P.get_Z()));           //-py.xy
        _J[1][5] =  -xi_[1]*((_P.get_X())/(_P.get_Z()));                                 //-py.x

    }
};
}


#endif // PERSPECTIVE_CAMERA_H
