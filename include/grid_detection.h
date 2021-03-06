#ifndef GRID_DETECTION_H
#define GRID_DETECTION_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>

namespace covis
{
struct dot
{
    double x_, y_, a_;
    dot(double _x, double _y, double _a)
    {
        x_ = _x;
        y_ = _y;
        a_ = _a;
    }
    double dist(const double &_x, const double &_y, const double &_a)
    {
        //cout << "area ratio: " << a_/_a << endl;
        if(a_/_a < 0.5)
            return -1;
        else if(a_/_a > 1.1)
            return 0;

        return (_x-x_)*(_x-x_) + (_y-y_)*(_y-y_) + 0*(_a-a_)*(_a-a_);
    }
};

// This function takes an image and returns the position (cog) of the 36 ellipses of interest
bool findDots(cv::Mat _im, std::vector<cv::Point> &_cog)
{
    // to grey level + blur
    cv::Mat img;
    cv::cvtColor(_im, img, cv::COLOR_BGR2GRAY);

    // cv::GaussianBlur(img, img, Size(7, 7), 1, 1);
    cv::Mat imth;
    cv::Canny(img, imth, 20, 80, 3);

    // contours
    std::vector<std::vector<cv::Point> > contours_tmp, contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(imth, contours_tmp, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    // hierarchy to detect doublons
    unsigned int cnt = 0;
    std::vector<dot> dots;
    cv::Moments mu;
    cv::RotatedRect rect;
    unsigned int i;

    for (i=0; i<contours_tmp.size(); i++)
        if (hierarchy[i][3] >= 0)   //has parent, inner (hole) contour of a closed edge (looks good)
        {
            mu = moments(contours_tmp[i], false);
            rect = cv::minAreaRect(contours_tmp[i]);
            if(mu.m00 > 0 && std::abs(mu.m00*4/(M_PI*rect.size.area())-1) < 0.1)
            {
                dots.push_back(dot(mu.m10/mu.m00, mu.m01/mu.m00, mu.m00));
                contours.push_back(contours_tmp[i]);
                cnt++;
            }
        }

    // 36 largest for first guess if enough detected
    if(contours.size()<36)
        return true;

    std::vector<unsigned int> candidates, others;
    unsigned int idx_small = 0;
    for(i=0;i<contours.size();++i)
    {
        if(i < 36)
        {
            candidates.push_back(i);
            if(dots[i].a_ < dots[idx_small].a_)
                idx_small = i;
        }
        else
        {
            if(dots[i].a_ > dots[idx_small].a_)
            {
                others.push_back(idx_small);
                *(std::find(candidates.begin(), candidates.end(),idx_small)) = i;
                idx_small = i;
                for(unsigned int j=0;j<candidates.size();++j)
                {
                    if(dots[candidates[j]].a_ < dots[idx_small].a_)
                        idx_small = candidates[j];
                }
            }
        }
    }

    if(contours.size() > 36)   // too many contours, have to check nearest ones
    {
        bool update = true;
        while(update)
        {
            update = false;
            // mean of current candidates
            double x=0, y=0, a=0;
            for(i=0;i<36;++i)
            {
                x += dots[candidates[i]].x_;
                y += dots[candidates[i]].y_;
                a += dots[candidates[i]].a_;
            }
            x *= 1./36;
            y *= 1./36;
            a *= 1./36;
            // max candidate distance
            double d = 0, d_max = 0;
            unsigned int idx_max;
            for(i=0;i<36;++i)
            {
                d = dots[candidates[i]].dist(x,y,a);
               if(d > d_max && d != -1)
                {
                    d_max = d;
                    idx_max = i;
                }
            }
            // min other distance
            unsigned int idx_min;
            for(i=0;i<others.size();++i)
            {
                d = dots[others[i]].dist(x,y,a);
                if(d < d_max && d != -1)
                {
                    d_max = d;
                    idx_min = i;
                    update = true;
                }
            }


            // switch
            if(update)
            {
                i = candidates[idx_max];
                candidates[idx_max] = others[idx_min];
                others[idx_min] = i;
            }
        }
    }

    // keep only candidates
    _cog.clear();
    for(i=0;i<36;++i)
        _cog.push_back(cv::Point(int(dots[candidates[i]].x_), int(dots[candidates[i]].y_)));

    return false;
}

// utility function
float euclideanDist(const cv::Point& _p, const cv::Point& _q) {
    cv::Point diff = _p - _q;
    return sqrt(diff.x*diff.x + diff.y*diff.y);
}

// utility function to draw current dot sequence
void drawSeq(const std::string &_window, cv::Mat &_im, const std::vector<cv::Point> &_X, std::vector<unsigned int> _idx = std::vector<unsigned int>())
{
    cv::Mat imdraw;
    _im.copyTo(imdraw);
    _idx.push_back(0);

    cv::circle(imdraw, _X[_idx[0]], 2, cv::Scalar(0,255,0),2);

    for(unsigned int i=1;i<_X.size();++i)
    {
        _idx.push_back(i);
        cv::circle(imdraw, _X[_idx[i]], 2, cv::Scalar(0,255,0),2);
        cv::line(imdraw, _X[_idx[i-1]], _X[_idx[i]], cv::Scalar(0,0,255), 2);
    }

    cv::imshow(_window, imdraw);
    //cv::waitKey(0);
}


double angle(const double &x0, const double &y0, const double &norm0, const double &x1, const double &y1)
{
    const double normProd = norm0*sqrt(x1*x1 + y1*y1);

    if(normProd == 0)
        return 2*M_PI;

    return std::atan2((x1*y0-x0*y1), (x1*x0+y1*y0));
}


unsigned int getNextPoint(cv::Point _X0, const unsigned int &_idx, const std::vector<cv::Point> &_X, std::vector<unsigned int> &_spiral, double &_alpha, const bool &_first = false)
{

    const cv::Point X1 = _X[_idx];

    std::vector<double> dist;
    double v;
    std::vector<unsigned int> ngb;
    unsigned int idx_max;

    const unsigned int N = _first?5:5;

    for(unsigned int i=0;i<_X.size();++i)
    {
        if(std::find(_spiral.begin(), _spiral.end(), i) == _spiral.end())
        {
            v = (X1.x - _X[i].x)*(X1.x - _X[i].x) + (X1.y - _X[i].y)*(X1.y - _X[i].y);
            //cout << "   dist to " << i << ": " << v << endl;
            if(dist.size() < N)
            {
                if(!dist.size())
                    idx_max = 0;
                else if(v > dist[idx_max])
                    idx_max = dist.size();
                dist.push_back(v);
                ngb.push_back(i);
                //   //cout << " -> added" << endl;
            }
            else if(v < dist[idx_max])
            {
                // smaller than one already found
                //    //cout << " -> replace " << ngb[idx_max] << endl;
                dist[idx_max] = v;
                ngb[idx_max] = i;
                for(unsigned j=0;j<N;++j)
                {
                    if(dist[j] > dist[idx_max])
                        idx_max = j;
                }
            }
            /* else
                //cout << " -> ignored" << endl;*/
        }
    }

    /*  //cout << "  candidates: ";
    for(unsigned int i=0;i<ngb.size();++i)
        //cout << ngb[i] << ", ";
    //cout << endl;*/

    // ngb = idx of N nearest neighboors

    double alpha;
    const double x = _X0.x-X1.x;
    const double y = _X0.y-X1.y;
    const double nor = sqrt(x*x+y*y);
    unsigned int ind1;
    _alpha = 2*M_PI;
    bool ok;
    for(unsigned int i=0;i<ngb.size();++i)
    {

        if(_first)
        {
            alpha = -angle(x, y, nor, -_X[ngb[i]].x+X1.x, -_X[ngb[i]].y+X1.y);
            ok = alpha > 0;
        }
        else
        {
            alpha = std::abs(angle(x, y, nor, -_X[ngb[i]].x+X1.x, -_X[ngb[i]].y+X1.y));
            ok = true;
        }

        if(ok && (alpha < _alpha || std::abs(alpha-_alpha) < 0.2))
        {
            // check if almost the same
            if(std::abs(alpha-_alpha) < 0.2)
            {
                if(euclideanDist(X1, _X[ind1]) > euclideanDist(X1, _X[ngb[i]]))
                {
                    _alpha = alpha;
                    ind1 = ngb[i];
                }
            }
            else
            {
                _alpha = alpha;
                ind1 = ngb[i];
            }
        }
        //cout << " angle to " << ngb[i] << ": " << alpha << endl;

    }


    //cout << " winner: " << ind1 << endl;
    _spiral.push_back(ind1);
    return ind1;
}


// build the sequence of points
void initAuto(std::vector<cv::Point> &_X)
{
    // if previous ordering is given simply take the nearest
    /* if(_Xprev.size() == -1)
    {
        std::vector<cv::Point> Xold = _X;
        unsigned int idx_min;
        double d_min, d;

        for(unsigned int i=0;i<36;++i)
        {
            d_min = 1000;
            // find nearest point
            for(unsigned int j=0;j<36;++j)
            {
                d = (_Xprev[j].x - Xold[i].x)*(_Xprev[j].x - Xold[i].x) + (_Xprev[j].y - Xold[i].y)*(_Xprev[j].y - Xold[i].y);
                if(d < d_min)
                {
                    d_min = d;
                    idx_min = j;
                }
            }
            _X[idx_min] = Xold[i];
        }
    }
    else*/
    {
        // first point: farest from the center of gravity
        double xCog=0, yCog=0;
        unsigned int i;

        for(i=0;i<_X.size();++i)
        {
            xCog += _X[i].x;
            yCog += _X[i].y;
        }

        xCog /= _X.size();
        yCog /= _X.size();
        cv::Point cog;cog.x = int(xCog);cog.y = int(yCog);

        double dMax = (xCog - _X[0].x)*(xCog - _X[0].x) + (yCog - _X[0].y)*(yCog - _X[0].y), d;
        unsigned int ind0 = 0;
        for(i=1;i<_X.size();++i)
        {
            d = (xCog - _X[i].x)*(xCog - _X[i].x) + (yCog - _X[i].y)*(yCog - _X[i].y);
            if(d > dMax)
            {
                d = dMax;
                ind0 = i;
            }
        }

        std::vector<unsigned int> spiral;
        spiral.push_back(ind0);
        // find other points from initiated sequence, counter-clockwise
        unsigned int ind1, ind2;
        bool check_corner = true;
        double alpha;
        while(spiral.size() < _X.size())
        {
            if(spiral.size() == 1)
            {
                //cout << "restarting from " << ind0 << endl;
                ind1 = getNextPoint(cog, ind0, _X, spiral, alpha, true);
            }
            else
            {
                //cout << "starting from (" << ind0 << ", " << ind1 << ")" << endl;
                ind2 = getNextPoint(_X[ind0], ind1, _X, spiral, alpha);
                ind0 = ind1;
                ind1 = ind2;
                // check corner, we may have begun on a side
                if(check_corner && alpha > M_PI/4)
                {
                    check_corner = false;
                    // reinit
                    spiral.clear();
                    spiral.push_back(ind0);
                }

            }

        }

        // build indices from spiral
        std::vector<unsigned int> idx(36);
        for(unsigned int i=0;i<36;++i)
        {
            //cout << spiral[i] << ", ";
            idx[i] = 36;
        }
        //cout << endl;
        int x=0, y=0, dx=1, dy=0;
        int xn=0,yn=0;
        bool turn;
        for(unsigned int i=0;i<36;++i)
        {
            turn = true;
            idx[x + 6*y] = spiral[i];
            //cout << "adding " << spiral[i] << " at (" << x << ", " << y << ")" << endl;
            xn = x+dx;
            yn = y+dy;
            // check if we stay on the grid
            if(xn>=0 && xn<6 && yn>=0 && yn<6)
            {
                if(idx[xn+6*yn] == 36)  // point was not written yet
                {
                    turn = false;
                }
            }
            if(turn)
            {
                //cout << "  turn" << endl;
                if(dy == 0)
                {
                    dy = dx;
                    dx = 0;
                }
                else
                {
                    dx = -dy;
                    dy = 0;
                }
                x = x+dx;
                y = y+dy;
            }
            else
            {
                x = xn;
                y = yn;
            }
        }

        //idx = spiral;

        // reorder
        std::vector<cv::Point> Xold = _X;
        for(unsigned int i=0;i<36;++i)
        {
            ////cout << idx[i] << ", ";
            _X[i] = Xold[idx[i]];
        }
        //  //cout << endl;
    }
}
}

#endif // GRID_DETECTION_H
